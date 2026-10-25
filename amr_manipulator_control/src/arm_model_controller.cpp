#include "amr_manipulator_control/arm_model_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace amr_manipulator_control
{

namespace
{
constexpr double kGravity = 9.81;  // [m/s^2], acting along -Z of the chain root frame
}

controller_interface::InterfaceConfiguration
ArmModelController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration
ArmModelController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // Order matters: state_interfaces_[i] = position of joint i,
  //                state_interfaces_[n + i] = velocity of joint i
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

controller_interface::CallbackReturn ArmModelController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::string>("chain_root", "");
    auto_declare<std::string>("chain_tip", "");
    auto_declare<std::string>("robot_description_node", "robot_state_publisher");
    auto_declare<double>("extra_mass", 0.0);
    auto_declare<double>("gravity_scale", 1.0);
    auto_declare<double>("pd_scale", 1.0);
    auto_declare<double>("ramp_time", 5.0);
    auto_declare<std::vector<double>>("target_positions", std::vector<double>());
    auto_declare<std::vector<double>>("kp", std::vector<double>());
    auto_declare<std::vector<double>>("kd", std::vector<double>());
    auto_declare<std::vector<double>>("effort_limits", std::vector<double>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool ArmModelController::fetch_urdf(const std::string & remote_node, std::string & urdf)
{
  // A throw-away node with its own internal executor: the controller's own node is
  // already spun by the controller manager, so we must not spin it here.
  auto helper = std::make_shared<rclcpp::Node>("arm_model_urdf_fetcher");
  rclcpp::SyncParametersClient client(helper, remote_node);
  if (!client.wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter service of '%s' not available",
      remote_node.c_str());
    return false;
  }
  const auto values = client.get_parameters({"robot_description"});
  if (values.empty() || values[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
    RCLCPP_ERROR(get_node()->get_logger(), "'%s' has no string parameter robot_description",
      remote_node.c_str());
    return false;
  }
  urdf = values[0].as_string();
  return true;
}

controller_interface::CallbackReturn ArmModelController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  std::string root, tip, urdf_node;
  double extra_mass = 0.0;

  try {
    joint_names_ = node->get_parameter("joints").as_string_array();
    root = node->get_parameter("chain_root").as_string();
    tip = node->get_parameter("chain_tip").as_string();
    urdf_node = node->get_parameter("robot_description_node").as_string();
    extra_mass = node->get_parameter("extra_mass").as_double();
    gravity_scale_ = node->get_parameter("gravity_scale").as_double();
    pd_scale_ = node->get_parameter("pd_scale").as_double();
    ramp_time_ = node->get_parameter("ramp_time").as_double();
    target_ = node->get_parameter("target_positions").as_double_array();
    kp_ = node->get_parameter("kp").as_double_array();
    kd_ = node->get_parameter("kd").as_double_array();
    effort_limits_ = node->get_parameter("effort_limits").as_double_array();
  } catch (const std::exception & e) {
    // Typical cause: an array written as [150, 150] (integers) instead of [150.0, 150.0]
    RCLCPP_ERROR(node->get_logger(), "Bad parameter: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  const std::size_t n = joint_names_.size();
  if (n == 0 || target_.size() != n || kp_.size() != n || kd_.size() != n ||
    effort_limits_.size() != n)
  {
    RCLCPP_ERROR(node->get_logger(),
      "joints, target_positions, kp, kd and effort_limits must all have the same non-zero size");
    return controller_interface::CallbackReturn::ERROR;
  }

  // URDF -> KDL tree -> chain
  std::string urdf;
  if (!fetch_urdf(urdf_node, urdf)) {
    return controller_interface::CallbackReturn::ERROR;
  }
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf, tree)) {
    RCLCPP_ERROR(node->get_logger(), "Could not build a KDL tree from the URDF");
    return controller_interface::CallbackReturn::ERROR;
  }
  chain_ = KDL::Chain();
  if (!tree.getChain(root, tip, chain_)) {
    RCLCPP_ERROR(node->get_logger(), "No KDL chain from '%s' to '%s'", root.c_str(), tip.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Point mass lumped at the tip frame (fingers now, grasped payload in step 3.6)
  if (extra_mass > 0.0) {
    chain_.addSegment(KDL::Segment(
      "extra_mass", KDL::Joint(), KDL::Frame::Identity(),
      KDL::RigidBodyInertia(extra_mass, KDL::Vector::Zero(), KDL::RotationalInertia(0, 0, 0))));
  }

  // The movable joints of the chain must be exactly our joints, in the same order
  std::vector<std::string> found;
  for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
    const std::string & name = chain_.getSegment(i).getJoint().getName();
    if (std::find(joint_names_.begin(), joint_names_.end(), name) != joint_names_.end()) {
      found.push_back(name);
    }
  }
  if (chain_.getNrOfJoints() != n || found != joint_names_) {
    RCLCPP_ERROR(node->get_logger(),
      "Chain has %u movable joints, expected the %zu listed in 'joints' in the same order",
      chain_.getNrOfJoints(), n);
    return controller_interface::CallbackReturn::ERROR;
  }

  dyn_ = std::make_unique<KDL::ChainDynParam>(chain_, KDL::Vector(0.0, 0.0, -kGravity));
  q_ = KDL::JntArray(n);
  g_ = KDL::JntArray(n);
  q0_.assign(n, 0.0);

    // Live-tunable parameters (ros2 param set ...)
  param_cb_handle_.reset();
  param_cb_handle_ = node->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : params) {
        if (p.get_name() != "gravity_scale" && p.get_name() != "pd_scale") {
          continue;
        }
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result.successful = false;
          result.reason = p.get_name() + " must be a double (write 1.0, not 1)";
          return result;
        }
        if (p.get_name() == "gravity_scale") {
          gravity_scale_ = p.as_double();
        } else {
          pd_scale_ = p.as_double();
        }
      }
      return result;
    });

  double total_mass = 0.0;
  for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
    total_mass += chain_.getSegment(i).getInertia().getMass();
  }
  RCLCPP_INFO(node->get_logger(),
    "KDL chain %s -> %s: %u segments, %u joints, total mass %.3f kg",
    root.c_str(), tip.c_str(), chain_.getNrOfSegments(), chain_.getNrOfJoints(), total_mass);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArmModelController::on_activate(
  const rclcpp_lifecycle::State &)
{
  // The reference ramp starts from wherever the arm is right now
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    q0_[i] = state_interfaces_[i].get_value();
  }
  t_ = 0.0;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArmModelController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // Effort commands are held by the simulator: leave zeros behind, not a stale torque
  for (auto & cmd : command_interfaces_) {
    cmd.set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ArmModelController::update(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  const std::size_t n = joint_names_.size();
  t_ += period.seconds();

  // Smoothstep ramp 0 -> 1 (zero velocity at both ends)
  const double x = (ramp_time_ > 0.0) ? std::min(t_ / ramp_time_, 1.0) : 1.0;
  const double s = x * x * (3.0 - 2.0 * x);

  const double gs = gravity_scale_.load();
  const double ps = pd_scale_.load();

  for (std::size_t i = 0; i < n; ++i) {
    q_(i) = state_interfaces_[i].get_value();
  }
  if (dyn_->JntToGravity(q_, g_) < 0) {
    return controller_interface::return_type::ERROR;
  }

  for (std::size_t i = 0; i < n; ++i) {
    const double qd = state_interfaces_[n + i].get_value();
    const double q_ref = q0_[i] + s * (target_[i] - q0_[i]);
    double tau = gs * g_(i) + ps * (kp_[i] * (q_ref - q_(i)) - kd_[i] * qd);
    tau = std::clamp(tau, -effort_limits_[i], effort_limits_[i]);
    if (!std::isfinite(tau)) {
      tau = 0.0;
    }
    command_interfaces_[i].set_value(tau);
  }
  return controller_interface::return_type::OK;
}

}  // namespace amr_manipulator_control

PLUGINLIB_EXPORT_CLASS(
  amr_manipulator_control::ArmModelController, controller_interface::ControllerInterface)
