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
constexpr double kPi = 3.14159265358979323846;
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
    auto_declare<std::vector<double>>("omega", std::vector<double>());
    auto_declare<double>("coriolis_scale", 1.0);
    auto_declare<int>("mode", 0);
    auto_declare<std::vector<double>>("velocity_limits", std::vector<double>());
    auto_declare<std::vector<double>>("friction_coulomb", std::vector<double>());
    auto_declare<std::vector<double>>("friction_viscous", std::vector<double>());
    auto_declare<double>("friction_smoothing", 0.05);
    auto_declare<double>("friction_scale", 1.0);
    auto_declare<double>("velocity_noise_std", 0.0);
    auto_declare<double>("velocity_filter_hz", 0.0);
    auto_declare<int>("noise_seed", 42);
    auto_declare<double>("adapt_gain", 0.0);
    auto_declare<double>("payload_estimate", 0.0);
    auto_declare<double>("payload_min", 0.0);
    auto_declare<double>("payload_max", 6.0);
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
  std::vector<double> omega;
  int noise_seed = 42;

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
    omega = node->get_parameter("omega").as_double_array();
    velocity_limits_ = node->get_parameter("velocity_limits").as_double_array();

    fric_coulomb_ = node->get_parameter("friction_coulomb").as_double_array();
    fric_viscous_ = node->get_parameter("friction_viscous").as_double_array();
    fric_smoothing_ = node->get_parameter("friction_smoothing").as_double();
    friction_scale_ = node->get_parameter("friction_scale").as_double();
    velocity_noise_std_ = node->get_parameter("velocity_noise_std").as_double();
    velocity_filter_hz_ = node->get_parameter("velocity_filter_hz").as_double();
    noise_seed = static_cast<int>(node->get_parameter("noise_seed").as_int());

    adapt_gain_ = node->get_parameter("adapt_gain").as_double();
    payload_init_ = node->get_parameter("payload_estimate").as_double();
    payload_min_ = node->get_parameter("payload_min").as_double();
    payload_max_ = node->get_parameter("payload_max").as_double();

    coriolis_scale_ = node->get_parameter("coriolis_scale").as_double();
    mode_ = static_cast<int>(node->get_parameter("mode").as_int());
  } catch (const std::exception & e) {
    // Typical cause: an array written as [150, 150] (integers) instead of [150.0, 150.0]
    RCLCPP_ERROR(node->get_logger(), "Bad parameter: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  const std::size_t n = joint_names_.size();
  if (n == 0 || target_.size() != n || kp_.size() != n || kd_.size() != n ||
    effort_limits_.size() != n || omega.size() != n || velocity_limits_.size() != n ||
    fric_coulomb_.size() != n || fric_viscous_.size() != n)
  {
    RCLCPP_ERROR(node->get_logger(),
      "joints, target_positions, kp, kd, effort_limits, omega, velocity_limits, "
      "friction_coulomb and friction_viscous must all have the same non-zero size");
    return controller_interface::CallbackReturn::ERROR;
  }
  for (const double v : velocity_limits_) {
    if (!(v > 0.0)) {
      RCLCPP_ERROR(node->get_logger(), "velocity_limits must be strictly positive");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  if (!(fric_smoothing_ > 0.0)) {
    RCLCPP_ERROR(node->get_logger(), "friction_smoothing must be strictly positive");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!(payload_min_ >= 0.0) || !(payload_max_ > payload_min_) ||
    !(payload_init_ >= payload_min_) || !(payload_init_ <= payload_max_) ||
    !(adapt_gain_.load() >= 0.0))
  {
    RCLCPP_ERROR(node->get_logger(),
      "payload_min/payload_max/payload_estimate/adapt_gain are inconsistent");
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

  // Point mass lumped at the tip frame (fingers and camera; the grasped payload is estimated)
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

  // Payload model: same kinematics, massless links, plus a 1 kg point mass at the tip
  payload_dyn_.reset();  // drop the reference to the old chain before replacing it
  payload_chain_ = KDL::Chain();
  for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
    const KDL::Segment & seg = chain_.getSegment(i);
    payload_chain_.addSegment(KDL::Segment(
      seg.getJoint(), seg.getFrameToTip(), KDL::RigidBodyInertia::Zero()));
  }
  payload_chain_.addSegment(KDL::Segment(
    "payload", KDL::Joint(), KDL::Frame::Identity(),
    KDL::RigidBodyInertia(1.0, KDL::Vector::Zero(), KDL::RotationalInertia(0, 0, 0))));
  payload_dyn_ = std::make_unique<KDL::ChainDynParam>(
    payload_chain_, KDL::Vector(0.0, 0.0, -kGravity));

  q_ = KDL::JntArray(n);
  g_ = KDL::JntArray(n);
  qdot_ = KDL::JntArray(n);
  c_ = KDL::JntArray(n);
  m_.resize(n);
  a_ = Eigen::VectorXd::Zero(n);
  tau_ = Eigen::VectorXd::Zero(n);

  // Payload model buffers and Slotine-Li variables (no allocation inside update())
  mp_.resize(n);
  gp_ = KDL::JntArray(n);
  cp_ = KDL::JntArray(n);
  qdot_r_ = KDL::JntArray(n);
  xy_ = KDL::JntArray(n);
  h_x_ = KDL::JntArray(n);
  h_y_ = KDL::JntArray(n);
  h_xy_ = KDL::JntArray(n);
  qdd_r_ = Eigen::VectorXd::Zero(n);
  s_ = Eigen::VectorXd::Zero(n);
  z_ = Eigen::VectorXd::Zero(n);
  y_ = Eigen::VectorXd::Zero(n);
  payload_hat_ = payload_init_;
  payload_set_pending_ = false;

  rng_.seed(static_cast<std::mt19937::result_type>(noise_seed));
  qd_filt_.assign(n, 0.0);
  filter_primed_ = false;

  // Computed torque gains: critically damped error dynamics e'' + 2w e' + w^2 e = 0
  ct_kp_.resize(n);
  ct_kd_.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    ct_kp_[i] = omega[i] * omega[i];
    ct_kd_[i] = 2.0 * omega[i];
  }

  // Trajectory state; the first trajectory is started in on_activate()
  q_start_.assign(n, 0.0);
  q_goal_.assign(n, 0.0);
  ref_q_.assign(n, 0.0);
  ref_qd_.assign(n, 0.0);
  pending_goal_.assign(n, 0.0);
  goal_pending_ = false;

  // Goal topic: [q_1 ... q_n, duration]
  goal_sub_.reset();
  goal_sub_ = node->create_subscription<Float64MultiArray>(
    "~/goal", rclcpp::SystemDefaultsQoS(),
    [this](const Float64MultiArray & msg) {on_goal(msg);});

  // Debug topic: [t_traj, T, q_d(n), q(n), qd_d(n), qd(n), tau(n)], published at 50 Hz
  debug_data_.assign(2 + 5 * n, 0.0);
  debug_valid_ = false;
  state_pub_ = node->create_publisher<Float64MultiArray>("~/state", rclcpp::SystemDefaultsQoS());
  payload_pub_ = node->create_publisher<Float64MultiArray>(
    "~/payload", rclcpp::SystemDefaultsQoS());
  debug_timer_.reset();
  debug_timer_ = node->create_wall_timer(
    std::chrono::milliseconds(20), [this]() {publish_debug();});

  // Live-tunable parameters (ros2 param set ...)
  param_cb_handle_.reset();
  param_cb_handle_ = node->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : params) {
        const std::string & name = p.get_name();
        std::atomic<double> * live = nullptr;  // live-tunable double parameters
        if (name == "gravity_scale") {
          live = &gravity_scale_;
        } else if (name == "pd_scale") {
          live = &pd_scale_;
        } else if (name == "coriolis_scale") {
          live = &coriolis_scale_;
        } else if (name == "friction_scale") {
          live = &friction_scale_;
        } else if (name == "velocity_noise_std") {
          live = &velocity_noise_std_;
        } else if (name == "velocity_filter_hz") {
          live = &velocity_filter_hz_;
        } else if (name == "adapt_gain") {
          live = &adapt_gain_;
        }

        if (live != nullptr) {
          if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
            result.successful = false;
            result.reason = name + " must be a double (write 1.0, not 1)";
            return result;
          }
          const double value = p.as_double();
          if (!std::isfinite(value) || value < 0.0) {
            result.successful = false;
            result.reason = name + " must be finite and >= 0";
            return result;
          }
          *live = value;
        } else if (name == "payload_estimate") {
          if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
            result.successful = false;
            result.reason = name + " must be a double (write 3.0, not 3)";
            return result;
          }
          const double value = p.as_double();
          if (!std::isfinite(value) || value < payload_min_ || value > payload_max_) {
            result.successful = false;
            result.reason = name + " must be within [payload_min, payload_max]";
            return result;
          }
          payload_set_ = value;  // applied by the control thread at the next cycle
          payload_set_pending_ = true;
        } else if (name == "mode") {
          if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
            result.successful = false;
            result.reason =
              "mode must be an integer (0 = PD + gravity, 1 = computed torque, 2 = adaptive payload)";
            return result;
          }
          const auto value = p.as_int();
          if (value < 0 || value > 2) {
            result.successful = false;
            result.reason = "mode must be 0, 1 or 2";
            return result;
          }
          mode_ = static_cast<int>(value);
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

void ArmModelController::on_goal(const Float64MultiArray & msg)
{
  const std::size_t n = joint_names_.size();
  if (msg.data.size() != n + 1) {
    RCLCPP_WARN(get_node()->get_logger(),
      "Goal ignored: expected %zu values (targets + duration), got %zu", n + 1, msg.data.size());
    return;
  }
  const double duration = msg.data[n];
  bool ok = std::isfinite(duration) && duration >= 0.2;
  for (std::size_t i = 0; i < n; ++i) {
    ok = ok && std::isfinite(msg.data[i]);
  }
  if (!ok) {
    RCLCPP_WARN(get_node()->get_logger(), "Goal ignored: non-finite value or duration < 0.2 s");
    return;
  }
  std::lock_guard<std::mutex> lock(goal_mutex_);
  pending_goal_.assign(msg.data.begin(), msg.data.begin() + n);
  pending_time_ = duration;
  goal_pending_ = true;
}

void ArmModelController::publish_debug()
{
  Float64MultiArray msg;
  Float64MultiArray payload_msg;
  {
    std::lock_guard<std::mutex> lock(debug_mutex_);
    if (!debug_valid_) {
      return;
    }
    msg.data = debug_data_;  // copy while holding the lock, publish after releasing it
    payload_msg.data = {debug_payload_};
  }
  state_pub_->publish(msg);
  payload_pub_->publish(payload_msg);
}

bool ArmModelController::coriolis_bilinear(
  KDL::ChainDynParam & dyn, const KDL::JntArray & x, const KDL::JntArray & y,
  KDL::JntArray & out)
{
  const unsigned int n = x.rows();
  for (unsigned int i = 0; i < n; ++i) {
    xy_(i) = x(i) + y(i);
  }
  // h(v) = C(q, v) v is what KDL returns; C(q, x) y is the polarisation of that quadratic form
  if (dyn.JntToCoriolis(q_, x, h_x_) < 0 || dyn.JntToCoriolis(q_, y, h_y_) < 0 ||
    dyn.JntToCoriolis(q_, xy_, h_xy_) < 0)
  {
    return false;
  }
  for (unsigned int i = 0; i < n; ++i) {
    out(i) = 0.5 * (h_xy_(i) - h_x_(i) - h_y_(i));
  }
  return true;
}

controller_interface::CallbackReturn ArmModelController::on_activate(
  const rclcpp_lifecycle::State &)
{
  // First trajectory: from the measured pose to the configured target
  double t_min = 0.0;
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    q_start_[i] = state_interfaces_[i].get_value();
    q_goal_[i] = target_[i];
    // Quintic peak speed is 1.875 * |delta| / T: stretch T to stay within the limit
    t_min = std::max(t_min, 1.875 * std::abs(target_[i] - q_start_[i]) / velocity_limits_[i]);
  }
  traj_time_ = std::max({ramp_time_, 0.2, t_min});
  t_ = 0.0;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    goal_pending_ = false;
  }
  filter_primed_ = false;  // the filter restarts from the first measurement
  payload_hat_ = payload_init_;  // every activation restarts from the configured estimate
  payload_set_pending_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArmModelController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // Effort commands are held by the simulator: leave zeros behind, not a stale torque
  for (auto & cmd : command_interfaces_) {
    cmd.set_value(0.0);
  }
  std::lock_guard<std::mutex> lock(debug_mutex_);
  debug_valid_ = false;  // stop publishing stale samples
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ArmModelController::update(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  const std::size_t n = joint_names_.size();
  const double dt = period.seconds();

  // 0. A payload estimate set from outside (ros2 param set) is applied by the control thread
  if (payload_set_pending_.exchange(false)) {
    payload_hat_ = std::clamp(payload_set_.load(), payload_min_, payload_max_);
  }

  // 1. Take a pending goal when the current trajectory is over; stretch T to respect
  //    the velocity limits (quintic peak speed is 1.875 * |delta| / T)
  if (t_ >= traj_time_) {
    std::unique_lock<std::mutex> lock(goal_mutex_, std::try_to_lock);
    if (lock.owns_lock() && goal_pending_) {
      double t_min = 0.0;
      for (std::size_t i = 0; i < n; ++i) {
        t_min = std::max(
          t_min, 1.875 * std::abs(pending_goal_[i] - q_goal_[i]) / velocity_limits_[i]);
      }
      q_start_ = q_goal_;
      q_goal_ = pending_goal_;
      traj_time_ = std::max(pending_time_, t_min);
      t_ = 0.0;
      goal_pending_ = false;
    }
  }
  t_ += dt;

  // 2. Quintic blending: s(u) = 10u^3 - 15u^4 + 6u^5, u = t/T in [0, 1]
  const double T = traj_time_;
  const double u = std::min(t_ / T, 1.0);
  const double s = u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
  const double ds = 30.0 * u * u * (1.0 - u) * (1.0 - u) / T;
  const double dds = 60.0 * u * (1.0 - u) * (1.0 - 2.0 * u) / (T * T);

  // 3. Measured state. Velocity = simulated sensor noise + first-order low-pass filter
  const int mode = mode_.load();
  const double gs = gravity_scale_.load();
  const double ps = pd_scale_.load();
  const double cs = coriolis_scale_.load();
  const double fs = friction_scale_.load();
  const double sigma = velocity_noise_std_.load();
  const double fc = velocity_filter_hz_.load();
  const double alpha = (fc > 0.0) ? dt / (1.0 / (2.0 * kPi * fc) + dt) : 1.0;  // 1 = no filter
  for (std::size_t i = 0; i < n; ++i) {
    q_(i) = state_interfaces_[i].get_value();
    const double v = state_interfaces_[n + i].get_value() + sigma * normal_(rng_);
    qd_filt_[i] = filter_primed_ ? qd_filt_[i] + alpha * (v - qd_filt_[i]) : v;
    qdot_(i) = qd_filt_[i];
  }
  filter_primed_ = true;

  // 4. Nominal model terms available before the references
  if (dyn_->JntToGravity(q_, g_) < 0) {
    return controller_interface::return_type::ERROR;
  }
  if (mode == 1) {
    if (dyn_->JntToMass(q_, m_) < 0 || dyn_->JntToCoriolis(q_, qdot_, c_) < 0) {
      return controller_interface::return_type::ERROR;
    }
  }

  // 5. Reference and feedback, joint by joint
  for (std::size_t i = 0; i < n; ++i) {
    const double delta = q_goal_[i] - q_start_[i];
    ref_q_[i] = q_start_[i] + s * delta;
    ref_qd_[i] = ds * delta;
    const double ref_qdd = dds * delta;
    const double e = ref_q_[i] - q_(i);
    const double ed = ref_qd_[i] - qdot_(i);
    if (mode >= 1) {
      a_(i) = ref_qdd + ct_kd_[i] * ed + ct_kp_[i] * e;  // commanded acceleration
    } else {
      tau_(i) = ps * (kp_[i] * e + kd_[i] * ed);          // PD part only
    }
    if (mode == 2) {
      // Slotine-Li variables with Lambda = omega = ct_kd / 2
      const double lambda = 0.5 * ct_kd_[i];
      s_(i) = ed + lambda * e;                  // filtered tracking error
      qdot_r_(i) = ref_qd_[i] + lambda * e;     // reference velocity
      qdd_r_(i) = ref_qdd + lambda * ed;        // reference acceleration
    }
  }

  // 6. Torque: PD + g (mode 0), M a + C qdot + g (mode 1), Slotine-Li with payload (mode 2)
  if (mode == 1) {
    tau_.noalias() = m_.data * a_;
    tau_ += cs * c_.data;
  } else if (mode == 2) {
    // Known model: M0 a + C0(q, qdot) qdot_r
    if (dyn_->JntToMass(q_, m_) < 0 || !coriolis_bilinear(*dyn_, qdot_, qdot_r_, c_)) {
      return controller_interface::return_type::ERROR;
    }
    // Unit payload: Mp, Cp(q, qdot) qdot_r, gp
    if (payload_dyn_->JntToMass(q_, mp_) < 0 || payload_dyn_->JntToGravity(q_, gp_) < 0 ||
      !coriolis_bilinear(*payload_dyn_, qdot_, qdot_r_, cp_))
    {
      return controller_interface::return_type::ERROR;
    }
    tau_.noalias() = m_.data * a_;
    tau_ += cs * c_.data;
    // Payload part of the control: m_hat * (Mp a + Cp qdot_r + gp)
    z_.noalias() = mp_.data * a_;
    z_ += cs * cp_.data;
    z_ += gs * gp_.data;
    tau_ += payload_hat_ * z_;
    // Regressor of the adaptation law (uses qdd_r, not a): Y = Mp qdd_r + Cp qdot_r + gp
    y_.noalias() = mp_.data * qdd_r_;
    y_ += cs * cp_.data;
    y_ += gs * gp_.data;
  }
  tau_ += gs * g_.data;

  // 7. Friction feedforward on the reference velocity, saturation and output
  bool saturated = false;
  for (std::size_t i = 0; i < n; ++i) {
    const double tau_fric = fs * (fric_viscous_[i] * ref_qd_[i] +
      fric_coulomb_[i] * std::tanh(ref_qd_[i] / fric_smoothing_));
    const double wanted = tau_(i) + tau_fric;
    double tau = std::clamp(wanted, -effort_limits_[i], effort_limits_[i]);
    if (!std::isfinite(tau)) {
      tau = 0.0;
    }
    if (tau != wanted) {
      saturated = true;  // clipped or not finite: do not adapt on this cycle
    }
    tau_(i) = tau;
    command_interfaces_[i].set_value(tau);
  }

  // 8. Adaptation law (Slotine-Li, projected): dm/dt = gamma * Y^T s
  const double gamma = adapt_gain_.load();
  if (mode == 2 && gamma > 0.0 && !saturated) {
    payload_hat_ = std::clamp(payload_hat_ + dt * gamma * y_.dot(s_), payload_min_, payload_max_);
  }

  // 9. Debug sample for the timer thread; skipped if that thread holds the lock
  {
    std::unique_lock<std::mutex> lock(debug_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      debug_data_[0] = t_;
      debug_data_[1] = traj_time_;
      for (std::size_t i = 0; i < n; ++i) {
        debug_data_[2 + i] = ref_q_[i];
        debug_data_[2 + n + i] = q_(i);
        debug_data_[2 + 2 * n + i] = ref_qd_[i];
        debug_data_[2 + 3 * n + i] = qdot_(i);
        debug_data_[2 + 4 * n + i] = tau_(i);
      }
      debug_payload_ = payload_hat_;
      debug_valid_ = true;
    }
  }
  return controller_interface::return_type::OK;
}

}  // namespace amr_manipulator_control

PLUGINLIB_EXPORT_CLASS(
  amr_manipulator_control::ArmModelController, controller_interface::ControllerInterface)