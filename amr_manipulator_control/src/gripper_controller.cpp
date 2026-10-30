#include "amr_manipulator_control/gripper_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace amr_manipulator_control
{

namespace
{
// One row per numeric parameter: the same table declares, reads and live-updates them
struct Entry
{
  const char * name;
  double GripperParams::* field;  // pointer to a member: p.*(entry.field) is that member of p
};

constexpr Entry kEntries[] = {
  {"kp_c", &GripperParams::kp_c},
  {"kd_c", &GripperParams::kd_c},
  {"kp_d", &GripperParams::kp_d},
  {"kd_d", &GripperParams::kd_d},
  {"fd_max", &GripperParams::fd_max},
  {"open_bias", &GripperParams::open_bias},
  {"open_speed", &GripperParams::open_speed},
  {"approach_speed", &GripperParams::approach_speed},
  {"approach_force", &GripperParams::approach_force},
  {"grip_force", &GripperParams::grip_force},
  {"grip_ramp", &GripperParams::grip_ramp},
  {"force_limit", &GripperParams::force_limit},
  {"settle_time", &GripperParams::settle_time},
  {"stall_ratio", &GripperParams::stall_ratio},
  {"stall_time", &GripperParams::stall_time},
  {"contact_force", &GripperParams::contact_force},
  {"q_max", &GripperParams::q_max},
  {"open_gap", &GripperParams::open_gap},
  {"empty_margin", &GripperParams::empty_margin},
  {"empty_force", &GripperParams::empty_force},
};

constexpr double kOpenTol = 5e-4;  // [m] a finger this close to the stop counts as open

// Consistency checks shared by on_configure() and the live parameter callback
bool is_valid(const GripperParams & p, std::string & why)
{
  for (const auto & e : kEntries) {
    if (!std::isfinite(p.*(e.field))) {
      why = std::string(e.name) + " must be finite";
      return false;
    }
  }
  if (!(p.kp_c > 0.0) || !(p.kp_d > 0.0) || !(p.kd_c >= 0.0) || !(p.kd_d >= 0.0)) {
    why = "kp_c and kp_d must be > 0, kd_c and kd_d >= 0";
    return false;
  }
  if (!(p.open_speed > 0.0) || !(p.approach_speed > 0.0) || !(p.grip_ramp > 0.0) ||
    !(p.stall_time > 0.0) || !(p.settle_time >= 0.0) || !(p.contact_force >= 0.0))
  {
    why = "open_speed, approach_speed, grip_ramp and stall_time must be > 0; "
      "settle_time and contact_force >= 0";
    return false;
  }
  if (!(p.stall_ratio > 0.0) || !(p.stall_ratio < 1.0)) {
    why = "stall_ratio must be in (0, 1)";
    return false;
  }
  if (!(p.force_limit > 0.0) || p.fd_max < 0.0 || p.fd_max > p.force_limit ||
    p.open_bias < 0.0 || p.open_bias > p.force_limit ||
    p.approach_force < 0.0 || p.approach_force > p.force_limit ||
    !(p.grip_force > 0.0) || p.grip_force > p.force_limit ||
    p.empty_force < 0.0 || p.empty_force > p.force_limit)
  {
    why = "forces must be >= 0 and not above force_limit (> 0); grip_force must be > 0";
    return false;
  }
  if (!(p.q_max > 0.0) || !(p.open_gap > 0.0) || p.empty_margin < 0.0 ||
    !(p.empty_margin < p.q_max))
  {
    why = "q_max and open_gap must be > 0, 0 <= empty_margin < q_max";
    return false;
  }
  return true;
}
}  // namespace

controller_interface::InterfaceConfiguration
GripperController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration
GripperController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // Order matters: state_interfaces_[0..1] = positions (left, right),
  //                state_interfaces_[2..3] = velocities (left, right)
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

controller_interface::CallbackReturn GripperController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    const GripperParams defaults;
    for (const auto & e : kEntries) {
      auto_declare<double>(e.name, defaults.*(e.field));
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GripperController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  GripperParams p;
  try {
    joint_names_ = node->get_parameter("joints").as_string_array();
    for (const auto & e : kEntries) {
      p.*(e.field) = node->get_parameter(e.name).as_double();
    }
  } catch (const std::exception & e) {
    // Typical cause: a number written as 25 (integer) instead of 25.0
    RCLCPP_ERROR(node->get_logger(), "Bad parameter: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (joint_names_.size() != 2) {
    RCLCPP_ERROR(node->get_logger(),
      "'joints' must list exactly two joints: [left finger, right finger]");
    return controller_interface::CallbackReturn::ERROR;
  }
  std::string why;
  if (!is_valid(p, why)) {
    RCLCPP_ERROR(node->get_logger(), "Invalid parameters: %s", why.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  {
    std::lock_guard<std::mutex> lock(params_mutex_);
    params_ = p;
  }
  p_ = p;
  params_dirty_ = false;

  // Command topic: [mode] or [mode, force]; mode 0 = open, 1 = close; force in N (0 = default)
  command_sub_.reset();
  command_sub_ = node->create_subscription<Float64MultiArray>(
    "~/command", rclcpp::SystemDefaultsQoS(),
    [this](const Float64MultiArray & msg) {on_command(msg);});

  // State topic: [state, xc, xd, vc, f_left, f_right, width, grip_target, fc], 50 Hz
  state_valid_ = false;
  state_pub_ = node->create_publisher<Float64MultiArray>("~/state", rclcpp::SystemDefaultsQoS());
  state_timer_.reset();
  state_timer_ = node->create_wall_timer(
    std::chrono::milliseconds(20), [this]() {publish_state();});

  // Live-tunable parameters (ros2 param set ...): validated as a whole, then handed over
  param_cb_handle_.reset();
  param_cb_handle_ = node->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      std::lock_guard<std::mutex> lock(params_mutex_);
      GripperParams candidate = params_;
      for (const auto & prm : params) {
        for (const auto & e : kEntries) {
          if (prm.get_name() != e.name) {
            continue;
          }
          if (prm.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
            result.successful = false;
            result.reason = prm.get_name() + " must be a double (write 25.0, not 25)";
            return result;
          }
          candidate.*(e.field) = prm.as_double();
        }
      }
      std::string reason;
      if (!is_valid(candidate, reason)) {
        result.successful = false;
        result.reason = reason;
        return result;
      }
      params_ = candidate;
      params_dirty_ = true;  // the control thread picks it up at the next cycle
      return result;
    });

  RCLCPP_INFO(node->get_logger(), "Gripper controller: left '%s', right '%s'",
    joint_names_[0].c_str(), joint_names_[1].c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GripperController::on_activate(
  const rclcpp_lifecycle::State &)
{
  {
    std::lock_guard<std::mutex> lock(params_mutex_);
    p_ = params_;
    params_dirty_ = false;
  }
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd_pending_ = false;  // forget commands sent while the controller was not active
  }
  // Start by opening: each finger ramps down from wherever it is (it may have fallen shut)
  open_ref_ = {state_interfaces_[0].get_value(), state_interfaces_[1].get_value()};
  state_ = State::kOpening;
  t_state_ = 0.0;
  stall_acc_ = 0.0;
  xr_c_ = 0.0;
  fc_contact_ = 0.0;
  fc_last_ = 0.0;
  grip_target_ = p_.grip_force;
  width_ = 0.0;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GripperController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // Effort commands are held by the simulator: leave zeros behind, not a stale force
  for (auto & cmd : command_interfaces_) {
    cmd.set_value(0.0);
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_valid_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

void GripperController::on_command(const Float64MultiArray & msg)
{
  if (msg.data.empty() || msg.data.size() > 2) {
    RCLCPP_WARN(get_node()->get_logger(), "Command ignored: expected [mode] or [mode, force]");
    return;
  }
  const double mode = msg.data[0];
  const double force = (msg.data.size() == 2) ? msg.data[1] : 0.0;
  double limit = 0.0;
  {
    std::lock_guard<std::mutex> lock(params_mutex_);
    limit = params_.force_limit;
  }
  if (!(mode == 0.0 || mode == 1.0) || !std::isfinite(force) || force < 0.0 || force > limit) {
    RCLCPP_WARN(get_node()->get_logger(),
      "Command ignored: mode must be 0 (open) or 1 (close), force in [0, %.1f] N", limit);
    return;
  }
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  cmd_mode_ = static_cast<int>(mode);
  cmd_force_ = force;
  cmd_pending_ = true;
}

void GripperController::apply_command(
  int mode, double force, double ql, double qr, double xc)
{
  const double target = (force > 0.0) ? force : p_.grip_force;
  t_state_ = 0.0;
  stall_acc_ = 0.0;
  if (mode == 0) {
    open_ref_ = {ql, qr};  // each finger ramps down from its own position
    state_ = State::kOpening;
    width_ = 0.0;
  } else if (state_ == State::kGrasp) {
    // Already holding: only re-target the force, ramping from the current value (no release)
    fc_contact_ = fc_last_;
    grip_target_ = target;
  } else {
    xr_c_ = xc;
    grip_target_ = target;
    width_ = 0.0;
    state_ = State::kClosing;
  }
}

void GripperController::publish_state()
{
  Float64MultiArray msg;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!state_valid_) {
      return;
    }
    msg.data.assign(state_data_.begin(), state_data_.end());  // copy under the lock
  }
  state_pub_->publish(msg);
}

controller_interface::return_type GripperController::update(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  const double dt = period.seconds();

  // 0. New parameters from the parameter thread (never block the control loop)
  if (params_dirty_.load()) {
    std::unique_lock<std::mutex> lock(params_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      p_ = params_;
      params_dirty_ = false;
    }
  }

  // 1. Measured state, in common (closing) and differential (de-centring) coordinates
  const double ql = state_interfaces_[0].get_value();
  const double qr = state_interfaces_[1].get_value();
  const double vl = state_interfaces_[2].get_value();
  const double vr = state_interfaces_[3].get_value();
  const double xc = 0.5 * (ql + qr);
  const double xd = 0.5 * (ql - qr);
  const double vc = 0.5 * (vl + vr);
  const double vd = 0.5 * (vl - vr);

  // 2. A new command, if one is waiting
  {
    std::unique_lock<std::mutex> lock(cmd_mutex_, std::try_to_lock);
    if (lock.owns_lock() && cmd_pending_) {
      cmd_pending_ = false;
      apply_command(cmd_mode_, cmd_force_, ql, qr, xc);
    }
  }

  // 3. State machine: forces towards the centre on the left (fl) and right (fr) finger
  t_state_ += dt;
  double fl = 0.0;
  double fr = 0.0;
  double fc = 0.0;
  bool centring = true;  // differential PD on, except while opening
  switch (state_) {
    case State::kOpen:
    case State::kOpening: {
      // Each finger follows its own reference down to the stop: works from any pose
      centring = false;
      const double step = p_.open_speed * dt;
      open_ref_[0] = std::max(0.0, open_ref_[0] - step);
      open_ref_[1] = std::max(0.0, open_ref_[1] - step);
      const double vref_l = (open_ref_[0] > 0.0) ? -p_.open_speed : 0.0;
      const double vref_r = (open_ref_[1] > 0.0) ? -p_.open_speed : 0.0;
      fl = p_.kp_c * (open_ref_[0] - ql) + p_.kd_c * (vref_l - vl);
      fr = p_.kp_c * (open_ref_[1] - qr) + p_.kd_c * (vref_r - vr);
      // Once the reference has reached the stop, keep pushing against it (gravity-proof)
      if (open_ref_[0] == 0.0) {
        fl -= p_.open_bias;
      }
      if (open_ref_[1] == 0.0) {
        fr -= p_.open_bias;
      }
      if (state_ == State::kOpening && open_ref_[0] == 0.0 && open_ref_[1] == 0.0 &&
        ql < kOpenTol && qr < kOpenTol)
      {
        state_ = State::kOpen;
      }
      break;
    }
    case State::kClosing: {
      // Reference moves at approach_speed; the tracking force is saturated
      xr_c_ = std::min(p_.q_max, xr_c_ + p_.approach_speed * dt);
      const double raw = p_.kp_c * (xr_c_ - xc) + p_.kd_c * (p_.approach_speed - vc);
      fc = std::clamp(raw, -p_.approach_force, p_.approach_force);
      // Contact = the fingers have slowed down although the controller keeps pushing
      const bool stalled =
        vc < p_.stall_ratio * p_.approach_speed && fc > p_.contact_force;
      stall_acc_ = (stalled && t_state_ > p_.settle_time) ? stall_acc_ + dt : 0.0;
      if (stall_acc_ >= p_.stall_time) {
        state_ = State::kGrasp;
        t_state_ = 0.0;
        fc_contact_ = fc;
        width_ = p_.open_gap - 2.0 * xc;
      } else if (xc > p_.q_max - p_.empty_margin) {
        state_ = State::kEmpty;  // closed on nothing
      }
      break;
    }
    case State::kGrasp: {
      // Force control: ramp from the contact force to the requested grip force
      const double a = std::min(1.0, t_state_ / p_.grip_ramp);
      fc = (1.0 - a) * fc_contact_ + a * grip_target_;
      if (xc > p_.q_max - p_.empty_margin) {
        state_ = State::kEmpty;  // the object slipped out
      }
      break;
    }
    case State::kEmpty: {
      fc = p_.empty_force;
      break;
    }
  }

  // 4. Combine common and differential modes: fl = fc + fd, fr = fc - fd
  if (centring) {
    const double fd = std::clamp(-p_.kp_d * xd - p_.kd_d * vd, -p_.fd_max, p_.fd_max);
    fl = fc + fd;
    fr = fc - fd;
  } else {
    fc = 0.5 * (fl + fr);
  }
  fc_last_ = fc;

  // 5. Saturation and output (non-finite values become zero force)
  const auto limit = [this](double f) {
      return std::isfinite(f) ? std::clamp(f, -p_.force_limit, p_.force_limit) : 0.0;
    };
  fl = limit(fl);
  fr = limit(fr);
  command_interfaces_[0].set_value(fl);
  command_interfaces_[1].set_value(fr);

  // 6. State sample for the timer thread; skipped if that thread holds the lock
  {
    std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      state_data_ = {static_cast<double>(static_cast<int>(state_)), xc, xd, vc, fl, fr,
        width_, grip_target_, fc};
      state_valid_ = true;
    }
  }
  return controller_interface::return_type::OK;
}

}  // namespace amr_manipulator_control

PLUGINLIB_EXPORT_CLASS(
  amr_manipulator_control::GripperController, controller_interface::ControllerInterface)
