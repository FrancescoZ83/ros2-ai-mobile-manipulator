#ifndef AMR_MANIPULATOR_CONTROL__GRIPPER_CONTROLLER_HPP_
#define AMR_MANIPULATOR_CONTROL__GRIPPER_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace amr_manipulator_control
{

// Every numeric parameter of the gripper controller. Plain data, so it can be copied
// between the parameter thread and the control thread. The defaults live here only.
struct GripperParams
{
  // Common mode (both fingers move together): PD on position/velocity, per finger [N/m, N s/m]
  double kp_c{1500.0};
  double kd_c{25.0};
  // Differential mode (keeps the object centred): PD on (qL - qR) / 2
  double kp_d{1500.0};
  double kd_d{25.0};
  double fd_max{10.0};          // [N] saturation of the centring force

  double open_bias{5.0};        // [N] push towards the open stop while open
  double open_speed{0.10};      // [m/s] opening speed
  double approach_speed{0.05};  // [m/s] closing speed before contact
  double approach_force{12.0};  // [N] saturation of the tracking force while closing
  double grip_force{25.0};      // [N] default force on each finger once the object is held
  double grip_ramp{0.25};       // [s] time to go from the contact force to the grip force
  double force_limit{40.0};     // [N] hard limit on each finger command

  // Contact detection while closing
  double settle_time{0.12};     // [s] ignore the start-up transient
  double stall_ratio{0.3};      // stalled when common speed < ratio * approach_speed ...
  double stall_time{0.03};      // ... for this long [s] ...
  double contact_force{1.5};    // ... while the controller pushes at least this hard [N]

  // Geometry
  double q_max{0.04};           // [m] travel of each finger
  double open_gap{0.085};       // [m] distance between the inner faces when open
  double empty_margin{0.0008};  // [m] closer than this to q_max = closed on nothing
  double empty_force{5.0};      // [N] holding force when closed on nothing
};

class GripperController : public controller_interface::ControllerInterface
{
public:
  GripperController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using Float64MultiArray = std_msgs::msg::Float64MultiArray;

  enum class State : int { kOpen = 0, kOpening = 1, kClosing = 2, kGrasp = 3, kEmpty = 4 };

  // Subscriber thread: validates a command and hands it over to the control thread
  void on_command(const Float64MultiArray & msg);
  // Control thread: applies a validated command to the state machine
  void apply_command(int mode, double force, double ql, double qr, double xc);
  // Timer thread: publishes the last state sample
  void publish_state();

  // joint_names_[0] = left finger, joint_names_[1] = right finger.
  // For both, a positive effort and a positive position move the finger towards the centre.
  std::vector<std::string> joint_names_;

  // Parameters: params_ is the master copy (parameter thread), p_ is the control thread's copy
  GripperParams params_;
  std::mutex params_mutex_;
  std::atomic<bool> params_dirty_{false};
  GripperParams p_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // State machine (control thread only)
  State state_{State::kOpening};
  double t_state_{0.0};             // time since the last state change [s]
  double stall_acc_{0.0};           // how long the stall condition has held [s]
  double xr_c_{0.0};                // common-mode position reference while closing [m]
  std::array<double, 2> open_ref_{{0.0, 0.0}};  // per-finger references while opening [m]
  double fc_contact_{0.0};          // common-mode force at the instant of contact [N]
  double fc_last_{0.0};             // common-mode force of the previous cycle [N]
  double grip_target_{25.0};        // force requested for the current grasp [N]
  double width_{0.0};               // width of the held object, 0 if none [m]

  // Command handover: subscriber thread -> control thread
  std::mutex cmd_mutex_;
  bool cmd_pending_{false};
  int cmd_mode_{0};
  double cmd_force_{0.0};
  rclcpp::Subscription<Float64MultiArray>::SharedPtr command_sub_;

  // State sample handover: control thread -> timer thread
  // layout: [state, xc, xd, vc, f_left, f_right, width, grip_target, fc]
  std::mutex state_mutex_;
  bool state_valid_{false};
  std::array<double, 9> state_data_{};
  rclcpp::Publisher<Float64MultiArray>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};

}  // namespace amr_manipulator_control

#endif  // AMR_MANIPULATOR_CONTROL__GRIPPER_CONTROLLER_HPP_
