#ifndef AMR_MANIPULATOR_CONTROL__ARM_MODEL_CONTROLLER_HPP_
#define AMR_MANIPULATOR_CONTROL__ARM_MODEL_CONTROLLER_HPP_

#include <Eigen/Core>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <random>

#include "controller_interface/controller_interface.hpp"
#include "kdl/chain.hpp"
#include "kdl/chaindynparam.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/jntspaceinertiamatrix.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace amr_manipulator_control
{

class ArmModelController : public controller_interface::ControllerInterface
{
public:
  ArmModelController() = default;

  // Which interfaces this controller claims (called by the controller manager)
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // Lifecycle callbacks
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  // Control loop, called at the controller_manager update rate
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using Float64MultiArray = std_msgs::msg::Float64MultiArray;

  // Reads the "robot_description" parameter of another node (robot_state_publisher)
  bool fetch_urdf(const std::string & remote_node, std::string & urdf);

  // C(q, x) y from KDL's h(v) = C(q, v) v, by polarisation: (h(x + y) - h(x) - h(y)) / 2
  bool coriolis_bilinear(
    KDL::ChainDynParam & dyn, const KDL::JntArray & x, const KDL::JntArray & y,
    KDL::JntArray & out);

  // Goal topic callback: runs in the executor thread, never in the control loop
  void on_goal(const Float64MultiArray & msg);

  // Timer callback: publishes the last debug sample, runs in the executor thread
  void publish_debug();

  // Parameters fixed at configure time
  std::vector<std::string> joint_names_;
  std::vector<double> kp_, kd_;        // PD mode gains
  std::vector<double> ct_kp_, ct_kd_;  // computed torque gains: omega^2 and 2*omega

  // Friction feedforward (nominal values, applied on the reference velocity)
  std::vector<double> fric_coulomb_, fric_viscous_;
  double fric_smoothing_{0.05};        // width of the tanh around zero velocity [rad/s]

  // Simulated velocity sensor: noise + first-order low-pass filter
  std::mt19937 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};
  std::vector<double> qd_filt_;        // filtered noisy velocity
  bool filter_primed_{false};

  std::vector<double> target_, effort_limits_;
  std::vector<double> velocity_limits_;  // per-joint speed limit used by the planner [rad/s]
  double ramp_time_{5.0};              // duration of the first move after activation

  // Parameters tunable at runtime (parameter thread writes, control thread reads)
  std::atomic<double> gravity_scale_{1.0};
  std::atomic<double> pd_scale_{1.0};
  std::atomic<double> coriolis_scale_{1.0};
  std::atomic<int> mode_{0};           // 0 = PD + gravity, 1 = computed torque, 2 = adaptive payload
  std::atomic<double> friction_scale_{1.0};
  std::atomic<double> velocity_noise_std_{0.0};  // [rad/s]
  std::atomic<double> velocity_filter_hz_{0.0};  // 0 = filter disabled
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // Adaptive payload estimate [kg] (point mass at the tip, on top of extra_mass)
  double payload_hat_{0.0};               // touched only by the control thread
  double payload_init_{0.0};
  double payload_min_{0.0};
  double payload_max_{6.0};
  std::atomic<double> adapt_gain_{0.0};   // gamma, 0 = adaptation off
  std::atomic<double> payload_set_{0.0};  // value requested with ros2 param set
  std::atomic<bool> payload_set_pending_{false};
  double debug_payload_{0.0};             // copy of the estimate for the timer thread
  rclcpp::Publisher<Float64MultiArray>::SharedPtr payload_pub_;

  // Dynamic model. chain_ must be declared before dyn_ (dyn_ keeps a reference to it)
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam> dyn_;
  KDL::JntArray q_, qdot_, g_, c_;     // preallocated: no allocation inside update()
  KDL::JntSpaceInertiaMatrix m_;
  Eigen::VectorXd a_, tau_;            // commanded acceleration and torque

  // Payload model: same kinematics, massless links, 1 kg point mass at the tip.
  // payload_chain_ must be declared before payload_dyn_ (ChainDynParam keeps a reference)
  KDL::Chain payload_chain_;
  std::unique_ptr<KDL::ChainDynParam> payload_dyn_;
  KDL::JntSpaceInertiaMatrix mp_;
  KDL::JntArray gp_, cp_, qdot_r_, xy_, h_x_, h_y_, h_xy_;
  Eigen::VectorXd qdd_r_, s_, z_, y_;  // Slotine-Li variables and payload terms

  // Quintic point-to-point trajectory
  std::vector<double> q_start_, q_goal_, ref_q_, ref_qd_;
  double traj_time_{1.0};              // duration T of the current trajectory
  double t_{0.0};                      // time since the trajectory started

  // Goal handover from the subscriber thread to the control thread
  std::mutex goal_mutex_;
  bool goal_pending_{false};
  std::vector<double> pending_goal_;
  double pending_time_{1.0};
  rclcpp::Subscription<Float64MultiArray>::SharedPtr goal_sub_;

  // Debug sample handover from the control thread to the timer thread
  std::mutex debug_mutex_;
  bool debug_valid_{false};
  std::vector<double> debug_data_;
  rclcpp::Publisher<Float64MultiArray>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
};

}  // namespace amr_manipulator_control

#endif  // AMR_MANIPULATOR_CONTROL__ARM_MODEL_CONTROLLER_HPP_