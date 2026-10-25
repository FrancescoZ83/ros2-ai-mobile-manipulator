#ifndef AMR_MANIPULATOR_CONTROL__ARM_MODEL_CONTROLLER_HPP_
#define AMR_MANIPULATOR_CONTROL__ARM_MODEL_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "kdl/chain.hpp"
#include "kdl/chaindynparam.hpp"
#include "kdl/jntarray.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

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
  // Reads the "robot_description" parameter of another node (robot_state_publisher)
  bool fetch_urdf(const std::string & remote_node, std::string & urdf);

  // Parameters fixed at configure time
  std::vector<std::string> joint_names_;
  std::vector<double> kp_, kd_, target_, effort_limits_;
  double ramp_time_{5.0};

  // Parameters tunable at runtime (written by the parameter callback thread,
  // read by the control thread, hence atomic)
  std::atomic<double> gravity_scale_{1.0};
  std::atomic<double> pd_scale_{1.0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // Dynamic model. chain_ must be declared before dyn_ (destroyed in reverse order)
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam> dyn_;
  KDL::JntArray q_, g_;  // preallocated: no memory allocation inside update()

  // Reference generation state
  std::vector<double> q0_;
  double t_{0.0};
};

}  // namespace amr_manipulator_control

#endif  // AMR_MANIPULATOR_CONTROL__ARM_MODEL_CONTROLLER_HPP_
