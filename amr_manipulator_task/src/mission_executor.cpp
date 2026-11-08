// Runs a BehaviorTree.CPP (v3) mission: the leaves of the tree are the skills of the skill server (scripts/skill_server.py), each an
// action of this package. The tree decides the order, the retries and the fallbacks; the skills do the work (Nav2, MoveIt, the images).
//
//   ros2 run amr_manipulator_task mission_executor --ros-args -p slot:=S1.L2.n1 -p article_class:=parcel -p bin:=bin_1 -p alt_bin:=bin_2
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "amr_manipulator_task/action/carry_article.hpp"
#include "amr_manipulator_task/action/navigate_to.hpp"
#include "amr_manipulator_task/action/pick_article.hpp"
#include "amr_manipulator_task/action/place_in_bin.hpp"
#include "amr_manipulator_task/action/recover.hpp"

namespace
{
using namespace std::chrono_literals;
namespace act = amr_manipulator_task::action;

/// A leaf that calls one skill: the goal is sent when the node starts, the node is RUNNING until the result arrives and then it is
/// SUCCESS if the skill says `success` and FAILURE if not (or if the action server is not there, rejects the goal or aborts).
/// Every leaf writes the `outcome` of the skill to its output port, so that the tree and the log say why it failed.
template <typename ActionT>
class SkillNode : public BT::StatefulActionNode
{
public:
  using Goal = typename ActionT::Goal;
  using Result = typename ActionT::Result;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;

  SkillNode(const std::string & name, const BT::NodeConfiguration & config, rclcpp::Node::SharedPtr node, const std::string & action)
  : BT::StatefulActionNode(name, config), node_(node), action_(action),
    client_(rclcpp_action::create_client<ActionT>(node, action)) {}

  BT::NodeStatus onStart() override
  {
    done_ = false;
    handle_.reset();
    if (!client_->wait_for_action_server(3s)) {
      RCLCPP_ERROR(node_->get_logger(), "%s: no action server %s", name().c_str(), action_.c_str());
      setOutput("outcome", std::string("no_server"));
      return BT::NodeStatus::FAILURE;
    }
    Goal goal;
    if (!fillGoal(goal)) {
      RCLCPP_ERROR(node_->get_logger(), "%s: a port of the goal is missing", name().c_str());
      return BT::NodeStatus::FAILURE;
    }
    typename rclcpp_action::Client<ActionT>::SendGoalOptions options;
    options.goal_response_callback = [this](typename GoalHandle::SharedPtr handle) {
        handle_ = handle;
        if (!handle) {
          rejected_ = true;
          done_ = true;
        }
      };
    options.result_callback = [this](const typename GoalHandle::WrappedResult & result) {
        result_ = result;
        done_ = true;
      };
    rejected_ = false;
    client_->async_send_goal(goal, options);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    rclcpp::spin_some(node_);
    if (!done_) {
      return BT::NodeStatus::RUNNING;
    }
    if (rejected_ || result_.code != rclcpp_action::ResultCode::SUCCEEDED) {
      setOutput("outcome", std::string(rejected_ ? "rejected" : "aborted"));
      return BT::NodeStatus::FAILURE;
    }
    return readResult(*result_.result) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  void onHalted() override
  {
    if (handle_ && !done_) {
      client_->async_cancel_goal(handle_);
    }
  }

protected:
  virtual bool fillGoal(Goal & goal) = 0;
  /// Write the output ports; true if the skill succeeded.
  virtual bool readResult(const Result & result) = 0;

  rclcpp::Node::SharedPtr node_;

private:
  std::string action_;
  typename rclcpp_action::Client<ActionT>::SharedPtr client_;
  typename GoalHandle::SharedPtr handle_;
  typename GoalHandle::WrappedResult result_;
  bool done_{false};
  bool rejected_{false};
};

class NavigateTo : public SkillNode<act::NavigateTo>
{
public:
  NavigateTo(const std::string & name, const BT::NodeConfiguration & config, rclcpp::Node::SharedPtr node)
  : SkillNode(name, config, node, "skills/navigate_to") {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("place"), BT::OutputPort<std::string>("outcome")};
  }

protected:
  bool fillGoal(Goal & goal) override
  {
    auto place = getInput<std::string>("place");
    if (!place) {return false;}
    goal.place = place.value();
    return true;
  }
  bool readResult(const Result & result) override
  {
    setOutput("outcome", result.outcome);
    return result.success;
  }
};

class PickArticle : public SkillNode<act::PickArticle>
{
public:
  PickArticle(const std::string & name, const BT::NodeConfiguration & config, rclcpp::Node::SharedPtr node)
  : SkillNode(name, config, node, "skills/pick_article") {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("slot"), BT::InputPort<std::string>("article_class"), BT::OutputPort<std::string>("outcome")};
  }

protected:
  bool fillGoal(Goal & goal) override
  {
    auto slot = getInput<std::string>("slot");
    auto cls = getInput<std::string>("article_class");
    if (!slot || !cls) {return false;}
    goal.slot = slot.value();
    goal.article_class = cls.value();
    return true;
  }
  bool readResult(const Result & result) override
  {
    setOutput("outcome", result.outcome);
    return result.success;
  }
};

class CarryArticle : public SkillNode<act::CarryArticle>
{
public:
  CarryArticle(const std::string & name, const BT::NodeConfiguration & config, rclcpp::Node::SharedPtr node)
  : SkillNode(name, config, node, "skills/carry_article") {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("speed_scale", 0.0, "speed scale of the arm, 0 = the default of the server"), BT::OutputPort<std::string>("outcome")};
  }

protected:
  bool fillGoal(Goal & goal) override
  {
    goal.speed_scale = static_cast<float>(getInput<double>("speed_scale").value_or(0.0));
    return true;
  }
  bool readResult(const Result & result) override
  {
    setOutput("outcome", result.outcome);
    return result.success;
  }
};

class PlaceInBin : public SkillNode<act::PlaceInBin>
{
public:
  PlaceInBin(const std::string & name, const BT::NodeConfiguration & config, rclcpp::Node::SharedPtr node)
  : SkillNode(name, config, node, "skills/place_in_bin") {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("bin"), BT::OutputPort<std::string>("outcome")};
  }

protected:
  bool fillGoal(Goal & goal) override
  {
    auto bin = getInput<std::string>("bin");
    if (!bin) {return false;}
    goal.bin = bin.value();
    return true;
  }
  bool readResult(const Result & result) override
  {
    setOutput("outcome", result.outcome);
    return result.success;
  }
};

class Recover : public SkillNode<act::Recover>
{
public:
  Recover(const std::string & name, const BT::NodeConfiguration & config, rclcpp::Node::SharedPtr node)
  : SkillNode(name, config, node, "skills/recover") {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("what"), BT::OutputPort<std::string>("outcome")};
  }

protected:
  bool fillGoal(Goal & goal) override
  {
    goal.what = getInput<std::string>("what").value_or("release_and_stow");
    return true;
  }
  bool readResult(const Result & result) override
  {
    setOutput("outcome", result.outcome);
    return result.success;
  }
};

/// SUCCESS if both outcomes are "delivered": the mission of two orders succeeds only if each of them did.
class AllDelivered : public BT::ConditionNode
{
public:
  AllDelivered(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("first"), BT::InputPort<std::string>("second")};
  }
  BT::NodeStatus tick() override
  {
    auto first = getInput<std::string>("first");
    auto second = getInput<std::string>("second");
    const bool ok = first && second && first.value() == "delivered" && second.value() == "delivered";
    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

template <typename T>
void add(BT::BehaviorTreeFactory & factory, const std::string & id, rclcpp::Node::SharedPtr node)
{
  factory.registerBuilder<T>(id, [node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<T>(name, config, node);
    });
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mission_executor");
  node->declare_parameter<std::string>("tree", "fetch_and_deliver.xml");
  node->declare_parameter<std::string>("slot", "");
  node->declare_parameter<std::string>("article_class", "");
  node->declare_parameter<std::string>("bin", "bin_1");
  node->declare_parameter<std::string>("alt_bin", "bin_2");
  node->declare_parameter<double>("timeout", 1500.0);       // [s] of the whole mission
  // the orders of a mission of two articles (trees/fetch_two.xml): o1_slot, o1_class, o1_bin, o1_alt, and the same for o2
  for (const char * order : {"o1", "o2"}) {
    for (const char * key : {"slot", "class", "bin", "alt"}) {
      node->declare_parameter<std::string>(std::string(order) + "_" + key, "");
    }
  }

  std::string tree_file = node->get_parameter("tree").as_string();
  if (tree_file.find('/') == std::string::npos) {
    tree_file = ament_index_cpp::get_package_share_directory("amr_manipulator_task") + "/trees/" + tree_file;
  }

  BT::BehaviorTreeFactory factory;
  add<NavigateTo>(factory, "NavigateTo", node);
  add<PickArticle>(factory, "PickArticle", node);
  add<CarryArticle>(factory, "CarryArticle", node);
  add<PlaceInBin>(factory, "PlaceInBin", node);
  add<Recover>(factory, "Recover", node);
  factory.registerNodeType<AllDelivered>("AllDelivered");

  auto blackboard = BT::Blackboard::create();
  for (const char * key : {"slot", "article_class", "bin", "alt_bin"}) {
    blackboard->set<std::string>(key, node->get_parameter(key).as_string());
  }
  for (const char * order : {"o1", "o2"}) {
    for (const char * key : {"slot", "class", "bin", "alt"}) {
      const std::string name = std::string(order) + "_" + key;
      blackboard->set<std::string>(name, node->get_parameter(name).as_string());
    }
  }
  auto tree = factory.createTreeFromFile(tree_file, blackboard);
  BT::StdCoutLogger logger(tree);                            // every change of status of every node, with the time

  RCLCPP_INFO(node->get_logger(), "mission %s: take the %s of %s to %s (or %s)", tree_file.c_str(),
    node->get_parameter("article_class").as_string().c_str(), node->get_parameter("slot").as_string().c_str(),
    node->get_parameter("bin").as_string().c_str(), node->get_parameter("alt_bin").as_string().c_str());
  const auto start = std::chrono::steady_clock::now();
  const auto limit = std::chrono::duration<double>(node->get_parameter("timeout").as_double());
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    status = tree.tickRoot();
    rclcpp::spin_some(node);
    if (status == BT::NodeStatus::RUNNING) {
      std::this_thread::sleep_for(100ms);
    }
    if (std::chrono::steady_clock::now() - start > limit) {
      RCLCPP_ERROR(node->get_logger(), "mission timeout");
      tree.haltTree();
      status = BT::NodeStatus::FAILURE;
    }
  }
  for (const char * key : {"nav_outcome", "pick_outcome", "carry_outcome", "place_outcome", "final_outcome", "o1_place", "o2_place"}) {   // what each skill said last
    try {
      if (auto * any = blackboard->getAny(key)) {
        RCLCPP_INFO(node->get_logger(), "  %s = %s", key, any->cast<std::string>().c_str());
      }
    } catch (const std::exception &) {
    }
  }
  const bool ok = status == BT::NodeStatus::SUCCESS;
  RCLCPP_INFO(node->get_logger(), "MISSION %s in %.1f s", ok ? "SUCCEEDED" : "FAILED",
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
