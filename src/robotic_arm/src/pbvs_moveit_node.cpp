#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "robotic_arm/control/pbvs_controller.hpp"
#include "robotic_arm/control/velocity_controller.hpp"
#include "robotic_arm/kinematics/kinematics_solver.hpp"
#include "robotic_arm/planning/moveit_planner.hpp"
#include "robotic_arm/state_machine/state_machine.hpp"
#include "robotic_arm/trajectory/quintic_trajectory.hpp"

namespace robotic_arm {

class PbvsMoveItNode : public rclcpp::Node {
public:
  PbvsMoveItNode()
      : Node("pbvs_moveit_node"),
        velocity_controller_(*this),
        kinematics_solver_("base_link", "tool0"),
        planner_(std::make_shared<rclcpp::Node>("moveit_planner_node"), "manipulator") {
    declare_parameter("joint_names", std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    declare_parameter("planner_id", std::string{"RRTConnectkConfigDefault"});
    declare_parameter("planning_time", 5.0);
    declare_parameter("pbvs.linear_gain", 0.6);
    declare_parameter("pbvs.angular_gain", 0.8);
    declare_parameter("pbvs.linear_limit", 0.2);
    declare_parameter("pbvs.angular_limit", 0.5);

    joint_names_ = get_parameter("joint_names").as_string_array();
    velocity_controller_.setJointNames(joint_names_);

    pbvs_controller_.setGains(get_parameter("pbvs.linear_gain").as_double(),
                              get_parameter("pbvs.angular_gain").as_double());
    pbvs_controller_.setMaxVelocity(get_parameter("pbvs.linear_limit").as_double(),
                                     get_parameter("pbvs.angular_limit").as_double());

    planner_.setPlannerId(get_parameter("planner_id").as_string());
    planner_.setPlanningTime(get_parameter("planning_time").as_double());

    target_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/target_pose", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          target_pose_ = msg->pose;
          has_target_pose_ = true;
        });

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, [this](sensor_msgs::msg::JointState::SharedPtr msg) {
          latest_joint_state_ = *msg;
          has_joint_state_ = true;
        });

    control_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() { onControlTick(); });

    configureStateMachine();
  }

private:
  void configureStateMachine() {
    state_machine_.addState(
        "plan",
        [this]() {
          if (!has_target_pose_ || !has_joint_state_) {
            return;
          }
          try {
            planned_traj_ = planner_.planToPose(target_pose_);
            buildQuinticTrajectory(planned_traj_);
            has_plan_ = true;
          } catch (const std::exception &ex) {
            RCLCPP_WARN(get_logger(), "Planning failed: %s", ex.what());
            has_plan_ = false;
          }
        },
        []() {},
        []() {});

    state_machine_.addState(
        "execute",
        []() {},
        [this]() {
          if (!has_plan_) {
            return;
          }
          const auto now = this->now();
          if (!traj_start_time_.has_value()) {
            traj_start_time_ = now;
          }
          const double elapsed = (now - *traj_start_time_).seconds();
          const auto positions = quintic_traj_.sample(elapsed);
          if (positions.empty()) {
            return;
          }
          const auto velocities = quintic_traj_.sampleVelocity(elapsed);
          velocity_controller_.sendVelocityCommand(velocities, 0.02);
        },
        [this]() { traj_start_time_.reset(); });

    state_machine_.addState(
        "pbvs",
        []() {},
        [this]() {
          if (!has_target_pose_ || !has_joint_state_) {
            return;
          }
          const auto current_pose = kinematics_solver_.solveFK(latest_joint_state_.position);
          const auto twist = pbvs_controller_.computeTwistCommand(current_pose, target_pose_);
          std::vector<double> velocity_command(joint_names_.size(), 0.0);
          velocity_command[0] = twist.linear.x;
          velocity_command[1] = twist.linear.y;
          velocity_command[2] = twist.linear.z;
          velocity_controller_.sendVelocityCommand(velocity_command, 0.02);
        },
        []() {});

    state_machine_.addTransition("plan", "execute", [this]() { return has_plan_; });
    state_machine_.addTransition("execute", "pbvs", [this]() {
      if (!traj_start_time_) {
        return false;
      }
      const double elapsed = (this->now() - *traj_start_time_).seconds();
      return elapsed >= planned_duration_;
    });

    state_machine_.setInitialState("plan");
  }

  void buildQuinticTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory) {
    const auto &points = trajectory.joint_trajectory.points;
    if (points.size() < 2) {
      throw std::runtime_error("Trajectory contains insufficient points for quintic smoothing.");
    }

    std::vector<double> times;
    std::vector<std::vector<double>> positions;
    std::vector<std::vector<double>> velocities;
    std::vector<std::vector<double>> accelerations;

    for (const auto &point : points) {
      const double time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
      times.push_back(time);
      positions.push_back(point.positions);
      velocities.push_back(point.velocities.empty() ? std::vector<double>(point.positions.size(), 0.0)
                                                    : point.velocities);
      accelerations.push_back(point.accelerations.empty() ? std::vector<double>(point.positions.size(), 0.0)
                                                          : point.accelerations);
    }

    quintic_traj_.setWaypoints(times, positions, velocities, accelerations);
    planned_duration_ = times.back();
  }

  void onControlTick() {
    try {
      state_machine_.tick();
    } catch (const std::exception &ex) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "State machine error: %s", ex.what());
    }
  }

  control::VelocityController velocity_controller_;
  control::PBVSController pbvs_controller_;
  kinematics::KinematicsSolver kinematics_solver_;
  planning::MoveItPlanner planner_;
  trajectory::QuinticTrajectory quintic_traj_;
  state_machine::StateMachine state_machine_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::vector<std::string> joint_names_;
  geometry_msgs::msg::Pose target_pose_;
  sensor_msgs::msg::JointState latest_joint_state_;

  bool has_target_pose_{false};
  bool has_joint_state_{false};
  bool has_plan_{false};
  double planned_duration_{0.0};
  std::optional<rclcpp::Time> traj_start_time_;
  moveit_msgs::msg::RobotTrajectory planned_traj_;
};

}  // namespace robotic_arm

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robotic_arm::PbvsMoveItNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
