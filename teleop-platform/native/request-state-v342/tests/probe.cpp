// No device IO. Compare complete Servo output with fresh/stale ROS state.
#include <moveit_servo/servo.hpp>
#include "baseline_servo.hpp"
#include "dynamic_gripper.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <sstream>
#include <thread>
#include <iomanip>
#include <limits>
using namespace moveit_servo;
using State = moveit::core::RobotState;
struct Result {
  KinematicState state;
  StatusCode status;
  CollisionSample collision;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("e6_request_probe",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto listener = std::make_shared<const servo::ParamListener>(node, "moveit_servo");
  const auto params = listener->get_params();
  if (params.use_smoothing) throw std::runtime_error("Probe requires independent unfiltered comparisons");
  auto monitor = createPlanningSceneMonitor(node, params);
  auto model = monitor->getRobotModel();
  auto group = model->getJointModelGroup(params.move_group_name);
  auto publisher = node->create_publisher<sensor_msgs::msg::JointState>(params.joint_topic, 1);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&] { executor.spin(); });
  int rc = 0;
  try {
    auto publish = [&](const State& s) {
      sensor_msgs::msg::JointState feedback;
      feedback.header.stamp = node->now();
      feedback.name = model->getVariableNames();
      feedback.position.assign(s.getVariablePositions(), s.getVariablePositions() + model->getVariableCount());
      publisher->publish(feedback);
      if (!monitor->getStateMonitor()->waitForCurrentState(feedback.header.stamp, 2.0))
        throw std::runtime_error("Probe publication timeout");
      const auto observed = monitor->getStateMonitor()->getCurrentState();
      for (size_t i = 0; i < model->getVariableCount(); ++i)
        if (std::abs(observed->getVariablePositions()[i] - s.getVariablePositions()[i]) > 1e-12)
          throw std::runtime_error("Probe publication mismatch");
    };
    State zero(model); zero.setToDefaultValues(); zero.update();
    publish(zero);
    Servo candidate(node, listener, monitor, CollisionCheckMode::REQUEST_STATE);
    BaselineServo baseline(node, listener, monitor, BaselineCollisionCheckMode::REQUEST_STATE);
    Servo background(node, listener, monitor);
    BaselineServo old_background(node, listener, monitor);
    background.setCollisionChecking(false); old_background.setCollisionChecking(false);
    DynamicGripper gripper(node, monitor);
    size_t poses = 0, comparisons = 0, moving = 0, sensitivity = 0, fallback = 0, rejected = 0;
    double maximum_error = 0.0;
    std::string phase;
    auto compare = [&](const Result& a, const Result& b) {
      if (a.status != b.status || a.state.joint_names != b.state.joint_names ||
          a.state.positions.size() != b.state.positions.size())
        throw std::runtime_error("Servo status/layout mismatch");
      for (auto pair : {std::make_pair(a.state.positions, b.state.positions),
                        std::make_pair(a.state.velocities, b.state.velocities),
                        std::make_pair(a.state.accelerations, b.state.accelerations)}) {
        if (!pair.first.allFinite() || !pair.second.allFinite())
          throw std::runtime_error("Nonfinite output");
        if (pair.first.size()) {
          const double error = (pair.first - pair.second).cwiseAbs().maxCoeff();
          maximum_error = std::max(maximum_error, error);
          if (error > 1e-10) {
            std::cerr << "Mismatch " << phase << " pose=" << poses << " compare=" << comparisons
                      << " error=" << error << " status=" << int(a.status)
                      << " expected=" << pair.first.transpose()
                      << " actual=" << pair.second.transpose() << std::endl;
            throw std::runtime_error("Servo numerical mismatch");
          }
        }
      }
      if (a.collision.checked != b.collision.checked ||
          a.collision.self_collision != b.collision.self_collision ||
          a.collision.scene_collision != b.collision.scene_collision ||
          a.collision.scale != b.collision.scale)
        throw std::runtime_error("Collision state mismatch");
      ++comparisons;
    };
    auto invoke = [&](auto& servo, const State& seed, const ServoInput& command,
                      const moveit::core::RobotStateConstPtr& actual) {
      servo.setCommandType(static_cast<CommandType>(command.index()));
      auto result = servo.getNextJointState(std::make_shared<State>(seed), command, actual);
      return Result{result, servo.getStatus(), servo.getCollisionSample()};
    };
    std::string line;
    while (std::getline(std::cin, line)) {
      std::istringstream input(line);
      Eigen::VectorXd q(6); int raw;
      for (auto& x : q) if (!(input >> x)) throw std::runtime_error("Bad probe pose");
      if (!(input >> raw)) throw std::runtime_error("Bad probe raw");
      auto actual = std::make_shared<State>(model); actual->setToDefaultValues();
      actual->setJointGroupPositions(group, q); gripper.position(*actual, raw);
      sensor_msgs::msg::JointState unused; gripper.feedback(unused, raw);
      auto seed = *actual;
      Eigen::VectorXd seed_q = q;
      seed_q[3] += .0008; seed_q[4] -= .0004;
      seed.setJointGroupPositions(group, seed_q);
      seed.setJointGroupVelocities(group, Eigen::VectorXd::Zero(6));
      seed.setJointGroupAccelerations(group, Eigen::VectorXd::Zero(6)); seed.update();
      auto stale = *actual;
      Eigen::VectorXd stale_q = q;
      stale_q[0] += .12; stale_q[3] -= .17; stale_q[5] += .21;
      stale.setJointGroupPositions(group, stale_q); stale.update();
      geometry_msgs::msg::TransformStamped tf;
      tf.header.frame_id = *getIKSolverBaseFrame(std::make_shared<State>(seed), params.move_group_name);
      tf.child_frame_id = "external_probe";
      tf.header.stamp = node->now(); tf.transform.translation.x = .13;
      tf.transform.translation.y = -.08;
      tf.transform.rotation.w = std::cos(.2); tf.transform.rotation.z = std::sin(.2);
      if (!monitor->getTFClient()->setTransform(tf, "isolated_probe", true))
        throw std::runtime_error("TF fixture rejected");
      std::vector<ServoInput> commands;
      for (auto frame : {"controller_base", "controller_flange", "grasp_tcp", "external_probe"}) {
        for (int axis = 0; axis < 6; ++axis)
          for (double sign : {-1., 1.}) {
            Eigen::Vector<double, 6> v = Eigen::Vector<double, 6>::Zero();
            v[axis] = sign * (axis < 3 ? .01 : .03);
            commands.emplace_back(TwistCommand{frame, v});
          }
        Eigen::Vector<double, 6> v; v << .005, -.003, .007, .01, -.02, .03;
        commands.emplace_back(TwistCommand{frame, v});
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() << .001, -.002, .003;
        commands.emplace_back(PoseCommand{frame, pose});
      }
      for (int axis = 0; axis < 6; ++axis) {
        std::vector<double> velocities(6, 0.); velocities[axis] = .03;
        commands.emplace_back(JointJogCommand{group->getActiveJointModelNames(), velocities});
      }
      std::vector<Result> expected, backgrounds;
      publish(*actual);
      for (const auto& command : commands) {
        expected.push_back(invoke(baseline, seed, command, actual));
        backgrounds.push_back(invoke(old_background, seed, command, nullptr));
        phase = "background-fresh/" + std::to_string(expected.size()-1);
        compare(backgrounds.back(), invoke(background, seed, command, nullptr));
        const auto& x = expected.back().state;
        if (x.velocities.size() && x.velocities.norm() > 1e-9) ++moving;
      }
      publish(stale);
      auto dirty = std::make_shared<State>(stale);
      dirty->setJointGroupPositions(group, q);  // Deliberately keep stale FK caches.
      for (size_t i = 0; i < commands.size(); ++i) {
        phase = "request-stale/" + std::to_string(i);
        compare(expected[i], invoke(candidate, seed, commands[i], actual));
        phase = "request-dirty/" + std::to_string(i);
        compare(expected[i], invoke(candidate, seed, commands[i], dirty));
        const auto wrong = invoke(baseline, seed, commands[i], actual);
        if (wrong.state.positions.size() == expected[i].state.positions.size() &&
            wrong.state.positions.size() &&
            (wrong.state.positions - expected[i].state.positions).norm() > 1e-9)
          ++sensitivity;
        // Default mode still follows the current monitor, as upstream did.
        phase = "background-stale/" + std::to_string(i);
        compare(invoke(old_background, seed, commands[i], nullptr),
                invoke(background, seed, commands[i], nullptr));
        if (i >= 42 && i < 56) ++fallback;
      }
      if (poses == 0) {
        auto expect_rejected = [&](auto&& action) {
          try { action(); } catch (const std::invalid_argument&) { ++rejected; return; }
          throw std::runtime_error("Invalid request state accepted");
        };
        expect_rejected([&] { invoke(candidate, seed, commands[0], nullptr); });
        expect_rejected([&] { invoke(background, seed, commands[0], actual); });
        auto bad = std::make_shared<State>(*actual);
        bad->setVariablePosition("joint5", std::numeric_limits<double>::quiet_NaN());
        expect_rejected([&] { invoke(candidate, seed, commands[0], bad); });
        auto second = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
        auto foreign = std::make_shared<State>(second->getRobotModel());
        foreign->setToDefaultValues(); foreign->update();
        expect_rejected([&] { invoke(candidate, seed, commands[0], foreign); });
      }
      ++poses;
    }
    if (poses < 20 || moving < 100 || sensitivity < 100 || rejected != 4 || fallback < 100)
      throw std::runtime_error("Insufficient nontrivial transform coverage");
    std::cout << std::setprecision(17) << "{\"poses\":" << poses
              << ",\"apply_twist_about_ee\":" << (params.apply_twist_commands_about_ee_frame ? "true" : "false")
              << ",\"comparisons\":" << comparisons << ",\"moving_outputs\":" << moving
              << ",\"stale_monitor_changes_baseline\":" << sensitivity
              << ",\"external_tf_cases\":" << fallback << ",\"invalid_rejections\":" << rejected
              << ",\"max_error\":" << maximum_error << ",\"physical_io\":false,\"passed\":true}\n";
  } catch (const std::exception& ex) {
    std::cerr << "PROBE FAILURE: " << ex.what() << std::endl; rc = 1;
  }
  executor.cancel(); spinner.join(); rclcpp::shutdown();
  return rc;
}
