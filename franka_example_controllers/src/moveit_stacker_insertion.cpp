#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/mesh_triangle.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometric_shapes/shape_operations.h>   // Für Mesh-Ladefunktion
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <thread> 
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/robot_state/robot_state.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <fstream>
#include <chrono>
#include <atomic>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <Eigen/Geometry>

#include <fstream>
#include <string>
#include <filesystem>  // C++17
#include <sstream>

namespace fs = std::filesystem;

// Global/latest effort data
std::vector<double> latest_effort;
std::mutex effort_mutex;
std::atomic<bool> effort_received{false};

void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(effort_mutex);
    latest_effort = msg->effort;  // Nur Effort-Werte speichern
    effort_received = true;
}

std::string getNextDatafileName(const std::string &folder, const std::string &base_name, const std::string &extension)
{
    int index = 0;
    std::string filename;

    do {
        std::ostringstream oss;
        oss << folder << "/" << base_name << "_" << index << extension;
        filename = oss.str();
        index++;
    } while (fs::exists(filename));

    return filename;
}

template<typename TrajectoryToolPath>
bool planAndMoveToJointGoal(
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    moveit_visual_tools::MoveItVisualTools &moveit_visual_tools,
    TrajectoryToolPath const &draw_trajectory_tool_path,
    rclcpp::Logger logger,
    std::vector<double> &joint_goal)
{
  // transform the goal to radians
  for (auto &angle : joint_goal) {
    angle = angle * M_PI / 180.0; // Convert degrees to radians
  }

  // Gelenkwinkel setzen
  move_group_interface.setJointValueTarget(joint_goal);

  // Scale speed and acceleration
  move_group_interface.setMaxVelocityScalingFactor(0.2);      // 20% of max speed
  move_group_interface.setMaxAccelerationScalingFactor(0.2);  // 20% of max acceleration

  RCLCPP_INFO(logger, "Planning to joint goal: [%f, %f, %f, %f, %f, %f]",
              joint_goal[0], joint_goal[1], joint_goal[2],
              joint_goal[3], joint_goal[4], joint_goal[5]);

  moveit::planning_interface::MoveGroupInterface::Plan plan_to_joint;

  bool success_joint = static_cast<bool>(move_group_interface.plan(plan_to_joint));

  int max_retries_joint = 3;
  int attempt_joint = 0;

  while (!success_joint && attempt_joint < max_retries_joint)
  {
    moveit_visual_tools.trigger();
    RCLCPP_WARN(logger, "Planning to joint goal failed! Retrying... (attempt %d/%d)",
                attempt_joint + 1, max_retries_joint);

    moveit::planning_interface::MoveGroupInterface::Plan retry_plan_joint;
    success_joint = static_cast<bool>(move_group_interface.plan(retry_plan_joint));
    if (success_joint)
    {
      plan_to_joint = retry_plan_joint;
    }
    attempt_joint++;
  }
  
  auto const prompt = [&moveit_visual_tools](auto text) {
  moveit_visual_tools.prompt(text);
  };

  if (success_joint)
  {
    draw_trajectory_tool_path(plan_to_joint.trajectory_);
    moveit_visual_tools.trigger();
    rclcpp::sleep_for(std::chrono::milliseconds(500)); // Give time for visualization
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    move_group_interface.execute(plan_to_joint);
    return true;
  }
  else
  {
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Failed to move to joint goal after %d attempts!", max_retries_joint);
    return false;
  }
}

template<typename TrajectoryToolPath>
bool planAndMoveToPoseGoal(
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    moveit_visual_tools::MoveItVisualTools &moveit_visual_tools,
    TrajectoryToolPath const &draw_trajectory_tool_path,
    rclcpp::Logger logger,
    geometry_msgs::msg::Pose &pose_goal)
{

  auto const prompt = [&moveit_visual_tools](auto text) {
  moveit_visual_tools.prompt(text);
  };

  move_group_interface.setPoseTarget(pose_goal);

  // Debug: Prüfen, ob Zielpose erreichbar ist
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group =
      current_state->getJointModelGroup(move_group_interface.getName());

  // Pose in Eigen umwandeln
  Eigen::Isometry3d pose_eigen;
  tf2::fromMsg(pose_goal, pose_eigen);

  bool ik_found = current_state->setFromIK(
    joint_model_group, pose_eigen, 0.1);  // Timeout 0.1 Sekunden

  if (!ik_found)
  {
    RCLCPP_ERROR(logger,
                "IK check failed: Pose [x=%.3f, y=%.3f, z=%.3f] is not reachable.",
                pose_goal.position.x, pose_goal.position.y, pose_goal.position.z);

    moveit_visual_tools.publishAxisLabeled(pose_goal, "unreachable_pose");
    moveit_visual_tools.trigger();

    return false;
  }
  else
  {
    RCLCPP_INFO(logger, "IK check succeeded: Pose is reachable.");
    moveit_visual_tools.publishAxisLabeled(pose_goal, "reachable_pose");
    moveit_visual_tools.trigger();
  }

  // Scale speed and acceleration
  move_group_interface.setMaxVelocityScalingFactor(0.2);      // 20% of max speed
  move_group_interface.setMaxAccelerationScalingFactor(0.2);  // 20% of max acceleration

  RCLCPP_INFO(logger, "Planning to pose goal: [x=%.3f, y=%.3f, z=%.3f, qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f]",
              pose_goal.position.x, pose_goal.position.y, pose_goal.position.z,
              pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w);

  moveit_visual_tools.publishAxisLabeled(pose_goal, "pose_goal");
  moveit_visual_tools.trigger();

  std::string planning_frame = move_group_interface.getPlanningFrame();
  RCLCPP_INFO(logger, "Planning frame: %s", planning_frame.c_str());

  RCLCPP_INFO(logger, "Current TCP (end effector link): %s", move_group_interface.getEndEffectorLink().c_str());

  // Create a plan to that target pose
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
  moveit_visual_tools.trigger();
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan, retry if planning fails
  int max_retries = 3;
  int attempt = 0;
  bool plan_success = success;
  moveit::planning_interface::MoveGroupInterface::Plan current_plan = plan;

  while (!plan_success && attempt < max_retries) {
    moveit_visual_tools.trigger();
    RCLCPP_WARN(logger, "Planning failed! Retrying... (attempt %d/%d)", attempt + 1, max_retries);
    auto const [retry_success, retry_plan] = [&move_group_interface] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();
    plan_success = retry_success;
    current_plan = retry_plan;
    attempt++;
  }

  if (plan_success) {
    draw_trajectory_tool_path(current_plan.trajectory_);
    moveit_visual_tools.trigger();
    rclcpp::sleep_for(std::chrono::milliseconds(500)); // Give time for visualization
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    moveit_visual_tools.trigger();
    move_group_interface.execute(current_plan);
  } else {
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed after %d attempts!", max_retries);
  }
  return plan_success;
}

void detach_and_move_object(moveit::planning_interface::PlanningSceneInterface &psi,
                            const std::string &object_id,
                            const shape_msgs::msg::Mesh &mesh,
                            const geometry_msgs::msg::Pose &new_pose)
{
  // 1. Detach the object from the robot's end-effector.
  moveit_msgs::msg::AttachedCollisionObject detach_attached;
  detach_attached.object.id = object_id;
  detach_attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  psi.applyAttachedCollisionObject(detach_attached);

  // 2. Remove the existing object from the planning scene. This is crucial to
  // prevent MoveIt from reusing the old object definition when you re-attach.
  std::vector<std::string> object_ids_to_remove;
  object_ids_to_remove.push_back(object_id);
  psi.removeCollisionObjects(object_ids_to_remove);

  // 3. Add a new CollisionObject message at the new pose using the mesh.
  moveit_msgs::msg::CollisionObject object;
  object.id = object_id;
  object.header.frame_id = "world"; // The planning frame
  object.mesh_poses.push_back(new_pose);
  object.meshes.push_back(mesh);

  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  psi.applyCollisionObject(object);
}


void attachObject(moveit::planning_interface::PlanningSceneInterface &psi,
                          const std::string &object_id,
                          const std::string &link_name,
                          const std::vector<std::string> &touch_links)
{
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = link_name;
  attached_object.object.id = object_id;
  attached_object.touch_links = touch_links;

  // The object is added to the attached list.
  // The correct operation type is on the nested CollisionObject message.
  attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;

  // The planning scene will automatically compute the relative pose based on
  // the object's current pose in the world and the link's pose.
  psi.applyAttachedCollisionObject(attached_object);
}

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Subscribe to the joint states topic
  auto joint_states_sub = node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, joint_states_callback);


  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "fr3_arm");

  moveit::planning_interface::MoveGroupInterface gripper_group_interface(node, "fr3_hand");
 
  auto jmg = move_group_interface.getRobotModel()->getJointModelGroup("fr3_arm");

  if (!jmg)
  {
    RCLCPP_ERROR(logger, "JointModelGroup 'fr3_arm' not found!");
  } else {
    RCLCPP_INFO(logger, "LinkModelNames in JointModelGroup 'fr3_arm':");
    for (const auto& link_name : jmg->getLinkModelNames())
    {
      RCLCPP_INFO(logger, "  %s", link_name.c_str());
    }
  }

  RCLCPP_INFO(logger, "Planning frame_move: %s", move_group_interface.getPlanningFrame().c_str());
  

  // Add the stacker object
  moveit_msgs::msg::CollisionObject collision_stacker;
  collision_stacker.header.frame_id = "base";
  collision_stacker.id = "collision_stacker";

  const std::string stacker_mesh_path = "package://franka_description/meshes/container_corner/Stacker_decimated.dae";
  shapes::Mesh* stacker_mesh = shapes::createMeshFromResource(stacker_mesh_path, Eigen::Vector3d(0.001, 0.001, 0.001));
  geometry_msgs::msg::Pose stacker_pose;

  if (!stacker_mesh) {
    RCLCPP_ERROR(logger, "Failed to load mesh from %s", stacker_mesh_path.c_str());
  } else {
    shape_msgs::msg::Mesh stacker_mesh_msg;
    shapes::ShapeMsg tmp_msg;
    shapes::constructMsgFromShape(stacker_mesh, tmp_msg);
    stacker_mesh_msg = boost::get<shape_msgs::msg::Mesh>(tmp_msg);

    collision_stacker.meshes.push_back(stacker_mesh_msg);

    stacker_pose.orientation.x = 0.7071068;
    stacker_pose.orientation.y = 0.0;
    stacker_pose.orientation.z = 0.0;
    stacker_pose.orientation.w = 0.7071068;
    stacker_pose.position.x = -0.20;
    stacker_pose.position.y = 0.51;
    stacker_pose.position.z = 0.12;
    collision_stacker.mesh_poses.push_back(stacker_pose);

    collision_stacker.operation = collision_stacker.ADD;

    delete stacker_mesh;
  }

  const std::string corner_mesh_path_1 = "package://franka_description/meshes/container_corner/Containerecken_oben_links.dae";
  const std::string corner_mesh_path_2 = "package://franka_description/meshes/container_corner/Containerecken_oben_rechts.dae";

  geometry_msgs::msg::Pose corner_pose_2; // Declare here
  geometry_msgs::msg::Pose corner_pose_1; // Declare here

  // Add first container on the floor
  moveit_msgs::msg::CollisionObject collision_corner_1;
  collision_corner_1.header.frame_id = "base";
  collision_corner_1.id = "corner1";  

  shapes::Mesh* corner_mesh_1 = shapes::createMeshFromResource(corner_mesh_path_1, Eigen::Vector3d(0.001, 0.001, 0.001));

  if (!corner_mesh_1) {
    RCLCPP_ERROR(logger, "Failed to load mesh from %s", corner_mesh_path_1.c_str());
  } else {
    shape_msgs::msg::Mesh corner_mesh_msg_1;
    shapes::ShapeMsg tmp_msg;
    shapes::constructMsgFromShape(corner_mesh_1, tmp_msg);
    corner_mesh_msg_1 = boost::get<shape_msgs::msg::Mesh>(tmp_msg);

    collision_corner_1.meshes.push_back(corner_mesh_msg_1);

    corner_pose_1.orientation.x = 0.0;
    corner_pose_1.orientation.y = 0.0;
    corner_pose_1.orientation.z = -0.7071068;
    corner_pose_1.orientation.w = 0.7071068;
    corner_pose_1.position.x = -0.11;
    corner_pose_1.position.y = 0.52;
    corner_pose_1.position.z = 0.0415;
    collision_corner_1.mesh_poses.push_back(corner_pose_1);

    collision_corner_1.operation = collision_corner_1.ADD;

    delete corner_mesh_1;
  }

  // Add second container corner
  moveit_msgs::msg::CollisionObject collision_corner_2;
  collision_corner_2.header.frame_id = "base";
  collision_corner_2.id = "corner2";  

  shapes::Mesh* corner_mesh_2 = shapes::createMeshFromResource(corner_mesh_path_2, Eigen::Vector3d(0.001, 0.001, 0.001));

  if (!corner_mesh_2) {
    RCLCPP_ERROR(logger, "Failed to load mesh from %s", corner_mesh_path_2.c_str());
  } else {
    shape_msgs::msg::Mesh corner_mesh_msg_2;
    shapes::ShapeMsg tmp_msg;
    shapes::constructMsgFromShape(corner_mesh_2, tmp_msg);
    corner_mesh_msg_2 = boost::get<shape_msgs::msg::Mesh>(tmp_msg);

    collision_corner_2.meshes.push_back(corner_mesh_msg_2);

    corner_pose_2.orientation.x = 0.0;
    corner_pose_2.orientation.y = -1.0;
    corner_pose_2.orientation.z = 0.0;
    corner_pose_2.orientation.w = 0.0;
    corner_pose_2.position.x = -0.0937;
    corner_pose_2.position.y = 0.222;
    corner_pose_2.position.z = 0.99;
    collision_corner_2.mesh_poses.push_back(corner_pose_2);

    collision_corner_2.operation = collision_corner_2.ADD;

    delete corner_mesh_2;
  }

  // Add ground plane
  moveit_msgs::msg::CollisionObject ground;
  ground.header.frame_id = "base";
  ground.id = "ground";

  // Define a box shape (large flat surface)
  shape_msgs::msg::SolidPrimitive ground_primitive;
  ground_primitive.type = ground_primitive.BOX;

  // Set size: x = length, y = width, z = thickness
  ground_primitive.dimensions.resize(3);
  ground_primitive.dimensions[0] = 2.0;  // length
  ground_primitive.dimensions[1] = 2.5;  // width
  ground_primitive.dimensions[2] = 0.01; // thickness

  // Define pose
  geometry_msgs::msg::Pose ground_pose;
  ground_pose.orientation.w = 1.0;
  ground_pose.position.x = 0.0;
  ground_pose.position.y = 0.85;


  ground.primitives.push_back(ground_primitive);
  ground.primitive_poses.push_back(ground_pose);

  ground.operation = ground.ADD;

  // Erstelle Wand
  moveit_msgs::msg::CollisionObject wall;
  wall.header.frame_id = "base"; // Use the base frame
  wall.id = "wall";

  // Definiere eine Box
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.01;  // Länge in x
  primitive.dimensions[1] = 2.0; // Breite in y
  primitive.dimensions[2] = 1.5;  // Höhe in z

  // Pose der Box
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.6;   
  pose.position.y = 1.0;
  pose.position.z = 0.75;   // halbe Höhe, damit Boden = 0
  pose.orientation.w = 1.0;

  wall.primitives.push_back(primitive);
  wall.primitive_poses.push_back(pose);
  wall.operation = wall.ADD;

  // Planning scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::vector<moveit_msgs::msg::CollisionObject> objs = {collision_stacker,  collision_corner_1, collision_corner_2, ground, wall};
  planning_scene_interface.applyCollisionObjects(objs);
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  auto scene_objects = planning_scene_interface.getObjects();
  RCLCPP_INFO(logger, "Current planning scene has %zu objects:", scene_objects.size());
  for (auto &entry : scene_objects) {
      RCLCPP_INFO(logger, "  - %s", entry.first.c_str());
  }

  // upper corner stacker pose
  geometry_msgs::msg::Pose stacker_upper_pose;
  stacker_upper_pose.position.x = 0.06;
  stacker_upper_pose.position.y = 0.12;
  stacker_upper_pose.position.z = 1.0;
  stacker_upper_pose.orientation.x = -0.4713405;
  stacker_upper_pose.orientation.y = 0.5003299;
  stacker_upper_pose.orientation.z = -0.5465041;
  stacker_upper_pose.orientation.w = 0.4783737;

  // lower corner stacker pose
  geometry_msgs::msg::Pose stacker_lower_pose;
  stacker_lower_pose.orientation.x = 0.7071068;
  stacker_lower_pose.orientation.y = 0.0;
  stacker_lower_pose.orientation.z = 0.0;
  stacker_lower_pose.orientation.w = 0.7071068;
  stacker_lower_pose.position.x = -0.20;
  stacker_lower_pose.position.y = 0.51;
  stacker_lower_pose.position.z = 0.12;


  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "base", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();
  
  // Wait a bit for RViz to be ready and ensure robot state is loaded
  rclcpp::sleep_for(std::chrono::milliseconds(1000));
  
  // Publish the current robot state to help with visualization
  moveit_visual_tools.publishRobotState(move_group_interface.getCurrentState(), rviz_visual_tools::GREEN);
  moveit_visual_tools.trigger();

  // Create a closures for visualization
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 2.05;
      msg.translation().y() = 0.8;
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XXXLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };
  
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools,
      jmg = move_group_interface.getRobotModel()->getJointModelGroup("fr3_arm"),
      ee_link = move_group_interface.getRobotModel()->getLinkModel("fr3_hand")]
      (auto const& trajectory) {
        moveit_visual_tools.publishTrajectoryLine(trajectory, ee_link, jmg);
      };

    // plan to upper corner
    prompt("Press 'Next' in the RvizVisualToolsGui window to plan to upper corner");
    draw_title("Planned move to upper corner");
    moveit_visual_tools.trigger();
    std::vector<double> my_goal = {-177.68, -10.78, -96.49, 1.61, -74.73, 58.64};
    planAndMoveToJointGoal(move_group_interface, moveit_visual_tools, draw_trajectory_tool_path, logger, my_goal);
    moveit_visual_tools.deleteAllMarkers();

  // Shutdown ROS
  rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
  spinner.join();  // <--- Join the thread before exiting
  return 0;
}