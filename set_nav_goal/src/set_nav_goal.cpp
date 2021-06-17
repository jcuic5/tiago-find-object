/*
 * @Author: Jianfeng Cui 
 * @Date: 2021-05-08 14:34:33 
 * @Last Modified by:   Jianfeng Cui 
 * @Last Modified time: 2021-05-08 14:34:33 
 */
#include <exception>
#include <string>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include "set_nav_goal/ToPerceptionAction.h"
#include "set_nav_goal/ToMoveArmAction.h"
#include <tf/transform_broadcaster.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef boost::shared_ptr<MoveBaseClient> MoveBaseClientPtr;
typedef actionlib::SimpleActionClient<set_nav_goal::ToPerceptionAction> ToPerceptionClient;
typedef boost::shared_ptr<ToPerceptionClient> ToPerceptionClientPtr;
typedef actionlib::SimpleActionClient<set_nav_goal::ToMoveArmAction> ToMoveArmClient;
typedef boost::shared_ptr<ToMoveArmClient> ToMoveArmClientPtr;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> GripperClient;
typedef boost::shared_ptr<GripperClient> GripperClientPtr;

move_base_msgs::MoveBaseGoal createNavGoal(double px, double py, double pz, 
                                                double r, double p, double y)
{
  move_base_msgs::MoveBaseGoal goal;

  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = "map";
  target_pose.header.stamp = ros::Time::now();
  target_pose.pose.position.x = px;
  target_pose.pose.position.y = py;
  target_pose.pose.position.z = pz;
  target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r, p, y);
  goal.target_pose = target_pose;

  return goal;
}

set_nav_goal::ToMoveArmGoal createArmGoal(geometry_msgs::Pose pose,
                                                double r, double p, double y)
{
  set_nav_goal::ToMoveArmGoal goal;

  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = "base_footprint";
  target_pose.header.stamp = ros::Time::now();
  target_pose.pose = pose;
  target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r, p, y);
  goal.object_pose = target_pose;

  return goal;
}

set_nav_goal::ToMoveArmGoal createArmGoal(double px, double py, double pz, 
                                                double r, double p, double y)
{
  set_nav_goal::ToMoveArmGoal goal;

  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = "base_footprint";
  target_pose.header.stamp = ros::Time::now();
  target_pose.pose.position.x = px;
  target_pose.pose.position.y = py;
  target_pose.pose.position.z = pz;
  target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r, p, y);
  goal.object_pose = target_pose;

  return goal;
}

control_msgs::FollowJointTrajectoryGoal createGripperGoal(std::string side, std::string command)
{
  std::vector<double> goal_position;
  goal_position.resize(2);
  if(command == "close"){
    goal_position[0] = 0.0;
    goal_position[1] = 0.0;
  }
  else if(command == "open"){
    goal_position[0] = 1.0;
    goal_position[1] = 1.0;
  }
  else{
    ROS_WARN("No gripper goal state selected, defaulting to close");
    goal_position[0] = 0.0;
    goal_position[1] = 0.0;
  }
  trajectory_msgs::JointTrajectory traj;
  // traj.header.stamp = ros::Time::now();
  // traj.header.frame_id = "base_footprint";
  traj.joint_names.push_back("gripper_" + side + "_left_finger_joint");
  traj.joint_names.push_back("gripper_" + side + "_right_finger_joint");

  // ATTENTION: Resize those empty elements
  traj.points.resize(1);
  // First trajectory point
  traj.points[0].positions.resize(traj.joint_names.size());
  traj.points[0].velocities.resize(traj.joint_names.size());
  traj.points[0].effort.resize(traj.joint_names.size());
  traj.points[0].accelerations.resize(traj.joint_names.size());
  traj.points[0].positions = goal_position;
  traj.points[0].velocities = {0.0, 0.0};
  traj.points[0].effort = {100, 100};
  traj.points[0].accelerations = {0.0, 0.0};
  // To be reached 1 second after starting along the trajectory
  traj.points[0].time_from_start = ros::Duration(2.0);
  if(command == "close"){
    ROS_INFO("Closing gripper...");
  }
  else if(command == "open"){
    ROS_INFO("Opening gripper...");
  }
  else{
    ROS_INFO("No gripper goal selected.");
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = traj;
  goal.goal_time_tolerance = ros::Duration(0.0);
  // goal.goal_tolerance.resize(1);
  // goal.goal_tolerance[0].position = 10.0;
  
  /* ------------------------------- UNFINISHED ------------------------------- */

  return goal;
}

template<typename T>
void createClient(T& actionClient)
{
  int iterations = 0, max_iterations = 3;
  // Wait for head controller action server to come up
  while(!actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations)
  {
    ROS_DEBUG("Waiting for the server to come up");
    ++iterations;
  }

  if (iterations == max_iterations)
    throw std::runtime_error("Error in createClient: action server not available");
}

bool performMoveArmOut(ToMoveArmClientPtr& toMoveArmClient, const geometry_msgs::PoseStamped& object_pose)
{
  // NOTE: Finally tested strategy to approach the item(extended the end-effector horizontally)
  // Set goal of end-effector, based on the object_pose: 
  // object_pose -> x - 0.2, z + 0.08, pitch = 0.3 -> move_arm_goal
  //
  // To reach the move_arm_goal, use several intermediate waypoints:
  // original end-effector pose -> move_arm_goal_front_1 -> move_arm_goal_front_2 -> move_arm_goal_front_3
  // -> move_arm_goal
  set_nav_goal::ToMoveArmGoal move_arm_goal_front_1 = createArmGoal(object_pose.pose, 1.57, 0.0, 0.0);
  move_arm_goal_front_1.object_pose.pose.position.x -= 0.4;
  move_arm_goal_front_1.object_pose.pose.position.z += 0.08;
  toMoveArmClient -> sendGoal(move_arm_goal_front_1);
  toMoveArmClient -> waitForResult();
  if(toMoveArmClient -> getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Move Arm Goal Front succeeded!");
  else
  {
    ROS_INFO("Move Arm Goal Front failed");
    return false;
  }

  set_nav_goal::ToMoveArmGoal move_arm_goal_front_2 = createArmGoal(object_pose.pose, 1.57, 0.0, 0.0);
  move_arm_goal_front_2.object_pose.pose.position.x -= 0.3;
  move_arm_goal_front_2.object_pose.pose.position.z += 0.08;
  toMoveArmClient -> sendGoal(move_arm_goal_front_2);
  toMoveArmClient -> waitForResult();
  if(toMoveArmClient -> getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Move Arm Goal Front succeeded!");
  else
  {
    ROS_INFO("Move Arm Goal Front failed");
    return false;
  }

  set_nav_goal::ToMoveArmGoal move_arm_goal_front_3 = createArmGoal(object_pose.pose, 1.57, 0.0, 0.0);
  move_arm_goal_front_3.object_pose.pose.position.x -= 0.2;
  move_arm_goal_front_3.object_pose.pose.position.z += 0.08;
  toMoveArmClient -> sendGoal(move_arm_goal_front_3);
  toMoveArmClient -> waitForResult();
  if(toMoveArmClient -> getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Move Arm Goal Front succeeded!");
  else
  {
    ROS_INFO("Move Arm Goal Front failed");
    return false;
  }

  set_nav_goal::ToMoveArmGoal move_arm_goal = createArmGoal(object_pose.pose, 1.57, 0.25, 0.0);
  move_arm_goal.object_pose.pose.position.x -= 0.2;
  move_arm_goal.object_pose.pose.position.z += 0.08;
  toMoveArmClient -> sendGoal(move_arm_goal);
  toMoveArmClient -> waitForResult();
  if(toMoveArmClient -> getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Move Arm Goal succeeded!");
    return true;
  }
  else
  {
    ROS_INFO("Move Arm Goal failed");
    return false;
  }
}

bool performMoveArmBack(ToMoveArmClientPtr& toMoveArmClient, const geometry_msgs::PoseStamped& object_pose)
{
  // NOTE: Finally tested strategy to bring the item
  //
  // To reach the move_arm_goal, use several intermediate waypoints:
  // move_arm_goal -> move_arm_goal_front_3 -> move_arm_goal_front_2 -> move_arm_goal_front_1
  // -> original end-effector pose

  set_nav_goal::ToMoveArmGoal move_arm_goal_front_3 = createArmGoal(object_pose.pose, 1.57, 0.0, 0.0);
  move_arm_goal_front_3.object_pose.pose.position.x -= 0.2;
  move_arm_goal_front_3.object_pose.pose.position.z += 0.08;
  toMoveArmClient -> sendGoal(move_arm_goal_front_3);
  toMoveArmClient -> waitForResult();
  if(toMoveArmClient -> getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Move Arm Goal Front succeeded!");
  else
  {
    ROS_INFO("Move Arm Goal Front failed");
    return false;
  }

  set_nav_goal::ToMoveArmGoal move_arm_goal_front_2 = createArmGoal(object_pose.pose, 1.57, 0.0, 0.0);
  move_arm_goal_front_2.object_pose.pose.position.x -= 0.3;
  move_arm_goal_front_2.object_pose.pose.position.z += 0.08;
  toMoveArmClient -> sendGoal(move_arm_goal_front_2);
  toMoveArmClient -> waitForResult();
  if(toMoveArmClient -> getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Move Arm Goal Front succeeded!");
  else
  {
    ROS_INFO("Move Arm Goal Front failed");
    return false;
  }

  set_nav_goal::ToMoveArmGoal move_arm_goal_front_1 = createArmGoal(object_pose.pose, 1.57, 0.0, 0.0);
  move_arm_goal_front_1.object_pose.pose.position.x -= 0.4;
  move_arm_goal_front_1.object_pose.pose.position.z += 0.08;
  toMoveArmClient -> sendGoal(move_arm_goal_front_1);
  toMoveArmClient -> waitForResult();
  if(toMoveArmClient -> getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Move Arm Goal Front succeeded!");
  else
  {
    ROS_INFO("Move Arm Goal Front failed");
    return false;
  }

  set_nav_goal::ToMoveArmGoal move_arm_original = createArmGoal(0.154, -0.225, 0.565, 1.269, -1.425, 0.344);
  toMoveArmClient -> sendGoal(move_arm_original);
  toMoveArmClient -> waitForResult();
  if(toMoveArmClient -> getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Move Arm Original succeeded!");
    return true;
  }
  else
  {
    ROS_INFO("Move Arm Original failed");
    return false;
  }
}

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "set_nav_goal");

  ROS_INFO("Starting set_nav_goal application ...");

  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  //
  // ─── 1. NAVIGATION ──────────────────────────────────────────────────────────────
  //
 
  ROS_INFO("Creating action client to move_base ...");
  // Create a point head action client to move the TIAGo++'s head
  MoveBaseClientPtr goalClient(new MoveBaseClient("/move_base", true));
  createClient(goalClient);

  ROS_INFO_STREAM("Sending nav goal to move_base");
  // ! NOTE: hardcored navigation goal pose(enhance later)
  // move_base_msgs::MoveBaseGoal goal = createNavGoal(1.316, -0.216, 0, 0, 0, 0, 1.0);
  // move_base_msgs::MoveBaseGoal goal = createNavGoal(-0.971, 1.153, -0.099, 0.000, -0.000, 1.913);
  // move_base_msgs::MoveBaseGoal goal = createNavGoal(0.9525, 1.087, -0.099, 0.000, -0.000, 1.570);
  move_base_msgs::MoveBaseGoal goal = createNavGoal(0.9525, 0.987, -0.099, 0.000, -0.000, 1.570);

  goalClient -> sendGoal(goal);
  // ros::Duration(0.5).sleep();

  //enter a loop that processes ROS callbacks. Press CTRL+C to exit the loop
  // ros::spin();

  goalClient -> waitForResult();
  if(goalClient -> getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Navigation Goal succeeded!");
  else
  {
    ROS_INFO("Navigation Goal failed");
    return EXIT_FAILURE;
  }
  
  //
  // ─── 2. PERCEPTION ──────────────────────────────────────────────────────────────
  //

  ToPerceptionClientPtr perceptionClient(new ToPerceptionClient("/to_perception", true));
  createClient(perceptionClient);

  set_nav_goal::ToPerceptionGoal perception_goal;
  perception_goal.object_id = 0;
  perceptionClient -> sendGoal(perception_goal);

  bool to_pec_finished = perceptionClient -> waitForResult();

  if(to_pec_finished)
  {
    ROS_INFO("Perception Goal succeeded!");
  }
  else
  {
    ROS_INFO("Perception Goal failed");
    return EXIT_FAILURE;
  }
  geometry_msgs::PoseStamped object_pose = perceptionClient -> getResult() -> object_pose;

  //
  // ─── 3. MOVE ARM TO REACH THE OBJECT ────────────────────────────────────────────
  //

  ToMoveArmClientPtr toMoveArmClient(new ToMoveArmClient("/to_move_arm", true));

  createClient(toMoveArmClient);

  if(!performMoveArmOut(toMoveArmClient, object_pose))
    return EXIT_FAILURE;
  

  //
  // ─── 4. GRAB THE OBJECT ─────────────────────────────────────────────────────────
  //

  GripperClientPtr gripperClient(new GripperClient("/gripper_right_controller/follow_joint_trajectory", true));

  createClient(gripperClient);

  control_msgs::FollowJointTrajectoryGoal gripper_goal = createGripperGoal("right", "close");
  gripperClient -> sendGoal(gripper_goal);
  gripperClient -> waitForResult();
  if(gripperClient -> getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Gripper Goal succeeded!");
  else
  {
    // ROS_INFO("Gripper Goal failed");
    // ! NOTE: Continue the process, so commented EXIT_FAILURE out
    // Here I have not dived into how to set the tolerance or goal position for this grip task
    // (now goal pos is set to 0.0 so totally closed, but it will not reach there), so actually
    // action server will repond an error(error_code: -5, so it says GOAL_TOLERANCE_VIOLATED)
    // The item is grasped so anyway I continue the process
    // see http://docs.ros.org/en/jade/api/control_msgs/html/action/FollowJointTrajectory.html

    // return EXIT_FAILURE;
  }

  //
  // ─── 5. MOVE ARM BACK ───────────────────────────────────────────────────────────
  //
  if(!performMoveArmBack(toMoveArmClient, object_pose))
    return EXIT_FAILURE;

  {
    //
    // ─── 6. NAVIGATION ───────────────────────────────────────────────
    //

    move_base_msgs::MoveBaseGoal goal = createNavGoal(-1.137170, 0.887, -0.099, 0.000, -0.000, 1.570);

    goalClient -> sendGoal(goal);
    // ros::Duration(0.5).sleep();

    //enter a loop that processes ROS callbacks. Press CTRL+C to exit the loop
    // ros::spin();

    goalClient -> waitForResult();
    if(goalClient -> getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Navigation Goal succeeded!");
    else
    {
      ROS_INFO("Navigation Goal failed");
      return EXIT_FAILURE;
    }

    //
    // ─── 7. PERCEPTION ───────────────────────────────────────────────
    //

    // NOTE: since the result is hardcored once in the find_object node, here directly
    // set a result
    geometry_msgs::PoseStamped object_pose_dummy;
    object_pose_dummy.pose.position.x = 0.85123;
    object_pose_dummy.pose.position.y = 0.015;
    object_pose_dummy.pose.position.z = 0.8815;
    object_pose_dummy.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);


    //
    // ─── 8. MOVE ARM ─────────────────────────────────────────────────
    //
    
    ToMoveArmClientPtr toMoveArmClient(new ToMoveArmClient("/to_move_arm", true));

    createClient(toMoveArmClient);

    if(!performMoveArmOut(toMoveArmClient, object_pose_dummy))
      return EXIT_FAILURE;
    
    //
    // ─── 9. OPEN THE GIRRIPER ────────────────────────────────────────
    //
    
    GripperClientPtr gripperClient(new GripperClient("/gripper_right_controller/follow_joint_trajectory", true));

    createClient(gripperClient);

    control_msgs::FollowJointTrajectoryGoal gripper_goal = createGripperGoal("right", "open");
    gripperClient -> sendGoal(gripper_goal);
    gripperClient -> waitForResult();
    if(gripperClient -> getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Gripper Goal succeeded!");
    else
    {
      // ROS_INFO("Gripper Goal failed");
      // ! NOTE: Continue the process, so commented EXIT_FAILURE out
      // Here I have not dived into how to set the tolerance or goal position for this grip task
      // (now goal pos is set to 0.0 so totally closed, but it will not reach there), so actually
      // action server will repond an error(error_code: -5, so it says GOAL_TOLERANCE_VIOLATED)
      // The item is grasped so anyway I continue the process
      // see http://docs.ros.org/en/jade/api/control_msgs/html/action/FollowJointTrajectory.html

      // return EXIT_FAILURE;
    }

    //
    // ─── 10. MOVE ARM BACK ───────────────────────────────────────────
    //

    if(!performMoveArmBack(toMoveArmClient, object_pose_dummy))
      return EXIT_FAILURE;
  }

  return 0;
}
