#include <iostream>
#include <ros/ros.h>
#include <brics_actuator/JointPositions.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

// global variables
//static const double INIT_POS[] = {0.02, 0.02, -0.02, 0.02, 0.02}; // folded
//static const double GRIPPER_INIT_POS[] = {0.0011,0.0012}; // closed
//static const uint NUM_ARM_JOINTS = 5;


int main (int argc, char** argv)
{
	ros::init(argc, argv, "arm_draw");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// allow rviz to come up
//	sleep(20);

	// setup MoveGroup
	moveit::planning_interface::MoveGroup group("my_arm");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	moveit_msgs::DisplayTrajectory display_trajectory;

	ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1,true);

	std::string REF_FRAME = "arm_link_0";
//	std::string EEF_LINK = "arm_link_5";

	// configure the group
	group.setPoseReferenceFrame(REF_FRAME);
//	group.setEndEffectorLink(EEF_LINK);

	group.setPlannerId("RRTConnectkConfigDefault");
	// set tolerance
	group.setGoalJointTolerance(0.001);
	group.setGoalPositionTolerance(0.01);
	group.setGoalOrientationTolerance(1);

	// show the name of the reference frame
	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
	// show the name of end effector
	ROS_INFO("End-effector: %s", group.getEndEffectorLink().c_str());

	// move to home position
	group.setNamedTarget("folded");
	group.move();
	ROS_INFO("Arm has been moved to home position.");
	sleep(2.0);

	// move to home position
	group.setNamedTarget("ready");
	group.move();
	ROS_INFO("Arm has been moved to ready position.");
	sleep(2.0);

	// move forward
	geometry_msgs::PoseStamped current_pose = group.getCurrentPose();
	geometry_msgs::Pose target_pose0 = current_pose.pose;
	target_pose0.position.x += 0.1;

	group.setPoseTarget(target_pose0);
//	group.setPositionTarget(0.214291, 0.0, 0.417,"arm_link_5");
	moveit::planning_interface::MoveGroup::Plan plan;
	bool success = group.plan(plan);
	ROS_INFO("Test 1 (work space goal): %s", success ? "Success" : "Failed");
	group.move();

//	sleep(2.0);
//
//	std::vector<double> arm_joint_values;
//	group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()),arm_joint_values);
//	arm_joint_values[4] = 2.9234;
//	group.setJointValueTarget(arm_joint_values);
//	success = group.plan(plan);
//	ROS_INFO("Test 2 (joint space goal): %s", success ? "Success" : "Failed");
//	group.move();

//	sleep(2.0);
//
//	group.setStartStateToCurrentState();
//	group.setPositionTarget(0.2, -0.1, 0.4, "arm_link_5");
//	success = group.plan(plan);
//	group.move();
//
//	group.setStartStateToCurrentState();
//	group.setPositionTarget(0.2, 0.1, 0.4, "arm_link_5");
//	success = group.plan(plan);
//	group.move();
//
//	group.setStartStateToCurrentState();
//	group.setPositionTarget(0.2, 0.1, 0.3, "arm_link_5");
//	success = group.plan(plan);
//	group.move();
//
//	group.setStartStateToCurrentState();
//	group.setPositionTarget(0.2, -0.1, 0.3, "arm_link_5");
//	success = group.plan(plan);
//	group.move();
//
//	group.setStartStateToCurrentState();
//	group.setPositionTarget(0.2, -0.1, 0.4, "arm_link_5");
//	success = group.plan(plan);
//	group.move();

	//sleep(5.0);
	ros::waitForShutdown();
	return 0;
}
