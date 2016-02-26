#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointTorques.h>
#include <brics_actuator/JointVelocities.h>
#include <boost/units/systems/si.hpp>
#include <boost/units/io.hpp>

static const uint NUM_ARM_JOINTS = 5;

int mode = 3;	// 1:current PID 2:velocity PID 3:position PID
int joint_index = 5; // joint to be tested, from 1 to 5

ros::Publisher armPositionsPublisher;
ros::Publisher armTorquePublisher;
ros::Publisher armVelocitiesPublisher;

double READY_POS[] = {0.2, 0.2, -0.2, 0.2, 0.2};
double HOME_POS[] = {0.05, 0.05, -0.05, 0.05, 0.1107};

brics_actuator::JointTorques generateJointTorqueMsg(double* joints)
{
  brics_actuator::JointTorques m_joint_torques;
  //Ros component negates torque values for joints with negative direction (all joints except joint 3)
  joints[2] = -joints[2];
  std::stringstream jointName;
  m_joint_torques.torques.clear();

  for (int i = 0; i < NUM_ARM_JOINTS; i++)
  {
    brics_actuator::JointValue joint;
    joint.value = joints[i];
    joint.unit = boost::units::to_string(boost::units::si::newton_meter);
    jointName.str("");
    jointName << "arm_joint_" << (i + 1);
    joint.joint_uri = jointName.str();

    m_joint_torques.torques.push_back(joint);
  }
  return m_joint_torques;
}

brics_actuator::JointPositions generateJointPositionMsg(double* joints)
{
  brics_actuator::JointPositions joint_position_msg;

  std::stringstream jointName;
  joint_position_msg.positions.clear();

  for (int i = 0; i < NUM_ARM_JOINTS; i++)
  {
    brics_actuator::JointValue joint;

    joint.value = joints[i];
    joint.unit = boost::units::to_string(boost::units::si::radians);
    jointName.str("");
    jointName << "arm_joint_" << (i + 1);
    joint.joint_uri = jointName.str();

    joint_position_msg.positions.push_back(joint);
  }

  return joint_position_msg;
}

brics_actuator::JointVelocities generateJointVelocityMsg(double* joints)
{
  brics_actuator::JointVelocities joint_velocity_msg;

  std::stringstream jointName;
  joint_velocity_msg.velocities.clear();

  for (int i = 0; i < 5; i++)
  {
    brics_actuator::JointValue joint;

    joint.value = joints[i];
    joint.unit = boost::units::to_string(boost::units::si::radian_per_second);
    jointName.str("");
    jointName << "arm_joint_" << (i + 1);
    joint.joint_uri = jointName.str();

    joint_velocity_msg.velocities.push_back(joint);
  }

  return joint_velocity_msg;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tune_PID_gains");
	ros::NodeHandle nh;
	ros::Rate rate(200);

	armPositionsPublisher = nh.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 1);
	armVelocitiesPublisher = nh.advertise<brics_actuator::JointVelocities>("arm_1/arm_controller/velocity_command", 1);
	armTorquePublisher = nh.advertise<brics_actuator::JointTorques>("arm_1/arm_controller/torques_command", 1);

	if (mode == 1)
	{
	/***************************************************************************************************************
	 * step response of current loop
	 * WARNING:
	 * (1) youbot drive has to be launched without arm joint calibration, so that the joint can be moved to
	 * 		it's stop manually!!!
	 * (2) give the torque in correct direction to remain joint at rest and prevent hardware damage!!!
	 * 		(e.g negative torque value for home position)
	 **************************************************************************************************************/

	brics_actuator::JointTorques arm_torque_cmd0;
	brics_actuator::JointTorques arm_torque_cmd1;
	brics_actuator::JointTorques arm_torque_cmd2;

	double torque_val1[] = {0.25, 0, 0, 0, 0};
	double torque_val2[] = {1, 0, 0, 0, 0};
	double torque_null[] = {0, 0, 0, 0, 0};
	arm_torque_cmd1 = generateJointTorqueMsg(torque_val1);
	arm_torque_cmd2 = generateJointTorqueMsg(torque_val2);
	arm_torque_cmd0 = generateJointTorqueMsg(torque_null);

	int k = 0;
	while (nh.ok() && k < 400)
	{
		armTorquePublisher.publish(arm_torque_cmd0);
		k = k+1;
	//	ros::spinOnce();
		rate.sleep();
	}
	k = 0;

	// step
	while (nh.ok() && k < 400)
	{
		armTorquePublisher.publish(arm_torque_cmd1);
		k = k+1;
//		ros::spinOnce();
		rate.sleep();
	}

	k=0;
	// reset
	while (nh.ok() && k < 400)
	{
		armTorquePublisher.publish(arm_torque_cmd0);
		k = k+1;
//		ros::spinOnce();
		rate.sleep();
	}
	k = 0;

	// step
	while (nh.ok() && k < 400)
	{
		armTorquePublisher.publish(arm_torque_cmd2);
		k = k+1;
//		ros::spinOnce();
		rate.sleep();
	}
	k = 0;

	// reset
	while (nh.ok() && k < 100)
	{
		armTorquePublisher.publish(arm_torque_cmd0);
		k = k+1;
//		ros::spinOnce();
		rate.sleep();
	}
	/*************************************************************************************************************/
	}
	else if (mode == 2)
	{
	/***************************************************************************************************************
	 * step and trapezoid response of velocity loop
	 * be careful if ramp generator is enabled!!!
	 **************************************************************************************************************/
	brics_actuator::JointPositions arm_position_cmd;
	brics_actuator::JointVelocities arm_velocity_cmd0;
	brics_actuator::JointVelocities arm_velocity_cmd1;


	double velocity_goal = 0.8;	// for trapezoid, must be adjusted for joint 3
	double velocity_val[] = {0, 0, 0, 0, 0.5}; // be careful with 3rd joint, negative direction
	double velocity_null[] = {0, 0, 0, 0, 0};

	arm_position_cmd = generateJointPositionMsg(HOME_POS);
	arm_velocity_cmd0 = generateJointVelocityMsg(velocity_null);
//	arm_velocity_cmd1 = arm_velocity_cmd0;
	arm_velocity_cmd1 = generateJointVelocityMsg(velocity_val);

	int k = 0;
	while (nh.ok() && k < 400)
	{
		armVelocitiesPublisher.publish(arm_velocity_cmd0);
		k = k+1;
		rate.sleep();
	}
	k = 0;

	while (nh.ok() && k < 800)	// 4s
	{
		armVelocitiesPublisher.publish(arm_velocity_cmd1);
		k = k+1;
		rate.sleep();
	}
	k = 0;

	// test following ability with trapezoid input,this segment must be adjusted for joint 3
//	while(nh.ok() && arm_velocity_cmd1.velocities[joint_index-1].value <= velocity_goal)
//	{
//		armVelocitiesPublisher.publish(arm_velocity_cmd1);
//		arm_velocity_cmd1.velocities[joint_index-1].value += 0.005;
//		rate.sleep();
//	}
//
//	while (nh.ok() && k < 400)
//	{
//		armVelocitiesPublisher.publish(arm_velocity_cmd1);
//		k = k+1;
//		rate.sleep();
//	}
//	k = 0;
//
//	while(nh.ok() && arm_velocity_cmd1.velocities[joint_index-1].value >= 0)
//	{
//		armVelocitiesPublisher.publish(arm_velocity_cmd1);
//		arm_velocity_cmd1.velocities[joint_index-1].value -= 0.005;
//		rate.sleep();
//	}

	while (nh.ok() && k < 400)
	{
		armVelocitiesPublisher.publish(arm_velocity_cmd0);
		k = k+1;
		rate.sleep();
	}

	// go back to home position (OVER CURRENT POTENTIAL IF RAMP GENERATOR DISABLED!)
//	armPositionsPublisher.publish(arm_position_cmd);
	/*************************************************************************************************************/
	}
	else if (mode == 3)
	{
	/***************************************************************************************************************
	* step response of position loop
	* normally only a P controller
	* Do not forget to enable arm calibration in youbot driver if disable for tuning current loop
	**************************************************************************************************************/
		brics_actuator::JointPositions arm_position_cmd0;
		brics_actuator::JointPositions arm_position_cmd1;
		brics_actuator::JointPositions arm_position_cmd2;


		arm_position_cmd0 = generateJointPositionMsg(READY_POS);
		arm_position_cmd1 = arm_position_cmd0;
		arm_position_cmd2 = arm_position_cmd0;

		if (joint_index == 3)
		{
			arm_position_cmd1.positions[joint_index-1].value = arm_position_cmd0.positions[joint_index-1].value - 0.5;
			arm_position_cmd2.positions[joint_index-1].value = arm_position_cmd0.positions[joint_index-1].value - 1.0;
		}
		else
		{
			arm_position_cmd1.positions[joint_index-1].value = arm_position_cmd0.positions[joint_index-1].value + 0.5;
			arm_position_cmd2.positions[joint_index-1].value = arm_position_cmd0.positions[joint_index-1].value + 1.0;
		}

		int k = 0;
		// go to ready position
		while (nh.ok() && k < 400)
		{
			armPositionsPublisher.publish(arm_position_cmd0);
			k = k+1;
			rate.sleep();
		}

		k = 0;
		// step, duration: 2s
//		while (nh.ok() && k < 400)
//		{
//			armPositionsPublisher.publish(arm_position_cmd1);
//			k = k+1;
//			rate.sleep();
//		}
//
//		k=0;
//
//		// reset
//		while (nh.ok() && k < 400)
//		{
//			armPositionsPublisher.publish(arm_position_cmd0);
//			k = k+1;
//			rate.sleep();
//		}
//		k = 0;

		// step
		while (nh.ok() && k < 600)
		{
			armPositionsPublisher.publish(arm_position_cmd2);
			k = k+1;
			rate.sleep();
		}
		k = 0;

		// reset
		while (nh.ok() && k < 400)
		{
			armPositionsPublisher.publish(arm_position_cmd0);
			k = k+1;
			rate.sleep();
		}

		// go back to home position
		armPositionsPublisher.publish(generateJointPositionMsg(HOME_POS));

	}
	else
	{
		ROS_INFO("invalid mode! 1: current, 2: velocity, 3: position.");
		return 1;
	}

	return 0;
}
