#include "ros/ros.h"
#include <ros/package.h>
#include "ros/time.h"
#include "origarm_ros/States.h"
#include <sensor_msgs/JointState.h>
#include "myData.h"
#include "string.h"
#include <urdf/model.h>
#include <iostream>
#include <vector>
#include "stdio.h"

std::string urdf_path_ab;

sensor_msgs::JointState jointStateRigid;
urdf::Model keModel;

float quantToDecimal(double val,uint8_t num){
    int n=1;
    for(int i=0;i<num;i++)
      n*=10;
    val=round((val*n))/n;
    return val;
}

void callback(const origarm_ros::States &msg)
{
  int k = 0;
  for (int i = 0; i < SEGNUM; i++)
  {
    jointStateRigid.position[k++] = quantToDecimal(msg.ABL.segment[i].B,2);
    jointStateRigid.position[k++] = quantToDecimal(msg.ABL.segment[i].A / 2.0f,2);
    jointStateRigid.position[k++] = quantToDecimal(msg.ABL.segment[i].L,0);
    jointStateRigid.position[k++] = quantToDecimal(msg.ABL.segment[i].A / 2.0f,2);
    jointStateRigid.position[k++] = quantToDecimal(-msg.ABL.segment[i].B,2);
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "kinematicsEquivalence_node");

  ros::NodeHandle n;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::Rate loop_rate(100);
  ros::Subscriber sub = n.subscribe("States", 100, &callback);
  ros::Publisher pub = n.advertise<sensor_msgs::JointState>("joint_states", 100);

  urdf_path_ab = ros::package::getPath("origarm_ros") + "/model/urdf/origarmRigidKE.urdf";

  if (!keModel.initFile(urdf_path_ab))
  {
    ROS_ERROR("Failed to parse urdf file");
    return -1;
  }


  /*Init the msg with joint names and 0 position, skip the first fixed joint*/
  for (auto it = ++keModel.joints_.begin(); it != keModel.joints_.end(); it++)
  {
    urdf::JointSharedPtr joint = it->second;
    jointStateRigid.name.push_back((*joint).name);
    jointStateRigid.position.push_back(0.0f);
  }

  // robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  // const moveit::core::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
  // ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  // // Using the :moveit_core:`RobotModel`, we can construct a
  // // :moveit_core:`RobotState` that maintains the configuration
  // // of the robot. We will set all joints in the state to their
  // // default values. We can then get a
  // // :moveit_core:`JointModelGroup`, which represents the robot
  // // model for a particular group, e.g. the "panda_arm" of the Panda
  // // robot.
  // moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  // kinematic_state->setToDefaultValues();
  // const moveit::core::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("panda_arm");

  // const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

  // // Get Joint Values
  // // ^^^^^^^^^^^^^^^^
  // // We can retreive the current set of joint values stored in the state for the Panda arm.
  // std::vector<double> joint_values;
  // kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  // for (std::size_t i = 0; i < joint_names.size(); ++i)
  // {
  //   ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  // }

  // // Joint Limits
  // // ^^^^^^^^^^^^
  // // setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
  // /* Set one joint in the Panda arm outside its joint limit */
  // joint_values[0] = 5.57;
  // kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

  // /* Check whether any joint is outside its joint limits */
  // ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  // /* Enforce the joint limits for this state and check again*/
  // kinematic_state->enforceBounds();
  // ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  // // Forward Kinematics
  // // ^^^^^^^^^^^^^^^^^^
  // // Now, we can compute forward kinematics for a set of random joint
  // // values. Note that we would like to find the pose of the
  // // "panda_link8" which is the most distal link in the
  // // "panda_arm" group of the robot.
  // kinematic_state->setToRandomPositions(joint_model_group);
  // const Eigen::Isometry3d &end_effector_state = kinematic_state->getGlobalLinkTransform("panda_link8");

  // /* Print end-effector pose. Remember that this is in the model frame */
  // ROS_INFO_STREAM("Translation: \n"
  //                 << end_effector_state.translation() << "\n");
  // ROS_INFO_STREAM("Rotation: \n"
  //                 << end_effector_state.rotation() << "\n");

  // // Inverse Kinematics
  // // ^^^^^^^^^^^^^^^^^^
  // // We can now solve inverse kinematics (IK) for the Panda robot.
  // // To solve IK, we will need the following:
  // //
  // //  * The desired pose of the end-effector (by default, this is the last link in the "panda_arm" chain):
  // //    end_effector_state that we computed in the step above.
  // //  * The timeout: 0.1 s
  // double timeout = 0.1;
  // bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

  // // Now, we can print out the IK solution (if found):
  // if (found_ik)
  // {
  //   kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  //   for (std::size_t i = 0; i < joint_names.size(); ++i)
  //   {
  //     ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  //   }
  // }
  // else
  // {
  //   ROS_INFO("Did not find IK solution");
  // }

  // // Get the Jacobian
  // // ^^^^^^^^^^^^^^^^
  // // We can also get the Jacobian from the :moveit_core:`RobotState`.
  // Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  // Eigen::MatrixXd jacobian;
  // kinematic_state->getJacobian(joint_model_group,
  //                              kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
  //                              reference_point_position, jacobian);
  // ROS_INFO_STREAM("Jacobian: \n"
  //                 << jacobian << "\n");
  // // END_TUTORIAL

  while (ros::ok())
  {

    jointStateRigid.header.stamp = ros::Time::now();
    pub.publish(jointStateRigid);
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}