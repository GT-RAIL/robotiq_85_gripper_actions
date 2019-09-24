#ifndef GRIPPER_ACTIONS_H_
#define GRIPPER_ACTIONS_H_

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <rail_manipulation_msgs/GripperAction.h>
#include <rail_manipulation_msgs/VerifyGraspAction.h>
#include <robotiq_85_msgs/GripperCmd.h>
#include <robotiq_85_msgs/GripperStat.h>

class GripperActions
{
public:
    /**
    * \brief Constructor
    */
    GripperActions();

private:
    /**
    * \brief Callback for the gripperCommand action sever, support for standard ROS gripper control messages
    * @param goal action goal
    */
    void executeGripperCommand(const control_msgs::GripperCommandGoalConstPtr &goal);

    /**
    * \brief Callback for the gripperManipulation action sever, closes the gripper until an object is grasped, alternatively opens the gripper fully
    * @param goal action goal
    */
    void executeGripperManipulation(const rail_manipulation_msgs::GripperGoalConstPtr &goal);

    /**
    * \brief Callback for the verifyGrasp action sever, tests whether the gripper is holding an object
    * @param goal action goal
    */
    void executeVerifyGrasp(const rail_manipulation_msgs::VerifyGraspGoalConstPtr &goal);

    /**
    * \brief Callback for joint state updates
    * @param msg joint state message
    */
    void gripperStatusCallback(const robotiq_85_msgs::GripperStat msg);

    //constants
    static constexpr float defaultGripperSpeed = .013;
    static constexpr float defaultGripperForce = 100;
    static constexpr float gripperClosedPosition = 0;
    static constexpr float gripperOpenPosition = 0.085;
    static constexpr float gripperMinSpeed = .013;
    static constexpr float gripperMinForce = 5;

    ros::NodeHandle node, privateNode;

    // Messages
    ros::Publisher gripperCmdPublisher;
    ros::Subscriber gripperStatusSubscriber;

    // Actionlib
    actionlib::SimpleActionServer<control_msgs::GripperCommandAction> asGripperCommand;
    actionlib::SimpleActionServer<rail_manipulation_msgs::GripperAction> asGripperManipulation;
    actionlib::SimpleActionServer<rail_manipulation_msgs::VerifyGraspAction> asVerifyGrasp;

    //gripper status
    robotiq_85_msgs::GripperStat gripperStatus;
};

#endif
