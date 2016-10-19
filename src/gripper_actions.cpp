#include <robotiq_85_gripper_actions/gripper_actions.h>

using namespace std;

GripperActions::GripperActions() : privateNode("~"),
    asGripperCommand(privateNode, "gripper_command", boost::bind(&GripperActions::executeGripperCommand, this, _1), false),
    asGripperManipulation(privateNode, "gripper_manipulation", boost::bind(&GripperActions::executeGripperManipulation, this, _1), false),
    asVerifyGrasp(privateNode, "verify_grasp", boost::bind(&GripperActions::executeVerifyGrasp, this, _1), false)
{
    // Messages
    gripperCmdPublisher = node.advertise<robotiq_85_msgs::GripperCmd>("/gripper/cmd", 1);

    gripperStatusSubscriber = node.subscribe("/gripper/stat", 1, &GripperActions::gripperStatusCallback, this);

    // Action servers
    asGripperCommand.start();
    asGripperManipulation.start();
    asVerifyGrasp.start();
}

void GripperActions::gripperStatusCallback(const robotiq_85_msgs::GripperStat msg)
{
    gripperStatus = msg;
}

void GripperActions::executeGripperCommand(const control_msgs::GripperCommandGoalConstPtr &goal)
{
    //TODO: cancel goal on the gripperManipulation action server

    control_msgs::GripperCommandResult result;

    //send command to gripper
    robotiq_85_msgs::GripperCmd cmd;
    cmd.emergency_release = false;
    cmd.stop = false;
    cmd.position = goal->command.position;
    cmd.speed = defaultGripperSpeed;
    cmd.force = goal->command.max_effort;
    gripperCmdPublisher.publish(cmd);

    //wait until it's finished
    ros::Rate loopRate(30);
    bool positionReached = false;
    bool gripperStopped = false;
    ros::Duration(0.25).sleep(); //give gripper time to start moving
    while (!(positionReached || gripperStopped))
    {
        gripperStopped = !gripperStatus.is_moving;
        positionReached = fabs(gripperStatus.position - cmd.position) < .001;

        loopRate.sleep();
    }

    //stop gripper
    cmd.stop = true;
    gripperCmdPublisher.publish(cmd);

    //publish result
    result.position = gripperStatus.position;
    result.reached_goal = positionReached;
    asGripperCommand.setSucceeded(result);
}

void GripperActions::executeGripperManipulation(const rail_manipulation_msgs::GripperGoalConstPtr &goal)
{
    //TODO: cancel goal on the gripperCommand action server

    rail_manipulation_msgs::GripperFeedback feedback;
    rail_manipulation_msgs::GripperResult result;

    if (goal->close)
        feedback.message = "Closing gripper...";
    else
        feedback.message = "Opening gripper...";
    asGripperManipulation.publishFeedback(feedback);

    //send command to gripper
    robotiq_85_msgs::GripperCmd cmd;
    cmd.emergency_release = false;
    cmd.stop = false;

    if (goal->close)
        cmd.position = gripperClosedPosition;
    else
        cmd.position = gripperOpenPosition;

    if (goal->speed == 0)
        cmd.speed = defaultGripperSpeed;
    else
        cmd.speed = goal->speed;

    if (goal->force == 0)
        cmd.force = defaultGripperForce;
    else
        cmd.force = goal->force;

    gripperCmdPublisher.publish(cmd);

    ros::Rate loopRate(30);
    ros::Duration(0.25).sleep(); //give gripper time to start moving
    while (gripperStatus.is_moving)
    {
        if (asGripperManipulation.isPreemptRequested())
        {
            result.success = false;
            asGripperManipulation.setPreempted(result);
        }
        loopRate.sleep();
    }

    //stop gripper
    cmd.stop = true;
    gripperCmdPublisher.publish(cmd);

    //publish result
    if (goal->close)
        feedback.message = "Gripper closed.";
    else
        feedback.message = "Gripper opened.";
    asGripperManipulation.publishFeedback(feedback);
    result.success = true;
    asGripperManipulation.setSucceeded(result);
}

void GripperActions::executeVerifyGrasp(const rail_manipulation_msgs::VerifyGraspGoalConstPtr &goal)
{
    rail_manipulation_msgs::VerifyGraspResult result;

    result.success = true;

    if (gripperStatus.obj_detected) //first condition: gripper status reports an object detected
    {
        result.grasping = true;
    }
    else if (gripperStatus.is_moving) //second condition: no object detected and gripper is moving
    {
        result.grasping = false;
    }
    else //third case: attempt a close for a short time duration to determine whether an object is detected
    {
        float prevPos = gripperStatus.position;

        //squeeze gripper in previously moved in direction
        robotiq_85_msgs::GripperCmd cmd;
        cmd.emergency_release = false;
        cmd.stop = false;
        if (gripperStatus.requested_position - gripperStatus.position <= 0)
            cmd.position = gripperClosedPosition;
        else
            cmd.position = gripperOpenPosition;
        cmd.speed = gripperMinSpeed;
        cmd.force = gripperMinForce;
        gripperCmdPublisher.publish(cmd);

        //wait so gripper can perform detection
        ros::Duration(0.25).sleep();

        //check result
        result.grasping = gripperStatus.obj_detected;

        //stop gripper
        cmd.stop = true;
        gripperCmdPublisher.publish(cmd);

        //if no object detected, return gripper to its previous position
        if (!result.grasping)
        {
            cmd.position = prevPos;
            cmd.stop = false;
            gripperCmdPublisher.publish(cmd);

            ros::Rate loopRate(30);
            ros::Duration(0.25).sleep(); //give gripper time to start moving
            while (gripperStatus.is_moving)
            {
                if (asVerifyGrasp.isPreemptRequested())
                {
                    result.success = false;
                    asVerifyGrasp.setPreempted(result);
                }
                loopRate.sleep();
            }

            //stop gripper
            cmd.stop = true;
            gripperCmdPublisher.publish(cmd);
        }
    }

    //report result
    asVerifyGrasp.setSucceeded(result);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gripper_actions");

    GripperActions ga;

    ros::spin();
}
