/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 * @date   2021/08/07
 *
 * <h2><center>&copy; COPYRIGHT 2021 Dobot CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */
// 本程序刘煌桦调整过，解除了不能微动的限制
// 本程序徐峥也修改过，引入包含时间戳的状态话题
#include <ros/ros.h>
#include <ros/param.h>
#include <dobot_bringup/cr5_robot.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2/LinearMath/Quaternion.h>

#include <dobot_bringup/ToolVectorActual.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
using namespace std;


#define PI acos(-1.0)

bool if_servop;
dobot_bringup::ToolVectorActual ToolPose;
void servop_call_back(const dobot_bringup::ToolVectorActual msg)
{
    if_servop = true;
    ToolPose = msg;
}

bool robot_motion(dobot_bringup::ToolVectorActual current, dobot_bringup::ToolVectorActual deg)
{
    double dx = abs(current.x - deg.x);
    double dy = abs(current.y - deg.y);
    double dz = abs(current.z - deg.z);
    double drx = abs(current.rx - deg.rx);
    double dry = abs(current.ry - deg.ry);
    double drz = abs(current.rz - deg.rz);
    bool motion_flag;
    if((dx+dy+dz)>20 || (drx+dry+drz)>10)
        motion_flag = true;
    else
        motion_flag = false;
    return motion_flag;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "CR5Robot");

    try
    {
        ros::NodeHandle node;
        ros::NodeHandle private_node("~");

        ros::AsyncSpinner async_spinner(1);
        async_spinner.start();

        sensor_msgs::JointState joint_state_msg;
        ros::Publisher joint_state_pub = private_node.advertise<sensor_msgs::JointState>("/joint_states", 100);
        dobot_bringup::RobotStatus robot_status_msg;
        ros::Publisher robot_status_pub = private_node.advertise<dobot_bringup::RobotStatus>("/dobot_bringup/msg/RobotStatus", 100);
        ros::Subscriber robot_servop_sub = private_node.subscribe("/dobot_bringup/RobotServop",100,servop_call_back);

        dobot_bringup::ToolVectorActual tool_vector_actual_msg;
        ros::Publisher tool_vector_pub =
            private_node.advertise<dobot_bringup::ToolVectorActual>("/dobot_bringup/msg/ToolVectorActual", 100);


        geometry_msgs::PoseStamped EndPose;
        ros::Publisher tool_pose_pub =
            private_node.advertise<geometry_msgs::PoseStamped>("/dobot_bringup/msg/ToolPose", 100);


        string z ="/";
        const char* robot_type = getenv("DOBOT_TYPE");
        string a = robot_type == nullptr ? "cr5" : robot_type;
        string b = "_robot/joint_controller/follow_joint_trajectory";
        string ss =  z + a+ b ;
        for (uint32_t i = 0; i < 6; i++)
        {
            joint_state_msg.position.push_back(0.0);
            joint_state_msg.name.push_back(std::string("joint") + std::to_string(i + 1));
        }

        CR5Robot robot(private_node, ss);

        double rate_vale = private_node.param("JointStatePublishRate", 30);

        robot.init();
        ros::Rate rate(rate_vale);
        double position[6];
        while (ros::ok())
        {
            //
            // publish joint state
            //
            robot.getJointState(position);
            joint_state_msg.header.stamp = ros::Time::now();
            joint_state_msg.header.frame_id = "dummy_link";
            for (uint32_t i = 0; i < 6; i++)
                joint_state_msg.position[i] = position[i];
            joint_state_pub.publish(joint_state_msg);

            double val[6];
            robot.getToolVectorActual(val);
            tool_vector_actual_msg.x = val[0];
            tool_vector_actual_msg.y = val[1];
            tool_vector_actual_msg.z = val[2];
            tool_vector_actual_msg.rx = val[3];
            tool_vector_actual_msg.ry = val[4];
            tool_vector_actual_msg.rz = val[5];
            tool_vector_actual_msg.header.frame_id = "dummy_link";
            tool_vector_actual_msg.header.seq = 1;
            tool_vector_actual_msg.header.stamp = ros::Time::now();
            tool_vector_pub.publish(tool_vector_actual_msg);




            EndPose.header.stamp = ros::Time::now();
            EndPose.header.seq = 1;
            EndPose.header.frame_id = "dummy_link";

            EndPose.pose.position.x = val[0];
            EndPose.pose.position.y = val[1];
            EndPose.pose.position.z = val[2];

            tf2::Quaternion myQuaternion;
            myQuaternion.setRPY( (val[3])/180*PI,  (val[4])/180*PI,  (val[5])/180*PI );
            myQuaternion.normalize();
            EndPose.pose.orientation.w = myQuaternion.w();
            EndPose.pose.orientation.x = myQuaternion.x();
            EndPose.pose.orientation.y = myQuaternion.y();
            EndPose.pose.orientation.z = myQuaternion.z();

            tool_pose_pub.publish(EndPose);



            //
            // publish robot status
            //
            robot_status_msg.is_enable = robot.isEnable();
            robot_status_msg.is_connected = robot.isConnected();
            robot_status_pub.publish(robot_status_msg);

            //
            //send robot tcp pose
            //
            if(if_servop==true)
            {
                dobot_bringup::ServoP srv;
                srv.request.x = ToolPose.x;
                srv.request.y = ToolPose.y;
                srv.request.z = ToolPose.z;
                srv.request.a = ToolPose.rx;
                srv.request.b = ToolPose.ry;
                srv.request.c = ToolPose.rz;
//                if(robot_motion(tool_vector_actual_msg,ToolPose)==true)
                //注释掉这一句，将对期望执行的运动幅度不做限制，可以微动
                    robot.servoP(srv.request, srv.response);
                if_servop = false;
            }

            ros::spinOnce();
            rate.sleep();
        }
    }
    catch (const std::exception& err)
    {
        ROS_ERROR("%s", err.what());
        return -1;
    }

    return 0;
}
