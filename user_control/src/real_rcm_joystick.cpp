/**
 * @file real_rcm_joystick.cpp
 * @brief 
 * @author Zheng Xu (xz200103@sjtu.edu.cn)
 * @version 1.0
 * @date 2024-03-02
 * 
 * @copyright Copyright (c) 2024 Robotics-GA
 * 
 * @par logs:
 * <table>
 * <tr><th>Date       <th>Version <th>Author   <th>Description
 * <tr><td>2024-03-02 <td>1.0     <td>Zheng Xu <td>Initial version
 * <tr><td>2024-03-14 <td>1.1     <td>Zheng Xu <td>Design fill-in-the-blank questions
 * </table>
 */
#include <ros/ros.h>

#include <dobot_bringup/ToolVectorActual.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <sensor_msgs/Joy.h>

#include <math.h>

#include <iostream>

#define CVUI_IMPLEMENTATION

#include "cvui.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include <boost/thread/thread.hpp>

using namespace std;
using namespace Eigen;

Vector3d transR2RPY(Matrix3d t, string xyz);

// pose callback for robotic arm
bool dobot_pose_flag = false;
dobot_bringup::ToolVectorActual RobotToolPose;

void dobot_pos_Callback(const dobot_bringup::ToolVectorActual msg) {
    RobotToolPose = msg;
    dobot_pose_flag = true;
}

// callback function for Xbox joystick input
bool joy_flag = false;
ArrayXd joy_button = ArrayXd::Zero(8);

void joy_Callback(const sensor_msgs::Joy::ConstPtr &msg) {
    joy_button = ArrayXd::Zero(8);

    for (int i = 0; i < 8; i++) {
        joy_button(i) = msg->buttons[i];
        if (joy_button(i) != 0)
            joy_flag = true;
    }

}


void to_blue(double &rcm_alpha, double &rcm_beta, double &rcm_trans);

void to_green(double &rcm_alpha, double &rcm_beta, double &rcm_trans);

void to_yellow(double &rcm_alpha, double &rcm_beta, double &rcm_trans);

cv::Point2d detectCenter(cv::Mat image);

int detectHSColor(const cv::Mat &image, double minHue, double maxHue, double minSat, double maxSat, cv::Mat &mask);


//static cv::Mat usb_img;

int main(int argc, char **argv) {
    ros::init(argc, argv, "real_rcm_joystick_node");
    ros::NodeHandle nh;

    ros::Subscriber dobot_subscriber = nh.subscribe<dobot_bringup::ToolVectorActual>(
            "/dobot_bringup/msg/ToolVectorActual", 1, dobot_pos_Callback);
    ros::Publisher dobot_pose_pub = nh.advertise<dobot_bringup::ToolVectorActual>("/dobot_bringup/RobotServop", 1);

    ros::Subscriber joystick_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, joy_Callback);


    // read tmp robotic arm pose
    while (ros::ok() && dobot_pose_flag == false) {
        ros::spinOnce();
    }
    dobot_pose_flag = false;
    cout << RobotToolPose.x << " " << RobotToolPose.y << " " << RobotToolPose.z << " " << RobotToolPose.rx << " "
         << RobotToolPose.ry << " " << RobotToolPose.rz << endl;

    /*********************************** CR5-moveit & RCM point init ************************************/
    // Get initial end pose (T_base_end_init)
    Matrix3d T_base_end_r_init;
    T_base_end_r_init = AngleAxisd((RobotToolPose.rz) / 180 * M_PI, Vector3d::UnitZ()) *
                        AngleAxisd((RobotToolPose.ry) / 180 * M_PI, Vector3d::UnitY()) *
                        AngleAxisd((RobotToolPose.rx) / 180 * M_PI, Vector3d::UnitX());
    Vector3d T_base_end_t_init;
    T_base_end_t_init << (RobotToolPose.x) / 1000, (RobotToolPose.y) / 1000, (RobotToolPose.z) / 1000;
    Matrix4d T_base_end_init = Matrix4d::Identity();
    T_base_end_init.block<3, 3>(0, 0) = T_base_end_r_init;
    T_base_end_init.block<3, 1>(0, 3) = T_base_end_t_init;
    // Get initial RCM point position, under end frame (prcm_end_init), under base frame (prcm_base_init)
    double rcm_len = 0.343;
    Vector4d prcm_end_init, prcm_base_init;
    prcm_end_init << 0, 0, rcm_len, 1;
    prcm_base_init = T_base_end_init * prcm_end_init;
    cout << "prcm_base_init" << endl;
    cout << prcm_base_init << endl;

    /********************************************** RCM param init *********************************************/
    double rcm_alpha, rcm_beta;
    rcm_alpha = 0;
    rcm_beta = 0;
    double rcm_trans;
    rcm_trans = 0.0;

    /********************************************** CAMERA init *********************************************/
/*    cv::namedWindow("usb_cam", cv::WINDOW_AUTOSIZE);
    //打开摄像头
    cv::VideoCapture cap(2);
    if (!cap.isOpened()) {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }*/
    /**********************************************************************************************************/

    // joystick control prompt words
    cout << "start" << endl;

    while (ros::ok()) {

        while (ros::ok() && (dobot_pose_flag == false || joy_flag == false)) {
            ros::spinOnce();
        }
        dobot_pose_flag = false;
        joy_flag = false;

    //    char key_input;
     //   cap >> usb_img;
/*        if (usb_img.empty()) {
            std::cout << "empty frame" << std::endl;
            continue;
        }
        //设置大小
        cv::resize(usb_img, usb_img, cv::Size(640, 480));
        //显示图像
        cv::imshow("usb_cam", usb_img);*/
   /*     key_input = cv::waitKey(10);
        if (key_input == 'q') {
            break;
        }*/
        /*************************************** fill-in-the-blank code block **************************************/
        // design and add your code for keyboard mapping
        // also feel free to create, read, update or delete any code in the whole file
        // one simple example:
        /*if (joy_button[2] == 1)
          rcm_beta += 0.3 / 180.0 * M_PI;*/
        if (joy_button[3] == 1)
            rcm_beta += 0.3 / 180.0 * M_PI;
        if (joy_button[1] == 1)
            rcm_beta -= 0.3 / 180.0 * M_PI;
        if (joy_button[2] == 1)
            rcm_alpha += 0.3 / 180.0 * M_PI;
        if (joy_button[0] == 1)
            rcm_alpha -= 0.3 / 180.0 * M_PI;
        if (joy_button[4] == 1)
            rcm_trans +=0.01;
        if (joy_button[5] == 1)
            rcm_trans-=0.01;


        /****************************************** RCM motion iteration *******************************************/
        // map RCM angle (rcm_alpha, rcm_beta) to RCM motion posture (rcm_rotation_update)
        Eigen::AngleAxisd rcm_alpha_m(Eigen::AngleAxisd(rcm_alpha, Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd rcm_beta_m(Eigen::AngleAxisd(rcm_beta, Eigen::Vector3d::UnitY()));
        Eigen::Matrix3d rcm_rotation_update = rcm_alpha_m.matrix() * rcm_beta_m.matrix();

        // tmp RCM point homogeneous coordinate transformation matrix (T_base_rcm_update)
        Matrix4d T_base_rcm_update = Matrix4d::Identity();
        T_base_rcm_update.block<3, 3>(0, 0) = T_base_end_init.block<3, 3>(0, 0) * rcm_rotation_update;
        T_base_rcm_update.block<3, 1>(0, 3) = prcm_base_init.block<3, 1>(0, 0);

        // tmp robotic arm end point considering RCM translation (rcm_trans)
        // under end frame (pend_rcm_update), under base frame (pend_base_update)
        double rcm_len_update = rcm_len - rcm_trans;
        Vector4d pend_rcm_update;
        pend_rcm_update << 0, 0, -rcm_len_update, 1;
        Vector4d pend_base_update;
        pend_base_update = T_base_rcm_update * pend_rcm_update;

        // robotic arm end pose to be input (T_base_end_update)
        Matrix4d T_base_end_update = Matrix4d::Identity();
        T_base_end_update.block<3, 3>(0, 0) = T_base_rcm_update.block<3, 3>(0, 0);
        T_base_end_update.block<3, 1>(0, 3) = pend_base_update.block<3, 1>(0, 0);

        // update robotic arm end pose
        dobot_bringup::ToolVectorActual RobotToolDegPose;
        RobotToolDegPose.x = T_base_end_update(0, 3) * 1000;
        RobotToolDegPose.y = T_base_end_update(1, 3) * 1000;
        RobotToolDegPose.z = T_base_end_update(2, 3) * 1000;
        Vector3d rpy_relative = transR2RPY(T_base_end_update.block<3, 3>(0, 0), "zyx");
        RobotToolDegPose.rx = (rpy_relative(2, 0) / M_PI) * 180;
        RobotToolDegPose.ry = (rpy_relative(1, 0) / M_PI) * 180;
        RobotToolDegPose.rz = (rpy_relative(0, 0) / M_PI) * 180;
        cout << RobotToolDegPose.x << " " << RobotToolDegPose.y << " " << RobotToolDegPose.z << " "
             << RobotToolDegPose.rx << " " << RobotToolDegPose.ry << " " << RobotToolDegPose.rz << endl;
        dobot_pose_pub.publish(RobotToolDegPose);

    }
}

Vector3d transR2RPY(Matrix3d t, string xyz) {
    Vector3d rpy;
    // t   x    y   z
    //     0    1   2   3
    // 0   nx   ox  ax   0
    // 1   ny   oy  ay   0
    // 2   nz   oz  az   0
    // 3    0    0   0   1

    if (xyz == "xyz") {
        if (fabs(t(2, 2)) < 1e-12 && fabs(t(1, 2)) < 1e-12) {                                   //% singularity
            rpy(0) = 0;                       //% roll is zero
            rpy(1) = atan2(t(0, 2), t(2, 2)); //% pitch
            rpy(2) = atan2(t(1, 0), t(1, 1)); //% yaw is sum of roll+yaw
        } else {
            rpy(0) = atan2(-t(1, 2), t(2, 2)); //% roll
            //% compute sin/cos of roll angle
            double sr = sin(rpy(0));
            double cr = cos(rpy(0));
            rpy(1) = atan2(t(0, 2), cr * t(2, 2) - sr * t(1, 2)); //% pitch
            rpy(2) = atan2(-t(0, 1), t(0, 0));                    //% yaw
        }
    } else if (xyz == "zyx") {
        //            % old ZYX order (as per Paul book)
        if (fabs(t(0, 0)) < 1e-12 && fabs(t(1, 0)) < 1e-12) {                                    //% singularity
            rpy(0) = 0;                        // roll is zero
            rpy(1) = atan2(-t(2, 0), t(0, 0)); // pitch
            rpy(2) = atan2(-t(1, 2), t(1, 1)); // yaw is difference yaw-roll
        } else {
            rpy(0) = atan2(t(1, 0), t(0, 0));
            double sp = sin(rpy(0));
            double cp = cos(rpy(0));
            rpy(1) = atan2(-t(2, 0), cp * t(0, 0) + sp * t(1, 0));
            rpy(2) = atan2(sp * t(0, 2) - cp * t(1, 2), cp * t(1, 1) - sp * t(0, 1));
        }
    }

    return rpy;
}


// Detect the center of the image
cv::Point2d detectCenter(cv::Mat image) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image.clone(), contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    std::vector<cv::Moments> mu(contours.size());
    for (int i = 0; i < contours.size(); i++) {
        mu[i] = cv::moments(contours[i], false);
    }
    vector<cv::Point2f> mc(contours.size());
    for (int i = 0; i < contours.size(); i++) {
        mc[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
    }

    cv::Point2d center;
    center.x = (mc[0].x);
    center.y = (mc[0].y);
    return center;
}

int detectHSColor(const cv::Mat &image, double minHue, double maxHue, double minSat, double maxSat, cv::Mat &mask) {
    cv::Mat hsv;
    cv::cvtColor(image, hsv, CV_BGR2HSV);
    std::vector<cv::Mat> channels;
    split(hsv, channels);
    cv::Mat mask1, mask2, hueMask;
    cv::threshold(channels[0], mask1, maxHue, 255, cv::THRESH_BINARY_INV);
    cv::threshold(channels[0], mask2, minHue, 255, cv::THRESH_BINARY);
    if (minHue < maxHue) {
        hueMask = mask1 & mask2;
    } else {
        hueMask = mask1 | mask2;
    }
    cv::Mat satMask;
    inRange(channels[1], minSat, maxSat, satMask);
    mask = hueMask & satMask;
    //检测色块的大小
    int nonZeroPixels = cv::countNonZero(mask);
    return nonZeroPixels;
}


/*
void to_blue(double &rcm_alpha, double &rcm_beta, double &rcm_trans) {

    double Kp = 0.6;
    double minHue = 110.0; // 蓝色的最小色调值
    double maxHue = 130.0; // 蓝色的最大色调值
    double minSat = 100.0; // 饱和度的最小值
    double maxSat = 255.0; // 饱和度的最大值
    cv::Mat mask; // 函数返回的掩膜
    cv::Mat world_mask;
    // 调用函数
    int nonZeroPixels = detectHSColor(usb_img, minHue, maxHue, minSat, maxSat, mask);
    if (nonZeroPixels == 0) {
        cout << "not found blue" << endl;
        if (cv::getWindowProperty("mask", cv::WND_PROP_AUTOSIZE) == -1) {
            return;
        } else {
            cv::destroyWindow("mask");
            return;
        }
    }
    cv::imshow("mask", mask);
    cv::Point2d blue_center = detectCenter(mask);
    cout << "blue_center: " << blue_center << endl;
    cout << "blue_size: " << nonZeroPixels << endl;
    rcm_alpha += (0.01 * (blue_center.x - 320) / 180.0 * M_PI) * Kp;
    rcm_beta += (0.01 * (blue_center.y - 240) / 180.0 * M_PI) * Kp;
    rcm_trans += 0.01 * Kp;
}

void to_green(double &rcm_alpha, double &rcm_beta, double &rcm_trans) {

    double Kp = 0.5;
    double minHue = 30.0; // 蓝色的最小色调值
    double maxHue = 60.0;// 蓝色的最大色调值
    double minSat = 100.0; // 饱和度的最小值
    double maxSat = 255.0; // 饱和度的最大值
    cv::Mat mask; // 函数返回的掩膜
    // 调用函数
    int nonZeroPixels = detectHSColor(usb_img, minHue, maxHue, minSat, maxSat, mask);
    if (nonZeroPixels == 0) {
        cout << "not found green" << endl;
        if (cv::getWindowProperty("mask", cv::WND_PROP_AUTOSIZE) == -1) {
            return;
        } else {
            cv::destroyWindow("mask");
            return;
        }
    }
    cv::imshow("mask", mask);
    cv::Point2d green_center = detectCenter(mask);
    cout << "green_center: " << green_center << endl;
    cout << "green_size: " << nonZeroPixels << endl;
    rcm_alpha += (0.01 * (green_center.x - 320) / 180.0 * M_PI) * Kp;
    rcm_beta += (0.01 * (green_center.y - 240) / 180.0 * M_PI) * Kp;
    rcm_trans += 0.01 * Kp;
}


void to_yellow(double &rcm_alpha, double &rcm_beta, double &rcm_trans) {

    double Kp = 0.4;
    double minHue = 0.0; // 黄色的最小色调值
    double maxHue = 30.0; // 黄色的最大色调值
    double minSat = 100.0; // 饱和度的最小值
    double maxSat = 255.0; // 饱和度的最大值
    cv::Mat mask; // 函数返回的掩膜
    // 调用函数
    int nonZeroPixels = detectHSColor(usb_img, minHue, maxHue, minSat, maxSat, mask);
    if (nonZeroPixels == 0) {
        cout << "not found yellow" << endl;
        if (cv::getWindowProperty("mask", cv::WND_PROP_AUTOSIZE) == -1) {
            return;
        } else {
            cv::destroyWindow("mask");
            return;
        }
    }
    cv::imshow("mask", mask);
    cv::Point2d yellow_center = detectCenter(mask);
    cout << "yellow_center: " << yellow_center << endl;
    cout << "yellow_size: " << nonZeroPixels << endl;
    rcm_alpha += (0.01 * (yellow_center.x - 320) / 180.0 * M_PI) * Kp;
    rcm_beta += (0.01 * (yellow_center.y - 240) / 180.0 * M_PI) * Kp;
    rcm_trans += 0.01 * Kp;
}*/
