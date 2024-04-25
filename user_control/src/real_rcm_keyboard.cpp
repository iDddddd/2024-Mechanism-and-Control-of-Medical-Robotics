/**
 * @file real_rcm_keyboard.cpp
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

#include <math.h>

#include <iostream>

#define CVUI_IMPLEMENTATION

#include "cvui.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include "sensor_msgs/Image.h"
#include "message_filters/subscriber.h"
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

using namespace std;
using namespace Eigen;
static cv::Mat usb_img;
static const double Kp = 10.0;

Vector3d transR2RPY(Matrix3d t, string xyz);

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv::Mat img_resized;
    cv::Size size(640, 480); // 新的尺寸，例如640x480
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::resize(img, img_resized, size, 0, 0, cv::INTER_LINEAR);
    usb_img = img_resized;
}

void to_blue(double &rcm_alpha, double &rcm_beta, double &rcm_trans);

cv::Point2d detectCenter(cv::Mat image);

int detectHSColor(const cv::Mat &image, double minHue, double maxHue, double minSat, double maxSat, cv::Mat &mask);

// pose callback for robotic arm
bool dobot_pose_flag = false;
dobot_bringup::ToolVectorActual RobotToolPose;

void dobot_pos_Callback(const dobot_bringup::ToolVectorActual msg) {
    RobotToolPose = msg;
    dobot_pose_flag = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "real_rcm_keyboard_node");
    ros::NodeHandle nh;

//    ros::Subscriber dobot_subscriber = nh.subscribe<dobot_bringup::ToolVectorActual>(
//            "/dobot_bringup/msg/ToolVectorActual", 1, dobot_pos_Callback);
    ros::Publisher dobot_pose_pub = nh.advertise<dobot_bringup::ToolVectorActual>("/dobot_bringup/RobotServop", 1);

    ros::Subscriber subscriber_image = nh.subscribe<sensor_msgs::Image>("/image_view/output_mouse_left", 10, imageCallback);

    // read tmp robotic arm pose
//    while (ros::ok() && dobot_pose_flag == false) {
//        ros::spinOnce();
//    }
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
    /**********************************************************************************************************/

    // keyboard control prompt words
    cout << "forward w, back s" << endl;
    cout << "left a, right d" << endl;
    cout << "up i, down k" << endl;

    while (ros::ok()) {

        cout << "input now!" << endl;
        char key_input;
        if (usb_img.empty()) {
            std::cout << "Image is empty. Cannot display image." << std::endl;
        } else {
            cv::imshow("usb_cam", usb_img);
        }

//        while (ros::ok() && dobot_pose_flag == false) {
//            ros::spinOnce();
//        }
        dobot_pose_flag = false;


        /*************************************** fill-in-the-blank code block **************************************/
        // design and add your code for keyboard mapping
        // also feel free to create, read, update or delete any code in the whole file
        // one simple example:
        /*if (key_input == 'd')
          rcm_beta += 1.0 / 180.0 * M_PI;*/
        switch (key_input) {
            case 'w':
                rcm_alpha += 1.0 / 180.0 * M_PI;
                break;
            case 's':
                rcm_alpha -= 1.0 / 180.0 * M_PI;
                break;
            case 'a':
                rcm_beta += 1.0 / 180.0 * M_PI;
                break;
            case 'd':
                rcm_beta -= 1.0 / 180.0 * M_PI;
                break;
            case 'u':
                rcm_trans += 0.01;
                break;
            case 'i':
                rcm_trans -= 0.01;
                break;
        }

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

void to_blue(double &rcm_alpha, double &rcm_beta, double &rcm_trans) {

    double minHue = 110.0; // 蓝色的最小色调值
    double maxHue = 130.0; // 蓝色的最大色调值
    double minSat = 100.0; // 饱和度的最小值
    double maxSat = 255.0; // 饱和度的最大值
    cv::Mat mask; // 这将是函数返回的掩膜
    // 调用函数
    int nonZeroPixels = detectHSColor(usb_img, minHue, maxHue, minSat, maxSat, mask);
    if (nonZeroPixels == 0) {
        cout << "not found blue" << endl;
        rcm_alpha = 0;
        rcm_beta = 0;
        rcm_trans = 0;
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
    rcm_alpha += (Kp * (blue_center.x - 320) / nonZeroPixels / 180.0 * M_PI);
    rcm_beta += (Kp * (blue_center.y - 240) / nonZeroPixels / 180.0 * M_PI);
    rcm_trans += 0.02;
}