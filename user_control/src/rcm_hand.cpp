//
// Created by idear on 24-4-11.
//
/**
 * @file rcm_keyboard.cpp
 * @brief
 * @author Zheng Xu (xz200103@sjtu.edu.cn)
 * @version 1.0
 * @date 2024-03-01
 *
 * @copyright Copyright (c) 2024 Robotics-GA
 *
 * @par logs:
 * <table>
 * <tr><th>Date       <th>Version <th>Author   <th>Description
 * <tr><td>2024-03-01 <td>1.0     <td>Zheng Xu <td>Initial version
 * <tr><td>2024-03-14 <td>1.1     <td>Zheng Xu <td>Design fill-in-the-blank questions
 * </table>
 */

#include <iostream>

// import the matrix operation library Eigen
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <math.h>
#include <termios.h>

#define CVUI_IMPLEMENTATION

#include "cvui.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include <ros/ros.h>
#include <gazebo_msgs/DeleteModel.h>
#include "sensor_msgs/Image.h"
#include "message_filters/subscriber.h"
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <std_msgs/Int32MultiArray.h>

#include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace std;
using namespace Eigen;

void CR5_setEndPose(moveit::planning_interface::MoveGroupInterface &group, Matrix4Xd CR5_EndPose_input);

Matrix4Xd CR5_getEndPose(moveit::planning_interface::MoveGroupInterface &group);

void CR5_setJointValue(moveit::planning_interface::MoveGroupInterface &group, VectorXd CR5_joint_angle);

VectorXd CR5_getJointValue(moveit::planning_interface::MoveGroupInterface &group,
                           const robot_state::JointModelGroup *joint_model_group);

static cv::Mat hmerge;
static cv::Mat hand;
int hand_x;
int hand_y;
int hand_z;
static const double Kp = 16.0;

char getch();

void target(double &rcm_alpha, double &rcm_beta);

void imageCallback(const sensor_msgs::ImageConstPtr &msg1, const sensor_msgs::ImageConstPtr &msg2);
void centerCallback(const std_msgs::Int32MultiArray::ConstPtr &msg);

cv::Point2d detectCenter(cv::Mat image);

int detectHSColor(const cv::Mat &image, double minHue, double maxHue, double minSat, double maxSat, cv::Mat &mask);

void drawing(cv::Mat &image);
int main(int argc, char **argv) {

    ros::init(argc, argv, "rcm_keyboard_node");
    ros::NodeHandle nh;
    //execlp("roslaunch", "roslaunch", "user_control","hand_tracking.launch", NULL);
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
    ros::service::waitForService("/gazebo/delete_model");

    message_filters::Subscriber<sensor_msgs::Image> subscriber_world(nh, "/world_camera/image_raw", 100,
                                                                     ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<sensor_msgs::Image> subscriber_arm(nh, "/camera/image_raw", 100,
                                                                   ros::TransportHints().tcpNoDelay());
    //接收消息
    ros::Subscriber sub = nh.subscribe<std_msgs::Int32MultiArray>("/hand_tracking", 100, centerCallback);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
    //message_filters::TimeSynchronizer<sensor_msgs::LaserScan,geometry_msgs::PoseWithCovarianceStamped> sync(subscriber_laser, subscriber_pose, 10);
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), subscriber_world, subscriber_arm);
    sync.registerCallback(boost::bind(imageCallback, _1, _2));


    ros::AsyncSpinner spinner(1);
    spinner.start();

    /*********************************** gazebo env param init ******************************************/
    bool green_flag = 0;
    bool yellow_flag = 0;
    bool blue_flag = 0;
    Vector3d p_yellow_wb(0.550602, -0.082265, 0.002491);
    Vector3d p_blue_wb(0.464286, -0.128259, 0.002491);
    Vector3d p_green_wb(0.361654, -0.094127, 0.002491);



    /*********************************** CR5-moveit & RCM point init ************************************/
    // moveit planning group init
    static const std::string PLANNING_GROUP = "cr5_arm";
    moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
    const robot_state::JointModelGroup *joint_model_group = group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    // Get original joint angle
    VectorXd CR5_joint_angle = CR5_getJointValue(group, joint_model_group);
    cout << CR5_joint_angle.transpose() << endl;
    // Set initial joint angle
    VectorXd CR5_joint_angle_init = VectorXd::Zero(6);
    CR5_joint_angle_init[0] = 0;
    CR5_joint_angle_init[1] = 0;
    CR5_joint_angle_init[2] = -M_PI / 2;
    CR5_joint_angle_init[3] = 0;
    CR5_joint_angle_init[4] = M_PI / 2;
    CR5_joint_angle_init[5] = 0;
    CR5_setJointValue(group, CR5_joint_angle_init);
    sleep(5);
    // Get initial end pose (T_base_end_init)
    Matrix4Xd T_base_end_init = CR5_getEndPose(group);
    cout << T_base_end_init << endl;
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

    //打开摄像头

    while (ros::ok()) {

        cv::imshow("mask", hmerge);
        cv::waitKey(500);

        /****************************************** gazebo contact check ****************************************/
        // get tmp end pose (CR5_EndPose)
        Matrix4Xd CR5_EndPose = CR5_getEndPose(group);
        // Get tmp instrument tip point position
        // under end frame (ptip_end), under base frame (ptip_base), under world frame (ptip_wb)
        Vector4d ptip_end, ptip_base;
        ptip_end << 0, 0, rcm_len, 1;
        ptip_base = CR5_EndPose * ptip_end;
        Vector3d ptip_wb = ptip_base.block<3, 1>(0, 0);
        ptip_wb(2) += 0.08;
        // Check whether the contact is successful
        if ((p_yellow_wb - ptip_wb).norm() < 0.015) {
            yellow_flag = 1;
            gazebo_msgs::DeleteModel deModel;
            deModel.request.model_name = "rcm_tp_yellow";
            client.call(deModel);
            cout << "rcm_tp_yellow" << endl;
        }
        if ((p_blue_wb - ptip_wb).norm() < 0.015) {
            blue_flag = 1;
            gazebo_msgs::DeleteModel deModel;
            deModel.request.model_name = "rcm_tp_blue";
            client.call(deModel);
            cout << "rcm_tp_blue" << endl;
        }
        if ((p_green_wb - ptip_wb).norm() < 0.015) {
            green_flag = 1;
            gazebo_msgs::DeleteModel deModel;
            deModel.request.model_name = "rcm_tp_green";
            client.call(deModel);
            cout << "rcm_tp_green" << endl;
        }
        if (green_flag == 1 && yellow_flag == 1 && blue_flag == 1) {
            cout << "success" << endl;
            break;
        }


        /*************************************** fill-in-the-blank code block **************************************/
        // design and add your code for keyboard mapping
        // also feel free to create, read, update or delete any code in the whole file
        // one simple example:
        /*  if (key_input == 'w')
                rcm_alpha += 1.0 / 180.0 * M_PI;
        */
        //target(rcm_alpha, rcm_beta);
        cout << "hand_x: " << hand_x << " hand_y: " << hand_y << " hand_z: " << hand_z << endl;
        if(hand_x!=-1&&hand_y!=-1&&hand_z!=-1){
            if(abs(hand_x-320)>220){
                if(hand_x>320){
                    rcm_alpha -= 0.5 / 180.0 * M_PI;
                } else if(hand_x<320){
                    rcm_alpha += 0.5 / 180.0 * M_PI;
                }
            }
            if(abs(hand_y-240)>140){
                if(hand_y>240){
                    rcm_beta -= 0.5 / 180.0 * M_PI;
                } else if(hand_y<240){
                    rcm_beta += 0.5 / 180.0 * M_PI;
                }
            }
            if(hand_z == 1){
                rcm_trans += 0.01;
            } else if(hand_z == 0) {
                rcm_trans -= 0.01;
            } else if(hand_z == 2){

            }


        }
        cout << "rcm_alpha: " << rcm_alpha << endl;
        cout << "rcm_beta: " << rcm_beta << endl;
        cout << "rcm_trans: " << rcm_trans << endl;

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
        CR5_setEndPose(group, T_base_end_update);


    }

    // 关闭所有窗口
    cv::destroyAllWindows();

    ros::shutdown();
    return 0;

}


void CR5_setEndPose(moveit::planning_interface::MoveGroupInterface &group, Matrix4Xd CR5_EndPose_input) {

    geometry_msgs::Pose target_pose1;

    Quaterniond q_curr_tmp = Quaterniond(CR5_EndPose_input.block<3, 3>(0, 0));
    q_curr_tmp.normalize();

    target_pose1.orientation.x = q_curr_tmp.x();
    target_pose1.orientation.y = q_curr_tmp.y();
    target_pose1.orientation.z = q_curr_tmp.z();
    target_pose1.orientation.w = q_curr_tmp.w();
    target_pose1.position.x = CR5_EndPose_input(0, 3);
    target_pose1.position.y = CR5_EndPose_input(1, 3);
    target_pose1.position.z = CR5_EndPose_input(2, 3);

    group.setPoseTarget(target_pose1);


    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = static_cast<bool>(group.plan(my_plan));
    ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    if (success)
        group.asyncExecute(my_plan);

}

Matrix4Xd CR5_getEndPose(moveit::planning_interface::MoveGroupInterface &group) {
    Matrix4Xd CR5_EndPose = Matrix4Xd::Identity(4, 4);

    geometry_msgs::PoseStamped CurrentPose = group.getCurrentPose();


    Eigen::Quaterniond q_curr_tmp;
    q_curr_tmp.x() = CurrentPose.pose.orientation.x;
    q_curr_tmp.y() = CurrentPose.pose.orientation.y;
    q_curr_tmp.z() = CurrentPose.pose.orientation.z;
    q_curr_tmp.w() = CurrentPose.pose.orientation.w;

    CR5_EndPose.block<3, 3>(0, 0) = q_curr_tmp.normalized().toRotationMatrix();

    CR5_EndPose(0, 3) = CurrentPose.pose.position.x;
    CR5_EndPose(1, 3) = CurrentPose.pose.position.y;
    CR5_EndPose(2, 3) = CurrentPose.pose.position.z;

    return CR5_EndPose;
}

void CR5_setJointValue(moveit::planning_interface::MoveGroupInterface &group, VectorXd CR5_joint_angle) {

    std::vector<double> joint_group_positions(&CR5_joint_angle[0],
                                              CR5_joint_angle.data() + CR5_joint_angle.cols() * CR5_joint_angle.rows());

    group.setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    if (success)
        group.asyncExecute(my_plan);
}


VectorXd CR5_getJointValue(moveit::planning_interface::MoveGroupInterface &group,
                           const robot_state::JointModelGroup *joint_model_group) {
    moveit::core::RobotStatePtr current_state = group.getCurrentState();

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    Eigen::VectorXd CR5_joint_angle = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(joint_group_positions.data(),
                                                                                    joint_group_positions.size());

    return CR5_joint_angle;

}

char getch() {
    system("stty -icanon");
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
        perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");
    return buf;
}

void target(double &rcm_alpha, double &rcm_beta) {
    char key_input;
    while (ros::ok()) {
        key_input = getch(); // 获取按键
        if (key_input == 'w')
            rcm_alpha += 1.0 / 180.0 * M_PI;
        else if (key_input == 's')
            rcm_alpha -= 1.0 / 180.0 * M_PI;
        else if (key_input == 'a')
            rcm_beta += 1.0 / 180.0 * M_PI;
        else if (key_input == 'd')
            rcm_beta -= 1.0 / 180.0 * M_PI;
        std::cout << "Key Pressed: " << key_input << std::endl;
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg1, const sensor_msgs::ImageConstPtr &msg2) {

    cv::Mat img1_resized;
    cv::Mat img2_resized;
    cv::Size size(640, 480); // 新的尺寸，例如640x480
    cv::Mat img1 = cv_bridge::toCvShare(msg1, "bgr8")->image;
    cv::Mat img2 = cv_bridge::toCvShare(msg2, "bgr8")->image;
    cv::resize(img1, img1_resized, size, 0, 0, cv::INTER_LINEAR);
    cv::resize(img2, img2_resized, size, 0, 0, cv::INTER_LINEAR);
    hand = img2_resized;
    cv::hconcat(img1_resized, img2_resized, hmerge);

}
void centerCallback(const std_msgs::Int32MultiArray::ConstPtr &msg){
    hand_x = msg->data[0];
    hand_y = msg->data[1];
    hand_z = msg->data[2];
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
    //绘制轮廓
    cv::Mat drawing = cv::Mat::zeros(image.size(), CV_8UC1);
    for (int i = 0; i < contours.size(); i++) {
        cv::drawContours(drawing, contours, i, cv::Scalar(255), 2, 8, hierarchy, 0, cv::Point());
        cv::circle(drawing, mc[i], 4, cv::Scalar(255), -1, 8, 0);
    }
    cv::imshow("outImage", drawing);

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
void drawing(cv::Mat &image) {
    //绘制凸包
    // Assuming 'o' is your source image
    cv::Mat gray, binary;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY); // Convert image to grayscale
    cv::threshold(gray, binary, 127, 255, cv::THRESH_BINARY); // Apply threshold
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
// Find contours in the binary image
    cv::findContours(binary, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
// Compute the convex hull of the first contour
    std::vector<cv::Point> hull;
    cv::convexHull(contours[0], hull);
// Draw the convex hull on the original image
    cv::polylines(image, hull, true, cv::Scalar(0, 255, 0), 2);

    cv::imshow("drawing", image);
    cv::imshow("binary", binary);
    cv::imshow("gray", gray);
}