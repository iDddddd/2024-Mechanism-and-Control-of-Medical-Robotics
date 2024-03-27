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
#include <unistd.h>


#include <ros/ros.h>
#include <gazebo_msgs/DeleteModel.h>

#include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace std;
using namespace Eigen;

void CR5_setEndPose(moveit::planning_interface::MoveGroupInterface &group, Matrix4Xd CR5_EndPose_input);
Matrix4Xd CR5_getEndPose(moveit::planning_interface::MoveGroupInterface &group);
void CR5_setJointValue(moveit::planning_interface::MoveGroupInterface &group, VectorXd CR5_joint_angle);
VectorXd CR5_getJointValue(moveit::planning_interface::MoveGroupInterface &group, const robot_state::JointModelGroup* joint_model_group);


int main(int argc, char **argv)
{

    ros::init(argc, argv, "rcm_keyboard_node");
    ros::NodeHandle nh; 

    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
    ros::service::waitForService("/gazebo/delete_model");

    ros::AsyncSpinner spinner(1);
    spinner.start();
 
    /*********************************** gazebo env param init ******************************************/
    bool green_flag = 0;
    bool yellow_flag = 0;
    bool blue_flag = 0;
    Vector3d p_yellow_wb(0.550602,-0.082265,0.002491);
    Vector3d p_blue_wb(0.464286,-0.128259,0.002491);
    Vector3d p_green_wb(0.361654,-0.094127,0.002491);



    /*********************************** CR5-moveit & RCM point init ************************************/
    // moveit planning group init 
    static const std::string PLANNING_GROUP = "cr5_arm";
    moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
    const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    // Get original joint angle 
    VectorXd CR5_joint_angle = CR5_getJointValue(group, joint_model_group);
    cout<<CR5_joint_angle.transpose()<<endl;
    // Set initial joint angle 
    VectorXd CR5_joint_angle_init = VectorXd::Zero(6);
    CR5_joint_angle_init[0] = 0;
    CR5_joint_angle_init[1] = 0;
    CR5_joint_angle_init[2] = -M_PI/2;
    CR5_joint_angle_init[3] = 0;    
    CR5_joint_angle_init[4] = M_PI/2;
    CR5_joint_angle_init[5] = 0;
    CR5_setJointValue(group, CR5_joint_angle_init);
    sleep(5);
    // Get initial end pose (T_base_end_init)
    Matrix4Xd T_base_end_init = CR5_getEndPose(group);
    cout<<T_base_end_init<<endl;
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

    while(ros::ok())
    {
        // get keyboard input
        cout << "input now!" << endl;
        char key_input;
        cin >> key_input;

    /****************************************** gazebo contact check ****************************************/
        // get tmp end pose (CR5_EndPose)
        Matrix4Xd CR5_EndPose = CR5_getEndPose(group);
        // Get tmp instrument tip point position
        // under end frame (ptip_end), under base frame (ptip_base), under world frame (ptip_wb)
        Vector4d ptip_end, ptip_base;
        ptip_end << 0, 0, rcm_len, 1;
        ptip_base = CR5_EndPose * ptip_end;
        Vector3d ptip_wb = ptip_base.block<3,1>(0,0);
        ptip_wb(2) += 0.08;
        // Check whether the contact is successful
        if((p_yellow_wb - ptip_wb).norm() < 0.015) 
        {
            yellow_flag = 1;
            gazebo_msgs::DeleteModel deModel;
            deModel.request.model_name = "rcm_tp_yellow";
            client.call(deModel);
            cout<<"rcm_tp_yellow"<<endl;
        }
        if((p_blue_wb - ptip_wb).norm() < 0.015)
        {
            blue_flag = 1;
            gazebo_msgs::DeleteModel deModel;
            deModel.request.model_name = "rcm_tp_blue";
            client.call(deModel);
            cout<<"rcm_tp_blue"<<endl;
        }
        if((p_green_wb - ptip_wb).norm() < 0.015)
        {
            green_flag = 1;
            gazebo_msgs::DeleteModel deModel;
            deModel.request.model_name = "rcm_tp_green";
            client.call(deModel);
            cout<<"rcm_tp_green"<<endl;
        }        
        if(green_flag == 1&&yellow_flag == 1&&blue_flag == 1)
        {
            cout<<"success"<<endl;
            break;
        }
    
    
    /*************************************** fill-in-the-blank code block **************************************/
    // design and add your code for keyboard mapping
    // also feel free to create, read, update or delete any code in the whole file
    // one simple example:
    /*  if (key_input == 'w')
            rcm_alpha += 1.0 / 180.0 * M_PI;
    */
        if(key_input == 'q')
            break;

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

  
    ros::shutdown(); 
    return 0;
    
}




void CR5_setEndPose(moveit::planning_interface::MoveGroupInterface &group, Matrix4Xd CR5_EndPose_input)
{

    geometry_msgs::Pose target_pose1;

    Quaterniond q_curr_tmp = Quaterniond(CR5_EndPose_input.block<3,3>(0,0));//旋转矩阵转为四元数
    q_curr_tmp.normalize();//转为四元数之后，需要进行归一化

    target_pose1.orientation.x= q_curr_tmp.x();
    target_pose1.orientation.y = q_curr_tmp.y();
    target_pose1.orientation.z = q_curr_tmp.z();
    target_pose1.orientation.w = q_curr_tmp.w();
    target_pose1.position.x = CR5_EndPose_input(0,3);
    target_pose1.position.y = CR5_EndPose_input(1,3);
    target_pose1.position.z = CR5_EndPose_input(2,3);

    group.setPoseTarget(target_pose1);

 
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = static_cast<bool>(group.plan(my_plan));
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");   

    if(success)
      group.asyncExecute(my_plan);

}

Matrix4Xd CR5_getEndPose(moveit::planning_interface::MoveGroupInterface &group)
{
    Matrix4Xd CR5_EndPose = Matrix4Xd::Identity(4,4);

    geometry_msgs::PoseStamped CurrentPose = group.getCurrentPose();

    
    Eigen::Quaterniond q_curr_tmp;
    q_curr_tmp.x() = CurrentPose.pose.orientation.x;
    q_curr_tmp.y() = CurrentPose.pose.orientation.y;
    q_curr_tmp.z() = CurrentPose.pose.orientation.z;
    q_curr_tmp.w() = CurrentPose.pose.orientation.w;

    CR5_EndPose.block<3,3>(0,0) = q_curr_tmp.normalized().toRotationMatrix();

    CR5_EndPose(0,3) = CurrentPose.pose.position.x;
    CR5_EndPose(1,3) = CurrentPose.pose.position.y;
    CR5_EndPose(2,3) = CurrentPose.pose.position.z;
    
    return CR5_EndPose;
}

void CR5_setJointValue(moveit::planning_interface::MoveGroupInterface &group, VectorXd CR5_joint_angle)
{
    
    std::vector<double> joint_group_positions(&CR5_joint_angle[0], CR5_joint_angle.data()+CR5_joint_angle.cols()*CR5_joint_angle.rows());

    cout<<"1111"<<endl;
    group.setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED"); 
    if(success)
        group.asyncExecute(my_plan);
}



VectorXd CR5_getJointValue(moveit::planning_interface::MoveGroupInterface &group, const robot_state::JointModelGroup* joint_model_group)
{
    moveit::core::RobotStatePtr current_state = group.getCurrentState();

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    Eigen::VectorXd CR5_joint_angle = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(joint_group_positions.data(), joint_group_positions.size());

    return CR5_joint_angle;

}

 