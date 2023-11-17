#ifndef ROBOT_H
#define ROBOT_H

/**
    @author: YuelinZOU
    @brief: This class/robot is set for ur_robot arm. 
    @copyright: Belongs to Multi-Scale Medecial Robotics Center, The Chinese University of Hong Kong.
**/
#include "ros/ros.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64MultiArray.h>
// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>

#include "function/ur_kine.h"
#include "function/Transformer.h"

#include <iostream>
#include <algorithm>
#include <chrono>
#include <vector>
#include <thread>
#include <mutex>
#include <cmath>

#define ROBOT_DOF           6
#define FORCEMIN           0.2
#define FORCEMAX           10.0
#define MOVETHRESHOLD      2e-4
#define GRAVITYACC          9.8
#define TIME_STEP         0.008


class robot
{
public:
    robot(ros::NodeHandle &_nh,bool disableforce=true);
    bool stepForward();
    bool moveForward();
    inline bool isOk(){ 
        if((flag_force && flag_position) || (disableForce && flag_position)) return true;
        else return false;
    }
    void stop(){ isMoving = false;}
    void start(){ isMoving = true;}

    Eigen::Matrix<double,ROBOT_DOF,ROBOT_DOF> jacobian();
    Eigen::Matrix4d forward();
    void inverKinematic(Eigen::Matrix4d Tu,double *q_sols);

    inline Eigen::Matrix4d getTransformation(){ return Trans_;}
    inline double* getTransformationArray(){ return T_;}
    inline Eigen::Matrix3d getRotation(){ return Rotation_;}
    inline Eigen::Vector3d getTranslation(){ return Translation_;}
    inline Eigen::Matrix<double,ROBOT_DOF,ROBOT_DOF> getJacoM(){ return Jaco_;}
    inline Eigen::Vector3d getForce(){ return cur_force;}
    double* getJointsStatus(){ return q_;}

    /**velocity control
        @parameter: velocity
        @return: the step from robot arm
        example:
            robot ur;
            ur.setGravity(0.180);
            ur.setBias(a,b,c);
            //check the stats of robot by using ur.isOk();
            //given velocity vel
            Eigen::VectorXd vstep = ur.calculateStep(vel);
            ur.setDestinationDelta(vstep);
            ur.stepForward();
    **/

    Eigen::VectorXd calculateStep(Eigen::VectorXd vel);
    Eigen::VectorXd calculateJointSpeed(Eigen::VectorXd vel);

    //utilities
    inline Eigen::Vector3d mapFromHandToWorld(Eigen::Vector3d data){
        return Rotation_*data;
    }
    inline Eigen::Vector3d mapFromWorldToHand(Eigen::Vector3d data){
        return Rotation_.transpose()*data;
    }

    void setBias(double dx = 0 , double dy = 0 , double dz = 0){
        dx_ = dx;
        dy_ = dy;
        dz_ = dz;
    }
    void setDestinationDelta(double* vStep);
    void setDestinationDelta(Eigen::VectorXd vStep);
    void setDestinationObsolute(double* tra);
    void setVelocity(double* vel);
    void setVelocity(Eigen::VectorXd vel);

    void setGravity(double MASS){ gravity_<< 0, 0, -GRAVITYACC*MASS;} //kg
    //obtain information
    inline double* getCurrentPos(){ return q_;}
    inline Eigen::Vector3d getGravity(){ return gravity_;}

    void kalmanFilter(Eigen::Vector3d &data);
    inline void resetKalmanFilter(){ dFflag = 1;} 

    //parameters for impedence control
    inline void setMBK(double M, double B, double K){
        M_ = M;
        B_ = B;
        K_ = K;
    }
    void impedenceControlXYZ(Eigen::Vector3d exterForce);
    inline Eigen::Vector3d getImpPose(){ return xpose;}
    //TODO: Set the same process for torque, perhaps you can realize it in the form of Euler angle.
    void impedenceControlTorque(Eigen::Vector3d exterTorque);

    inline void judgeForce(Eigen::Vector3d &deltF){
        //Compenstate the drift
        for(int i = 0; i<3; i++){
            deltF[i] = abs(deltF[i])<FORCEMIN? 0:deltF[i];
            deltF[i] = abs(deltF[i])<FORCEMAX? deltF[i] : std::copysign(FORCEMAX,deltF[i]);
        }
    }

protected:
    double T_[16]={0};
    double q_[6]={0};
    void jointStateCallback (const sensor_msgs::JointState& js_msg){
    //Copy the message from topic.
        for(int i = 0; i<ROBOT_DOF; i++){
            q_[Indexjoint[i]] = js_msg.position[i];
            traj_msg.points[0].positions[i] = js_msg.position[i];
        }
        flag_position = true;
    }

    void forceStateCallback (const geometry_msgs::WrenchStamped& force_msg){
        //Initialize the tcp force.
        cur_force[0] = force_msg.wrench.force.x;
        cur_force[1] = force_msg.wrench.force.y;
        cur_force[2] = force_msg.wrench.force.z;
        //Torque
        //  static_torque = cur_torque;
        cur_torque[0] = force_msg.wrench.torque.x;
        cur_torque[1] = force_msg.wrench.torque.y;
        cur_torque[2] = force_msg.wrench.torque.z;
        //Ensure the cur_force is not zero vector.
        flag_force = true;
    }
    void forceInitial(){
        std::this_thread::sleep_for(std::chrono::seconds(2));
        static_force = compaForce;
        isMoving = true;
    }

    ros::Publisher arm_pub;
    ros::Publisher vel_pub;
    ros::Subscriber force_sub; 
    ros::Subscriber joint_state_sub;
private:
    /* data */
    const std::string JOINTS[ROBOT_DOF] = {"elbow_joint","shoulder_lift_joint",
                                            "shoulder_pan_joint",
                                            "wrist_1_joint",
                                            "wrist_2_joint",
                                            "wrist_3_joint"};
    const int Indexjoint[ROBOT_DOF] = {2, 1, 0, 3, 4, 5};

    bool flag_position ;
    bool flag_force ;
    bool isMoving ;
    bool hasSetDes;
    bool disableForce;

    Eigen::Vector3d cur_force;
    Eigen::Vector3d static_force;
    Eigen::Vector3d compaForce;
    Eigen::Vector3d cur_torque;
    Eigen::Vector3d static_torque;
    Eigen::Vector3d compaTorque;

    //Kalman filter parameters.
    Eigen::Vector3d dFk_1;
    Eigen::Vector3d dFk_2;
    Eigen::Vector3d dFk_3;
    Eigen::Vector3d dFk_4;
    Eigen::Matrix3d Pk,Qk,Rk;
    Eigen::Vector3d dForceP;
    Eigen::Vector3d dForcePost;
    // inverkinematic
    Eigen::Matrix4d Tw;
    Eigen::Matrix4d Tt;
    //forward
    Eigen::Matrix4d Trans_;
    Eigen::Matrix3d Rotation_;
    Eigen::Vector3d Translation_;
    Eigen::Matrix<double,ROBOT_DOF,ROBOT_DOF> Jaco_;

    trajectory_msgs::JointTrajectory traj_msg;
    std_msgs::Float64MultiArray vel_msg;
    Eigen::Vector3d gravity_;
    int dFflag;
    double dx_;
    double dy_;
    double dz_;
    double M_,B_,K_;
    Eigen::Vector3d xacc;
    Eigen::Vector3d xvel;
    Eigen::Vector3d xpose;
    bool setJacobian;
};


#endif // ROBOT_H