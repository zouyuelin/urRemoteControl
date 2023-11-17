#include "ros/ros.h"
#include "function/robot.h"
#include <iostream>
#include <fstream>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "std_msgs/Float64MultiArray.h"

using namespace std;

Eigen::Vector3d Pe(0,0,0.180);

geometry_msgs::TwistStamped omeInitPosition;
geometry_msgs::TwistStamped omePosition;
geometry_msgs::TwistStamped velocity_;
geometry_msgs::Vector3 forceOmega;
Eigen::Vector3d previousImpose;

double angle(0);

void OmegaPositionCallback (const geometry_msgs::TwistStamped position){
    omePosition = position;
}

void OmegaVelocityCallback (const geometry_msgs::TwistStamped vel){
    velocity_ = vel;
}

void OmegaAngluarCallback (const std_msgs::Float64MultiArray angle_){
    angle = angle_.data[0];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracking");
  ros::NodeHandle nh;

  ros::Publisher force_pub = nh.advertise<geometry_msgs::Vector3>("/haptic/force", 10);
  ros::Subscriber omegp_sub = nh.subscribe("/haptic/position",1,OmegaPositionCallback);
  ros::Subscriber omegv_sub = nh.subscribe("/haptic/velocity",1,OmegaVelocityCallback); //"/haptic/button_state";
  ros::Subscriber omegAngle_sub = nh.subscribe("/haptic/button_state",1,OmegaAngluarCallback);

  robot ur_(nh,false);
  ur_.setGravity(0.180);
  ur_.setBias(Pe[0],Pe[1],Pe[2]);
  // ur_.setMBK(20,500,89.44);

  ros::Rate loop_rate(1./TIME_STEP);

  while (ros::ok())
  {
    while(!ur_.isOk()){
      ros::spinOnce();
      loop_rate.sleep();
      cout<<"Wait fo initiation..."<<endl;
    }
    //Count the processing time.
    auto start = std::chrono::high_resolution_clock::now();

    printf("-----------------------------step information-----------------------------\n");

    //Obtain the joint states from topic.
    ur_.forward();
    // auto Jaco  = ur_.jacobian();

    Eigen::Vector3d f_ = ur_.getForce();
    Eigen::Vector3d force = ur_.mapFromHandToWorld(f_) + ur_.getGravity();

    static bool setForce = false;
    static Eigen::Vector3d static_force;
    static Eigen::Matrix4d init_pos;
    if(!setForce){
        static_force=force;
        setForce = true;
        init_pos = ur_.getTransformation();
    }

    static Eigen::Vector3d initpose_omega;
    static bool setOmegaPos = false;
    if(!setOmegaPos){
      initpose_omega[0] = omePosition.twist.linear.x;
      initpose_omega[1] = omePosition.twist.linear.y;
      initpose_omega[2] = omePosition.twist.linear.z;
      setOmegaPos = true;
    }
    Eigen::Vector3d currentpose_omega;
    currentpose_omega[0] = omePosition.twist.linear.x;
    currentpose_omega[1] = omePosition.twist.linear.y;
    currentpose_omega[2] = omePosition.twist.linear.z;

    Eigen::Vector3d incrementF = (force-static_force);
    ur_.judgeForce(incrementF);
    // ur_.kalmanFilter(incrementF);

    Eigen::Vector3d exterForce(0,0,0);

    ur_.impedenceControlXYZ(incrementF);
    Eigen::Vector3d xpose = ur_.getImpPose();

    forceOmega.x = 120*xpose[0];
    forceOmega.y = 120*xpose[1];
    forceOmega.z = 120*xpose[2];
    force_pub.publish(forceOmega);
    cout<<forceOmega.x<<" "<<forceOmega.y<<" "<<forceOmega.z<<endl;
    cout<<incrementF.transpose()<<endl;
    

    Eigen::VectorXd vel(6,1);
    vel.head(3)<< velocity_.twist.linear.x *0.06,velocity_.twist.linear.y*0.06,velocity_.twist.linear.z *0.06;
    // vel.tail(3)<< 0,0,0;
    // Eigen::VectorXd velocity = ur_.calculateJointSpeed(vel);

    Eigen::Matrix4d Tu = ur_.getTransformation();
    previousImpose = xpose;
    Tu.col(3).head(3) = ur_.getTranslation()+vel.head(3)+xpose;
    double q_sols[8*6];
    double *q;
    q = ur_.getJointsStatus();

    ur_.inverKinematic(Tu,q_sols);

    // tip rotation rx ry rz 
    Eigen::VectorXd twistgy(6,1);
    twistgy.head(3)<<0,0,0;
    twistgy.tail(3)<<velocity_.twist.angular.x*0.05,velocity_.twist.angular.y*0.05,velocity_.twist.angular.z*0.05;
    ur_.jacobian();
    Eigen::VectorXd velIncrement = ur_.calculateJointSpeed(twistgy);

    // printf("%.4f %.4f %.4f\n",velocity[0],velocity[1],velocity[2]);
    ur_.setDestinationObsolute(q_sols);
    ur_.setDestinationDelta(velIncrement);
    printf("%.4f %.4f %.4f %.4f %.4f %.4f\n",q[0]-q_sols[0],q[1]-q_sols[1],q[2]-q_sols[2],q[3]-q_sols[3],q[4]-q_sols[4],q[5]-q_sols[5]);
    cout<<velIncrement.transpose()<<endl;
    // //move robot
    if(angle>0.015) ur_.stepForward();

    ros::spinOnce();
    loop_rate.sleep();

    //Finish Code to be timed.
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    printf("Elapsed time: %.5f  ms\n", elapsed.count());
  }
  return 0;
}
