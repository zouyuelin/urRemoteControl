#include "ros/ros.h"
#include "function/robot.h"
#include <iostream>
#include <fstream>

using namespace std;

Eigen::Vector3d Pe(0,0,0.180);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracking");
  ros::NodeHandle nh;

  robot ur_(nh,false);
  ur_.setGravity(0.180);
  ur_.setBias(Pe[0],Pe[1],Pe[2]);

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

    Eigen::Vector3d incrementF = (force-static_force);
    ur_.judgeForce(incrementF);
    // ur_.kalmanFilter(incrementF);

    ur_.impedenceControlXYZ(incrementF);
    Eigen::Vector3d xpose = ur_.getImpPose();
    

    Eigen::Matrix4d Tu = init_pos;
    Tu.col(3).head(3) = init_pos.col(3).head(3)+xpose;
    double q_sols[8*6];
    double *q;
    q = ur_.getJointsStatus();

    ur_.inverKinematic(Tu,q_sols);

    // Eigen::VectorXd vel(6,1);
    //     vel.head(3)<< incrementF*0.03;
    //     vel.tail(3)<< 0,0,0;
    // Eigen::VectorXd vStep = ur_.calculateStep(vel);
    
    printf("%.4f %.4f %.4f\n",incrementF[0],incrementF[1],incrementF[2]);
    //set destination
    // ur_.setDestinationDelta(vStep);
    ur_.setDestinationObsolute(q_sols);
    printf("%.4f %.4f %.4f %.4f %.4f %.4f\n",q[0]-q_sols[0],q[1]-q_sols[1],q[2]-q_sols[2],q[3]-q_sols[3],q[4]-q_sols[4],q[5]-q_sols[5]);
    //move robot
    ur_.stepForward();

    ros::spinOnce();
    loop_rate.sleep();

    //Finish Code to be timed.
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    printf("Elapsed time: %.5f  ms\n", elapsed.count());
  }
  return 0;
}
