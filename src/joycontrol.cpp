#include "ros/ros.h"
#include "function/robot.h"
#include <iostream>
#include <fstream>

using namespace std;

Eigen::Vector3d Pe(0,0,0.180);
bool flag_position = false;
Eigen::VectorXd cur_pos(6);

void jointStateCallback (const sensor_msgs::JointState& js_msg){
  //Copy the message from topic.
  for(int i = 0; i<6; i++)
    cur_pos[i] = js_msg.position[i];
    flag_position = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joycontrol");
  ros::NodeHandle nh;

  robot ur_(nh,true);
  ur_.setGravity(0.180);
  ur_.setBias(Pe[0],Pe[1],Pe[2]);

  ros::Subscriber arm_sub = nh.subscribe("/joystick",1,jointStateCallback);

  ros::Rate loop_rate(1./TIME_STEP);

  while (ros::ok())
  {
    while(!ur_.isOk() && flag_position){
      ros::spinOnce();
      loop_rate.sleep();
      cout<<"Wait fo initiation..."<<endl;
    }
    //Count the processing time.
    auto start = std::chrono::high_resolution_clock::now();

    printf("-----------------------------step information-----------------------------\n");

    //Obtain the joint states from topic.
    ur_.forward();
    auto Jaco  = ur_.jacobian();

    Eigen::VectorXd messageJoy(6,1);
    messageJoy<<cur_pos;

    Eigen::VectorXd vel(6,1);
        vel.head(3)<< messageJoy.head(3)*0.2;
        vel.tail(3)<< messageJoy.tail(3)*0.01;
    Eigen::VectorXd vStep = ur_.calculateStep(vel);
    
    // printf("%.4f %.4f %.4f\n",incrementF[0],incrementF[1],incrementF[2]);
    //set destination
    ur_.setDestinationDelta(vStep);

    printf("%.4f %.4f %.4f %.4f %.4f %.4f\n",vStep[0],vStep[1],vStep[2],vStep[3],vStep[4],vStep[5]);
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
