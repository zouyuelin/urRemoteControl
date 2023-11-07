#include "ros/ros.h"
// sensor messages
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
// standard library
#include <iostream>
#include <chrono>
#include <vector>
#include <thread>
#include <mutex>
// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
// Kinematic solution
#include "function/ur_kine.h"
#include "function/Transformer.h"

using namespace std;
using namespace transform;

#define ROBOT_DOF            6
#define TIME_STEP         0.008
#define COEFFICIENT         0.03 //map the force to the velocity in TCP system.
#define COEFFICIENT_ANGLE   12.0  //it functions like pid with respect to Kp
#define GRAVITYACC          9.8
#define MASS               0.90 //the quality or mass of object. 1.66 kg
#define FORCEMIN           0.2
#define FORCEMAX           10.0
#define THRESHOLDVELOCITY  4.0
//Virtual boundaries
#define MINIMALDIS         0.010
#define MAXIMALDIS         0.020

//Parameters
Eigen::Vector3d gravity;
Eigen::VectorXd cur_pos(6);
Eigen::Vector3d cur_force,static_force,compaForce;
Eigen::Vector3d cur_torque,static_torque,compaTorque;

//Kalman filter parameters.
Eigen::Vector3d dFk_1;
Eigen::Vector3d dFk_2;
Eigen::Vector3d dFk_3;
Eigen::Vector3d dFk_4;
Eigen::Matrix3d Pk,Qk,Rk;
Eigen::Vector3d dForceP;
Eigen::Vector3d dForcePost;
int dFflag = 1;

//From point cloud
Eigen::VectorXd normAngle(8);
//Position of the end effector.
Eigen::Vector3d Pe(0.0, 0.030, 0.097);
//End-effector
double q_sols[8*6];
Eigen::Matrix4d Tw;
Eigen::Matrix4d Tt;


//flags
int flag_force = 0;
int flag_position = 0;
bool isRunning = true;
bool isMoving = false;
mutex forceMu;

//Define the joints' names.
const std::string JOINTS[ROBOT_DOF] = {"elbow_joint",
                                       "shoulder_lift_joint",
                                       "shoulder_pan_joint",
                                       "wrist_1_joint",
                                       "wrist_2_joint",
                                       "wrist_3_joint"};
//The corresponding indexs are 2 1 0 3 4 5;
const int Indexjoint[ROBOT_DOF] = {2, 1, 0, 3, 4, 5};

// Callback function
void jointStateCallback (const sensor_msgs::JointState& js_msg);
void forceStateCallback (const geometry_msgs::WrenchStamped& force_msg);
void normAngleCallback (const std_msgs::Float64MultiArray& normAngle_msg);
void forceInitial();
void judgeForce(Eigen::Vector3d &deltF);
void kalmanFilter(Eigen::Vector3d &data);
Eigen::Matrix4d impedenceControl(Eigen::Vector3d force,
                      Eigen::Matrix4d plannedPath,
                      Eigen::Matrix4d Trans, double *q,
                      Eigen::Vector3d surfNorm,
                      Eigen::Vector3d surfPoint,
                      double forceLimit = 3.0);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur_autonomous");
  ros::NodeHandle nh;

  //Control the real UR robot arm.
  ros::Publisher arm_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/scaled_pos_joint_traj_controller/command", 10);
  ros::Subscriber force_sub = nh.subscribe("/SRI_force_topic",1,forceStateCallback);
  ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, jointStateCallback);

  //parameter initiation.
  trajectory_msgs::JointTrajectory traj_msg;
    traj_msg.header.frame_id = "Next";
    traj_msg.joint_names.resize(6);
    traj_msg.points.resize(1);
    traj_msg.points[0].positions.resize(6);
  for(int i =0; i<ROBOT_DOF;i++) traj_msg.joint_names[i] = JOINTS[i];

    //initialization
  double T[16]={0};
  double q[6]={0};
  cur_pos << 0, 0, 0, 0, 0, 0;
  static_force << 0, 0, 0;
  static_torque << 0, 0, 0;
  normAngle << 0, 0, 0, 0, 0, 0, 0,0;
  //Noise distribution.
  Qk << 0.007,    0,    0,
           0, 0.007,    0,
           0,    0, 0.007;
  Rk <<  0.1,    0,   0,
           0, 0.1,    0,
           0,    0, 0.1;
  Pk << 0.005,    0,    0,
            0, 0.005,    0,
            0,    0, 0.005;

  Tw<< -1 , 0,  0, 0,
         0,-1,  0, 0,
         0, 0,  1, 0,
         0, 0,  0, 1;
  Tt<<   0, 0,  1, 0,
         -1, 0,  0, 0,
         0, -1,  0, 0,
         0, 0,  0, 1;

  //calculate the gravity.
  gravity<< 0, 0, -GRAVITYACC*MASS;
  //start another thread.
  thread initalForce(&forceInitial);
  initalForce.detach();

  ros::Rate loop_rate(1./TIME_STEP);
  //initialize the force by spin once.
  while (ros::ok())
  {
    //Wait for the data from topic.
    while(flag_force==0 || flag_position==0){
    ros::spinOnce();
    loop_rate.sleep();
    cout<<"Wait fo initiation..."<<endl;
    }

    //Count the processing time.
    auto start = std::chrono::high_resolution_clock::now();

    printf("-----------------------------step information-----------------------------\n");

    //Obtain the joint states from topic.
    for(int i=0; i<ROBOT_DOF;i++)
      q[Indexjoint[i]] = traj_msg.points[0].positions[i] = cur_pos[i];

    unique_lock<mutex> lc(forceMu);
    //kinematic forward
    ur_Kinematics::forward(q,T);

    Eigen::Map<Eigen::Matrix<double,4,4,Eigen::RowMajor>> Trans(T);
    Eigen::Vector3d gValue = Trans.block(0,0,3,3).inverse()* gravity; //
      cout<<"GValue is :"<<gValue.transpose()<<endl;
    //Gravity compensation.
    compaForce = cur_force - gValue;
//    printf("Position of joints is :\n %1.6f %1.6f %1.6f %1.6f %1.6f %1.6f \n",cur_pos[0],cur_pos[1],cur_pos[2],cur_pos[3],cur_pos[4],cur_pos[5]);
    printf("Current Tcp-force x y z is :%1.6f %1.6f %1.6f \n",cur_force[0],cur_force[1],cur_force[2]);
    printf("Compenstated Tcp-force x y z is :\n %1.6f %1.6f %1.6f \n",compaForce[0],compaForce[1],compaForce[2]);
//    printf("Current Tcp-Torque rx ry rz is :%1.6f %1.6f %1.6f \n",cur_torque[0],cur_torque[1],cur_torque[2]);
    printf("Static force is :\n %1.6f %1.6f %1.6f \n",static_force[0],static_force[1],static_force[2]);

    //get the jacobian of robot arm.
    double Jr[6*ROBOT_DOF] = {0};
    ur_Kinematics::jacobian(q,Jr,0,Pe[1],Pe[2]);
    Eigen::Map<Eigen::Matrix<double,6,ROBOT_DOF,Eigen::RowMajor>> Jaco(Jr);
//      cout<<"Jacobian is :"<<endl<<Jaco<<endl;

    //v = Jr* {\dot q}---->{\dot q} = Jr^+ v.
    //Observation--->deltaForce.
    Eigen::Vector3d deltaForce = compaForce-static_force;
    deltaForce = Trans.block(0,0,3,3) * deltaForce;
    //constrict the force.
    judgeForce(deltaForce);
    //Kalman filtering
    if(isMoving)
        kalmanFilter(deltaForce);
        printf("delta force is :\n %1.5f %1.5f %1.5f \n",deltaForce[0],deltaForce[1],deltaForce[2]);
    Eigen::Vector3d v = COEFFICIENT*deltaForce;
    lc.unlock();

    // vel = {v,w};
    // ------------------------translation according to the force--------------------------------
    Eigen::VectorXd vel(6,1);
      vel.head(3) << v;

      //-----------------------rotation according to the surface normal-------------------------------
      //set a threshold of angle error, default 0.04 rad.
    double rxW = abs(normAngle[3])>0.12? normAngle[3]/abs(normAngle[3])*1.44:normAngle[3]*COEFFICIENT_ANGLE;
    double ryW = abs(normAngle[4])>0.12? normAngle[4]/abs(normAngle[4])*1.44:normAngle[4]*COEFFICIENT_ANGLE;
    double rzW = abs(normAngle[5])>0.12? normAngle[5]/abs(normAngle[5])*1.44:normAngle[5]*COEFFICIENT_ANGLE;

    // if rx and ry exceed 0.03 rad.
      if(abs(normAngle[3]) > 0.005 || abs(normAngle[4]) > 0.005)
        vel.tail(3) << rxW, ryW, rzW;
      else{
        vel.head(3) << v ;
        vel.tail(3) << 0, 0, 0;
      }

      //------------------------------soft boundary: re-plan the trajectory of robot arm----------------.
      Eigen::Vector3d cur_pos = (Trans.block(0,0,3,3)*Pe+Trans.col(3).head(3));
      Eigen::Vector3d nextPoint = vel.head(3)*TIME_STEP + cur_pos;
      Eigen::Vector3d pOnsurf = normAngle.head(3);

    // ------------------------------------planned path setting------------------------------.
    Eigen::MatrixXd Ji(6,6);
      Ji = Jaco.transpose()*((Jaco*Jaco.transpose()).inverse());
    //Joints's velocities.
    Eigen::VectorXd velocityJoint = Ji*vel;
    Eigen::VectorXd vStep = velocityJoint*TIME_STEP;
    Eigen::VectorXd vStepAbsolute(6,1);
    for(int i = 0; i<6;i++){
        if(abs(vStep[Indexjoint[i]])>1e-4)
            vStepAbsolute[Indexjoint[i]] = traj_msg.points[0].positions[i] + vStep[Indexjoint[i]];
        else
            vStepAbsolute[Indexjoint[i]] = traj_msg.points[0].positions[i];
    }
    double q_plan[ROBOT_DOF];
    double T_plan[16];
    for(int i=0; i<ROBOT_DOF;i++)
        q_plan[i] = vStepAbsolute[i];

    ur_Kinematics::forward(q_plan,T_plan);
    Eigen::Map<Eigen::Matrix<double,4,4,Eigen::RowMajor>> planPath_(T_plan);
    Eigen::Matrix4d nextPoseForMove;
    nextPoseForMove = impedenceControl(deltaForce, planPath_, Trans, q, pOnsurf-cur_pos,pOnsurf, 2.0);

    double speed = (nextPoseForMove.col(3).head(3)-Trans.col(3).head(3)).norm() / TIME_STEP;
    cout<<"Current speed is "<<speed<<endl;
    if(isMoving){
        //if velocity exceeds the 1.5m/s
      if(speed > 1.5){
        for(int i = 0; i<6;i++)
            q_sols[i] = q[i];
      }
    }

    printf("Joint velocity is :\n %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f \n",velocityJoint[0],
            velocityJoint[1],velocityJoint[2],velocityJoint[3],velocityJoint[4],velocityJoint[5]);
    printf("delta q is :\n %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f \n",q_sols[0] - q[0],
            q_sols[1] - q[1],q_sols[2] - q[2],q_sols[3] - q[3],q_sols[4] - q[4],q_sols[5] - q[5]);
    printf("vStep is :\n %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f \n",vStep[0],
            vStep[1],vStep[2],vStep[3],vStep[4],vStep[5]);

    if(isRunning && isMoving){
        //Move the robot!
        for(int i = 0; i<6;i++)
          if(abs(q_sols[Indexjoint[i]] - q[Indexjoint[i]]) > 1e-4)
                traj_msg.points[0].positions[i] = q_sols[Indexjoint[i]];
        traj_msg.header.stamp = ros::Time::now();
        traj_msg.points[0].time_from_start = ros::Duration(0.01);
        //Publish the trajectory message.
        arm_pub.publish(traj_msg);
    }

    //Interval time is 1000/125 mm.
    ros::spinOnce();
    loop_rate.sleep();

    //Finish Code to be timed.
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    printf("Elapsed time: %.5f  ms\n", elapsed.count());
  }
  return 0;
}

void jointStateCallback (const sensor_msgs::JointState& js_msg){
  //Copy the message from topic.
    for(int i = 0; i<ROBOT_DOF; i++)
      cur_pos[i] = js_msg.position[i];
    flag_position = 1;
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
  flag_force = 1;
}

void forceInitial(){
    this_thread::sleep_for(chrono::seconds(2));
    unique_lock<mutex> lc(forceMu);
    static_force = compaForce;
    isMoving = true;
}

void judgeForce(Eigen::Vector3d &deltF){
  //Compenstate the drift
  for(int i = 0; i<3; i++){
      deltF[i] = abs(deltF[i])<FORCEMIN? 0:deltF[i];
      deltF[i] = abs(deltF[i])<FORCEMAX? deltF[i]:deltF[i]/abs(deltF[i])*FORCEMAX;
  }
}

void kalmanFilter(Eigen::Vector3d &data){
    //--------Start: kalman filter---
    //Step 1: prediction of the increased force.
    dForceP = dFk_1 + (dFk_1 - dFk_4)/3.0;
    Pk = Pk + Qk;
    //Step 2: optimization of the force.
    Eigen::Matrix3d Kk = Pk*(Pk + Rk).inverse(); //kalman gain
    Pk = (Eigen::Matrix3d::Identity() - Kk)*Pk;
    dForcePost = dForceP + Kk*(data - dForceP);
        printf("Optimized delta force is :\n %1.5f %1.5f %1.5f \n",dForcePost[0],dForcePost[1],dForcePost[2]);
    //----------End: Kalman filter---
    data = dForcePost;

    //Update the former force records.
    //Prepare for kalman filter.
    if((dFflag) == 1){
        dFk_1 = dForceP;
        dFflag++;
    }
    else if(dFflag == 2){
        dFk_1 = dForceP;
        dFk_2 = dFk_1;
        dFflag++;
    }
    else if(dFflag == 3){
        dFk_1 = dForceP;
        dFk_2 = dFk_1;
        dFk_3 = dFk_2;
        dFflag++;
    }
    else if(dFflag == 4){
        dFk_1 = dForcePost;
        dFk_2 = dFk_1;
        dFk_3 = dFk_2;
        dFk_4 = dFk_3;
    }
}


Eigen::Matrix4d impedenceControl(Eigen::Vector3d force,
                      Eigen::Matrix4d plannedPath,
                      Eigen::Matrix4d Trans, double *q,
                      Eigen::Vector3d surfNorm,
                      Eigen::Vector3d surfPoint,
                      double forceLimit){
    static Eigen::Vector3d xacc(0.0,0.0,0.0),xvel(0.0,0.0,0.0),xpose(0.0,0.0,0.0);
    double M,B,K;
    M = 20.0; K = 600; B = 109.5;
    Eigen::Vector3d exterForce(0,0,0);

    static Eigen::Vector3d initPos;
    static Eigen::Matrix3d initOrien;
    static double curDistance = 0.0;
    static bool hasSetPos = false;
    if(!hasSetPos && isMoving) {
        initPos = Trans.col(3).head(3);
        initOrien = Trans.block(0,0,3,3);
        curDistance = surfNorm.norm();
        hasSetPos = true;
    }

    // set the normal vector's direction
    surfNorm = Trans.col(1).head(3);

    // limit the force within 3F
    if(abs(force.norm()) > 1e-3){
        if(force.norm()>2) force = 2*force/force.norm();
        else force = (4 - force.norm()) * force/force.norm();
    }

    double surfNormforce;
    surfNormforce = surfNorm.transpose() * force;
    surfNormforce = surfNormforce/surfNorm.norm();

    Eigen::Vector3d forceSurf = surfNormforce*surfNorm/surfNorm.norm();

    cout<<"force on surface is" << surfNormforce <<endl;

    // refined path
    Eigen::Matrix4d Tours = Eigen::Matrix4d::Identity();
    if(isMoving){

        // impedence control steps.
        xacc = 1.0/M*(forceSurf+exterForce - B*xvel - K*xpose);
        xvel = xacc*TIME_STEP + xvel;
        xpose = xpose + xvel*TIME_STEP;

        //reset the planned path.
//        if(abs(surfNormforce)<forceLimit) plannedPath.col(3).head(3) = initPos;

        Eigen::Vector3d next_pos = (plannedPath.block(0,0,3,3)*Pe+plannedPath.col(3).head(3));
        Eigen::Vector3d nextNorm = next_pos - surfPoint;
        Eigen::Vector3d refinedPos = nextNorm/nextNorm.norm()*curDistance + surfPoint;

        initPos = plannedPath.col(3).head(3) + (refinedPos - next_pos);

        // ultimate path.
        Eigen::Vector3d nextPos = initPos + xpose;//+ xpose


            Tours(0,3) = nextPos[0];
            Tours(1,3) = nextPos[1];
            Tours(2,3) = nextPos[2];
        Tours.block(0,0,3,3) =  plannedPath.block(0,0,3,3);
        // Tours = Tw*Toff*Tt, Toff= Tw^-1*Tours*Tt^-1
        // kinematic forward to match the official frame.
        Eigen::Matrix4d Transoff = Tw.inverse() * Tours * Tt.inverse();//Tw.inverse() * Tours * Tt.inverse();

        double T_[16];
        for(int i = 0; i<4; i++)
            for(int j = 0; j<4; j++)
                T_[i*4+j] =  Transoff(i,j);
        ur_Kinematics::inverseK(T_,q,q_sols,q[5],false);
    }
    else{
        for(int i=0; i<ROBOT_DOF;i++)
            q_sols[i] = q[i];
    }

    return Tours;
}
