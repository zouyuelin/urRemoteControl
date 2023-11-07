#include "ros/ros.h"
// sensor messages
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
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
#include "follow_drag/ur_kine.h"
#include <follow_drag/Transformer.h>

#include <fstream>

using namespace std;
using namespace transform;

#define ROBOT_DOF            6
#define TIME_STEP         0.008
#define COEFFICIENT         0.03 //map the force to the velocity in TCP system.
#define COEFFICIENT_ANGLE   12.0  //it functions like pid with respect to Kp
#define GRAVITYACC          9.8
#define MASS               1.45 //the quality or mass of object. 1.66 kg
#define FORCEMIN           0.2
#define FORCEMAX           10.0
#define THRESHOLDVELOCITY  4.0
//Virtual boundaries
#define MINIMALDIS         0.010
#define MAXIMALDIS         0.020

#define SQUARE(x,y) sqrt(std::pow(x,2)+std::pow(y,2))
#define VECPRODT(x1,y1,x2,y2) (x1*x2+y1*y2)/(SQUARE(x1,y1)*SQUARE(x2,y2))

//Parameters
Eigen::VectorXd cur_pos(6);

int dFflag = 1;

//From point cloud
Eigen::VectorXd normAngle(8);
//Position of the end effector.
Eigen::Vector3d Pe(0.0, 0.00, 0.218);
//End-effector
double q_sols[8*6];
Eigen::Matrix4d Tw;
Eigen::Matrix4d Tt;


//flags
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur_autonomous");
  ros::NodeHandle nh;

  //Control the real UR robot arm.
  ros::Publisher arm_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/scaled_pos_joint_traj_controller/command", 10);
  ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, jointStateCallback);

  //parameter initiation.
  trajectory_msgs::JointTrajectory traj_msg;
    traj_msg.header.frame_id = "Next";
    traj_msg.joint_names.resize(6);
    traj_msg.points.resize(1);
    traj_msg.points[0].positions.resize(6);
  for(int i =0; i<ROBOT_DOF;i++) traj_msg.joint_names[i] = JOINTS[i];
  
  string strSettingPath="/home/rcclab/config/eyehand.yaml";
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    //Pe[0] = fSettings["dx"];
    //Pe[1] = fSettings["dy"];
    //Pe[2] = fSettings["dz"];


    //initialization
  double T[16]={0};
  double q[6]={0};
  cur_pos << 0, 0, 0, 0, 0, 0;

  Tw<< -1 , 0,  0, 0,
         0,-1,  0, 0,
         0, 0,  1, 0,
         0, 0,  0, 1;
  Tt<<   0, 0,  1, 0,
         -1, 0,  0, 0,
         0, -1,  0, 0,
         0, 0,  0, 1;

  ros::Rate loop_rate(1./TIME_STEP);
  //initialize the force by spin once.
  
  const string filename[5] = {"0","1","2","3","4"};
  std::ifstream newfile;
  
  std::vector<std::vector<double>> vec(5);
  std::string line;
  for(int i = 0; i<5; i++){
  	  newfile.open(string("/home/rcclab/config/ver2/")+filename[i]+ string(".txt"),ios::in);
  	  cout<<"read the file successfully"<<endl;
	  while(std::getline(newfile, line)){
		std::istringstream ss(line);
		double x;
		while (ss >> x) // Read next int from `ss` to `x`, stop if no more ints.
		vec[i].push_back(x); 
		}
	  newfile.close();
    }
  while (ros::ok())
  {
    //Wait for the data from topic.
    while( flag_position==0){
    ros::spinOnce();
    loop_rate.sleep();
    cout<<"Wait fo initiation..."<<endl;
    isMoving = true;
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
    
    static bool hasReach = false;

    //read data from txt and seperate it into 6 parts
    static long int pointerVec = 0;
    static int chapter = 0;
    if(hasReach){
    if(chapter%2==0){
    	if(pointerVec==vec[chapter].size()/6-1){
    		pointerVec = vec[chapter+1].size()/6-1;
    		chapter++;
    		goto escape;
    	}
        pointerVec++;
    }
        
     if(chapter%2==1){
        if(pointerVec == 0){
    		pointerVec = 0;
    		chapter++;
    		goto escape;
    	}
        pointerVec--;
     }
    }
    
    
    escape:
    cout<<"change"<<endl;
    
    if(chapter == 5) break;
        
	static Eigen::Matrix3d rotation;
	static bool hasInitiate = false;
	if(!hasInitiate){
		rotation = Trans.block(0,0,3,3);
		hasInitiate = true;
	}
	
	Eigen::Matrix4d Tours = Trans;
	Tours.block(0,0,3,3) = rotation; 
	
	Eigen::Vector3d endP(vec[chapter][pointerVec*6+0],vec[chapter][pointerVec*6+1]-0.020,vec[chapter][pointerVec*6+2]+0.037);
	Eigen::Vector3d startP=Trans.block(0,0,3,3)*Pe + Trans.col(3).head(3);
	Eigen::Vector3d direction = (endP-startP).normalized();
	Eigen::Vector3d nextP=direction*0.001+Trans.col(3).head(3);
	
	//----------------Calculate the rx ry rz. normB ah-----------------
	Eigen::Vector3d normB(vec[chapter][pointerVec*6+3],vec[chapter][pointerVec*6+4],vec[chapter][pointerVec*6+5]);
	Eigen::Vector3d ah = Trans.col(1).head(3);
        Eigen::Vector3d az = Trans.col(2).head(3);
        double qx = VECPRODT(normB[1],normB[2],az[1],az[2]);
        Eigen::Vector3d a1(0,az[1],az[2]);
        Eigen::Vector3d nor1(0,normB[1],normB[2]);
        if(qx<0){
            qx = -qx;
            nor1 = -nor1;
        }
        int signalN = (a1.cross(nor1))[0]>0? 1:-1;

        double rx = 0.3*signalN * abs(acos(qx)) ;

        double qy = VECPRODT(normB[0],normB[2],az[0],az[2]);
        Eigen::Vector3d a2(az[0],0,az[2]);
        Eigen::Vector3d nor2(normB[0],0,normB[2]);
        if(qy<0){
            qy = -qy;
            nor2 = -nor2;
        }
        signalN = (a2.cross(nor2))[1]>0? 1:-1;

//        cout<<(a2.cross(nor2)).transpose()<<endl;
        double ry = 0.4 * signalN * abs(acos(qy)) ;

        double qz = VECPRODT(0,-1,az[0],az[1]);
        Eigen::Vector3d a3(az[0],az[1],0);
        Eigen::Vector3d nor3(0,-1,0);
        if(qz<0){
            qz = -qz;
            nor3 = -nor3;
        }
        signalN = (a3.cross(nor3))[2]>0? 1:-1;

//        cout<<(a3.cross(nor3)).transpose()<<endl;
        double rz = signalN * abs(acos(qz)) ;
        
        double rxW = abs(rx)>0.15? rx/abs(rx)*1.0:rx/abs(rx)*0.16;
    	double ryW = abs(ry)>0.15? ry/abs(ry)*1.0:ry/abs(ry)*0.16;
    	double rzW = abs(rz)>0.12? rz/abs(rz)*0.1:rz*0.0;
    	
    	double Jr[6*ROBOT_DOF] = {0};
    	ur_Kinematics::jacobian(q,Jr,Pe[0],Pe[1],Pe[2]);
    	Eigen::Map<Eigen::Matrix<double,6,ROBOT_DOF,Eigen::RowMajor>> Jaco(Jr);

      	Eigen::VectorXd vel(6,1);
      	vel.head(3) << direction*0.08;
      	vel.tail(3) << rxW, ryW, 0;
      	
    	Eigen::MatrixXd Ji(6,6);
      	Ji = Jaco.transpose()*((Jaco*Jaco.transpose()).inverse());
    	//Joints's velocities.
    	Eigen::VectorXd velocityJoint = Ji*vel;
    	Eigen::VectorXd vStep = velocityJoint*TIME_STEP;
    	
    	cout<<"angle "<<rx<<" "<<rxW<<" "<<rz<<endl;
    	cout<<vStep.transpose()<<endl;
        //----------------------end calculation-------------------------
        
        
	Tours.col(3).head(3) << nextP[0],nextP[1],nextP[2];
	
	if((endP-startP).norm()<0.005) hasReach = true;
	else hasReach = false;
	
	Eigen::Matrix4d Transoff = Tw.inverse() * Tours * Tt.inverse();//Tw.inverse() * Tours * Tt.inverse();

        double T_[16];
        for(int i = 0; i<4; i++)
            for(int j = 0; j<4; j++)
                T_[i*4+j] =  Transoff(i,j);
        ur_Kinematics::inverseK(T_,q,q_sols,q[5],false);


    double speed = (Tours.col(3).head(3)-Trans.col(3).head(3)).norm() / TIME_STEP;
    cout<<"Current speed is "<<speed<<endl;
    if(isMoving){
      if(speed > 1.5){
        for(int i = 0; i<6;i++)
            q_sols[i] = q[i];
      }
    }
    
    if(isRunning && isMoving){
        //Move the robot!
        for(int i = 0; i<6;i++)
          if(abs(q_sols[Indexjoint[i]] - q[Indexjoint[i]]) > 1e-4)
                traj_msg.points[0].positions[i] += vStep[Indexjoint[i]] ; //+ vStep[Indexjoint[i]]
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

