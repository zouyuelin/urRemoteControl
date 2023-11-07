#include "ros/ros.h"
#include "function/robot.h"
#include <iostream>
#include <fstream>

#define SQUARE(x,y) sqrt(std::pow(x,2)+std::pow(y,2))
#define VECPRODT(x1,y1,x2,y2) (x1*x2+y1*y2)/(SQUARE(x1,y1)*SQUARE(x2,y2))

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

  const string filename[5] = {"0","1","2","3","4"};
  std::ifstream newfile;
  
  std::vector<std::vector<double>> vec(5);
  std::string line;
  for(int i = 0; i<5; i++){
  	  newfile.open(string("/home/yuelinzou/ver2/")+filename[i]+ string(".txt"),ios::in);
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
    std::cout<<ur_.getTransformation()<<std::endl;

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
		rotation = ur_.getRotation();
		hasInitiate = true;
	}
	
	Eigen::Matrix4d Tours = ur_.getTransformation();
	Tours.block(0,0,3,3) = rotation; 
	
	Eigen::Vector3d endP(vec[chapter][pointerVec*6+0],vec[chapter][pointerVec*6+1]-0.1,vec[chapter][pointerVec*6+2]+0.100);
	Eigen::Vector3d startP=ur_.getRotation()*Pe + ur_.getTranslation();
	Eigen::Vector3d direction = (endP-startP).normalized();
	Eigen::Vector3d nextP=direction*0.0018+ur_.getTranslation();
	
	//----------------Calculate the rx ry rz. normB ah-----------------
	Eigen::Vector3d normB(vec[chapter][pointerVec*6+3],vec[chapter][pointerVec*6+4],vec[chapter][pointerVec*6+5]);
	Eigen::Vector3d ah = ur_.getTransformation().col(1).head(3);
        Eigen::Vector3d az = ur_.getTransformation().col(2).head(3);
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
    	
      auto Jaco  = ur_.jacobian();

      	Eigen::VectorXd vel(6,1);
      	vel.head(3) << direction*0.12;
      	vel.tail(3) << rxW, ryW, 0;
      	
      Eigen::VectorXd vStep = ur_.calculateStep(vel);
    	
    	cout<<"angle "<<rx<<" "<<rxW<<" "<<rz<<endl;
    	cout<<vStep.transpose()<<endl;
        //----------------------end calculation-------------------------
        
        
	  Tours.col(3).head(3) << nextP[0],nextP[1],nextP[2];
	
	  if((endP-startP).norm()<0.005) hasReach = true;
	  else hasReach = false;

    double q_sols[8*6];
	  ur_.inverKinematic(Tours, q_sols);

    double speed = (Tours.col(3).head(3)-ur_.getTranslation()).norm() / TIME_STEP;
    cout<<"Current speed is "<<speed<<endl;

    double* q;
    q = ur_.getJointsStatus();

    // for(int i = 0; i<6;i++)
    //         cout<<q_sols[i]<<" "<<q[i]<<endl;

    ur_.setDestinationDelta(vStep);
    // ur_.setDestinationObsolute(q_sols);
    ur_.stepForward();

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
