#include "function/robot.h"

robot::robot(ros::NodeHandle &_nh,bool disableforce):flag_position(false),flag_force(false),
    isMoving(false),hasSetDes(false),dFflag(1),M_(20.0),K_(600), B_(109.5),dx_(0),dy_(0),dz_(0),setJacobian(false),disableForce(disableforce){

	traj_msg.header.frame_id = "Next";
    traj_msg.joint_names.resize(6);
    traj_msg.points.resize(1);
    traj_msg.points[0].positions.resize(6);
  	for(int i =0; i<ROBOT_DOF;i++) traj_msg.joint_names[i] = JOINTS[i];

    vel_msg.data={0,0,0,0,0,0.0};
    vel_msg.layout.data_offset=1;

    static_force << 0, 0, 0;
    static_torque << 0, 0, 0;

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
    //initiate the parameters of impedence control
    xacc << 0, 0, 0;
    xvel << 0, 0, 0;
    xpose << 0, 0, 0;

	arm_pub = _nh.advertise<trajectory_msgs::JointTrajectory>("/scaled_pos_joint_traj_controller/command", 10);
    vel_pub = _nh.advertise<std_msgs::Float64MultiArray>("/joint_group_vel_controller/command",10);
    if(!disableForce)
  	    force_sub = _nh.subscribe("/SRI_force_topic",1,&robot::forceStateCallback,this);
  	joint_state_sub = _nh.subscribe("/joint_states", 1, &robot::jointStateCallback,this);
	ros::spinOnce();
	// std::thread initalForce(&robot::forceInitial,this);
  	// initalForce.detach();
	isMoving = true;
}
void robot::setDestinationDelta(double* vStep){
	for(int i = 0; i<ROBOT_DOF;i++)
            traj_msg.points[0].positions[i] += vStep[Indexjoint[i]];
	hasSetDes = true;
}

void robot::setDestinationDelta(Eigen::VectorXd vStep){
	for(int i = 0; i<ROBOT_DOF;i++)
            traj_msg.points[0].positions[i] += vStep[Indexjoint[i]];
	hasSetDes = true;
}

void robot::setDestinationObsolute(double* tra){
	for(int i = 0; i<ROBOT_DOF;i++)
            traj_msg.points[0].positions[i] = tra[Indexjoint[i]];
	hasSetDes = true;
}

bool robot::stepForward(){//MOVETHRESHOLD->min setp for running
		if(!hasSetDes) return false;
        double stepMax = 0;
        for(int i = 0; i<ROBOT_DOF;i++){
                auto stepMax_t = abs(traj_msg.points[0].positions[i] - q_[Indexjoint[i]]);
                stepMax = std::max(stepMax_t,stepMax);
        }

        hasSetDes = false;
        setJacobian = false;

        if(stepMax>MOVETHRESHOLD && isMoving){
                traj_msg.header.stamp = ros::Time::now();
                traj_msg.points[0].time_from_start = ros::Duration(0.01);
                arm_pub.publish(traj_msg);
                return true;
        }
        else {
                return false;
        }   
}

void robot::setVelocity(double* vel){
    if(!vel_msg.data.empty()) vel_msg.data.clear();
    for(int i = 0; i<ROBOT_DOF; i++)
        vel_msg.data.push_back(vel[i]);
}

void robot::setVelocity(Eigen::VectorXd  vel){
    if(!vel_msg.data.empty()) vel_msg.data.clear();
    for(int i = 0; i<ROBOT_DOF; i++)
        vel_msg.data.push_back(vel[i]);
}

bool robot::moveForward(){//MOVETHRESHOLD->min setp for running
        setJacobian = false;
        if( isMoving){
                vel_pub.publish(vel_msg);
                return true;
        }
        else {
                return false;
        }   
}

Eigen::Matrix4d robot::forward(){
	ur_Kinematics::forward(q_,T_);
	Eigen::Map<Eigen::Matrix<double,4,4,Eigen::RowMajor>> Trans(T_);
	Trans_ = Trans;
	Rotation_ = Trans.block(0,0,3,3);
	Translation_ = Trans.col(3).head(3);
	return Trans;
}

Eigen::Matrix<double,ROBOT_DOF,ROBOT_DOF> robot::jacobian(){
        double Jr[ROBOT_DOF*ROBOT_DOF] = {0};
        ur_Kinematics::jacobian(q_,Jr,dx_,dy_,dz_);
        Eigen::Map<Eigen::Matrix<double,ROBOT_DOF,ROBOT_DOF,Eigen::RowMajor>> Jaco(Jr);
		Jaco_ = Jaco;
        setJacobian = true;
        return Jaco;
}

Eigen::VectorXd robot::calculateStep(Eigen::VectorXd vel){
    //if it has not been set, run the jacobian function
    if(!setJacobian) jacobian();
    Eigen::MatrixXd Ji(6,6);
    Ji = Jaco_.transpose()*((Jaco_*Jaco_.transpose()).inverse());
    Eigen::VectorXd velocityJoint = Ji*vel;
    Eigen::VectorXd vStep = velocityJoint*TIME_STEP;
    return vStep;
}

Eigen::VectorXd robot::calculateJointSpeed(Eigen::VectorXd vel){
    //if it has not been set, run the jacobian function
    if(!setJacobian) jacobian();
    Eigen::MatrixXd Ji(6,6);
    Ji = Jaco_.transpose()*((Jaco_*Jaco_.transpose()).inverse());
    Eigen::VectorXd velocityJoint = Ji*vel;
    return velocityJoint;
}

void robot::inverKinematic(Eigen::Matrix4d Tu,double *q_sols){
	Eigen::Matrix4d Transoff = Tw.inverse() * Tu * Tt.inverse();//Tw.inverse() * Tours * Tt.inverse();
	double T[16];
	for(int i = 0; i<4; i++)
		for(int j = 0; j<4; j++)
			T[i*4+j] =  Transoff(i,j);
	ur_Kinematics::inverseK(T,q_,q_sols,q_[5],false); //q_[5]
}

void robot::kalmanFilter(Eigen::Vector3d &data){
    //--------Start: kalman filter---
    //Step 1: prediction of the increased force.
    dForceP = dFk_1 + (dFk_1 - dFk_4)/3.0;
    Pk = Pk + Qk;
    //Step 2: optimization of the force.
    Eigen::Matrix3d Kk = Pk*(Pk + Rk).inverse(); //kalman gain
    Pk = (Eigen::Matrix3d::Identity() - Kk)*Pk;
    dForcePost = dForceP + Kk*(data - dForceP);
    //printf("Optimized delta force is :\n %1.5f %1.5f %1.5f \n",dForcePost[0],dForcePost[1],dForcePost[2]);
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

void robot::impedenceControlXYZ(Eigen::Vector3d exterForce){
    xacc = 1.0/M_*(exterForce - B_*xvel - K_*xpose);
    xvel = xacc*TIME_STEP + xvel;
    xpose = xpose + xvel*TIME_STEP;
}
void robot::impedenceControlTorque(Eigen::Vector3d exterTorque){
    //TODO: define the process of impedence control for torque.
}