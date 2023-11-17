#include "omega/HapticDevice.h"

// haptic device API
#include "omega/dhdc.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "eigen3/Eigen/Eigen"
#include <boost/bind.hpp>

#define REFRESH_INTERVAL  0.1   // sec

#define KP    100.0
#define KVP    10.0
#define MAXF    4.0
#define KR      0.3
#define KWR     0.02
#define MAXT    0.1
#define KG    100.0
#define KVG     5.0
#define MAXG    1.0

using namespace Eigen;

HapticDevice::HapticDevice(ros::NodeHandle & node, float loop_rate, bool set_force): loop_rate_(loop_rate)
{
    nh_ = node;

    dev_id_ = -2; // we set a value not in API defined range
    device_enabled_ = -1;

    set_force_ = set_force;

    for (int i = 0; i<7;i++) {
        velocity_[i] = position_[i] = 0.0;
    }
    for (int i = 0; i<DHD_MAX_DOF;i++) {
        v[i] = p[i] = 0.0;
    }

    button0_state_ = false;
    keep_alive_ = false;
    force_released_ = true;

    force_.resize(3);
    force_[0] = 0.0;
    force_[1] = 0.0;
    force_[2] = 0.0;

    SetForceLimit(3.0, 3.0, 3.0);

    // connect to hardware device
    device_count_ = dhdGetDeviceCount();

    // we only accept one haptic device.
    if ( device_count_ >= 1) {
        dev_id_ = dhdOpenID(0); // if open failed, we will get -1, and sucess with 0.
        if ( dev_id_ < 0) {
            ROS_INFO("error: handler device: %s\n", dhdErrorGetLastStr());
            device_enabled_ = false;
            return;
        }
    } else {
        ROS_INFO("No handler device find! %s\n", dhdErrorGetLastStr());
        device_enabled_ = false;
        return;
    }

    device_enabled_ =true;
}


HapticDevice::~HapticDevice()
{
    dev_id_ = -1;
    device_count_ = 0;
    keep_alive_ = false;
    if (dev_op_thread_)
    {
        dev_op_thread_->join();
    }
}

void HapticDevice::PublishHapticData()
{
    // geometry_msgs::Vector3Stamped pos;
    geometry_msgs::TwistStamped pos;
    pos.header.frame_id = ros::this_node::getName();
    pos.header.stamp = ros::Time::now();
    pos.twist.linear.x = position_[0];
    pos.twist.linear.y = position_[1];
    pos.twist.linear.z = position_[2];
    pos.twist.angular.x = position_[3];
    pos.twist.angular.y = position_[4];
    pos.twist.angular.z = position_[5];

    // geometry_msgs::Vector3Stamped vec;
    geometry_msgs::TwistStamped vec;
    vec.header.frame_id = ros::this_node::getName();
    vec.header.stamp = ros::Time::now();
    vec.twist.linear.x = velocity_[0];
    vec.twist.linear.y = velocity_[1];
    vec.twist.linear.z = velocity_[2];
    vec.twist.angular.x = velocity_[3];
    vec.twist.angular.y = velocity_[4];
    vec.twist.angular.z = velocity_[5];

    std_msgs::Float64MultiArray button_stat;
    button_stat.data.push_back(button0_state_);

    position_pub_.publish(pos);
    velocity_pub_.publish(vec);
    button_state_pub_.publish(button_stat);
}

void HapticDevice::RegisterCallback()
{
    position_topic_ = "/haptic/position";
    buttons_topic_ = "/haptic/button_state";
    force_topic_ = "/haptic/force";
    velocity_topic_ = "/haptic/velocity";

    position_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(position_topic_.c_str(),1);
    velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(velocity_topic_.c_str(),1);
    button_state_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(buttons_topic_.c_str(), 1);
    force_sub_ = nh_.subscribe<geometry_msgs::Vector3>(force_topic_.c_str(),1, &HapticDevice::ForceCallback,this);
}

void HapticDevice::ForceCallback(const geometry_msgs::Vector3::ConstPtr &data)
{
    // wrapper force
    SetForce(data->x, data->y, data->z);
}

void HapticDevice::GetHapticDataRun()
{   // get and we will publish immediately

    if (drdOpen () < 0) {
        printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr ());
        dhdSleep (2.0);
      }

      // print out device identifier
      if (!drdIsSupported ()) {
        printf ("unsupported device\n");
        printf ("exiting...\n");
        dhdSleep (2.0);
        drdClose ();
      }
      printf ("%s haptic device detected\n\n", dhdGetSystemName ());

      // perform auto-initialization
      if (!drdIsInitialized () && drdAutoInit () < 0) {
        printf ("error: auto-initialization failed (%s)\n", dhdErrorGetLastStr ());
        dhdSleep (2.0);
      }
      else if (drdStart () < 0) {
        printf ("error: regulation thread failed to start (%s)\n", dhdErrorGetLastStr ());
        dhdSleep (2.0);
      }
      double nullPose[DHD_MAX_DOF] = { 0.0, 0.0, 0.0,  // translation
                                       0.0, 0.0, 0.0,  // rotation (joint angles)
                                       0.0 };          // gripper
      //move to center
      drdMoveTo (nullPose);
      drdSetForceAndTorqueAndGripperForce (0.0, 0.0, 0.0,  // force
                                             0.0, 0.0, 0.0,  // torque
                                             0.0);           // gripper force
      drdRegulatePos  (false);
      drdRegulateRot  (false);
      drdRegulateGrip (false);

      dhdEnableForce (DHD_ON);

    double feed_force[3] = {0.0, 0.0, 0.0};

    ros::spinOnce();

    while (ros::ok() && (keep_alive_ == true)) {

        if (device_count_ >= 1 && dev_id_ >= 0) {
//                dhdGetPosition(&current_position[0], &current_position[1], &current_position[2]);
                drdWaitForTick ();
                drdGetPositionAndOrientation (&(p[0]), &(p[1]), &(p[2]),
                                              &(p[3]), &(p[4]), &(p[5]),
                                              &(p[6]), r);
                // dhdGetGripperAngleDeg(angle);
                // get position/orientation/gripper velocity
                drdGetVelocity (&(v[0]), &(v[1]), &(v[2]),
                                &(v[3]), &(v[4]), &(v[5]),
                                &(v[6]));
                position_[0] = p[0];
                position_[1] = p[1];
                position_[2] = p[2];
                position_[3] = p[3];
                position_[4] = p[4];
                position_[5] = p[5];
                position_[6] = p[6];

                velocity_[0] = v[0];
                velocity_[1] = v[1];
                velocity_[2] = v[2];
                velocity_[3] = v[3];
                velocity_[4] = v[4];
                velocity_[5] = v[5];
                velocity_[6] = v[6];
                button0_state_ = p[6];
                ROS_INFO("position is: %f ,%f ,%f\n %f ,%f ,%f, %f", position_[0], position_[1], position_[2],position_[3],p[4],p[5],p[6]);
        }

        PublishHapticData();

        // apply force
        if (set_force_) {
//            val_lock_.lock();
            feed_force[0] = force_[0];
            feed_force[1] = force_[1];
            feed_force[2] = force_[2];
//            dhdSetForce(feed_force[0], feed_force[1], feed_force[2]);
            drdSetForceAndTorqueAndGripperForce (feed_force[0], feed_force[1], feed_force[2],  // force
                                                   0.0, 0.0, 0.0,  // torque
                                                   0.0);           // gripper force
            ROS_INFO("Force is: %f ,%f ,%f", feed_force[0], feed_force[1], feed_force[2]);
//            val_lock_.unlock();
        }


        ros::spinOnce();
        loop_rate_.sleep();
    }

}

void HapticDevice::SetForce(double x, double y, double z)
{
    double input_force[3] = {0.0, 0.0, 0.0};

    if (set_force_)
    {
        val_lock_.lock();
        input_force[0] = x;
        input_force[1] = y;
        input_force[2] = z;
        VerifyForceLimit(input_force, force_);
        force_released_ = false;
        val_lock_.unlock();
    }
}

void HapticDevice::SetForceLimit(double x, double y, double z)
{
    force_x_limit_ = x;
    force_y_limit_ = y;
    force_z_limit_ = z;
}


void HapticDevice::VerifyForceLimit(double input_force[], std::vector<double> & output)
{

    if (output.size() != 3) {
        output.resize(3);
    }

    output[0] = input_force[0];
    output[1] = input_force[1];
    output[2] = input_force[2];

    if (input_force[0] < -force_x_limit_) output[0] = -force_x_limit_;
    if (input_force[1] < -force_y_limit_) output[1] = -force_y_limit_;
    if (input_force[2] < -force_z_limit_) output[2] = -force_z_limit_;

    if (input_force[0] > force_x_limit_) output[0] = force_x_limit_;
    if (input_force[1] > force_y_limit_) output[1] = force_y_limit_;
    if (input_force[2] > force_z_limit_) output[2] = force_z_limit_;
}


void HapticDevice::Start()
{   
    if (!device_enabled_)
    {
        return; 
    }
    

    RegisterCallback();
    ros::AsyncSpinner spinner(2);
    spinner.start();

    dev_op_thread_ = std::make_shared<boost::thread>(boost::bind(&HapticDevice::GetHapticDataRun, this));

//    gravityCompensa_thread_ = std::make_shared<boost::thread>(boost::bind(&HapticDevice::gravityCompensation, this));

    keep_alive_ = true;

    while (ros::ok() && (keep_alive_ == true)) {
        ros::Duration(0.001).sleep();
        // ROS_INFO("working in main loop");
        //  usleep(1000);
    }

    keep_alive_ = false;
    spinner.stop();
}
