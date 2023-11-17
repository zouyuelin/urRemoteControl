///////////////////////////////////////////////////////////////////////////////
//
//  Copyright (C) 2001-2022 Force Dimension, Switzerland.
//  All Rights Reserved.
//
//  Force Dimension SDK 3.15.0
//
///////////////////////////////////////////////////////////////////////////////


#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
using namespace Eigen;

#include "omega/dhdc.h"

// size of sphere
const double SphereRadius = 0.03;

// size of finger
const double FingerRadius = 0.005;

// status flags
bool SimulationOn;
bool SimulationFinished;

// device-specific globals
int      FingerCount;
Vector3d FingerPosGlobal[2];
bool     HasRot;
bool     HasGrip;

// position of sphere in global world coordinates
Vector3d SpherePosGlobal;

// text overlay globals
double LastTime;
double Freq;
char   Perf[50];
bool   ShowRate = true;

// object properties
const double Stiffness = 1000.0;

// exit callback
void
Close (void)
{
  // finish haptic loop
  SimulationOn = false;
  while (!SimulationFinished) dhdSleep (0.1);

  // close device
  dhdClose ();
}



// haptic thread
void*
HapticsLoop (void* pUserData)
{
  static double t0 = dhdGetTime ();
  double        t  = t0 + 0.001;

  int      i;
  Vector3d newFingerPosGlobal;
  Vector3d newFingerPosLocal;
  Vector3d forceGlobal[2];

  // start haptic simulation
  SimulationOn       = true;
  SimulationFinished = false;

  // start with no force
  forceGlobal[0].setZero ();
  forceGlobal[1].setZero ();

  // enable force
  dhdEnableForce (DHD_ON);

  // main haptic simulation loop
  while (SimulationOn) {

    t  = dhdGetTime ();
    t0 = t;

    double x, y, z;

    // adapt behavior to device capabilities
    if (HasGrip) {
      dhdGetGripperThumbPos (&x, &y, &z);
      FingerPosGlobal[0] << x, y, z;

      dhdGetGripperFingerPos (&x, &y, &z);
      FingerPosGlobal[1] << x, y, z;
    }
    else {
      dhdGetPosition (&x, &y, &z);
      FingerPosGlobal[0] << x, y, z;
    }

    // compute interaction between sphere and each finger
    for (i=0; i<FingerCount; i++) {

      // compute penetration
      Vector3d dir  = (FingerPosGlobal[i] - SpherePosGlobal).normalized();
      double   dist = (FingerPosGlobal[i] - SpherePosGlobal).norm() - SphereRadius - FingerRadius;

      // compute force
      if (dist < 0.0) forceGlobal[i] = -dist * Stiffness * dir;
      else            forceGlobal[i].setZero ();
    }

    // compute projected force on each gripper finger
    Vector3d force;
    double   gripperForce = 0.0;
    if (HasGrip) {

      // compute total force
      force = forceGlobal[0] + forceGlobal[1];
      Vector3d gripdir = FingerPosGlobal[1] - FingerPosGlobal[0];

      // if force is not null
      if (gripdir.norm() > 0.00001) {

        // project force on mobile gripper finger (forceGlobal[1]) onto gripper opening vector (gripdir)
        gripdir.normalize ();
        Vector3d gripper = (forceGlobal[1].dot (gripdir) / (gripdir.squaredNorm())) * gripdir;
        gripperForce = gripper.norm();

        // compute the direction of the force based on the angle between
        // the gripper force vector (gripper) and the gripper opening vector (gripdir)
        if (force.norm() > 0.001) {
          double cosangle = gripdir.dot (gripper) / (gripdir.norm()*gripper.norm());
          if      (cosangle >  1.0) cosangle =  1.0;
          else if (cosangle < -1.0) cosangle = -1.0;
          double angle = acos(cosangle);
          if ((angle > M_PI/2.0) || (angle < -M_PI/2.0)) gripperForce = -gripperForce;
        }
      }

      // invert if necessary for left-handed devices
      if (dhdIsLeftHanded()) gripperForce = -gripperForce;
    }
    else force = forceGlobal[0];

    // apply all forces at once
    dhdSetForceAndGripperForce (force(0), force(1), force(2), gripperForce);
  }

  // close connection with haptic device
  dhdClose ();

  // simulation is now exiting
  SimulationFinished = true;

  // return
  return NULL;
}

// haptic thread
void*
RunningNode (void* pUserData)
{
  static double t0 = dhdGetTime ();
  double        t  = t0 + 0.001;

  int      i;
  Vector3d newFingerPosGlobal;
  Vector3d newFingerPosLocal;
  Vector3d forceGlobal[2];

  // start haptic simulation
  SimulationOn       = true;
  SimulationFinished = false;

  // start with no force
  forceGlobal[0].setZero ();
  forceGlobal[1].setZero ();

  // enable force
  dhdEnableForce (DHD_ON);

  // main haptic simulation loop
  while (SimulationOn) {

    t  = dhdGetTime ();
    t0 = t;

    double x, y, z;

    // adapt behavior to device capabilities
    if (HasGrip) {
      dhdGetGripperThumbPos (&x, &y, &z);
      FingerPosGlobal[0] << x, y, z;

      dhdGetGripperFingerPos (&x, &y, &z);
      FingerPosGlobal[1] << x, y, z;
    }
    else {
      dhdGetPosition (&x, &y, &z);
      FingerPosGlobal[0] << x, y, z;
    }

    // compute interaction between sphere and each finger
    for (i=0; i<FingerCount; i++) {

      // compute penetration
      Vector3d dir  = (FingerPosGlobal[i] - SpherePosGlobal).normalized();
      double   dist = (FingerPosGlobal[i] - SpherePosGlobal).norm() - SphereRadius - FingerRadius;

      // compute force
      if (dist < 0.0) forceGlobal[i] = -dist * Stiffness * dir;
      else            forceGlobal[i].setZero ();
    }

    // compute projected force on each gripper finger
    Vector3d force;
    double   gripperForce = 0.0;
    if (HasGrip) {

      // compute total force
      force = forceGlobal[0] + forceGlobal[1];
      Vector3d gripdir = FingerPosGlobal[1] - FingerPosGlobal[0];

      // if force is not null
      if (gripdir.norm() > 0.00001) {

        // project force on mobile gripper finger (forceGlobal[1]) onto gripper opening vector (gripdir)
        gripdir.normalize ();
        Vector3d gripper = (forceGlobal[1].dot (gripdir) / (gripdir.squaredNorm())) * gripdir;
        gripperForce = gripper.norm();

        // compute the direction of the force based on the angle between
        // the gripper force vector (gripper) and the gripper opening vector (gripdir)
        if (force.norm() > 0.001) {
          double cosangle = gripdir.dot (gripper) / (gripdir.norm()*gripper.norm());
          if      (cosangle >  1.0) cosangle =  1.0;
          else if (cosangle < -1.0) cosangle = -1.0;
          double angle = acos(cosangle);
          if ((angle > M_PI/2.0) || (angle < -M_PI/2.0)) gripperForce = -gripperForce;
        }
      }

      // invert if necessary for left-handed devices
      if (dhdIsLeftHanded()) gripperForce = -gripperForce;
    }
    else force = forceGlobal[0];

    // apply all forces at once
    dhdSetForceAndGripperForce (force(0), force(1), force(2), gripperForce);
  }

  // close connection with haptic device
  dhdClose ();

  // simulation is now exiting
  SimulationFinished = true;

  // return
  return NULL;
}

// haptic devices initialization
int
InitHaptics ()
{
  if (dhdOpen () >= 0) {
    printf ("%s device detected\n", dhdGetSystemName());

    // set device capabilities
    FingerCount = 1;
    HasRot      = dhdHasWrist ();
    HasGrip     = dhdHasGripper ();

    if (HasGrip) FingerCount = 2;
    else         FingerCount = 1;
  }

  else {
    printf ("no device detected\n");
    dhdSleep (2.0);
    exit (0);
  }

  printf ("\n");

  // register exit callback
  atexit (Close);

  return 0;
}


int
main (int   argc,
      char *argv[])
{
  // message
  std::cout << "Force Dimension - OpenGL Sphere Example " << dhdGetSDKVersionStr() << std::endl;
  std::cout << "Copyright (C) 2001-2022 Force Dimension" << std::endl;
  std::cout << "All Rights Reserved." << std::endl << std::endl;

  // initialize haptic devices
  InitHaptics ();
  // create a high priority haptic thread
#if defined(WIN32) || defined(WIN64)
  DWORD ThreadId;
  CreateThread (NULL, 0, (LPTHREAD_START_ROUTINE)(HapticsLoop), NULL, NULL, &ThreadId);
  SetThreadPriority(&ThreadId, THREAD_PRIORITY_ABOVE_NORMAL);
#else
  pthread_t handle;
  pthread_create (&handle, NULL, HapticsLoop, NULL);
  struct sched_param sp;
  memset (&sp, 0, sizeof(struct sched_param));
  sp.sched_priority = 10;
  pthread_setschedparam (handle, SCHED_RR, &sp);
#endif

  // display instructions
  printf ("\n");
  printf ("commands:\n\n");
  printf ("   'r' to toggle display of haptic rate\n");
  printf ("   'q' to quit\n");
  printf ("\n\n");
  // exit
  return 0;
}
