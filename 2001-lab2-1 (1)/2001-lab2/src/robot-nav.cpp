/**
 * robot-nav.cpp is where you should put navigation routines.
 */

#include "robot.h"
#include "math.h"
#include "chassis.h"

void Robot::UpdatePose(const Twist& twist)
{
    
    /** 
     * TODO: Add your FK algorithm to update currPose here.
     */
    double prevtheta = currPose.theta;
    currPose.theta += twist.omega * 0.02;
    double thetastar = (currPose.theta + prevtheta)/2.0;
    currPose.x += ((0.02) * (twist.u)*(cos(thetastar)));
    currPose.y += ((0.02) * (twist.u)*(sin(thetastar)));
       
        
    

#ifdef __NAV_DEBUG__
    TeleplotPrint("x", currPose.x);
    TeleplotPrint("y", currPose.y);
    TeleplotPrint("theta", currPose.theta * RAD_TO_DEG );
#endif

}

/**
 * Sets a destination in the lab frame.
 */
void Robot::SetDestination(const Pose& dest)
{
    /**
     * TODO: Turn on LED, as well.
     */
    Serial.print("Setting dest to: ");
    Serial.print(dest.x);
    Serial.print(", ");
    Serial.print(dest.y);
    Serial.print('\n');

    destPose = dest;
    robotState = ROBOT_DRIVE_TO_POINT;
}

bool Robot::CheckReachedDestination(void)
{
    bool retVal = false;
    /**
     * TODO: Add code to check if you've reached destination here.
     */

    return retVal;
}

void Robot::DriveToPoint(void)
{

    if(robotState == ROBOT_DRIVE_TO_POINT)
    {
        /**
         * TODO: Add your IK algorithm here. 
         */

#ifdef __NAV_DEBUG__
        // Print useful stuff here.
#endif

        /**
         * TODO: Call chassis.SetMotorEfforts() to command the motion, based on your calculations above.
         */
    }
}

void Robot::HandleDestination(void)
{
    /**
     * TODO: Stop and change state. Turn off LED.
     */
}