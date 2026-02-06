/**
 * robot-nav.cpp is where you should put navigation routines.
 */

#include "robot.h"
#include "chassis.h"

void Robot::UpdatePose(const Twist& twist)
{
    /**
     * TODO: Add your FK algorithm to update currPose here.
     */
    currPose.theta = twist.omega;
    currPose.x = (currPose.x +(0.2*(twist.u)*(cos(twist.omega))));
    currPose.y = (currPose.y +(0.2*(twist.u)*(sin(twist.omega))));


#ifdef __NAV_DEBUG__
    TeleplotPrint("x", currPose.x);
    TeleplotPrint("y", currPose.y);
    TeleplotPrint("theta", currPose.theta);
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
    if (currPose.x > (destPose.x - 3) && currPose.x < (destPose.x + 3)){
        if (currPose.y > (destPose.y - 3) && currPose.y< (destPose.y + 3)){
            retVal = true;
        }
    }

    return retVal;
}

void Robot::DriveToPoint(void)
{
    if(robotState == ROBOT_DRIVE_TO_POINT)
    {
        float distError;
        float headingError;
        distError = sqrt(square(destPose.x - currPose.x) + square(destPose.y - currPose.y));
        headingError = atan2((destPose.y - currPose.y), (destPose.x - currPose.x)) - currPose.theta;
        if (headingError >= PI){
            headingError -= (2*PI);
        }
        else if(headingError < -PI){
            headingError += (2*PI);
        }

#ifdef __NAV_DEBUG__
        // Print useful stuff here.
#endif

        /**
         * TODO: Call chassis.SetMotorEfforts() to command the motion, based on your calculations above.
         */
        float kp_distance = 0.5;
        float kp_theta = 0.5;
        float left_effort = (kp_distance * distError) + (kp_theta * headingError);
        float right_effort = (kp_distance * distError) + (kp_theta * headingError);
        if (headingError > 0){
            chassis.SetMotorEfforts(left_effort, -right_effort);
        }else if(headingError < 0){
            chassis.SetMotorEfforts(-left_effort, right_effort);
        }

        
    }
}

void Robot::HandleDestination(void)
{
    /**
     * TODO: Stop and change state. Turn off LED.
     */
}