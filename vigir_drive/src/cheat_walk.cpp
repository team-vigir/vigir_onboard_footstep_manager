///=================================================================================================
// Copyright (c) 2013, David Conner, TORC Robotics
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/subscriber.h>

#include <vigir_ocs_msg2/OCSDrive.h>

using namespace message_filters;

class CheatWalker
{

protected:
    vigir_ocs_msg2::OCSDrive drive_cmd;
    ros::Publisher&          twist_pub;

public:

    CheatWalker(ros::Publisher& _tp) : twist_pub(_tp) { };

    void storeDriveCmd(const vigir_ocs_msg2::OCSDrive& cmd)
    {
        drive_cmd = cmd;
        ROS_INFO(" Recv'd: (% 4d, % 4d)\n", drive_cmd.throttle, drive_cmd.steer);
    };

    void pubTimerCallback(const ros::TimerEvent&)
    {

        geometry_msgs::Twist twist;
        //http://answers.ros.org/question/53930/roscpp-warning-in-rospublisherpublish/
        twist.linear.x  =  1.5*((double)(drive_cmd.throttle)/255.0); //  m/s forward speed
        twist.angular.z = (M_PI*((double)(drive_cmd.steer)/127.0));  // rad/s turning rate
        twist_pub.publish(twist); // publish the velocity twist
        ROS_INFO(" publish: (%g, %g)\n", twist.linear.x, twist.angular.z);
    };


};

int main(int argc, char** argv)
{

    ros::init(argc,argv,"cheat_drive");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // Advertise our topics
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/atlas/cmd_vel", 200);

    // Create class to handle conversions
    CheatWalker walker(twist_pub);

    // Subscribe to tele-op command and register callback
    //message_filters::Subscriber<vigir_ocs_msg2::OCSDrive> driver_cmd(nh,"drive_cmd",1);
    //driver_cmd.registerCallback(&CheatDriver::storeDriveCmd, &driver);
    ros::Subscriber sub = nh.subscribe("drive_cmd", 1, &CheatWalker::storeDriveCmd, &walker);

    // Publish updates at regular rate
    ros::Timer pub_timer = pnh.createTimer(ros::Duration(0.1), &CheatWalker::pubTimerCallback, &walker, false);

    // OK - do it!
    ros::spin();
  return(0);
}

