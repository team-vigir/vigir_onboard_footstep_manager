/*********************************************************************
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Christian Rose
 *                      Team Hector,
 *                      Technische Universität Darmstadt
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Technische Universität Darmstadt nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <vigir_atlas_sim_ros_control/atlas_sim_ros_control.h>

#include <controller_manager/controller_manager.h>
#include <std_msgs/String.h>

namespace Atlas_Sim_Ros_Control
{

  void Atlas_Sim_Ros_Control::jointStatesCb(const sensor_msgs::JointState::ConstPtr &_js)
  {
    // We don´t know how our joint state looks like till we received the first instance of it, so we initialize the hardware interface here
    if (joint_state_.name.size() == 0){
      joint_state_ = *_js;


      for (size_t i = 0; i < joint_state_.name.size(); ++i){
        hardware_interface::JointStateHandle state_handle(joint_state_.name[i], &joint_state_.position[i], &joint_state_.velocity[i], &joint_state_.velocity[i]);
        joint_state_interface_.registerHandle(state_handle);


        hardware_interface::JointHandle pos_handle(joint_state_interface_.getHandle(joint_state_.name[i]), &jointcommands.position[i]);
        position_joint_interface_.registerHandle(pos_handle);

        jointcommands.position[i] = joint_state_.position[i];

      }

      registerInterface(&joint_state_interface_);
      registerInterface(&position_joint_interface_);
    }else{

      joint_state_ = *_js;

    }

  }


Atlas_Sim_Ros_Control::Atlas_Sim_Ros_Control()
{
  ros::NodeHandle* rosnode = new ros::NodeHandle();

  rosnode->setCallbackQueue(&subscriber_queue_);

  ros::Time last_ros_time_;
  bool wait = true;
  while (wait)
  {
    last_ros_time_ = ros::Time::now();
    if (last_ros_time_.toSec() > 0)
      wait = false;
  }

  // must match those inside AtlasPlugin
  jointcommands.name.push_back("atlas::back_lbz");
  jointcommands.name.push_back("atlas::back_mby");
  jointcommands.name.push_back("atlas::back_ubx");
  jointcommands.name.push_back("atlas::neck_ay");
  jointcommands.name.push_back("atlas::l_leg_uhz");
  jointcommands.name.push_back("atlas::l_leg_mhx");
  jointcommands.name.push_back("atlas::l_leg_lhy");
  jointcommands.name.push_back("atlas::l_leg_kny");
  jointcommands.name.push_back("atlas::l_leg_uay");
  jointcommands.name.push_back("atlas::l_leg_lax");
  jointcommands.name.push_back("atlas::r_leg_uhz");
  jointcommands.name.push_back("atlas::r_leg_mhx");
  jointcommands.name.push_back("atlas::r_leg_lhy");
  jointcommands.name.push_back("atlas::r_leg_kny");
  jointcommands.name.push_back("atlas::r_leg_uay");
  jointcommands.name.push_back("atlas::r_leg_lax");
  jointcommands.name.push_back("atlas::l_arm_usy");
  jointcommands.name.push_back("atlas::l_arm_shx");
  jointcommands.name.push_back("atlas::l_arm_ely");
  jointcommands.name.push_back("atlas::l_arm_elx");
  jointcommands.name.push_back("atlas::l_arm_uwy");
  jointcommands.name.push_back("atlas::l_arm_mwx");
  jointcommands.name.push_back("atlas::r_arm_usy");
  jointcommands.name.push_back("atlas::r_arm_shx");
  jointcommands.name.push_back("atlas::r_arm_ely");
  jointcommands.name.push_back("atlas::r_arm_elx");
  jointcommands.name.push_back("atlas::r_arm_uwy");
  jointcommands.name.push_back("atlas::r_arm_mwx");

  unsigned int n = jointcommands.name.size();
  jointcommands.position.resize(n);
  jointcommands.velocity.resize(n);
  jointcommands.effort.resize(n);
  jointcommands.kp_position.resize(n);
  jointcommands.ki_position.resize(n);
  jointcommands.kd_position.resize(n);
  jointcommands.kp_velocity.resize(n);
  jointcommands.i_effort_min.resize(n);
  jointcommands.i_effort_max.resize(n);

  for (unsigned int i = 0; i < n; i++)
  {
    std::vector<std::string> pieces;
    boost::split(pieces, jointcommands.name[i], boost::is_any_of(":"));

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/p",
      jointcommands.kp_position[i]);

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i",
      jointcommands.ki_position[i]);

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/d",
      jointcommands.kd_position[i]);

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp",
      jointcommands.i_effort_min[i]);
    jointcommands.i_effort_min[i] = -jointcommands.i_effort_min[i];

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp",
      jointcommands.i_effort_max[i]);

    jointcommands.velocity[i]     = 0;
    jointcommands.effort[i]       = 0;
    jointcommands.kp_velocity[i]  = 0;
  }

  // ros topic subscribtions
  ros::SubscribeOptions jointStatesSo =
    ros::SubscribeOptions::create<sensor_msgs::JointState>(
    "/atlas/joint_states", 1,boost::bind(&Atlas_Sim_Ros_Control::jointStatesCb, this, _1),
    ros::VoidPtr(), rosnode->getCallbackQueue());

  // Because TCP causes bursty communication with high jitter,
  // declare a preference on UDP connections for receiving
  // joint states, which we want to get at a high rate.
  // Note that we'll still accept TCP connections for this topic
  // (e.g., from rospy nodes, which don't support UDP);
  // we just prefer UDP.
  jointStatesSo.transport_hints = ros::TransportHints().unreliable();

  subJointStates_ = rosnode->subscribe(jointStatesSo);
  // ros::Subscriber subJointStates =
  //   rosnode->subscribe("/atlas/joint_states", 1000, SetJointStates);

  pub_joint_commands_ =
    rosnode->advertise<osrf_msgs::JointCommands>(
    "/atlas/joint_commands", 1, true);

  subscriber_spinner_.reset(new ros::AsyncSpinner(1, &subscriber_queue_));
  subscriber_spinner_->start();

}

void Atlas_Sim_Ros_Control::cleanup()
{
    subscriber_spinner_->stop();
}

void Atlas_Sim_Ros_Control::read(ros::Time time, ros::Duration period)
{
  //Normally we would read joint angles from hardware here, but we get those via ROS topic as we use Gazebo
}

void Atlas_Sim_Ros_Control::write(ros::Time time, ros::Duration period)
{
  jointcommands.header.stamp = time;
  pub_joint_commands_.publish(jointcommands);
}

}

int main(int argc, char** argv){

try{
        ROS_INFO("starting");
        ros::init(argc, argv, "atlas_sim_ros_control");

        Atlas_Sim_Ros_Control::Atlas_Sim_Ros_Control atlas_sim_ros_control_interface;

        sleep(2);

        // Publish Atlas user mode
        ros::NodeHandle nh;

        ros::Publisher pub_user_mode_ = nh.advertise<std_msgs::String>("/atlas/control_mode",1,true);
        std_msgs::String msg;
        std::stringstream ss;
        ss << "User";
        msg.data = ss.str();
        pub_user_mode_.publish(msg);

        controller_manager::ControllerManager cm(&atlas_sim_ros_control_interface);

        ros::AsyncSpinner spinner(4);
        spinner.start();

        ros::Rate loop_rate(50);

        ros::Time last_time = ros::Time::now();

        while (ros::ok())
                {
                    //ROS_INFO("in main loop");
                    loop_rate.sleep();
                    ros::spinOnce();

                    ros::Time current_time = ros::Time::now();
                    ros::Duration elapsed_time = current_time - last_time;
                    last_time = current_time;

                    //ROS_INFO("before read");
                    atlas_sim_ros_control_interface.read(current_time, elapsed_time);
                    //ROS_INFO("after read");

                    //ROS_INFO("before cm.update");
                    cm.update(current_time, elapsed_time);
                    //ROS_INFO("after cm.update");

                    //ROS_INFO("before write");
                    atlas_sim_ros_control_interface.write(current_time, elapsed_time);
                    //ROS_INFO("after write");
                }

        atlas_sim_ros_control_interface.cleanup();
    }
    catch(...)
    {
        ROS_ERROR("Unhandled exception!");
        return -1;
    }

    return 0;
}
