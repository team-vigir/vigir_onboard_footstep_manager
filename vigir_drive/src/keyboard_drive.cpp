/*
 * teleop_pr2_keyboard_drive
 *
 * Based on teleop_pr2_keyboard
 * 
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <flor_ocs_msgs/OCSDrive.h>

#define KEYCODE_SPACE 0x20
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77 
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
#define KEYCODE_Q_CAP 0x51
#define KEYCODE_E_CAP 0x45

class KeyboardDrive
{
  private:
  double walk_vel, run_vel, yaw_rate, yaw_rate_run;
  flor_ocs_msgs::OCSDrive drive_cmd;

  ros::NodeHandle n_;
  ros::Publisher drive_pub_;

  public:
  void init()
  { 
    drive_cmd.throttle = drive_cmd.steer = 0;

    drive_pub_ = n_.advertise<flor_ocs_msgs::OCSDrive>("drive_cmd", 1);

    ros::NodeHandle n_private("~");

  }
  
  ~KeyboardDrive()   { }
  void keyboardLoop();

};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  puts("Done!");
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyboard_drive");

  KeyboardDrive tpk;
  tpk.init();

  signal(SIGINT,quit);

  tpk.keyboardLoop();

  return(0);
}

void KeyboardDrive::keyboardLoop()
{
  char c;
  bool dirty=false;
  int32_t tmp;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("---------------------------");
  puts("---------------------------");
  puts("Basic Driving Interface");
  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use 'w' to accelerate");
  puts("    's' to slow down");
  puts("    'a' to turn left (increase steering)");
  puts("    'd' to turn right (decrease steering)");
  puts("Shift for larger change");
  puts("Press 'Space' to halt");
  puts(    " (throttle, steer):");
  printf("\r (   % 4d , % 4d )", drive_cmd.throttle, drive_cmd.steer);
  fflush(stdout);
  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    switch(c)
    {
      // Walking
    case KEYCODE_W:
      if (drive_cmd.throttle < 255) ++drive_cmd.throttle;
      dirty = true;
      break;
    case KEYCODE_S:
      if (drive_cmd.throttle > 0) --drive_cmd.throttle;
      dirty = true;
      break;
    case KEYCODE_A:
      if (drive_cmd.steer < 127) ++drive_cmd.steer;
      dirty = true;
      break;
    case KEYCODE_D:
      if (drive_cmd.steer > -127) --drive_cmd.steer;
      dirty = true;
      break;
      // Running 
    case KEYCODE_W_CAP:
      tmp = drive_cmd.throttle + 10; // will stay < 127
      if (tmp > 255) 
        drive_cmd.throttle = 255;
      else 
        drive_cmd.throttle = (uint8_t)tmp;
      dirty = true;
      break;
    case KEYCODE_S_CAP:
      tmp = drive_cmd.throttle - 10;
      if (tmp < 0) 
        drive_cmd.throttle = 0;
      else
        drive_cmd.throttle = (int8_t)tmp;
      dirty = true;
      break;
    case KEYCODE_A_CAP:
      tmp = drive_cmd.steer + 10;
      if (tmp > 127) 
        drive_cmd.steer = 127;
      else 
        drive_cmd.steer = tmp;
      dirty = true;
      break;
    case KEYCODE_D_CAP:
      tmp = drive_cmd.steer - 10;
      if (tmp < -127) 
        drive_cmd.steer = -127;
      else 
        drive_cmd.steer = tmp;
      dirty = true;
      break;
    case KEYCODE_SPACE:
      drive_cmd.throttle = drive_cmd.steer = 0;
      dirty = true;
      break;
    }

    
    if (dirty == true)
    {
      printf("\r (   % 4d , % 4d )", drive_cmd.throttle, drive_cmd.steer);
      fflush(stdout);
      drive_pub_.publish(drive_cmd);
      dirty = false;
    }


  }
}
