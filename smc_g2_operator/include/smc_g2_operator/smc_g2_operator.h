/*******************************************************************************
Copyright (c) 2020, Hidaka Sato
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met: 

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies, 
either expressed or implied, of the FreeBSD Project.
*******************************************************************************/

#ifndef SMC_G2_OPERATOR_H
#define SMC_G2_OPERATOR_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Trigger.h>
#include <smc_g2_io/smc_g2_io.h>

class SMCOperator
{
private:
  SimpleMotorControllerG2 smc_;
  ros::NodeHandle node_handle_;
  ros::Subscriber smc_vel_sub_;
  ros::Subscriber smc_brake_sub_;
  ros::ServiceServer exit_safe_start_service_;
public:
  SMCOperator();
  ~SMCOperator();
  void smcVelocityCallback(const std_msgs::Int32::ConstPtr &cmd);
  void smcBrakeCallback(const std_msgs::Int8::ConstPtr &cmd);
  bool exitSafeStartMsgCallback(std_srvs::Trigger::Request &req,
                               std_srvs::Trigger::Response &res);
  void initSubscriber();
  void initServiceServer();
  int initSMC(const char * device = "/dev/ttyACM0", uint32_t baud_rate = 9600);
  void closeSMCPort();
};

#endif  // SMC_G2_OPERATOR_H