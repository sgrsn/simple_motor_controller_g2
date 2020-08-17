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

#include <smc_g2_operator/smc_g2_operator.h>

SMCOperator::SMCOperator() : node_handle_("")
{
}

SMCOperator::~SMCOperator()
{
}

void SMCOperator::smcVelocityCallback(const std_msgs::Int32::ConstPtr &cmd)
{
  ROS_INFO("Setting target speed to %d.", cmd->data);
  int result = smc_.setTargetSpeed(cmd->data);
  if (result)
  {
    ROS_ERROR("failed to set new target speed.");
  }
}

void SMCOperator::smcBrakeCallback(const std_msgs::Int8::ConstPtr &cmd)
{
  ROS_INFO("Setting target brake to %d.", cmd->data);
  int result = smc_.setMotorBrake(cmd->data);
  if (result)
  {
    ROS_ERROR("failed to set new target brake.");
  }
}

bool SMCOperator::exitSafeStartMsgCallback(std_srvs::Trigger::Request &req,
                              std_srvs::Trigger::Response &res)
{
  int result = smc_.exitSafeStart();
  if (result)
  {
    ROS_ERROR("failed to exit safe start");
    res.success = false;
    res.message = "Failed to exit safe start";
    return true;
  }
  ROS_INFO("Exit from safe start.");
  res.success = true;
  res.message = "Success to exit safe start";
  return true;
}

void SMCOperator::initSubscriber()
{
  smc_vel_sub_ = node_handle_.subscribe<std_msgs::Int32>("smc_vel", 10, &SMCOperator::smcVelocityCallback, this);
  smc_brake_sub_ = node_handle_.subscribe<std_msgs::Int8>("smc_brake", 10, &SMCOperator::smcBrakeCallback, this);
}

void SMCOperator::initServiceServer()
{
  exit_safe_start_service_ = node_handle_.advertiseService("exit_safe_start", &SMCOperator::exitSafeStartMsgCallback, this);
}

int SMCOperator::initSMC(const char * device, uint32_t baud_rate)
{
  smc_ = SimpleMotorControllerG2(device, baud_rate);
  if(!smc_.hasFileDescriptor())
  {
    ROS_ERROR("failed to open the device %s", device);
    return 1;
  }

  int result = smc_.exitSafeStart();
  if (result)
  {
    ROS_ERROR("failed to exit safe start");
    return 1;
  }
  ROS_INFO("Exit from safe start.");

  uint16_t error_status;
  result = smc_.getErrorStatus(&error_status);
  if (result)
  {
    ROS_ERROR("Please check the smc g2 user guide for more information.");
    ROS_ERROR("Error status: 0x%04x", error_status);
    return 1;
  }
  ROS_INFO("Error status: 0x%04x", error_status);
  
  return 0;
}

void SMCOperator::closeSMCPort()
{
  if(smc_.hasFileDescriptor())
  {
    ROS_INFO("Close the SMC port.");
    smc_.closePort();
  }
}