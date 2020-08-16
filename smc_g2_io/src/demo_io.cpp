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

#include "ros/ros.h"
#include "smc_g2_io/smc_g2_io.h"

int main()
{
    const char * device = "/dev/ttyACM0";
    uint32_t baud_rate = 9600;

    HighPowerG2 my_smc(device, baud_rate);
 
    if(!my_smc.hasFileDescriptor()){ return 1; }
 
    int result = my_smc.exitSafeStart();
    if (result) { return 1; }
 
    uint16_t error_status;
    result = my_smc.getErrorStatus(&error_status);
    if (result) { return 1; }
    ROS_INFO("Error status: 0x%04x", error_status);
    
    int16_t target_speed;
    result = my_smc.getTargetSpeed(&target_speed);
    if (result) { return 1; }
    ROS_INFO("Target speed is %d.", target_speed);
    
    int16_t new_speed = (target_speed <= 0) ? 3200 : 0;
    ROS_INFO("Setting target speed to %d.", new_speed);
    result = my_smc.setTargetSpeed(new_speed);
    if (result) { return 1; }

    ROS_INFO("Waiting 500 ms");
    usleep(500000);

    ROS_INFO("Setting target speed to 0.");
    result = my_smc.setTargetSpeed(0);
    if (result) { return 1; }
    
    my_smc.closePort();
    return 0;
}