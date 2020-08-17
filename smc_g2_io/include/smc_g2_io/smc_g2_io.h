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

#ifndef SMC_G2_IO_H
#define SMC_G2_IO_H

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <termios.h>
#include <smc_g2_io/protocol.h>

class SimpleMotorControllerG2
{
private:
  int fd_;  // file descriptor for smc
public:
  SimpleMotorControllerG2();
  SimpleMotorControllerG2(const char * device, uint32_t baud_rate);
  ~SimpleMotorControllerG2();
  int openSerialPort(const char * device, uint32_t baud_rate);
  void closePort();
  bool hasFileDescriptor();
  int writePort(const uint8_t * buffer, size_t size);
  ssize_t readPort(uint8_t * buffer, size_t size);
  int getValue(uint8_t variable_id, uint16_t * value);
  
  int getTargetSpeed(int16_t * value);
  int getErrorStatus(uint16_t * value);
  int exitSafeStart();
  int setTargetSpeed(int speed);
};

#endif  // SMC_G2_IO_H