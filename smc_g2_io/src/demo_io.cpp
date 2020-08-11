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
    printf("Error status: 0x%04x\n", error_status);
    
    int16_t target_speed;
    result = my_smc.getTargetSpeed(&target_speed);
    if (result) { return 1; }
    printf("Target speed is %d.\n", target_speed);
    
    int16_t new_speed = (target_speed <= 0) ? 3200 : 0;
    printf("Setting target speed to %d.\n", new_speed);
    result = my_smc.setTargetSpeed(new_speed);
    if (result) { return 1; }
    
    my_smc.closePort();
    return 0;
}