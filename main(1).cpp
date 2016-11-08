#include "mbed.h"
#include "FXOS8700Q.h"
 
//I2C lines for FXOS8700Q accelerometer/magnetometer
FXOS8700Q_acc acc( PTE25, PTE24, FXOS8700CQ_SLAVE_ADDR1);
FXOS8700Q_mag mag( PTE25, PTE24, FXOS8700CQ_SLAVE_ADDR1);
 
//Temrinal enable 
Serial pc(USBTX, USBRX);
 
MotionSensorDataUnits mag_data;
MotionSensorDataUnits acc_data;
 

int main() 
{
    float faX, faY, faZ;
    float fmX, fmY, fmZ;
    float faX_prev=0,faY_prev=0,faZ_prev=0;
    float deltaY = 0;
    float deltaX = 0;
    float deltaZ = 0;
  
    acc.enable();
    printf("\r\n\nFXOS8700Q Who Am I= %X\r\n", acc.whoAmI());

    while (true) 
    {
        acc.getAxis(acc_data);
        mag.getAxis(mag_data);
        printf("FXOS8700Q ACC: X=%1.4f Y=%1.4f Z=%1.4f  ", acc_data.x, acc_data.y, acc_data.z);
        printf("    MAG: X=%4.1f Y=%4.1f Z=%4.1f\r\n", mag_data.x, mag_data.y, mag_data.z);
        acc.getX(&faX);
        deltaX = abs(faX-faX_prev);
        
        if(deltaX >=.300)
        {
            wait(5);
            printf(" \r\ndeltaX=%1.4f ",deltaX);
            }
        
        acc.getY(&faY);
        deltaY = faY-faY_prev;
        if(deltaY >=.300)
        {
            wait(5);
            printf(" \r\ndeltaY=%1.4f ",deltaY);
            }
        acc.getZ(&faZ);
        mag.getX(&fmX);
        mag.getY(&fmY);
        mag.getZ(&fmZ);
        printf("FXOS8700Q ACC: X=%1.4f Y=%1.4f Z=%1.4f ", faX, faY, faZ);
        printf("    MAG: X=%4.1f Y=%4.1f Z=%4.1f\r\n", fmX, fmY, fmZ);
        
        wait(1.0);
    }
}