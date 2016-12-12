#include "mbed.h"
#include "FXOS8700Q.h"
//#include "mbed_rpc.h"
#include "SerialRPCInterface.h"

 
//I2C lines for FXOS8700Q accelerometer/magnetometer
FXOS8700Q_acc acc( PTE25, PTE24, FXOS8700CQ_SLAVE_ADDR1);
FXOS8700Q_mag mag( PTE25, PTE24, FXOS8700CQ_SLAVE_ADDR1);
 
//Temrinal enable 
SerialRPCInterface SerialInterface (USBTX, USBRX);

//define Funtion for RPC
void getdeltaY(char * input,char * output);
void getdeltaX(char * input,char * output);

//RPCVariable<float> rpcdeltaY(&getdeltaY, "deltaY");
//RPCVariable<float> rpcdeltaX(&getdeltaX, "deltaX");
 
MotionSensorDataUnits mag_data;
MotionSensorDataUnits acc_data;

float deltaY = 0;
float deltaX = 0;
float deltaZ = 0;

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
        printf("FXOS8700Q ACC: X=%1.4f Y=%1.4f Z=%1.4f  ", acc_data.x, acc_data.y, acc_data.z);//printing out X,Y,Z Axis of Accel.
        printf("    MAG: X=%4.1f Y=%4.1f Z=%4.1f\r\n", mag_data.x, mag_data.y, mag_data.z);//printing out X,Y,Z Axis of Magn.
        acc.getX(&faX);
        deltaX = abs(faX-faX_prev);// absolute value of delta X 
        
        if(deltaX >=.300)// if the difference is greater thn .3 in positive or negative 
        {
            wait(5);// 5 second delay 
            printf(" \r\ndeltaX=%1.4f\n ",deltaX);// print value 
            }
        
        acc.getY(&faY);
        deltaY = abs(faY-faY_prev);// absolute value of delta X 
        
        
        if(deltaY >=.300)// if the difference is greater thn .3 in positive or negative 
        {
            wait(5);//5 second delay 
            printf("\r\ndeltaY=%1.4f\n ",deltaY);// print value
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
    
void getdeltaY(char * input, char * output){
    
}
void getdeltaX(char * input, char * output){
    
}

