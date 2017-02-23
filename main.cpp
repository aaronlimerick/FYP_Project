#include "MBed_Adafruit_GPS.h"
#include "FXOS8700Q.h"
#include "mbed.h"
 
Serial * gps_Serial;
Serial pc(USBTX,USBRX);
 //I2C lines for FXOS8700Q accelerometer/magnetometer
FXOS8700Q_acc acc( PTE25, PTE24, FXOS8700CQ_SLAVE_ADDR1);
FXOS8700Q_mag mag( PTE25, PTE24, FXOS8700CQ_SLAVE_ADDR1);

MotionSensorDataUnits mag_data;
MotionSensorDataUnits acc_data;

void GPS_read (Adafruit_GPS myGPS,Timer refresh_Timer, const int refresh_Time);

// Analog read pin 
AnalogIn senseLine(A0);
 
// Below are the calibrated loads and reading for the Load Cell in Grams
// 
float aReading = 14.90;
float aLoad =  20;// measured in grams
float bReading = 17.50;
float bLoad = 100; // measured in grams


 
int main() { 

    pc.baud( 9600); //sets COM serial communication to high rate; 
    gps_Serial = new Serial(PTC17,PTC16); //serial ports used on FRDM-K64F
    Adafruit_GPS myGPS(gps_Serial); //object of Adafruit's GPS class
    char c; // character stored here
    Timer refresh_Timer; //sets up a timer for use in loop; how often do we print GPS info?
    const int refresh_Time = 2000; //refresh time in ms
    
    myGPS.begin(9600);  //sets baud rate for GPS communication
                        
    
    myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //these commands are defined in MBed_Adafruit_GPS.h; 
    myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    myGPS.sendCommand(PGCMD_ANTENNA);
    
    printf("Connection established at  9600 baud...\n");
    wait(1);
    refresh_Timer.start();  //starts the clock on the timer    

int count = 0; 
float faX, faY, faZ;
    float fmX, fmY, fmZ;
    float faX_prev=0,faY_prev=0,faZ_prev=0;
    float deltaY = 0;
    float deltaX = 0;
    float deltaZ = 0;
  
    acc.enable();
    printf("\r\n\nFXOS8700Q Who Am I= %X\r\n", acc.whoAmI());

  
    while(1) {
       c = myGPS.read();   //queries the GPS
        
        if (c) { pc.printf("%c", c); } //this line will echo the GPS data if not paused
        
        //If message received parse Data
        if ( myGPS.newNMEAreceived() ) {
            printf("bla  ");
            if ( !myGPS.parse(myGPS.lastNMEA()) ) {
                //continue;   
                GPS_read (myGPS,refresh_Timer, refresh_Time);
                printf("bla bla ");
            }    
        }
        
        //check if enough time has passed to warrant printing GPS info to screen
        
      /*  if (refresh_Timer.read_ms() >= refresh_Time) {
            refresh_Timer.reset();
            printf("Time: %d:%d:%d\n\r", myGPS.hour, myGPS.minute, myGPS.seconds);   
            printf("Date: %d/%d/20%d\n\r", myGPS.day, myGPS.month, myGPS.year);
            printf("Fix: %d\n\r", (int) myGPS.fix);
            printf("Quality: %d\n\r", (int) myGPS.fixquality);
            if (myGPS.fix) {
                printf("Location: %5.2f%c, %5.2f%c\n\r", myGPS.latitude, myGPS.lat, myGPS.longitude, myGPS.lon);
                printf("Speed: %5.2f knots\n\r", myGPS.speed);
                printf("Angle: %5.2f\n\r", myGPS.angle);
                printf("Altitude: %5.2f\n\r", myGPS.altitude);
                printf("Satellites: %d\n\r", myGPS.satellites);
            }
        }*/
       float pinReading = 100*senseLine.read();
    //Base Pin Value needs to read 10.0
    //float AvgReading = (std::ceil(pinReading)+std::floor(pinReading))/2;
    //float Vol = 14.474*pinReading - 787.19;
    //float AvgVol = (std::ceil(Vol)+std::floor(Vol))/2;
        acc.getAxis(acc_data);
        //mag.getAxis(mag_data);
        printf("FXOS8700Q ACC: X=%1.4f Y=%1.4f Z=%1.4f ", acc_data.x, acc_data.y, acc_data.z);//printing out X,Y,Z Axis of Accel.
       // printf("    MAG: X=%4.1f Y=%4.1f Z=%4.1f\r\n", mag_data.x, mag_data.y, mag_data.z);//printing out X,Y,Z Axis of Magn.
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
        printf("FXOS8700Q ACC: X=%1.4f Y=%1.4f Z=%1.4f\t\r\n ", faX, faY, faZ);
        //printf("    MAG: X=%4.1f Y=%4.1f Z=%4.1f\r\n", fmX, fmY, fmZ);
         
         
      pc.printf("%.2f %d \t", pinReading, count);
         count++;
         if (count == 5)
         {
             pc.printf("\t\t\n\r");
             count = 0 ;
         }
         wait(.01);
              
    }
}
void GPS_read (Adafruit_GPS myGPS, Timer refresh_Timer, const int refresh_Time){
    if (refresh_Timer.read_ms() >= refresh_Time) {
            refresh_Timer.reset();
            printf("Time: %d:%d:%d\n\r", myGPS.hour, myGPS.minute, myGPS.seconds);   
            printf("Date: %d/%d/20%d\n\r", myGPS.day, myGPS.month, myGPS.year);
            printf("Fix: %d\n\r", (int) myGPS.fix);
            printf("Quality: %d\n\r", (int) myGPS.fixquality);
            if (myGPS.fix) {
                printf("Location: %5.2f%c, %5.2f%c\n\r", myGPS.latitude, myGPS.lat, myGPS.longitude, myGPS.lon);
                printf("Speed: %5.2f knots\n\r", myGPS.speed);
                printf("Angle: %5.2f\n\r", myGPS.angle);
                printf("Altitude: %5.2f\n\r", myGPS.altitude);
                printf("Satellites: %d\n\r", myGPS.satellites);
            }
        }
    }
    
    
    
    
    
