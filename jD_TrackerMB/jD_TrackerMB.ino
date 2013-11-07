// Ardustation Mega
// Created 2013 By Colin G http://www.diydrones.com/profile/ColinG
//
// Special thanks go to the ArduPilot and Mavlink dev teams and Michael Smith

// AVR runtime
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <math.h>

// Mavlink libraries - Path requires updating per install
#include "Z:\Repositories\ArdustationMega\mavlink\v1.0\ardupilotmega\mavlink.h"
#include "Z:\Repositories\ArdustationMega\mavlink\v1.0\common\common.h"

// Arduino Libraries
#include <Servo.h>
#include <Wire.h>
#include <SD.h>
#include <EEPROM.h>

/////////////////////////////   Antenna Tracker additions ////////////////////////////////////////////////
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
int16_t ax, ay, az;
#define MPU6050_DLPF_5HZ      MPU6050_DLPF_CFG_6 //Set digital LPF bandwidth to 5Hz
// Compass:
// class default I2C address is 0x1E
// specific I2C addresses may be passed as a parameter here
// this device only supports one I2C address (0x1E)
HMC5883L mag;
int16_t mx, my, mz;
int CompassHeading = 0;
static unsigned long LevelingTimer =0;   // For getting accelerometer 10times/s
static unsigned long HeadingTimer =0;
const int numReadings = 5;
const int magReadings = 5;
long AccX[numReadings];
long AccZ[numReadings];
long averageX = 0;  
long averageZ = 0;      
long totalX = 0; 
long totalZ = 0;
int magX[magReadings];
int magY[magReadings];
int averagemagX = 0;  
int averagemagY = 0;      
int totalmagX = 0; 
int totalmagY = 0;
int index =0; 
int magindex = 0;
float angle=0;
char testing;
// the index of the current reading
// the running total
// the average

float bearing;
float HeadingError;
float AngleError;
byte levelflag = 0;
byte HeadingFlag = 0;


/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// GPS Includes
#include "AP_GPS.h"
//#include "AP_GPS_UBLOX.h"
//#include "AP_GPS_MTK.h"
//#include "AP_GPS_MTK19.h"
//#include "AP_GPS_Auto.h"

// LCD Includes
#include "glcd.h"
#include "fonts/allFonts.h"
#include "Z:\Repositories\ArdustationMega\bitmaps\icon_altitude_small.h"
#include "Z:\Repositories\ArdustationMega\bitmaps\sat.h"
#include "Z:\Repositories\ArdustationMega\bitmaps\icn_conn.h"
#include "Z:\Repositories\ArdustationMega\bitmaps\icn_batt.h"
#include "Z:\Repositories\ArdustationMega\bitmaps\icn_speed.h"

// Local modules
#include "GCS.h"                // Controls the ground station comms
#include "RotaryEncoder.h";     // Handles the rotary encoder events
#include "Tracker.h"            // Controls the antenna tracker
#include "Buttons.h"            // Routines for button presses
#include "Beep.h"               // Sounds the piezo buzzer
#include "nvram.h"              // For saving to EEPROM
// Variables and definitions
#include "hardware.h"           // Definitions for the ground station's hardware
#include "uav_params.h"         // Class containing the UAV parameters
#include "uav.h"                // Class containing the UAV variables
#include "asm.h"                // Class containing the ardustation mega's variables
#include "pages.h"	            // Contains the LCD pages
#include "pageSettings.h"	    // Contains the Settings LCD pages
#include "pagesPlane.h"	        // Contains the Plane LCD pages
#include "pagesRover.h"	        // Contains the Rover LCD pages
#include "pagesCopter.h"	    // Contains the Copter LCD pages



// GPS declarations
#define T3 1000
#define T6 1000000
#define T7 10000000
//AP_GPS_UBLOX g_gps(&Serial1);

// All GPS access should be through this pointer.
static GPS         *g_gps;

// GPS Selection
#if   GPS_PROTOCOL == GPS_PROTOCOL_AUTO
AP_GPS_Auto     g_gps_driver(&Serial1, &g_gps);

#elif GPS_PROTOCOL == GPS_PROTOCOL_NMEA
AP_GPS_NMEA     g_gps_driver(&Serial1);

#elif GPS_PROTOCOL == GPS_PROTOCOL_SIRF
AP_GPS_SIRF     g_gps_driver(&Serial1);

#elif GPS_PROTOCOL == GPS_PROTOCOL_UBLOX
AP_GPS_UBLOX    g_gps_driver(&Serial1);

#elif GPS_PROTOCOL == GPS_PROTOCOL_MTK
AP_GPS_MTK      g_gps_driver(&Serial1);

#elif GPS_PROTOCOL == GPS_PROTOCOL_MTK19
AP_GPS_MTK19    g_gps_driver(&Serial1);

#elif GPS_PROTOCOL == GPS_PROTOCOL_NONE
AP_GPS_None     g_gps_driver(NULL);

#else
#error Unrecognised GPS_PROTOCOL setting.
#endif // GPS PROTOCOL


//AP_GPS_UBLOX g_gps(&Serial1);
//static GPS         *g_gps;
//AP_GPS_Auto     g_gps_driver(&Serial1, &g_gps);


// EEPROM Declaration
NVRAM           nvram;                          ///< NVRAM driver

////////////////////////////////////////////////////////////////////////////////
// GCS selection
////////////////////////////////////////////////////////////////////////////////
//
GCS_MAVLINK gcs0(110);
GCS_MAVLINK gcs3(110);
//static uint8_t      apm_mav_system; 
static uint8_t apm_mav_component;

// Flag for passing mavlink through usb, for pc gcs
boolean gcs_passthrough = 0;

// Flag denoting that we're downloading the parameters
boolean downloading = 0;
unsigned long download_start_time;
int16_t download_index=0;

////////////////////////////////////////////////////////////////////////////////
// System Timers
////////////////////////////////////////////////////////////////////////////////
// Time in miliseconds of start of main control loop.  Milliseconds
static unsigned long fast_loopTimer;
static unsigned long med_loopTimer;
static unsigned long slow_loopTimer;
static unsigned long vslow_loopTimer;



unsigned long loopwait;
unsigned long maxloopwait = 0;

// Heartbeat counter
unsigned long hbcount = 0;

void setup() {
  // Initialise the display driver object
  GLCD.Init(NON_INVERTED);

  // load the NVRAM
  nvram.load();

  // Print the welcome message
  //lcd.print("Starting up");

  // Initialize the keypad
  Wire.begin();

  // SD Card
  pinMode(chipSelect, OUTPUT);
  init_sdcard();

  // Attach the rotary encoder
  attachInterrupt(0, doEncoder, CHANGE);
  attachInterrupt(1, doEncoder, CHANGE);
  rotary.configure(&ASM.encoderval, 500, 0, -4);

  // Initialize stuff that needs to go in a class
  init_batt();
  uav.sysid = 0;
  uav.onboard_param_count = 0;
  uav.bln_requested_params = 0;
  ASM.num_sats = 0;
  uint8_t i;
  for (i=0;i<PARAM_COUNT_PLANE;i++) {
    uav.param_plane_avail[i] = 0;
  }
  for (i=0;i<PARAM_COUNT_PLANE_CTUN;i++) {
    uav.param_plane_ctun_avail[i] = 0;
  }
  for (i=0;i<PARAM_COUNT_PLANE_NTUN;i++) {
    uav.param_plane_ntun_avail[i] = 0;
  }
  for (i=0;i<PARAM_COUNT_PLANE_TECS;i++) {
    uav.param_plane_tecs_avail[i] = 0;
  }
  for (i=0;i<PARAM_COUNT_ROVER;i++) {
    uav.param_rover_avail[i] = 0;
  }
  for (i=0;i<PARAM_COUNT_COPTER;i++) {
    uav.param_copter_avail[i] = 0;
  }

  // Initialise the serial ports
  Serial.begin(57600);   // USB comm port
  Serial2.begin(57600);  // Motor driver board
  Serial1.begin(38400);  // GPS
  Serial3.begin(57600);  // Telemetry original = 57600

  // Initialise the GCS
  gcs0.init(&Serial);
  gcs3.init(&Serial3);

  // Initialise the GPS
  //	stderr = stdout; 
  // Do GPS init
  g_gps = &g_gps_driver;
  // GPS Initialization
  g_gps->init(GPS::GPS_ENGINE_STATIONARY);

  // Write centre positions to servos
  Pan.attach(6, 800, 2200); // Ultimately make the end points as variables on some input screen
  Tilt.attach(7, 800, 2200);
  //  Pan.write(90);
  //  Tilt.write(90);

  // Start the first page
  Pages::enter();

  accelgyro.initialize();
  mag.initialize();

}

void loop() {
  uint8_t buttonid;

  // Update comms as fast as possible
  if (gcs3.initialised) {
    gcs3.update();
  } 
  else {
    Serial.println("GCS not initialised");
  }

  // Update the GPS as fast as possible
  g_gps->update();

  // This loop is to execute at 50Hz
  // -------------------------------------------
  loopwait = millis() - fast_loopTimer;
  if (loopwait > 19) {
    maxloopwait = max(loopwait, maxloopwait);

    // Listen for button presses
    buttonid = keypad.pressed();
    switch (buttonid) {
      // By default all keypad presses are sent to the pages
    default:
      Pages::interact(buttonid);
      break;
    }

    // Listen for encoder updates, notify the pages
    if (rotary.haschanged())
      Pages::interact(B_ENCODER);

    // update the currently-playing tune
    beep.update();

    // Update the antenna tracker
    //tracker.update();

    // Update the fast loop timer
    fast_loopTimer = millis();

    // This loop is to execute at 10Hz
    // -------------------------------------------
    if (millis() - med_loopTimer > 99) {
      // Sample battery sensor
      sample_batt();

      // Update the pages
      Pages::refresh_med();
      if (ASM.encoderval == 20) {
        beep.play(BEEP_LAND);
        ASM.encoderval = 0;
      }

      // Update the medium loop timer
      med_loopTimer = millis();
    }

    //    // This loop is to execute at 5Hz
    //    // -------------------------------------------
    //    if (millis()-slow_loopTimer > 199) {
    //      slow_loopTimer = millis();
    //    }

    // This loop is to execute at 0.5Hz
    // -------------------------------------------
    if (millis() - vslow_loopTimer > 1999) {
      Pages::refresh_slow();
      maxloopwait = 0;

      // If we're downloading parameters, check the progress
      if (downloading) {
        if (millis() - download_start_time >= 1000) {
          downloading = 0;
          Serial.println("Download timed out");
        }
      }

      // Automatically download parameters if we haven't already
      if (uav.connected && !uav.bln_requested_params) {
        // Don't do it straight away
        if (millis() - uav.connTime > 1000) {
          uav.bln_requested_params = 1;

          // Quick hack to start data streaming for copters
          if (uav.type == MAV_TYPE_HELICOPTER
            || uav.type == MAV_TYPE_TRICOPTER
            || uav.type == MAV_TYPE_QUADROTOR
            || uav.type == MAV_TYPE_HEXAROTOR
            || uav.type == MAV_TYPE_OCTOROTOR) {
            gcs3.data_stream_request();
            delay(50);
          }

          // Request the parameters
          gcs3.param_request(0);
        }
      }

      vslow_loopTimer = millis();
    }


tracker.update();

  }
}
float read_IMU(){
  accelgyro.getAcceleration(&ax, &ay, &az);
  // subtract the last reading:
  totalX= totalX - AccX[index];   
  totalZ= totalZ - AccZ[index];         
  // read from the sensor:  
  AccX[index] = ax; 
  AccZ[index] = az;
  // add the reading to the total:
  totalX= totalX + AccX[index];
  totalZ= totalZ + AccZ[index];  
  // advance to the next position in the array:  
  index = index + 1;                    

  // if we're at the end of the array...
  if (index >= numReadings)              
    // ...wrap around to the beginning: 
    index = 0;                           

  // calculate the average:
  averageX = totalX / numReadings;   
  averageZ = totalZ / numReadings;
  angle = atan2(averageX,averageZ);
  angle = angle * 57.2957795; // rad to degrees 
  return float(angle);
}


int Read_compass() {
  mag.getHeading(&mx, &my, &mz);
//subtract the last reading:
  totalmagX= totalmagX - magX[magindex];   
  totalmagY= totalmagY - magY[magindex];         
  //read from the sensor:  
  magX[magindex] = mx; 
  magY[magindex] = my;
  // add the reading to the total:
  totalmagX= totalmagX + magX[magindex];  
  totalmagY= totalmagY + magY[magindex];  
  // advance to the next position in the array:  
  magindex = magindex + 1;                    
  //  // if we're at the end of the array...
  if (magindex >= magReadings)              
    // ...wrap around to the beginning: 
    magindex = 0;                           

  // calculate the average:
  averagemagX = totalmagX / magReadings;   
  averagemagY = totalmagY / magReadings;
  float heading = atan2(averagemagY, averagemagX);
  if(heading < 0)
    heading += 2 * M_PI;
  Serial.print("  heading:\t");
  Serial.println(heading * 180/M_PI);
  return int(heading * 180/M_PI);

}










