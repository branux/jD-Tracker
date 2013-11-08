// Antenna tracking class
// Created 2011 By Colin G http://www.diydrones.com/profile/ColinG

Tracker::Tracker()
{
  // Time stamp of last update
  _last_update = millis();
}

//void
//Tracker::notify(void *arg, mavlink_message_t *messageData)
//{
//
//}

void
Tracker::update()
{
  //  if (millis() - _last_update > TRACKERPERIOD) {
  //    _last_update = millis();
  _update();
  //  }
}

float
Tracker::get_dist(void)
{
  return _uavDist;
}

float
Tracker::get_bearing(void)
{
  return _uavBear;
}

void
Tracker::_update()
{
  // Calculate the distance and bearing to the UAV
  _calcs(ASM.lat, uav.lat, ASM.lon, uav.lon);

  _uavElev = toDeg(atan((uav.alt-ASM.alt)/_uavDist));


}

void
Tracker::_calcs(float lat1, float lat2, float lon1, float lon2)
{
  float bearing;


  float x = 69.1 * (lat2 - lat1);
  float y = 69.1 * (lon2 - lon1) * cos(lat1/57.3);

  _uavBear = toDeg(atan2(y,x));
  _uavDist = (float)sqrt((float)(x*x) + (float)(y*y))*1609.344;

  if(_uavBear < 0) _uavBear = 360.0 + _uavBear; 

  //Antenna Leveling loop 100times per second
  //if (millis() - LevelingTimer > 25) {

  angle = read_IMU();
//  if(!gcs_passthrough){
//  Serial.print("angle");
//  Serial.print(" ");
//  Serial.println(angle);
//  Serial.print("AngleError");
//  Serial.print(" ");
//  Serial.println(AngleError);
//  Serial.print("LevelFlag");
//  Serial.print(" ");
//  Serial.println(levelflag);
//  }


  AngleError = angle - _uavElev;
  // Serial.println("Averaging done");
  if(((AngleError) >8)&& (levelflag != 1)){
    Serial2.print('A');
    levelflag = 1;
  }
  else if((AngleError >4) && (AngleError < 8) && (levelflag != 2 )){
    Serial2.print('B');

    levelflag = 2;
  }
  else if((AngleError >1) && (AngleError < 4) && (levelflag != 3)){
    Serial2.print('C');

    levelflag = 3;
  }
  else if( (-1 < AngleError) && (AngleError <1) && (levelflag != 7)){
    Serial2.print('G');

    levelflag = 7;
  }
  else if((AngleError < -8) && (levelflag !=4)){
    Serial2.print('D');

    levelflag = 4;
  }
  if((AngleError) <-4 && (AngleError >-8) && (levelflag != 5)){
    Serial2.print('E');

    levelflag = 5;
  }
  else if((AngleError < -1) && (AngleError > -4) && (levelflag != 6)){
    Serial2.print('F');

    levelflag = 6;
  }
  // Update the Leveling loop timer
  LevelingTimer = millis();
  // }


  //  //Antenna heading loop
 //if (millis() - HeadingTimer > 10) {
    CompassHeading = Read_compass();

    HeadingError = (_uavBear - CompassHeading);

//  if(!gcs_passthrough){
//   Serial.print("HeadingError and uav bearing /t");
//    Serial.println(HeadingError);
//    Serial.println(_uavBear);
//        Serial.print("Compass Heading: ");
//    Serial.println(CompassHeading);
//  }
    
    if((CompassHeading < 90) && (_uavBear >270)&& (HeadingError<348)) {
      Serial2.print('H');
  
    }
    else if((CompassHeading > 270)&&(_uavBear<90) && (HeadingError>-348)) {
      Serial2.print('K');

    }
    else  if(HeadingError <-12){
      Serial2.print('H');

    }
     else  if((HeadingError <-8)&& (HeadingError>-12)){
      Serial2.print('J');

    }
//     else  if((HeadingError <-2) && (HeadingError > -8)){
//      Serial2.print('J');
//      Serial.println("Driving Left J");
//    }
    else if (HeadingError >12){
      Serial2.print('K');

    }
         else  if((HeadingError >8)&& (HeadingError<12)){
      Serial2.print('M');

    }
//     else  if((HeadingError >2) && (HeadingError <8)){
//      Serial2.print('M');
//      Serial.println("Driving Right M");
//  }
    else if((HeadingError <2) && (HeadingError >-2)){
      Serial2.print('N'); 

  }


    // Update the Heading loop timer

    HeadingTimer = millis();


// }
}





