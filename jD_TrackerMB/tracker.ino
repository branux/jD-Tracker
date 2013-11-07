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

//lat1=13.7179;
//lon1=100.6965;
/////////////////////////////////Testing//////////////////////////////////////////////////////
//if (Serial.available()) {
//    testing = Serial.read();
//
//    if(testing == '0'){
//      _uavElev = 45.0;
//      lat2 =13.71932;
//      lon2 = 100.69602;
//    }
//    if(testing == '1'){
//      _uavElev = 55.0;
//      lat2 =13.71848;
//      lon2 = 100.69670;
//    } 
//    if(testing == '2'){
//      _uavElev = 65.0;
//      lat2 =13.71788;
//      lon2 = 100.69771;
//    } 
//    if(testing == '3'){
//      _uavElev = 15.0;
//      lat2 =13.71715;
//      lon2 = 100.69782;
//    } 
//    if(testing == '4'){
//      _uavElev = 25.0;
//      lat2 =13.71608;
//      lon2 = 100.69732;
//    } 
//    if(testing == '5'){
//      _uavElev = 0;
//      lat2 =13.71539;
//      lon2 = 100.69597;
//    } 
//    if(testing == '6'){
//      _uavElev = 10.0;
//      lat2 =13.71660;
//      lon2 = 100.69437;
//    }  
//    if(testing == '7'){
//      _uavElev = 5.0;
//      lat2 =13.7181;
//      lon2 = 100.6933;
//    }
//    if(testing == '8'){
//      _uavElev = 30.0;
//      lat2 =13.7194;
//      lon2 = 100.6931;
//    } 
//    if(testing == '9'){
//      _uavElev = 10.0;
//      lat2 =13.7198;
//      lon2 = 100.6948;
//    }   
//}
  float x = 69.1 * (lat2 - lat1);
  float y = 69.1 * (lon2 - lon1) * cos(lat1/57.3);

  _uavBear = toDeg(atan2(y,x));
  _uavDist = (float)sqrt((float)(x*x) + (float)(y*y))*1609.344;

  if(_uavBear < 0) _uavBear = 360.0 + _uavBear; 

  //Antenna Leveling loop 100times per second
  //if (millis() - LevelingTimer > 25) {

  angle = read_IMU();
  Serial.print("angle");
  Serial.print(" ");
  Serial.println(angle);
  Serial.print("AngleError");
  Serial.print(" ");
  Serial.println(AngleError);
  Serial.print("LevelFlag");
  Serial.print(" ");
  Serial.println(levelflag);


  AngleError = angle - _uavElev;
  // Serial.println("Averaging done");
  if(((AngleError) >8)&& (levelflag != 1)){
    Serial2.print('A');
    Serial.println("Driving Up");
    levelflag = 1;
  }
  else if((AngleError >4) && (AngleError < 8) && (levelflag != 2 )){
    Serial2.print('B');
    Serial.println("Driving Up");
    levelflag = 2;
  }
  else if((AngleError >1) && (AngleError < 4) && (levelflag != 3)){
    Serial2.print('C');
    Serial.println("Driving Up");
    levelflag = 3;
  }
  else if( (-1 < AngleError) && (AngleError <1) && (levelflag != 7)){
    Serial2.print('G');
    Serial.println("Antenna Leveled");
    levelflag = 7;
  }
  else if((AngleError < -8) && (levelflag !=4)){
    Serial2.print('D');
    Serial.println("Driving Down");
    levelflag = 4;
  }
  if((AngleError) <-4 && (AngleError >-8) && (levelflag != 5)){
    Serial2.print('E');
    Serial.println("Driving Up");
    levelflag = 5;
  }
  else if((AngleError < -1) && (AngleError > -4) && (levelflag != 6)){
    Serial2.print('F');
    Serial.println("Driving Up");
    levelflag = 6;
  }
  // Update the Leveling loop timer
  Serial.println("Loop finished");
  Serial.println(angle);
  LevelingTimer = millis();
  // }


  //  //Antenna heading loop
 //if (millis() - HeadingTimer > 10) {
    CompassHeading = Read_compass();

    HeadingError = (_uavBear - CompassHeading);
       Serial.print("position ");
      Serial.println(testing);
  
   Serial.print("HeadingError and uav bearing /t");
    Serial.println(HeadingError);
    Serial.println(_uavBear);
    if((CompassHeading < 90) && (_uavBear >270)&& (HeadingError<348)) {
      Serial2.print('H');
      Serial.println("Driving Left H");
    }
    else if((CompassHeading > 270)&&(_uavBear<90) && (HeadingError>-348)) {
      Serial2.print('K');
      Serial.println("Driving Right K");
    }
    else  if(HeadingError <-12){
      Serial2.print('H');
      Serial.println("Driving Left H");
    }
     else  if((HeadingError <-8)&& (HeadingError>-12)){
      Serial2.print('J');
      Serial.println("Driving Left I");
    }
//     else  if((HeadingError <-2) && (HeadingError > -8)){
//      Serial2.print('J');
//      Serial.println("Driving Left J");
//    }
    else if (HeadingError >12){
      Serial2.print('K');
      Serial.println("Driving Right K");
    }
         else  if((HeadingError >8)&& (HeadingError<12)){
      Serial2.print('M');
      Serial.println("Driving Right L");
    }
//     else  if((HeadingError >2) && (HeadingError <8)){
//      Serial2.print('M');
//      Serial.println("Driving Right M");
//  }
    else if((HeadingError <2) && (HeadingError >-2)){
      Serial2.print('N'); 
  Serial.println("Anteanna Headed N");  
  }


    // Update the Heading loop timer
    Serial.println("Heading Loop finished");
    Serial.println(CompassHeading);
    HeadingTimer = millis();


// }
}





