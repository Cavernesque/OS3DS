/*
 * Open-Source 3D Scanner Project
 * Summer 2021
 * Darren Paetz, Matt Kantor, Philip Mees
 * MacEwan University, Edmonton, Alberta, Canada, 2021
 * 
 * This firmware is designed to be used with the 3D Scanner Interface application.
 * That application will gather the data from the scanner and convert it to an
 * industry standard file format (E57).
 * 
 * 
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <SoftwareSerial.h>

/***************
  Globals
***************/
// --Scanner--
// Parameters
unsigned int sensor_type = 0; // Parameterization from app - MEMSAVE: byte
unsigned int scan_angle = 0; // Parameterization from app
float scan_resolution = 0; // Parameterization from app
// Scanner angle limits
//  Angle phi is measured downwards from the zenith, relative to the vertical axis of the scanner
//  Straight up (Zenith) -> 0°   Straight down (Nadir) -> 180°
//  Max vertical (highside_angle): 0, but realistically should be 15 degrees.
//  Min vertical (lowside_angle): 150, limited by scanner body.
unsigned int highside_angle = 20;  // Parameterization from app
unsigned int lowside_angle = 135;  // Parameterization from app
unsigned int scan_point_trimming = 0;  // Parameterization from app
unsigned int scan_data_validation = 0; // Parameterization from app
// Scanner data collection
unsigned int distance = 0; // Distance will be used by either sensor type, and takes the same form in memory.
unsigned int intensity = 0; // Strength will be used exclusively by the LiDAR, both TF02 Pro and TFMini
// Scan statistics and flags
bool scan_complete = false;  // Flag for scan completion status
bool scanner_configured = false;  // Flag for scanner successful configuration
byte scanner_current_line = 1;  // Tracks current line to adjust for point interpolation.
unsigned int scan_points_acquired = 0;  // Tracks how many points have currently been scanned.
unsigned long anticipated_points = 0;  // Calculated number of points scan parameters should yield.
unsigned int validation_fails = 0;  // Tracks amount of failed transmissions from sensors
unsigned long vertical_step_cnt = 0;  // Tracks the number of steps to travel back to the top

// ----External Devices----
// --Steppers--
// We'll be using the steppers in full step mode, meaning we get 2037 steps per revolution
// This is based on independent findings that the 28BYJ-48 stepper has 4074 steps per full revolution in half step mode
const int revolution_steps = 2037; // 2037 steps per revolution of stepper (11.311 degrees/64:1 gear ratio)
const float horiz_deg_steps = 20.0871; // 20.0871 steps per horizontal degree (with scanner gearing)
const float vert_deg_steps = 5.6583; // 5.6583 steps per vertical degree (only stepper gearing)
const int full_steps = 7342; // 7342 steps per full horizontal revolution of scanner
float horiz_position = 0; // Initialize horizontal position tracking variable
float vert_position = 90; // Initialize vertical position tracking variable, will be set during homingSequence
bool vert_motor_direction = true; // Vertical motor direction: False is up, True is down

// --LiDAR--
// Default address of the LiDAR is 0x10 (DEC: 16)
// MEMSAVE: Place only in PROGMEM
int lid_addr = 0x10; // Leave as variable so it can be configured to prevent address collisions

/********************
 Stepper object setup
********************/
//Use default I2C addr 0x60 (DEC: 96)
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
//Create stepper object with (Steps/rev,Motor Port) on Motor Port 1 (M1&M2) and 2 (M3&M4) 
Adafruit_StepperMotor *horiz_Stepper = AFMS.getStepper(revolution_steps,1); 
Adafruit_StepperMotor *vert_Stepper = AFMS.getStepper(revolution_steps,2);

/******************
 Physical I/O setup
******************/
// Reserve serial connection for the sonar if used
SoftwareSerial sonar_Serial(2,3); // RX 2, TX 3
// Limit switch for vertical axis
const int limit_pin = 7;

/*****************
 Initial functions
*****************/
void setup() {
  // Open serial connection to the application.
  Serial.begin(115200);
  // We'll always need motorshield communication, start it immediately.
  AFMS.begin();
  // Default Stepper Speed is 10 RPM
  horiz_Stepper->setSpeed(10);  // TEST: Change to 15 rpm?
  vert_Stepper->setSpeed(10);  // TEST: Change to 15 rpm?
  // Set pins for limit switch and sonar serial connection
  pinMode(limit_pin, INPUT_PULLUP);
  pinMode(2, INPUT);
  pinMode(3, OUTPUT);
}

void serialEvent(){
  /*  The first character of any serial transmission will be a command code.
   *  A: Configure scanner
   *  B: Jog motors
   *  C: Scan
   *  serialEvent is only called at the end of loop(), so we don't need to worry about
   *  potentially interfering with a configuration frame once in that function.
   */
  char command_code = (char)Serial.read();
  switch(command_code){
    case 65:
      codeAConfigure();
      break;
    case 66:
      codeBJog();
      break;
    case 67:
      codeCScan();
      break;
  }
}

/***************************
 Configuration (A) functions
***************************/

void codeAConfigure(){
  bool get_config = false, begin_scan = false;
  // Purge the serial buffer to prepare to read configuration data
  while(Serial.available()){
    char byte_dump = Serial.read();
  }
  // Until we can successfully read a configuration frame from the application, do nothing.
  while(get_config == false){
    get_config = getConfiguration();
  }
  // Until we can successfully get a confirmation message from the application, do nothing.
  while(begin_scan == false){
    begin_scan = confirmScan();
  }
  // We've configured successfully, proceed to scan. Set the serial timeout to match the application.
  Serial.setTimeout(500);
  // sensor_type 0 -> SONAR
  //             1 -> LiDAR
  switch(sensor_type){
    case 0: // Sonar
      pinMode(2,INPUT);
      pinMode(3,OUTPUT);
      sonar_Serial.begin(9600);
      sonar_Serial.listen();
      break;
    case 1: // Lidar
      // Begin takes the argument of the Slave address for Arduino to take
      // If no address is given, it joins the I2C bus as a Master
      Wire.begin();
      // Set the wire clock to Fast mode, 400kbps
      Wire.setClock(400000);
      // Set up I2C communication mode to LiDAR
      Wire.beginTransmission(lid_addr);
      // Write the command to change the measurement unit to mm instead of cm
      // HEX: {5A, 05, 05, 06, 6A}     DEC: {90, 5, 5, 6, 106}
      byte message[] = {90, 5, 5, 6, 106};
      Wire.write(message,5);
      // TODO: Add in a message to change the slave address of the I2C if necessary
      // 210618, DP: Likely unnecessary.
      Wire.endTransmission();
      delay(10);  // Wait minimum time for LiDAR response
      break;
  }
  anticipated_points = calcScanPoints();
  homingSequence();
}

bool getConfiguration(){
  /*  Listen on the USB UART for configuration data from the application
   *  Save settings to globals
   *  If data has an invalid parameter, pass back false
   */
  String sensor_string = "", angle_string = "", hiangle_string = "";
  String loangle_string = "", resolution_string = "", valid_string = "", trim_string = "";
  // Set the serial timeout to be very long to allow for potential manual configuration
  // Get configuration data from the application
  // Leave 10ms in between reads to give time for the serial buffer to fill
  Serial.setTimeout(10000);
  sensor_string = Serial.readStringUntil('\n');
  delay(10);
  angle_string = Serial.readStringUntil('\n');
  delay(10);
  hiangle_string = Serial.readStringUntil('\n');
  delay(10);
  loangle_string = Serial.readStringUntil('\n');
  delay(10);
  resolution_string = Serial.readStringUntil('\n');
  delay(10);
  valid_string = Serial.readStringUntil('\n');
  delay(10);
  trim_string = Serial.readStringUntil('\n');
  Serial.setTimeout(1000);
  // Use string methods to pull configuration data from transmission
  sensor_type = (unsigned int)sensor_string.toInt();
  scan_angle = (unsigned int)angle_string.toInt();
  highside_angle = (unsigned int)hiangle_string.toInt();
  lowside_angle = (unsigned int)loangle_string.toInt();
  scan_resolution = (float)resolution_string.toFloat();
  scan_point_trimming = (unsigned int)trim_string.toInt();
  scan_data_validation = (unsigned int)valid_string.toInt();
  
  // Make sure we've changed at least one parameter
  if((scan_angle==0)){
    return false;
  }
  else{
    scanner_configured = true;
    return true;
  }
  return false; // If we've gotten here, something's gone wrong.
}

bool confirmScan(){
  // Take the configuration the scanner has and push it back to the application
  // Once permission is given from application, begin scan.
  // If permission is not granted, pass back false.
  String frame = "", response = "";
  // TODO: Add vertical angle validation
  frame += (String(sensor_type) + ','
    + String(scan_angle) + ','
    + String(highside_angle) + ','
    + String(lowside_angle) + ','
    + String(scan_data_validation) + ','
    + String(scan_point_trimming));
  Serial.println(frame);
  // Set serial timeout long for potential manual configuration
  Serial.setTimeout(10000);
  delay(10);
  response = Serial.readStringUntil('\n');
  Serial.setTimeout(1000);
  if(response == "scan"){
    return true;
  }
  else{
    return false;
  }
  return false; // If we've gotten here, something has gone wrong.
}

void homingSequence(){
  // Initialize the position of the vertical stepper motor
  // Rotate CCW until the limit switch activates
  // Once limit switch is hit, move CW to the highest possible scan angle
  bool limit_state = false;
  // limit_pin is HIGH when switch is closed and closed to ground
  limit_state = !digitalRead(limit_pin);
  // Increment the motor 1 degree in the FORWARD direction until we close the limit switch
  while(!limit_state){
    vert_Stepper->step(20,FORWARD,DOUBLE);  // TEST: Change to smaller step value
    limit_state = !digitalRead(limit_pin);
  }
  // Limit switch reached, go BACKWARD to top vertical angle
  int init_pos_steps = vert_deg_steps*highside_angle;  // TEST: Angle that scanner reaches after taking the steps
  vert_Stepper->step(init_pos_steps,BACKWARD,DOUBLE);
  vert_position = highside_angle;
}

unsigned long calcScanPoints(){
  // Calculate the number of points we should be scanning
  unsigned long points = 0;
  // TODO: Re-introduce point trimming calculation
  // Steps : Resolution
  // (1 : 4.0), (2 : 2.0), (5 : 1.0), (11 : 0.5), (22 : 0.25), (56 : 0.1)
  unsigned int prc_adj_vert_step = vert_deg_steps/scan_resolution;
  // (4 : 4.0), (10 : 2.0), (20 : 1.0), (40 : 0.5), (80 : 0.25), (200 : 0.1)
  unsigned int prc_adj_horiz_step = horiz_deg_steps/scan_resolution;
  // Position change (Degrees) : Resolution
  // (0.1767 : 4.0), (0.3535 : 2.0), (0.8837 : 1.0), (1.944 : 0.5), (3.888 : 0.25), (9.897 : 0.1)
  float prc_adj_vert_pos = prc_adj_vert_step*(1/vert_deg_steps);
  // (0.1991 : 4.0), (0.4978 : 2.0), (0.9957 : 1.0), (1.991 : 0.5), (3.983 : 0.25), (9.957 : 0.1)
  float prc_adj_horiz_pos = prc_adj_horiz_step*(1/horiz_deg_steps);
  unsigned long vertical_positions = ((lowside_angle-highside_angle)/prc_adj_vert_pos) + 1; // Add 1 to ensure we go over the angle limit
  unsigned long horizontal_positions = (scan_angle/prc_adj_horiz_pos) + 1; // Add 1 to ensure we go over the angle limit
  points = vertical_positions*horizontal_positions;
  return points;
}

/*****************
 Jog (B) functions
*****************/

void codeBJog(){
  /*  Each jog button will only send a single character on UART.
   *  Wait to receive it from the application.
   */
  delay(100);
  char direct_code = (char)Serial.read();
  switch(direct_code){
    // Each button press should correspond to 4 degrees of turn
    // TODO: This amount of steps should be configurable.
    case 117:  // 'u'
      vert_Stepper->step(20,FORWARD,DOUBLE);
      break;
    case 100:  // 'd'
      vert_Stepper->step(20,BACKWARD,DOUBLE);
      break;
    case 108:  // 'l'
      horiz_Stepper->step(80,BACKWARD,DOUBLE);
      break;
    case 114:  // 'r'
      horiz_Stepper->step(80,FORWARD,DOUBLE);
      break;
  }
}

/***************************************
 Scan and data acquisition (C) functions
***************************************/

void codeCScan(){
  // Loop until we have enough points to satisfy the scan.
  while(!scan_complete){
    bool data_state = false;
    bool scan_the_point = false;
    // ----------------------------------
    // Gather data from configured sensor
    // ----------------------------------
    // If point interpolation enabled, check if we need to measure a point at the current scanner position.
    scan_the_point = scanPoint();
    if(scan_the_point){
      // LiDAR routine
      if((sensor_type) && (!scan_complete)){
        data_state = readLidar();
        // If we are not using data validation, and the data is invalid, pass 0's.
        if((!scan_data_validation) && (!data_state)){
          distance = 0;
          intensity = 0;
          data_state = true; // Pass a true data state so program proceeds.
        }
        // Otherwise, we're using data validation and need to continue polling sensor until we get a good value
        else{
          while(!data_state){
            data_state = readLidar();
          }
        }
      }
      // Sonar routine
      else if((!sensor_type) && (!scan_complete)){
        data_state = readSonar();
        // If we are not using data validation, and the data is invalid, pass 0.
        if((!scan_data_validation) && (!data_state)){
          distance = 0; // Sonar doesn't yield intensity.
          data_state = true; // Pass a true data state so program proceeds.
        }
        // Otherwise, we're using data validation and need to continue polling sensor until we get a good value
        else{
          while(!data_state){
            data_state = readSonar();
          }
        }
      }
    }
    // Increment our point count, irrespective of data validity condition
    scan_points_acquired += 1;
    /*  Push data
     *  If data validation is enabled, and the data is bad, loop and try reading data again. App will wait.
     *  Data frame is structured as:
     *  DATA: Distance, Itensity, Horizontal angle, Vertical angle, Checksum
     *  TYPE: unsigned int, unsigned int, float, float, unsigned long
     *  UNIT: [mm], [value], [angle°], [angle°], [integer]
     */
    if(data_state){
      unsigned long checksum = genChecksum();
      Serial.print(distance);
      Serial.print(',');
      Serial.print(intensity);
      Serial.print(',');
      Serial.print(horiz_position);
      Serial.print(',');
      Serial.print(vert_position);
      Serial.print(',');
      Serial.print(checksum);
      Serial.print('\r');
      Serial.print('\n');
    }
    else
    {
      Serial.print("0,0,0,0,0");
      Serial.print('\r');
      Serial.print('\n');
    }
    // Move the scanner to the next position.
    moveScanner();
    // Check to see if the scan has completed, set the flag if it is.
    if(scan_points_acquired >= anticipated_points){
      scan_complete = true;
    }
  }
  // Return to starting position after loop is finished, "anti-tangle"
  horiz_Stepper->step((full_steps*(horiz_position/360)),FORWARD,DOUBLE); // Rotate horizontally clockwise
  horiz_position = 0;
  // Scan is complete, pass control back to loop() and allow serial commands again.
  Serial.print("complete,");
  if(scan_data_validation){
    Serial.println(validation_fails);
  }
  else{
    Serial.println("0");
  }
  horiz_Stepper->release();
  vert_Stepper->release();
}

void moveScanner(){
  /*  Take scan parameters and move the steppers accordingly
   *  How many steps the motor moves between data points is dependent on scan resolution
   *  When method returns true, scan is complete.
   *  (Scan resolution : Vertical points per horizontal position)
   *  (2.0 : 180), (1.0 : 90), (0.5 : 45), (0.25 : 22), (0.1 : 9)
   *  Scan resolution : Horizontal degrees in full rotation
   *  (2.0 : 720), (1.0 : 360), (0.5 : 180), (0.25 : 90), (0.1 : 36)
   * INFO: There is theoretically an available "extremely fine" scan where we take 11 data points per degree
   *       This would mean 990 points per horizontal degree, and 3960 horizontal points
   *       A whopping total of 3.92 MILLION points!
   */
  // Steps : Resolution
  // (1 : 4.0), (2 : 2.0), (5 : 1.0), (11 : 0.5), (22 : 0.25), (56 : 0.1)
  int prc_adj_vert_step = vert_deg_steps/scan_resolution;
  // (4 : 4.0), (10 : 2.0), (20 : 1.0), (40 : 0.5), (80 : 0.25), (200 : 0.1)
  int prc_adj_horiz_step = horiz_deg_steps/scan_resolution;
  // Position change (Degrees) : Resolution
  // (0.1767 : 4.0), (0.3535 : 2.0), (0.8837 : 1.0), (1.944 : 0.5), (3.888 : 0.25), (9.897 : 0.1)
  float prc_adj_vert_pos = prc_adj_vert_step*(1/vert_deg_steps);
  // (0.1991 : 4.0), (0.4978 : 2.0), (0.9957 : 1.0), (1.991 : 0.5), (3.983 : 0.25), (9.957 : 0.1)
  float prc_adj_horiz_pos = prc_adj_horiz_step*(1/horiz_deg_steps);
  // Control the vertical motor based on the direction we need to travel
  switch(vert_motor_direction){
    case 0: // Rotate servo upwards
      vert_Stepper->step(vertical_step_cnt,FORWARD,DOUBLE);
      vert_position = highside_angle;
      vertical_step_cnt = 0;
      vert_motor_direction = !vert_motor_direction;
      break;
    case 1: // Rotate servo downwards
      vert_Stepper->step(prc_adj_vert_step,BACKWARD,DOUBLE);
      vert_position += prc_adj_vert_pos;
      vertical_step_cnt += prc_adj_vert_step;
      break;
  }
  if(vert_position >= lowside_angle){
    horiz_Stepper->step(prc_adj_horiz_step,BACKWARD,DOUBLE); // Rotate horizontally anti-clockwise
    horiz_position += prc_adj_horiz_pos;
    vert_motor_direction = !vert_motor_direction;
    // Cycle the scanner line around the pattern
    if(scanner_current_line >= 8){
      scanner_current_line = 1;
    }
    else{
      scanner_current_line += 1;
    }
  }
  return;
}

bool readLidar() {
  /*  Read distance information from the Benewake TF02 Pro or TF Mini Plus
   *  Information is communicated over I2C bus at 400 kbps (Fast Mode), must be requested
   *  When data is read successfully, write it to the global variable
   */
  distance = 0;
  byte i = 1;  // Iterator for dataframe from lidar
  byte b = 0;  // Byte holder for dataframe from lidar
  word checksum = 0;
  bool check_pass = false;
  // Set up the delay time between request and read from the LiDAR on I2C
  const byte lid_dly = 10; // Value shouldn't change, make constant
  Wire.beginTransmission(lid_addr);
  /*  The write command will send a request to the LiDAR to obtain a data frame
   *  Measurement will change based on command
   *  HEX: {5A 05 00 01 60} will give cm
   *  DEC: {90 5 0 1 96}
   *  HEX: {5A 05 00 06 65} will give mm
   *  DEC: {90 5 0 6 101} 
   */
  byte data_req_from_lid[] = {90, 5, 0, 6, 101};
  Wire.write(data_req_from_lid,5);
  // endTransmission by default ends with a "stop" message on the I2C bus - the Pro and Mini require this message to be present
  Wire.endTransmission();
  // It's recommended that after initiating a request for data that you wait 100ms before reading the response
  // However, this can be reduced dramatically, to near 10ms to allow for fast data acquisition
  delay(lid_dly);
  // Request the slave write the data frame onto the bus
  Wire.requestFrom(lid_addr,9);
  // Read the response from the LiDAR, interpret it and push it.
  while(Wire.available()){
    // Get the next available byte
    b = Wire.read();
    // Bytes 3 and 4 are the distance data bytes, process them
    if(i==3){
      distance = b;
    }
    if(i==4){
      // Correct for Little Endian format, and add to the total
      distance = distance + (b << 8);
    }
    // Bytes 5 and 6 are the signal strength data bytes, process them
    if(i==5){
      intensity = b;
    }
    if(i==6){
      // Correct for Little Endian format, and add to the total
      intensity = intensity + (b << 8);
    }
    // If this isn't the checksum byte from LiDAR, add it to the local checksum
    if(i != 9){
      checksum += b;
    }
    // Otherwise, we are looking at the checksum byte. Validate our checksum against it.
    // If our checksum matches, we've received the data correctly. Push it to serial.
    // Return whether the checksum matched or not to the main program.
    else{
      if(lowByte(checksum) == b){
        // If the data matches the LiDAR checksum, data is good.
        check_pass = true;
      }
      else{
        // We've failed a checksum validation, add to the statistics.
        validation_fails += 1;
      }
      // The return value of the function will dictate if we use the data or not.
      return check_pass;
    }
    // Increment byte counter
    i++;
  }
  return false; // If we've gotten here, something went wrong with the I2C. Don't use the data.
}

bool readSonar(){
  // Read distance information from theDYP-A01-V2.0 sonar
  // Information is communicated over UART at 9600 baud, unprompted
  // When data is read successfully, write it to the global variable
  distance = 0;
  word incoming_checksum = 0;
  byte frame[2];
  byte discard = 0;
  if(sonar_Serial.available()>=4){
    byte incoming_byte = byte(sonar_Serial.read());
    if(incoming_byte == 255){
      frame[0] = byte(sonar_Serial.read());
      frame[1] = byte(sonar_Serial.read());
      frame[2] = byte(sonar_Serial.read());
      distance = frame[0] << 8;
      distance += frame[1];
      incoming_checksum += frame[0] + frame[1] + 255;
      // Leave time for another piece of data to be generated
      delay(300);
      // Discard any data in the serial buffer that accumulated after we got the data frame
      while(sonar_Serial.available()>0){
        discard = byte(sonar_Serial.read());
      }
      // If checksum is valid and the data is not the maximum value (likely erroneous), data is good.
      if(lowByte(incoming_checksum) == frame[2]){
        return true;
      }
      else{
        validation_fails += 1;
        return false;
      }
    }
  }
  return false; // If we've gotten here, something has gone wrong. Discard the data.
}

bool scanPoint(){
  // Take the current vertical position and return whether or not to measure a point based on scanner line.
  unsigned int comparison_high_angle = 0, comparison_low_angle = 0;
  // If point trimming is disabled, scan every single point.
  if(!scan_point_trimming){
    return true;
  }
  // Set the angle limits for point interpolation.
  comparison_low_angle = lowside_angle;
  if((scanner_current_line % 8)== 0)
  {
    comparison_high_angle = highside_angle;
  }
  else if((scanner_current_line % 4)== 0)
  {
    comparison_high_angle = max(highside_angle,7);
  }
  else if((scanner_current_line % 2)== 0)
  {
    comparison_high_angle = max(highside_angle,14);
  }
  else
  {
    comparison_high_angle = max(highside_angle,30);
  }
  // If the scanner's current vertical position is within the range of angle where we want a point on this line
  // return true to the program
  if((vert_position >= comparison_high_angle) && (vert_position <= comparison_low_angle)){
    return true;
  }
  else{
    return false;
  }
  return false;  // If we've gotten here, something's gone wrong. Default return value.
}

unsigned long genChecksum(){
  // Generate a checksum for our transmission to the application
  unsigned long checksum = 0;
  checksum += (distance + intensity + (int)horiz_position + (int)vert_position);
  return checksum;
}

void loop() {
}
