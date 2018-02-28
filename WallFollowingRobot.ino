/*
 * ObjectAvoidingRobot.ino
 * 
 * DATE: 2-23-2018
 * AUTHOR: Calvin Kielas-Jensen
 * CONTRIBUTORS: Hussein, Hussein, Merrick
 * COURSE: ME 492 - Robotic Systems
 * 
 * DESCRIPTION: 
 *  
 */

#define DEBUG // Uncomment this for serial debugging
#define DRIVE // Uncomment this to drive the motors

#ifdef DEBUG
  #define dbPrint( x ) Serial.print( x )
  #define dbPrintln( x ) Serial.println( x )
#else
  #define dbPrint( x )
  #define dbPrintln( x )
#endif

#include <Wire.h>
#include <Adafruit_MotorShield.h>

#define CTRL_PER 25 // Period at which the control system is run [default is 40 Hz, or 25 ms] (ms)
#define DEFAULT_SPEED 150 // Default motor speed [0-255] (DN)
#define SPEED_OF_SOUND 346 // Speed of sound at room temperature (m/s)
#define PULSE_TO 11600 // Timeout for the pulseIn function [11600 times out at a USR distance of 2m] (us)
#define DIST_MAX 60 // Maximum distance the USR can measure, larger values will return a distance of zero (cm)
#define DIST_MIN 2 // Minimum distance the USR can measure, smaller values will return a distance of zero (cm)
#define DIST_THRESH 20 // Any measured distance smaller than this will cause the robot to rotate right (cm)
#define DIST_HYST 8000 // Distance hysteresis, the robot will not change course as long as it is within +/- this distnace (cm)
#define RIGHT_TURN_TIME 1250 // Amount of time to spend turning right when a corner is met (ms)
#define P_GAIN 10.0 // Proportional gain for the control system
//#define SP_GAIN 8 // How aggresively to turn if we aren't the right distance away from the wall
#define DIST_FL_OFFSET 0.25 // Add a small distance to distFL to slightly turn the robot to the right (negative will turn it to the left)

// USR trigger and echo pins
#define TRIG1 2
#define ECHO1 3
#define TRIG2 4
#define ECHO2 5
#define TRIG3 6
#define ECHO3 7

// Since we don't have a feedback loop, we may need to tune the motor speeds to each other (offsets are in DN [0-255])
#define M1_OFFSET 13 // Back Left
#define M2_OFFSET -5 // Back Right
#define M3_OFFSET -5 // Front Right
#define M4_OFFSET 13 // Front Left

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Create 4 motor objects
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *motor3 = AFMS.getMotor(3);
Adafruit_DCMotor *motor4 = AFMS.getMotor(4);

// Enumerate the three USRs
enum USR {
  USR_FL,
  USR_F,
  USR_BL
};

// Variables
#ifdef DEBUG
  uint8_t debugCount = 0;
#endif

float distFL, distF, distBL;
unsigned long curTime;
unsigned long lastTime = 0;

void setup() {
  
  #ifdef DEBUG
    Serial.begin(9600);
    Serial.println("DEBUG ON");
    Serial.println("Starting...");
    Serial.println("---");
  #endif

  // Set the apropriate input and output pins
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO3, INPUT);

  // Create with the default frequency 1.6KHz
  AFMS.begin();
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  motor1->setSpeed(DEFAULT_SPEED);
  motor2->setSpeed(DEFAULT_SPEED);
  motor3->setSpeed(DEFAULT_SPEED);
  motor4->setSpeed(DEFAULT_SPEED);

  // Turn all four motors off
  motor1->run(RELEASE);
  motor2->run(RELEASE);
  motor3->run(RELEASE);
  motor4->run(RELEASE);

  // Delay for a second so that we can set the robot down if it was being programmed
  delay(1000);
  
}

void loop() {

  curTime = millis();
  dbPrint("Time: ");
  dbPrintln(curTime);
  
  // Run the control system at a known frequency
  if ( curTime - lastTime >= CTRL_PER ) {

    // Read USR distances
    distFL = readUSR( USR_FL );
    distBL = readUSR( USR_BL );
    distF = readUSR( USR_F );
    dbPrint("DistFL: ");
    dbPrint(distFL);
    dbPrint(", DistBL: ");
    dbPrint(distBL);
    dbPrint(", DistF: ");
    dbPrint(distF);

    // Driving algorithm
    doDrive(distFL, distBL, distF);
    
    // Update the time
    lastTime = curTime;
  }

//  #ifdef DEBUG
//    if ( !(debugCount%100) ) {
//      Serial.println("---");
//      Serial.println("Dist FL\tDist F\tDist BL");
//      Serial.println("---");
//    }
//    Serial.print(distFL);
//    Serial.print('\t');
//    Serial.print(distF);
//    Serial.print('\t');
//    Serial.println(distBL);
//    debugCount++;
//  #endif

  dbPrintln(' ');
  
}

/*
 * doDrive
 * 
 * Drives the robot. Keeps a constant distance from the wall. Will turn if it reaches a corner.
 * 
 * INPUTS:
 *  distFL - Distance from the front left USR
 *  distBL - Distance from the back left USR
 *  distF - Distance from the front USR
 */

void doDrive( float distFL, float distBL, float distF ) {
  
  float avgDist;
  int8_t setPoint;
  float diffVal, err;
  int16_t driveVal;

  // If we see a corner, turn right
  if (distF < DIST_THRESH) {
    
    rotateR();

  // If we didn't see a corner, follow the wall
  } else {
    
    avgDist = (distFL + distBL) / 2;
    dbPrint(", Avg Dist: ");
    dbPrint(avgDist);
    
//    // Determine the direction we need to face
//    if (avgDist > DIST_THRESH + DIST_HYST) {
//      setPoint = -SP_GAIN;
//    } else if (avgDist < DIST_THRESH - DIST_HYST) {
//      setPoint = SP_GAIN;
//    } else {
//      setPoint = 0;
//    }

//    setPoint = 0;
//    
//    diffVal = distFL - distBL;
//    err = (float) setPoint - abs(diffVal);
//    driveVal = (int16_t) (P_GAIN*err);
//    if (diffVal > 0) driveMotors( DEFAULT_SPEED + driveVal, FORWARD, DEFAULT_SPEED, FORWARD );
//    else if (diffVal < 0) driveMotors( DEFAULT_SPEED , FORWARD, DEFAULT_SPEED + driveVal, FORWARD );
//    else driveMotors( DEFAULT_SPEED, FORWARD, DEFAULT_SPEED, FORWARD );

//    diffVal = distFL - distBL;
//    err = setPoint - diffVal;
//    driveVal = P_GAIN*err;
//    if (driveVal > DEFAULT_SPEED) driveMotors( DEFAULT_SPEED, FORWARD, 2*DEFAULT_SPEED - driveVal, FORWARD );
//    else if (driveVal < DEFAULT_SPEED) driveMotors( driveVal , FORWARD, DEFAULT_SPEED, FORWARD );
//    else driveMotors( DEFAULT_SPEED, FORWARD, DEFAULT_SPEED, FORWARD );

    diffVal = P_GAIN * (distFL - distBL);
    
    if (diffVal > 0) {
      if (diffVal > DEFAULT_SPEED) driveVal = DEFAULT_SPEED;
      else driveVal = diffVal;
      driveMotors( DEFAULT_SPEED - driveVal, FORWARD, DEFAULT_SPEED, FORWARD );
    }
    
    else if (diffVal < 0) {
      if (-diffVal > DEFAULT_SPEED) driveVal = -DEFAULT_SPEED;
      else driveVal = diffVal;
      driveMotors( DEFAULT_SPEED , FORWARD, DEFAULT_SPEED + driveVal, FORWARD );
    }
    
    else {
      driveMotors( DEFAULT_SPEED, FORWARD, DEFAULT_SPEED, FORWARD );
    }
    
    dbPrint(", diffVal: ");
    dbPrint(diffVal);
    dbPrint(", err: ");
    dbPrint(err);
    dbPrint(", driveVal: ");
    dbPrint(driveVal);
    
  }
}

/*
 * rotateR
 * 
 * Rotates the robot for a defined amout of time.
 */
void rotateR() {
  dbPrintln("Rotating Right");
  driveMotors( DEFAULT_SPEED, FORWARD, DEFAULT_SPEED, BACKWARD );
  delay(RIGHT_TURN_TIME);
  return;
}

/*
 * readUSR
 * 
 * Reads data from the USR sensors. According to the datasheet, the USRs should not be read
 * higher than a frequency of 40 Hz. Because of that, this function should not be called faster
 * than that (1/40 Hz = 25 ms).
 * 
 * INPUTS:
 *  num - USR index number
 *  
 * OUTPUTS:
 *  distance - Distance, in cm, to the object in front of the USR. According to the datasheet,
 *    it is constrained from 2 cm to 400 cm with a resolution of 0.3 cm.
 */
float readUSR( uint8_t num ) {

  float distance;
  unsigned long duration;
  int trigPin, echoPin;

  // Decide which USR we are reading
  switch (num) {
    case USR_FL:
      trigPin = TRIG1;
      echoPin = ECHO1;
      break;
    case USR_F:
      trigPin = TRIG2;
      echoPin = ECHO2;
      break;
    case USR_BL:
      trigPin = TRIG3;
      echoPin = ECHO3;
      break;
    default:
      return 0;
      break;
  }

  // Clear the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echoPin by returning the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH, PULSE_TO);
  
  // Calculate the distance in cm (divide by 2 for sound to reach object and come back)
  // 1us * (1s/1000000us) * 346m/s * 100cm/1m / 2 = 1us * 346m/s / 20,000 
  distance = duration * SPEED_OF_SOUND / 20000.0;

  if ( distance > DIST_MAX ) distance = DIST_MAX;
  else if ( distance < DIST_MIN ) distance = 0;
  
  return (distance) ? distance : DIST_MAX;
  
}

/*
 * driveMotors
 * 
 * Drives the motors using a tank setup. Assumes motors 1 and 4 are on the left side and motors
 * 2 and 3 are on the right side. DRIVE must be defined in order to drive the motors. If it is not,
 * the motors will not turn.
 * 
 * INPUTS:
 *  speedL - Left side speed in DN. Can range from 0 to 255.
 *  dirL - Direction of the left wheels where 0 is backward and 1 is forward
 *  speedR - Same as speedL but for right side
 *  dirR - Same as dirL but for right side
 */
void driveMotors( uint8_t speedL, uint8_t dirL, uint8_t speedR, uint8_t dirR ) {

  dbPrint(", Motor L Speed: ");
  dbPrint( speedL + M1_OFFSET );
  dbPrint(", Motor R Speed: ");
  dbPrint( speedR + M2_OFFSET );

  #ifdef DRIVE
  
    // For the robot, motors 1 and 4 are on the left, motors 2 and 3 are on the right
    // Set left speed
    motor1->setSpeed(speedL+M1_OFFSET);
    motor4->setSpeed(speedL+M4_OFFSET);
    // Set right speed
    motor2->setSpeed(speedR+M2_OFFSET);
    motor3->setSpeed(speedR+M3_OFFSET);
  
    // Set direction
    // Left motors
    motor1->run(dirL);
    motor4->run(dirL);
    // Right motors
    motor2->run(dirR);
    motor3->run(dirR);
    
  #else
  
    motor1->run(RELEASE);
    motor2->run(RELEASE);
    motor3->run(RELEASE);
    motor4->run(RELEASE);
    
  #endif

  return;
  
}










