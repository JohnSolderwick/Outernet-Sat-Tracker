#include <Wire.h>
#include <Stepper.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <Adafruit_INA219.h>
Adafruit_INA219 ina219;


Servo myservoX; // Elevation 

boolean stringComplete = false;

String inputString = "";
int ytarget = 0;
int xpos = 1;
int xtarget = 0;
int Xcurrent = 0;
int cal = 0;
int down = 0;
int up = 0;
int olcount = 0;
int dirPin = 8;                  // output pin for stepper motor direction
int stepPin = 9;                 // output pin for motor movement
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

/**************************************************************************/
/*
 Displays some basic information on this sensor from the unified
 sensor API sensor_t type (see Adafruit_Sensor for more information)
 */
/**************************************************************************/
void displaySensorDetails(void) {
  sensor_t sensor;
  bno.getSensor(&sensor);
  //Serial.println("------------------------------------");
  //Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  //Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  //Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  //Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  //Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  //Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  //Serial.println("------------------------------------");
  //Serial.println("");
  //delay(500);
}

/**************************************************************************/
/*
 Display some basic info about the sensor status
 */
/**************************************************************************/
void displaySensorStatus(void) {
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  //delay(500);
}

/**************************************************************************/
/*
 Display sensor calibration status
 */
/**************************************************************************/
void displayCalStatus(void) {
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  //Serial.print("\t");
  if (!system) {

  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  
  cal = (mag, DEC);
}

/**************************************************************************/
/*
 Arduino setup function (automatically called at startup)
 */
/**************************************************************************/
void setup(void) {
  Serial.begin(9600);
  Serial.println("Orientation Sensor Calibration");
  Serial.println("");
  //inputString.reserve(16);
  /* Initialise the sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print(
        "Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  uint32_t currentFrequency;
  pinMode(7,OUTPUT);
  digitalWrite(7,0);
        delay(2000);
  ina219.begin();
  pinMode(dirPin, OUTPUT);       // Assign output mode to pin for direction
  pinMode(stepPin, OUTPUT);      // Assign output mode to pin for setp
  digitalWrite(dirPin, LOW);     // Initialize dir pin 
  digitalWrite(stepPin, HIGH);
 
  myservoX.attach(6); // Attach X servo to pin 6
  
  // Rotate one full turn then tilt up and down to calibrate position sensor. 
  myservoX.write(1);
  for (int y = 1; y < 2000; y = y + 1) {
    
    digitalWrite(stepPin, HIGH);
    delay(6);
    digitalWrite(stepPin, LOW);
    delay(6);

 }
 for (int y = 1; y < 75; y = y + 2) {
myservoX.write(y);
delay(50);
 }
  for (int y = 74; y > 1; y = y - 2) {
myservoX.write(y);
delay(50);
 }

 

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  bno.setExtCrystalUse(true);
}


void loop(void) {

  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;

  current_mA = ina219.getCurrent_mA();
  
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.println(event.orientation.z, 4);
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.println("");
  /* Optional: Display calibration status */
  displayCalStatus();

  /* Optional: Display sensor status (debug only) */
  //displaySensorStatus();
  /* New line for the next sample */
  //Serial.println("");
  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);

  

  ytarget=210;
  xtarget=39;
  
  
  
  // Y rotation axis   AZIMUTH  ********************

  if (((int) event.orientation.x) < ytarget -3 ) {
    up = ytarget - (int) event.orientation.x; //calculate shortest trans-meridian path to target
    down = (360 - ytarget) + (int) event.orientation.x;

    if (up < down) {
      digitalWrite(dirPin, LOW); //set direction
        for (int y = 1; y < 10; y = y + 1) {
           digitalWrite(stepPin, HIGH);
           delay(6);
           digitalWrite(stepPin, LOW);
           delay(6);

 }
    } else {
      digitalWrite(dirPin, HIGH); //set direction
         for (int y = 1; y < 10; y = y + 1) {
           digitalWrite(stepPin, HIGH);
           delay(6);
           digitalWrite(stepPin, LOW);
           delay(6);
         }  
    }
  }

  if (((int) event.orientation.x) > ytarget +3 ) {
    up = (360 - (int) event.orientation.x) + ytarget; //calculate shortest trans-meridian path to target
    down = (int) event.orientation.x - ytarget;

    if (up < down) {
      digitalWrite(dirPin, LOW); //Clockwise direction.
        for (int y = 1; y < 10; y = y + 1) {
           digitalWrite(stepPin, HIGH);
           delay(6);
           digitalWrite(stepPin, LOW);
           delay(6);
         }
    } else {
      digitalWrite(dirPin, HIGH); //Counter clockwise direction.
        for (int y = 1; y < 10; y = y + 1) {
           digitalWrite(stepPin, HIGH);
           delay(6);
           digitalWrite(stepPin, LOW);
           delay(6);
         }

      
    }
  }

  // X axis ELEVATION  ***********************
  Xcurrent = ((int) event.orientation.z +90);
  Serial.println(Xcurrent);
  if (Xcurrent < xtarget - 3) {
    xpos -= 1;
    if (xpos < 1) {
      xpos = 1;
    }

  }

  if (Xcurrent > xtarget + 3) {
    xpos += 1;
    if (xpos > 150) {
      xpos = 150;
    }

  }

  
  // **Check for overload**
  current_mA=current_mA*-1; //convert power reading to positive number
  if ((int)current_mA > 1000) // if overload detected
  {
    olcount+=1;
                if (olcount > 4) {
                digitalWrite(7,1); //shut down power to servos
    delay(1000);
                }
  }
  else
  {
    digitalWrite(7,0); // Turn power on to servos
                olcount=0;
  }
  
}




