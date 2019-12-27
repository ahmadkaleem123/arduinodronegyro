#include <math.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int motorl = 9;
int motorb = 6;
int motorr = 5;
int motorf = 3;
int led = 13;
float initx;
float inity;
float int_cte[2] = {0.0, 0.0};
float cte[2];
float diff_cte[2];
float prev_cte[2] = {0, 0};
float prop_cte[2];
float Acc_angle[2];
float Gyro_angle[2];
float Total_angle[2];
float px[3] = {2.75, 0, 5};   /*3.55,0.005,2.05*/                    /*p, i ,d*/ /* Make seperate PID ones for each side of rotation*/
float py[3] = {2.75, 0, 5};  // PID constants for both axes
float pid[2];
float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180 / 3.141592654;
int b;
int count = 0;
float vall = 200;                          //Starting values for each of the four motors 
float valb = 200;
float valr = 200;
float valf = 200;
float angle;
int first = 0;
float throttle = 200;              //Throttle(PID value is added or subtracted from this value at each cycle of the loop 
void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  pinMode(motorl, OUTPUT);
  pinMode(motorb, OUTPUT);
  pinMode(motorr, OUTPUT);
  pinMode(motorf, OUTPUT);
  pinMode(led, OUTPUT);
  // join I2C bus (I2Cdev library doesn't do this automatically)


  // initialize serial communication
  Serial.begin(38400);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  analogWrite(motorl, 0);
  analogWrite(motorb, 0);
  analogWrite(motorr, 0);
  analogWrite(motorf, 0);
  digitalWrite(led, LOW);
  time = millis();
  delay(5000);      //Wait 5 seconds before starting the motors. 

}

void loop() {
  while (time < 20000) {
    // read raw accel/gyro measurements from device
    timePrev = time;
    time = millis();
    elapsedTime = (time - timePrev) / 1000;
    //Serial.println(elapsedTime);
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Acc_angle[0] = atan((ay / 16384.0) / sqrt(pow((ax / 16384.0), 2) + pow((az / 16384.0), 2))) * rad_to_deg; // X Axis Angle
    Acc_angle[1] = atan(-1 * (ax / 16384.0) / sqrt(pow((ay / 16384.0), 2) + pow((az / 16384.0), 2))) * rad_to_deg; // Y Axis Angle
    Gyro_angle[0] = gx / 131.0;
    Gyro_angle[1] = gy / 131.0;
    Total_angle[0] = 0.98 * (Total_angle[0] + Gyro_angle[0] * elapsedTime) + 0.02 * Acc_angle[0]; // X Axis Total Angle
    Total_angle[1] = 0.98 * (Total_angle[1] + Gyro_angle[1] * elapsedTime) + 0.02 * Acc_angle[1]; // Y Axis Total Angle
    //Serial.println(Total_angle[1]);
    // Calibration to determine both initial angles at rest
    if (count < 2000) {
      if (count == 1999) {
        initx = Total_angle[0];
        inity = Total_angle[1];
        Serial.println(initx);
        Serial.println(inity);
      }
      count++;
    }                        // Calibrates for some time to ensure that the calibrated value is stable
    else {
      // PID Control Calculations
      cte[0] = Total_angle[0] - initx;     // Cross Track Error in both axes. (Difference between current angle and the initial calibrated angle).
      cte[1] = Total_angle[1] - inity;
      /*Serial.print("Initial");
        Serial.println(initx);
        Serial.println(Total_angle[0]);*/
      prop_cte[0] = px[0] * cte[0];      // P Value for PID
      //Serial.println(prop_cte[0]);
      prop_cte[1] = py[0] * cte[1];
      if (-2 < cte[0] < 2)
      {
        int_cte[0] = int_cte[0] + (px[1] * cte[0]);    //I Value
      }
      //Serial.println(int_cte[0]);
      if (-2 < cte[1] < 2)
      {
        int_cte[1] = int_cte[1] + (py[1] * cte[1]);
      }
      if(int(time) % 50 == 0){
        diff_cte[0] = px[2] * ((cte[0] - prev_cte[0])); /*/elapsedTime);*/   // D value found every 50 ms
        diff_cte[1] = py[2] * ((cte[1] - prev_cte[1])); /*/elapsedTime);*/
        prev_cte[0] = cte[0];            // The previous cross track error is now assigned to the current for the next cycle of the loop
        prev_cte[1] = cte[1];
      }
      //Serial.println(diff_cte[0]);

      // Final PID as a sum of the three components for each axis
      pid[0] = prop_cte[0] + int_cte[0] + diff_cte[0]; // X Axis
      //Serial.print(pid[0]);
      //Serial.print(" ");
      pid[1] = prop_cte[1] + int_cte[1] + diff_cte[1];   // Y Axis
      //Serial.print(pid[1]);
      //Serial.print(" ");
      motormove(pid[0], pid[1]);      // Function to calculate the PWM value to be assigned to each motor. 
      //Serial.println(valr);
      
      analogWrite(motorl, int(vall));    // Giving an analog PWM signal to each of the four motors based on the values calculated by the function motormove.
      analogWrite(motorb, int(valb));
      analogWrite(motorr, int(valr));
      analogWrite(motorf, int(valf));
      digitalWrite(led, HIGH);
      Serial.print(int(valb));
      Serial.print(" ");
      Serial.print(int(valf));
      Serial.print(" ");
      Serial.print(int(vall));
      Serial.print(" ");
      Serial.println(int(valr));
    }

  }
  analogWrite(motorl, 0);
  analogWrite(motorb, 0);
  analogWrite(motorr, 0);
  analogWrite(motorf, 0);
  digitalWrite(led, LOW);
}
float motormove(float angle1, float angle2) {
  valr = throttle + angle1 + angle2;                // Function which calculates the PWM value to be assigned to each motor by adding or subtracting the PID value for each axis to the throttle. 
  vall = throttle - angle1 - angle2;                
  valf = throttle - angle1 + angle2;
  valb = throttle + angle1 - angle2;

  if (valr > 255) {                                // Conditions to ensure that the value assigned to the motors is not higher than 255(maximum throttle) and not lower than 145 (minimum throttle). 
    valr = 255;                                    
  }
  if (vall > 255) {
    vall = 255;
  }
  if (valf > 255) {
    valf = 255;
  }
  if (valb > 255) {
    valb = 255;
  }
  if (valr < 145) {
    valr = 145;
  }
  if (vall < 145) {
    vall = 145;
  }
  if (valf < 145) {
    valf = 145;
  }
  if (valb < 145) {
    valb = 145;
  }
}

