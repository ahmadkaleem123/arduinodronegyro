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
float int_cte[2] = {0.0,0.0};
float cte[2];
float diff_cte[2];
float prev_cte[2] = {0, 0};
float prop_cte[2];
float Acc_angle[2];
float Gyro_angle[2];
float Total_angle[2];
float p[3] = {2,0.005,1};   /*3.55,0.005,2.05*/                    /*p, i ,d*/
float pid[2];
float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;    
int b;
int count = 0;
/*int first = 0;*/
/*int right;
int up;*/
float vall = 200;
float valb = 200;
float valr = 200;
float valf = 200;
float angle;
int first = 0;
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
    delay(5000);

}

void loop() {
    while(time<15000){
      // read raw accel/gyro measurements from device
        timePrev = time;
        time = millis();
        elapsedTime = (time - timePrev)/1000;
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        Acc_angle[0] = atan((ay/16384.0)/sqrt(pow((ax/16384.0),2) + pow((az/16384.0),2)))*rad_to_deg;    // X Axis Angle
         Acc_angle[1] = atan(-1*(ax/16384.0)/sqrt(pow((ay/16384.0),2) + pow((az/16384.0),2)))*rad_to_deg; // Y Axis Angle
    
        
    
      /*  #ifdef OUTPUT_READABLE_ACCELGYRO
            // display tab-separated accel/gyro x/y/z values
            Serial.print("a/g:\t");
            Serial.print(ax); Serial.print("\t");
            Serial.print(ay); Serial.print("\t");
            Serial.print(az); Serial.print("\t");
            Serial.print(gx); Serial.print("\t");
            Serial.print(gy); Serial.print("\t");
            Serial.println(gz);
        #endif
    
        #ifdef OUTPUT_BINARY_ACCELGYRO
            Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
            Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
            Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
            Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
            Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
            Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
        #endif*/
        Gyro_angle[0] = gx/131.0; 
        Gyro_angle[1] = gy/131.0;
        Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acc_angle[0];   // X Axis Angle
        Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acc_angle[1];   // Y Axis Angle

        // Calibration to determine both initial angles at rest 
        if(count<500){
          initx = Total_angle[0];     
          inity = Total_angle[1];
          count++;
        }
        else{
          // PID Control Calculations
          cte[0] = Total_angle[0] - initx;
          cte[1] = Total_angle[1] - inity;
          /*Serial.print("Initial");
          Serial.println(initx);
          Serial.println(Total_angle[0]);*/
          prop_cte[0] = p[0] * cte[0];
          prop_cte[1] = p[0] * cte[1];
          if(-3 <cte[0] <3)
          {
            int_cte[0]= int_cte[0]+(p[1]*cte[0]);  
          }
          if(-3 <cte[1] <3)
          {
            int_cte[1]= int_cte[1]+(p[1]*cte[1]);  
          }
          
          diff_cte[0] = p[2]*((cte[0] - prev_cte[0]));/*/elapsedTime);*/
          diff_cte[1] = p[2]*((cte[1] - prev_cte[1]));/*/elapsedTime);*/

          // Final PID as a sum of the three components for each axis
          pid[0] = prop_cte[0] + int_cte[0] + diff_cte[0];   // X Axis
          pid[1] = prop_cte[1] + int_cte[1] + diff_cte[1];   // Y Axis
          motormove(pid[0], pid[1]); 
          analogWrite(motorl,int(vall));     // Giving an analog PWM signal to each of the four motors based on the values calculated by the function motormove.
          analogWrite(motorb,int(valb));
          analogWrite(motorr,int(valr));
          analogWrite(motorf,int(valf));
          digitalWrite(led, HIGH);
          Serial.print("  Back: ");
          Serial.print(valb);
          Serial.print("  Front: ");
          Serial.print(valf);
          Serial.print("  Left: ");
          Serial.print(vall);
          Serial.print("  Right: ");
          Serial.println(valr);
          prev_cte[0] = cte[0];            // The previous cross track error is now assigned to the current for the next cycle of the loop
          prev_cte[1] = cte[1];
        }
        
}
  analogWrite(motorl,0);
  analogWrite(motorb,0);
  analogWrite(motorr,0);
  analogWrite(motorf,0);
  digitalWrite(led, LOW);
}
float motormove(float angle1, float angle2){
    valr = valr + 0.01 * angle1 + 0.01 * angle2;                /* Function to determine the next value of pwm output given to each motor based on the value of the pid variable for each axis */
    vall = vall - 0.01 * angle1 - 0.01 * angle2;
    valf = valf - 0.01 * angle1 + 0.01 * angle2;
    valb = valb + 0.01 * angle1 - 0.01 * angle2;
    
    if(valr>255){
       valr = 255;
    }
    if(vall>255){
       vall = 255;
    }
    if(valf>255){
       valf = 255;
    }
    if(valb>255){
       valb = 255;
    }
    if(valr<150){
       valr = 150;
    }
    if(vall<150){
       vall = 150;
    }
    if(valf<150){
       valf = 150;
    }
    if(valb<150){
       valb = 150;
    }
}

