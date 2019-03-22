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
int motorf = 3;     /* Pins 11 and 10 should be changed to 5 and3*/
int led = 13;
int calib = 0;
int count = 0;
int initx;
int inity;
double angley;
double anglex;
int int_cte1;
int int_cte2;
int cte1;
int cte2;
int diff_cte1;
int diff_cte2;
int prev_cte1;
int prev_cte2;
double tau_p = 0.002;
double tau_d = 0.03;
double tau_i = 0.0004;                 /*Change these values looking at what each of these is doing to the calculated angle! Check values which Thrun used Try to optimize these values*/

int b;
/*int first = 0;*/
/*int right;
int up;*/
double vall = 220;
double valb = 220;
double valr = 220;
double valf = 220;
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
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
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
    delay(5000);

}

void loop() {
  // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO
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
    #endif
    if (calib == 0){
        digitalWrite(led,HIGH);
        delay(2000);
        initx = ax;
        inity = ay;
        calib++;
        Serial.println(initx);
        Serial.println(inity);
        digitalWrite(led,LOW);
        delay(1000);
    }
    else{
      if(count < 500 && first == 0){
        vall = 220;
        valb = 220;
        valr = 220;
        valf = 220;
        int_cte1 = 0;
        int_cte2 = 0;
        cte1 = ay - inity;
        prev_cte1 = cte1;
        prev_cte2 = cte2;
        cte2 = ax - initx;
        int_cte1 = int_cte1 + cte1;
        int_cte2 = int_cte2 + cte2;
        diff_cte1 = cte1 - prev_cte1;
        diff_cte2 = cte2 - prev_cte2;
        prev_cte1 = cte1;
        prev_cte2 = cte2;
        angley = -tau_p * cte1 - tau_d * diff_cte1 - tau_i * int_cte1;
        Serial.println(angley);
        anglex = -tau_p * cte2 - tau_d * diff_cte2 - tau_i * int_cte2;                  /*YOu might have to use different constants for each angle!*/
        /*Add angle stuff over here. Make a function to calculate the speed of the motor given the angle!*/
        motormove(angley, anglex);
        vall = int(vall);
        valb = int(valb);
        valr = int(valr);
        valf = int(valf);
        analogWrite(motorl,vall);
        analogWrite(motorb,valb);
        analogWrite(motorr,valr);
        analogWrite(motorf,valf);
        digitalWrite(led, HIGH);
        /*Serial.print("  Back: ");
        Serial.print(valb);
        Serial.print("  Front: ");
        Serial.print(valf);
        Serial.print("  Left: ");
        Serial.print(vall);
        Serial.print("  Right: ");
        Serial.println(valr);*/
        first++;
      }
      if(count < 500 && first > 0){
        vall = 220;
        valb = 220;
        valr = 220;
        valf = 220;
        cte1 = ay - inity;
        cte2 = ax - initx;
        int_cte1 = int_cte1 + cte1;
        int_cte2 = int_cte2 + cte2;
        diff_cte1 = cte1 - prev_cte1;
        diff_cte2 = cte2 - prev_cte2;
        prev_cte1 = cte1;
        prev_cte2 = cte2;
        angley = -tau_p * cte1 - tau_d * diff_cte1 - tau_i * int_cte1;
        anglex = -tau_p * cte2 - tau_d * diff_cte2 - tau_i * int_cte2;
        Serial.println(angley);
        motormove(angley, anglex);
        vall = int(vall);
        valb = int(valb);
        valr = int(valr);
        valf = int(valf);
        /*Add angle stuff over here*/
        analogWrite(motorl,vall);
        analogWrite(motorb,valb);
        analogWrite(motorr,valr);
        analogWrite(motorf,valf);
        digitalWrite(led, HIGH);
       /* Serial.print("  Back: ");
        Serial.print(valb);
        Serial.print("  Front: ");
        Serial.print(valf);
        Serial.print("  Left: ");
        Serial.print(vall);
        Serial.print("  Right: ");
        Serial.println(valr);
        */
      }
      else{
        analogWrite(motorl,0);
        analogWrite(motorb,0);
        analogWrite(motorr,0);
        analogWrite(motorf,0);
        digitalWrite(led, LOW);
      }
      delay(20);
      count++;
    }
    
}
double motormove(double angle1, double angle2){
    if(angle1 > 0){
        valr = valr + 1.5 * angle1;                /* Fix the way the angle is calcualted. Max = 255!!!!!!!!!!*/
        vall = vall - 1.5 * angle1;
    }
    else if(angle1 < 0){
        valr = valr + 1.5 * angle1;
        vall = vall - 1.5 * angle1;
    }
    if(angle2 > 0){
        valf = valf - 1.5 * angle2;
        valb = valb + 1.5 * angle2;
    }
    else if(angle2 < 0){
        valf = valf - 1.5 * angle2;
        valb = valb + 1.5 * angle2;
    }
}

