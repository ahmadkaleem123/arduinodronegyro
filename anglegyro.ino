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
int a;
int b;
int first = 0;
int right;
int up;
int vall = 255;
int valb = 255;
int valr = 255;
int valf = 255;
float angle;
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
      if(first == 0){
        digitalWrite(led,HIGH);
        delay(2000);
        a = az;
        b = ay;
        first++;
        Serial.println(a);
        Serial.println(b);
        digitalWrite(led,LOW);
        delay(1000);
      }
      else if(first==1){
        digitalWrite(led,HIGH);
        delay(2000);
        right = ay;
        right = right - b;
        right = abs(right);
        first++;
        Serial.println(right);
        digitalWrite(led,LOW);
        delay(1000);
        
      }
      else if(first==2){     /* The angle of tilting both left and right is 45*/
        digitalWrite(led,HIGH);  
        delay(2000);
        up = az;
        up = up  - a;  
        up = abs(up);
        Serial.println(up);
        calib++;
        digitalWrite(led,LOW);
        if(up==0 || right==0){ /*Warning if gyro connections are not correct*/
           digitalWrite(led,HIGH);
           delay(200);
           digitalWrite(led,LOW);
           delay(200);
           digitalWrite(led,HIGH);
           delay(200);
           digitalWrite(led,LOW);
           delay(200);
           digitalWrite(led,HIGH);
           delay(200);
           digitalWrite(led,LOW);
           delay(200);
           digitalWrite(led,HIGH);
           delay(200);
           digitalWrite(led,LOW);
           delay(200);
           digitalWrite(led,HIGH);
           delay(200);
           digitalWrite(led,LOW);
           delay(200);
        }
        delay(2500);
      }
    }
    else{
      if(count < 500){
        vall = 255;
        valb = 255;
        valr = 255;
        valf = 255;
        if(az < a - (up/5)){
          /*Serial.println(az);*/  
          angle = az-a;
          angle = angle/up;
          angle = angle*30;
          angle = abs(angle);
          angle = round(angle);
          /*Serial.println(angle);*/
          valb = map(angle,6,30,200,30);
        }
        if(az > a + (up/5)){
          angle = az-a;
          angle = angle/up;
          angle = angle*30;
          angle = abs(angle);
          angle = round(angle);
          valf = map(angle,6,30,200,30);
        }
        if(ay < b - (right/5)){
          angle = ay-b;
          angle = angle/right;
          angle = angle*30;
          angle = abs(angle);
          angle = round(angle);
          vall = map(angle,6,30,200,30);
        }
        if(ay > b + (right/5)){
          angle = ay-b;
          angle = angle/right;
          angle = angle*30;
          angle = abs(angle);
          angle = round(angle);
          valr = map(angle,6,30,200,30);
        }
        analogWrite(motorl,vall);
        analogWrite(motorb,valb);
        analogWrite(motorr,valr);
        analogWrite(motorf,valf);
        digitalWrite(led, HIGH);
        Serial.print("  Back: ");
        Serial.print(valb);
        Serial.print("  Front: ");
        Serial.print(valf);
        Serial.print("  Left: ");
        Serial.print(vall);
        Serial.print("  Right: ");
        Serial.println(valr);
        
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
