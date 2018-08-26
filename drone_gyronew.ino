#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int motor1 = 9;
int motor2 = 6;
int motor3 = 11;
int motor4 = 10;
int led = 13;
int calib = 0;
int count = 0;
int a;
int b;
int val;
int val2;
void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
      pinMode(motor1, OUTPUT);
      pinMode(motor2, OUTPUT);
      pinMode(motor3, OUTPUT);
      pinMode(motor4, OUTPUT);
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
    analogWrite(motor1, 0);
    analogWrite(motor2, 0);
    analogWrite(motor3, 0);
    analogWrite(motor4, 0);
    digitalWrite(led, LOW);
    delay(6000);

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
      delay(1000);
      a = az;
      b = ay;
      calib++;
    }
    if(count<400){
        if((az < a - 600) && (ay < b - 1000)){
            val = map(az,a-600,a-6000,220,75);
            val2 = map(ay,b-1000,b-7000,220,75);
            analogWrite(motor1, val2);
            analogWrite(motor2, val);
            analogWrite(motor3, 255);
            analogWrite(motor4, 255);
            digitalWrite(led, HIGH);
        }
        else if((az < a - 600) && (ay > b + 1000)){
            val = map(az,a-600,a-6000,220,75);
            val2 = map(ay,b+1000,b+7000,220,75);
            analogWrite(motor1, 255);
            analogWrite(motor2, val);
            analogWrite(motor3, val2);
            analogWrite(motor4, 255);
            digitalWrite(led, HIGH);
        }
        else if((az > a + 600) && (ay < b - 1000)){
            val = map(az,a+600,a+6000,220,75);
            val2 = map(ay,b-1000,b-7000,220,75);
            analogWrite(motor1, val2);
            analogWrite(motor2, 255);
            analogWrite(motor3, 255);
            analogWrite(motor4, val);
            digitalWrite(led, HIGH);
        }
        else if((az > a + 600) && (ay > b + 1000)){
            val = map(az,a+600,a+6000,220,75);
            val2 = map(ay,b+1000,b+7000,220,75);
            analogWrite(motor1, 255);
            analogWrite(motor2, 255);
            analogWrite(motor3, val2);
            analogWrite(motor4, val);
            digitalWrite(led, HIGH);
        }
        else if(az < a - 600){   /*-7500*/
            val = map(az,a-600,a-6000,220,75);
            analogWrite(motor1, 255);
            analogWrite(motor2, val);
            analogWrite(motor3, 255);
            analogWrite(motor4, 255);
            digitalWrite(led, HIGH);
        }
        else if(az > a + 600){    /*-4000*/
            val = map(az,a+600,a+6000,220,75);
            analogWrite(motor1, 255);
            analogWrite(motor2, 255);
            analogWrite(motor3, 255);
            analogWrite(motor4, val);
            digitalWrite(led, HIGH);    
        }
        else if(ay < b - 1000){        /*-2100*/
            val2 = map(ay,b-1000,b-7000,220,75);
            analogWrite(motor1, val2);
            analogWrite(motor2, 255);
            analogWrite(motor3, 255);
            analogWrite(motor4, 255);
            digitalWrite(led, HIGH);     
        }
        else if(ay > b + 1000){       /*4600*/
            val2 = map(ay,b+1000,b+7000,220,75);
            analogWrite(motor1, 255);
            analogWrite(motor2, 255);
            analogWrite(motor3, val2);
            analogWrite(motor4, 255);
            digitalWrite(led, HIGH);
        }
        else{
            analogWrite(motor1, 255);
            analogWrite(motor2, 255);
            analogWrite(motor3, 255);
            analogWrite(motor4, 255);
            digitalWrite(led, HIGH);
        }
    }
    else if(count >= 400 && count<650){
        if((az < a - 600) && (ay < b - 1000)){
            val = map(az,a-600,a-6000,160,50);
            val2 = map(ay,b-1000,b-7000,160,50);
            analogWrite(motor1, val2);
            analogWrite(motor2, val);
            analogWrite(motor3, 180);
            analogWrite(motor4, 180);
            digitalWrite(led, HIGH);
        }
        else if((az < a - 600) && (ay > b + 1000)){
            val = map(az,a-600,a-6000,160,50);
            val2 = map(ay,b+1000,b+7000,160,50);
            analogWrite(motor1, 180);
            analogWrite(motor2, val);
            analogWrite(motor3, val2);
            analogWrite(motor4, 180);
            digitalWrite(led, HIGH);
        }
        else if((az > a + 600) && (ay < b - 1000)){
            val = map(az,a+600,a+6000,160,50);
            val2 = map(ay,b-1000,b-7000,160,50);
            analogWrite(motor1, val2);
            analogWrite(motor2, 180);
            analogWrite(motor3, 180);
            analogWrite(motor4, val);
            digitalWrite(led, HIGH);
        }
        else if((az > a + 600) && (ay > b + 1000)){
            val = map(az,a+600,a+1000,160,50);
            val2 = map(ay,b+1000,b+7000,160,50);
            analogWrite(motor1, 180);
            analogWrite(motor2, 180);
            analogWrite(motor3, val2);
            analogWrite(motor4, val);
            digitalWrite(led, HIGH);
        }
      else if(az < a - 600){   /*-7500*/
            val = map(az,a-600,a-6000,160,50);
            analogWrite(motor1, 180);
            analogWrite(motor2, val);
            analogWrite(motor3, 180);
            analogWrite(motor4, 180);
            digitalWrite(led, HIGH);
        }
        else if(az > a + 600){    /*-4000*/
            val = map(az,a+600,a+6000,160,50);
            analogWrite(motor1, 180);
            analogWrite(motor2, 180);
            analogWrite(motor3, 180);
            analogWrite(motor4, val);
            digitalWrite(led, HIGH);    
        }
        else if(ay < b - 1000){        /*-2100*/
            val2 = map(ay,b-1000,b-7000,160,50);
            analogWrite(motor1, val2);
            analogWrite(motor2, 180);
            analogWrite(motor3, 180);
            analogWrite(motor4, 180);
            digitalWrite(led, HIGH);     
        }
        else if(ay > b + 1000){       /*4600*/
            val2 = map(ay,b+1000,b+7000,160,50);
            analogWrite(motor1, 180);
            analogWrite(motor2, 180);
            analogWrite(motor3, val2);
            analogWrite(motor4, 180);
            digitalWrite(led, HIGH);
        }
        else{
            analogWrite(motor1, 180);
            analogWrite(motor2, 180);
            analogWrite(motor3, 180);
            analogWrite(motor4, 180);
            digitalWrite(led, HIGH);
        }
    }
    else if(count>=650){
      analogWrite(motor1, 0);
      analogWrite(motor2, 0);
      analogWrite(motor3, 0);
      analogWrite(motor4, 0);
      digitalWrite(led, LOW);
    }
    count++;
    delay(20);

}
