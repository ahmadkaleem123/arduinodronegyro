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
float p[3] = {1,0,0};   /*3.55,0.005,2.05*/                    /*p, i ,d*/
float pid[2];/*Change these values looking at what each of these is doing to the calculated angle!  ADD TWIDDLE*/
float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;
int b;
/*int first = 0;*/
/*int right;
int up;*/
double vall = 200;
double valb = 200;
double valr = 200;
double valf = 200;
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
    time = millis();
    delay(7000);

}

void loop() {
    while(time<15000){
      // read raw accel/gyro measurements from device
        timePrev = time;
        time = millis();
        elapsedTime = (time - timePrev)/1000;
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        Acc_angle[0] = atan((ay/16384.0)/sqrt(pow((ax/16384.0),2) + pow((az/16384.0),2)))*rad_to_deg;
         /*---Y---*/
         Acc_angle[1] = atan(-1*(ax/16384.0)/sqrt(pow((ay/16384.0),2) + pow((az/16384.0),2)))*rad_to_deg;
    
        // these methods (and a few others) are also available
        //accelgyro.getAcceleration(&ax, &ay, &az);
        //accelgyro.getRotation(&gx, &gy, &gz);
    
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
       /*---Y---*/
        Gyro_angle[1] = gy/131.0;
        /*---X axis angle---*/
        Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acc_angle[0];
       /*---Y axis angle---*/
        Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acc_angle[1];
        if(time<1000){
          initx = Total_angle[0];
          inity = Total_angle[1];
        }
        else{
          cte[0] = Total_angle[0] - initx;
          cte[1] = Total_angle[1] - inity;
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
          
          pid[0] = prop_cte[0] + int_cte[0] + diff_cte[0];
          pid[1] = prop_cte[1] + int_cte[1] + diff_cte[1];
          motormove(pid[0], pid[1]); 
              /*Add angle stuff over here*/
         /* vall = int(vall);
          valb = int(valb);
          valr = int(valr);
          valf = int(valf);*/
          analogWrite(motorl,int(vall));
          analogWrite(motorb,int(valb));
          analogWrite(motorr,int(valr));
          analogWrite(motorf,int(valf));
          digitalWrite(led, HIGH);
          //Serial.print("  Back: ");
          Serial.println(valr);
          /*Serial.print("  Front: ");
          Serial.print(valf);
          Serial.print("  Left: ");
          Serial.print(vall);
          Serial.print("  Right: ");
          Serial.println(valr);*/
          prev_cte[0] = cte[0];
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
    valr = valr + 0.01 * angle1;                /* Fix the way the angle is calcualted. Max = 255!!!!!!!!!!*/
    vall = vall - 0.01 * angle1;
    valf = valf + 0.01 * angle2;
    valb = valb - 0.01 * angle2;
    
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
    if(valr<100){
       valr = 100;
    }
    if(vall<100){
       vall = 100;
    }
    if(valf<100){
       valf = 100;
    }
    if(valb<100){
       valb = 100;
    }
}

