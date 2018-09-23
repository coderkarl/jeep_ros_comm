#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

unsigned long serialdata;
int inbyte;
int pinNumber;
int sensorVal;
int digitalState;
bool accelReady = true, gyroReady = false, magReady = true;

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup()
{
  Serial.begin(115200);
  /* Initialise the sensors */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    // Send CAN ERROR MESSAGE, Software Reset, continue, do NOT use while(1)
    //while(1);
  }
  else
  {
    delay(200);
    bno.setExtCrystalUse(true);
    gyroReady = true;
  }
}

void loop()
{
  
  if(Serial.read() == 'A')
  {
    getSerial();
    switch(serialdata)
    {
      case 5: // A5/
      {
        //IMU data
        //sensors_event_t event;
        getSerial();
        switch(serialdata) //A5-1
        {
          case 1: // A5/1/ accel X, Y, Z
          {
            imu::Vector<3> grav_accel = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
            int temp_ax = (grav_accel.x()+30)*100;
            Serial.println(temp_ax);
            int temp_ay = (grav_accel.y()+30)*100;
            Serial.println(temp_ay);
            int temp_az = (grav_accel.z()+30)*100;
            Serial.println(temp_az);
            break;
          }
          case 2: // A5/2/ gyro X,Y,Z
          {
            imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
            int temp_gx = (gyro.x()+10)*100;
            Serial.println(temp_gx);
            int temp_gy = (gyro.y()+10)*100;
            Serial.println(temp_gy);
            int temp_gz = (gyro.z()+10)*100;
            Serial.println(temp_gz);
            break;
          }   
          case 6: // A5/6/ gyro Z
          {
            imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
            int temp_gz = (gyro.z()+10)*100;
            Serial.println(temp_gz);
            break;
          }
          case 7: // A5/7/ compass heading
          {
            sensors_event_t event; 
            bno.getEvent(&event);
            float heading = (float)event.orientation.x;
            //float y = (float)event.orientation.y;
            //float z = (float)event.orientation.z;
            /*Serial.print("orientation x, y, z: ");
            Serial.print(heading );
            Serial.print(", ");
            Serial.print(y );
            Serial.print(", ");
            Serial.println(z );*/
            //if(heading >= 180.0)
            //{
            //    heading -= 360.0;
            //}
            int headingDeciDeg = int(heading*10.0); //centi-deg +/-180 deg
            Serial.println(headingDeciDeg);
            break;
          }
          case 8: // A5/8/ mag X,Y,Z
          {
            uint8_t system_cal, gyro_cal, accel_cal, mag_cal;
            system_cal = gyro_cal = accel_cal = mag_cal = 0;
            bno.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);
            Serial.println(mag_cal);
            imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
            int temp_mx = (mag.x()+0)*10;
            Serial.println(temp_mx);
            int temp_my = (mag.y()+0)*10;
            Serial.println(temp_my);
            int temp_mz = (mag.z()+0)*10;
            Serial.println(temp_mz);
            break;
          }   
        } //switch serial data inner
        break; //case 5 //A5
      }
    } //switch serialdata outer
  } // if Serial.read == 'A'
} //void loop

long getSerial()
{
  serialdata = 0;
  while (inbyte != '/')
  {
    inbyte = Serial.read(); 
    if (inbyte > 0 && inbyte != '/')
    {
      serialdata = serialdata * 10 + inbyte - '0';
    }
  }
  inbyte = 0;
  return serialdata;
}
