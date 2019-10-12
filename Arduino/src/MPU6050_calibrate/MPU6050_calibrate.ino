#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <Wire.h>
#endif
//#define LATITUDE -33.95 //Cape Town, South Africa //Enable if want to get units in m.s^-2 
/**
 * Sketch to calibrate MPU6050 using method presented in doi: 10.1109/sii.2016.7844027
 * Uses MPU6050 and I2C libraries by Jeff Rowberg at: https://github.com/jrowberg/i2cdevlib
 *  ============================================
 *  MPU6050 calibration code is placed under the MIT license
 *  Copyright (c) 2019 Yusuf Heylen
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  the above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
*/
//double g = 9.806 - 0.5*(9.832 -  9.780)*cos(2*(LATITUDE)*PI/(180)); //Enable if want to get units in m.s^-2
double unitConversionFactor;  //This converts the raw readings into values of 'g;

typedef struct
{
	double Ax;
	double Ay;
	double Az;
}Accel_Reading;

MPU6050 imu;
int16_t raw_Ax, raw_Ay, raw_Az;
String input = "";
uint8_t n;


int8_t accel_full_scale_range = MPU6050_ACCEL_FS_2;  //See MPU6050.h for the different scale ranges available 



unsigned long time; 
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  switch (accel_full_scale_range)
	{
		case MPU6050_ACCEL_FS_2:
		  unitConversionFactor = (1/1000.0f)*(2000.0f/32768.0f);  //Gives units in 'g' - use (g/1000.0f)*(2000.0f/32768.0f) to get units in m.s^-2
		  break;
		
		case MPU6050_ACCEL_FS_4:
		  unitConversionFactor = (1/1000.0f)*(4000.0f/32768.0f);
		  break;
		
		case MPU6050_ACCEL_FS_8:
			unitConversionFactor = (1/1000.0f)*(8000.0f/32768.0f);
			break;
		
		case MPU6050_ACCEL_FS_16:
			unitConversionFactor = (1/1000.0f)*(16000.0f/32768.0f);
			break;
		
		default:
			break;
	}
  Serial.begin(115200);

  Serial.println("Calibration routine for MPU6050");
  Serial.println("Initializing...");

  imu.setClockSource(MPU6050_CLOCK_INTERNAL);  //Less accurate than using MPU6050_CLOCK_PLL_XGYRO or external but uses less power, change if gyro's will be used
  imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); //Change depending on usage 
  imu.setSleepEnabled(false);

  Serial.println("Testing device connections...");
  Serial.println(imu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");  
  Serial.println("NOTE: Wait at least 5 - 15 min for device to warm up");
//  Serial.println("Value of g used in Latitude " + String(LATITUDE) + " is " + String(g,10));
//  Serial.println("NOTE: Change LATITUDE definition for approximate location device will be used in");
  Serial.println("Save averaged values MANUALLY and use with accelCalibration.py to calculate SEM parameters");
  Serial.println("---------------------------------------------------------------------------------");

  Serial.println("Enter number of rotations to be done:");
  while(input.equals("")) 
  {
    if( Serial.available() > 0) 
    {
      input = Serial.readString();
      Serial.println("Number of rotations entered: " + String(input.toInt() ));
      if(input.toInt() == 0)
      {
        Serial.println("Invalid Argument! Retry...");
        input = "";
        flush();
      }
    } 
  }
  n = input.toInt();


  Accel_Reading avg[n];
  calibrate(avg);
  Serial.println("---------------------------------------------------------------------------------");
  for(uint8_t row = 0; row < n; ++row)
  {
    Serial.print( (avg[row]).Ax, 11 ); Serial.print(","); Serial.print((avg[row]).Ay , 11); Serial.print(","); Serial.print((avg[row]).Az , 11); Serial.println();
  }

}
void flush()
{
  while (Serial.available())
  {
    Serial.read();
  };
}

void calibrate(Accel_Reading *average_values)
{
  for(int i = 0; i < n; ++i)
  {

    flush();
    String input = "";
    // Serial.println("Rotation Number: " + String(i+1));
    Serial.println("Send '1' once device has been set in new stationary orientation to continue...");
    while(!(input.toInt() == 1))
    {
      if(Serial.available() > 0)
      {
        input = Serial.readString(); 
        // Serial.println("You entered: " + String(int(input.toInt())) );
        if(input.toInt() == 1 ) 
        {
          // Serial.flush();
          Serial.print("Busy");
          Serial.flush();
          long sum_Ax = 0, sum_Ay = 0, sum_Az = 0;
          for(int16_t k = 0; k < 10000; ++k) 
          {
            if(k%1000 == 0)
            {
              Serial.print(".");
              Serial.flush();
            }//track progress
            imu.getAcceleration(&raw_Ax, &raw_Ay, &raw_Az);
            sum_Ax += raw_Ax;
            sum_Ay += raw_Ay;
            sum_Az += raw_Az;
          }
          Serial.println("Done!");
          if(i == 0){
            (average_values[i]).Ax = 0;
          }//Initialize first one, hack fix
            (average_values[i]).Ax = ((sum_Ax+0.0d)/10000)*unitConversionFactor;
            (average_values[i]).Ay = ((sum_Ay+0.0d)/10000)*unitConversionFactor; 
            (average_values[i]).Az = ((sum_Az+0.0d)/10000)*unitConversionFactor;

        }
      }
    }
  } 
}

void loop() {

}
 
