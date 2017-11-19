#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <TimerOne.h>
#include "Adafruit_SI1145.h"
#include <Adafruit_BMP280.h>


#define MPU9250_ADDRESS 0x68
#define MAG_ADDRESS 0x0C
 
#define GYRO_FULL_SCALE_250_DPS 0x00 
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18
 
#define ACC_FULL_SCALE_2_G 0x00 
#define ACC_FULL_SCALE_4_G 0x08
#define ACC_FULL_SCALE_8_G 0x10
#define ACC_FULL_SCALE_16_G 0x18
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10
 
Adafruit_ADS1015 ads;     /* Use thi for the 12-bit version */
Adafruit_SI1145 uv = Adafruit_SI1145();
Adafruit_BMP280 bmp; 

void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
// Set register address
Wire.beginTransmission(Address);
Wire.write(Register);
Wire.endTransmission();
 
// Read Nbytes
Wire.requestFrom(Address, Nbytes); 
uint8_t index=0;
while (Wire.available())
Data[index++]=Wire.read();
}
 
 
// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
// Set register address
Wire.beginTransmission(Address);
Wire.write(Register);
Wire.write(Data);
Wire.endTransmission();
}
 
// Initial time
long int ti;
volatile bool intFlag=false;
 
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;
unsigned long previousMillis=0;
unsigned long previousMillis2=0;
void setup(void) 
{
  Wire.begin();
Serial.begin(115200);
  Serial.println("Hello!");
  // Set accelerometers low pass filter at 5Hz
I2CwriteByte(MPU9250_ADDRESS,29,0x06);
// Set gyroscope low pass filter at 5Hz
I2CwriteByte(MPU9250_ADDRESS,26,0x06);
 
 
// Configure gyroscope range
I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);
// Configure accelerometers range
I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);
// Set by pass mode for the magnetometers
I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
 
// Request continuous magnetometer measurements in 16 bits
I2CwriteByte(MAG_ADDRESS,0x0A,0x16);
 
pinMode(13, OUTPUT);
Timer1.initialize(10000); // initialize timer1, and set a 1/2 second period
Timer1.attachInterrupt(callback); // attaches callback() as a timer overflow interrupt
 
  
  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");
  
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  
  ads.begin();
}
// Counter
long int cpt=0;
 
void callback()
{ 
intFlag=true;
digitalWrite(13, digitalRead(13) ^ 1);
}
void loop(void ) 
{
  unsigned long currentMillis=millis();
  
  if(currentMillis - previousMillis2 >=2000){
     Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
    
    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1013.25)); // this should be adjusted to your local forcase
    Serial.println(" m");
    
    Serial.println();
     previousMillis2 = currentMillis;
    }
  if (currentMillis - previousMillis >= 1000) {
 int16_t adc0;

  adc0 = ads.readADC_SingleEnded(0);
  //Serial.print("AIN0: "); Serial.println(adc0);
  voMeasured = adc0*0.1875; // read the dust value
 

  // 0 - 3.3V mapped to 0 - 1023 integer values
  // recover voltage
  calcVoltage = voMeasured;
 

  dustDensity = 0.17 * calcVoltage - 0.1;
 
  Serial.print("Raw Signal Value (0-1023): ");
  Serial.print(voMeasured);
 
  Serial.print(" - Voltage: ");
  Serial.print(calcVoltage);
 
  Serial.print(" - Dust Density: ");
  Serial.println(dustDensity);
     //SI1145
Serial.print("Vis: "); Serial.println(uv.readVisible());
  Serial.print("IR: "); Serial.println(uv.readIR());
  
  // Uncomment if you have an IR LED attached to LED pin!
  //Serial.print("Prox: "); Serial.println(uv.readProx());

  float UVindex = uv.readUV();
  // the index is multiplied by 100 so to get the
  // integer index, divide by 100!
  UVindex /= 100.0;  
  Serial.print("UV: ");  Serial.println(UVindex);
    previousMillis = currentMillis;
  }

  
  
  //Accelerometer
  if(millis()%100==0){
  while (!intFlag);
intFlag=false;
 
// Display time
Serial.print (millis()-ti,DEC);
Serial.print ("\t");
 
 
// _______________
// ::: Counter :::
 
// Display data counter
// Serial.print (cpt++,DEC);
// Serial.print ("\t");
 
 
 
// ____________________________________
// ::: accelerometer and gyroscope :::
 
// Read accelerometer and gyroscope
uint8_t Buf[14];
I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
 
// Create 16 bits values from 8 bits data
 
// Accelerometer
int16_t ax=-(Buf[0]<<8 | Buf[1]);
int16_t ay=-(Buf[2]<<8 | Buf[3]);
int16_t az=Buf[4]<<8 | Buf[5];
 
// Gyroscope
int16_t gx=-(Buf[8]<<8 | Buf[9]);
int16_t gy=-(Buf[10]<<8 | Buf[11]);
int16_t gz=Buf[12]<<8 | Buf[13];
 
// Display values
 
// Accelerometer
Serial.print (ax,DEC); 
Serial.print ("\t");
Serial.print (ay,DEC);
Serial.print ("\t");
Serial.print (az,DEC); 
Serial.print ("\t");
 
// Gyroscope
Serial.print (gx,DEC); 
Serial.print ("\t");
Serial.print (gy,DEC);
Serial.print ("\t");
Serial.print (gz,DEC); 
Serial.print ("\t");
 
 
// _____________________
// ::: Magnetometer :::
 
 
// Read register Status 1 and wait for the DRDY: Data Ready
 
uint8_t ST1;
do
{
I2Cread(MAG_ADDRESS,0x02,1,&ST1);
}
while (!(ST1&0x01));
 
// Read magnetometer data 
uint8_t Mag[7]; 
I2Cread(MAG_ADDRESS,0x03,7,Mag);
 
// Create 16 bits values from 8 bits data
 
// Magnetometer
int16_t mx=-(Mag[3]<<8 | Mag[2]);
int16_t my=-(Mag[1]<<8 | Mag[0]);
int16_t mz=-(Mag[5]<<8 | Mag[4]);
 
 
// Magnetometer
Serial.print (mx+200,DEC); 
Serial.print ("\t");
Serial.print (my-70,DEC);
Serial.print ("\t");
Serial.print (mz-700,DEC); 
Serial.print ("\t");
 
 
 
// End of line
Serial.println("");
}
 
  
  
}
