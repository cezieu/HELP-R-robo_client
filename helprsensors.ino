#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <TimerOne.h>
#include "Adafruit_SI1145.h"
#include <Adafruit_BMP280.h>
#include<math.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Servo.h>
//The number of points marked on the curves in data sheets
#define POINTS 9
#define DHTPIN            9  //pinulconectat la senzor
#define DHTTYPE           DHT22
float Ro = 98800, Vref = 5.0, R1 = 7700;
// Change the following values for different gasses from the corresponding curves from datasheet
//The following valure are corresponding X and Y coordinate values for the marked points on curves

float ltx[POINTS] = {200, 500, 800, 1000, 1600, 2000, 3000, 5000, 10000};
float lty[POINTS] = {1.67, 1.11, 0.88, 0.78, 0.64, 0.56, 0.46, 0.36, 0.26};
//loop control variables
int counter1, counter2;
int temp;
int rawAnalogValue[100]; 
DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;

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

int16_t adc0,adc1,adc2;
Servo S0,S1,S2,S3,S4;
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
int initAltitude;
void setup(void) 
{
 
  Wire.begin();
Serial.begin(115200);
  pinMode(13,OUTPUT);
  pinMode(12,OUTPUT);
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
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
    dht.humidity().getSensor(&sensor);
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
  initAltitude=bmp.readAltitude(1013.25);
  S0.attach(5);
  S1.attach(6);
  S2.attach(7);
  S3.attach(8);
  ads.begin();
}



void loop(void ) {
 while(!Serial.available());
  switch(Serial.read())
  {
    case 255:
    {
       Serial.println("Reading Voltage");
      //tensiunea bateriei , 0.375v/bit
      Serial.write((byte)(ads.readADC_SingleEnded(2)*0.375*0.000032941));
      break;
    }
    case 254: {
      //DHT22 0.5grade /bit
       Serial.println("Reading DHT22");
      sensors_event_t event;
      Serial.write((byte)(event.temperature*2));
      Serial.write((byte)(event.relative_humidity*2));
      break;
    }
    case 253: {
      //Servouri
      Serial.println("Writing servo data");
      S0.write(Serial.read());
      S1.write(Serial.read());
      S2.write(Serial.read());
      S3.write(Serial.read());
      S4.write(Serial.read());
      break;
    }
    case 252: {
      //BMP280
       Serial.println("Reading BMP280");
    Serial.write((byte)(bmp.readTemperature()*2));
    int pressure=(int)(bmp.readPressure());
    Serial.write((byte)(pressure&0xFF));
    Serial.write((byte)((pressure&0xFF00)>>8));
    Serial.write((byte)((pressure&0xFF0000)>>16));
    Serial.write((byte)((pressure&0xFF000000)>>24));
    Serial.write((byte)(10*(bmp.readAltitude(1013.25)-initAltitude)));
    break;
    }
    case 251: {
      //SI1145
       Serial.println("Reading SI1145");
  Serial.write((byte)(uv.readVisible()/2));
  Serial.write((byte)(uv.readIR()/2));
  float UVindex = uv.readUV();
  UVindex /= 100.0;  
  Serial.write((byte)(UVindex));
  break;
}
      case 250: {
  //MQ2
 //Serial.write((byte)(ads.readADC_SingleEnded(1)));
  Serial.println("Reading MQ2");
  Serial.write((byte)(digitalRead(4)?255:0));
    }
    case 249: {
      //accelerometer
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  int16_t ax=-(Buf[0]<<8 | Buf[1]);
  int16_t ay=-(Buf[2]<<8 | Buf[3]);
  int16_t az=Buf[4]<<8 | Buf[5];
  Serial.println(ax,DEC);
  Serial.println("\n"); 
  Serial.println(ay,DEC);
  Serial.println("\n");
  Serial.println(az,DEC); 
  Serial.println("\n");
  break;
    }
    case 248: {
      //gyroscope
      // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  // Gyroscope
  int16_t gx=-(Buf[8]<<8 | Buf[9]);
  int16_t gy=-(Buf[10]<<8 | Buf[11]);
  int16_t gz=Buf[12]<<8 | Buf[13];
  // Gyroscope
  Serial.println(gx,DEC); 
  Serial.println("\n");
  Serial.println(gy,DEC);
  Serial.println("\n");
  Serial.println(gz,DEC);
  Serial.println("\n");
  break;
    }
    case 247: {
      //magnetometer
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
  Serial.println(mx+200,DEC); 
  Serial.println("\n");
  Serial.println(my-70,DEC);
  Serial.println("\n");
  Serial.println(mz-700,DEC);
  Serial.println("\n"); 
  break;
    }
    case 246: {
      //light on
      digitalWrite(13, HIGH);
    }
    case 245: {
      //light off
      digitalWrite(13, LOW);
    }
    case 244: {
      digitalWrite(12, LOW);
    }
    }

}


