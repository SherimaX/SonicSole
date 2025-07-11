// NPi Sensor Read and Data Logging

#include <iostream>
#include <fstream>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <chrono>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <inttypes.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <time.h>
#include <vector>

// WiringPi libraries
#include <wiringPi.h>
#include <wiringSerial.h>
#include <wiringPiI2C.h>
#include <termios.h>
// #include "wiringSerial.h"

// User-defined libraries
#include "RPi_combined_Header.h"

// libraries from ADC code
#include <wiringPiSPI.h>


#define N_STR 512
#define LENGTH_SINGLE_PACKET 57
#define LENGTH_BUFFER_BLOCK 40926 //28500//4047
#define NUMBER_BUFFER_PACKET 25 //718 //71

using namespace std;

// channel is the wiringPi name for the chip select (or chip enable) pin.
// Set this to 0 or 1, depending on how it's connected.
static int SPI_CHANNEL = 0;


static int CS = 17;

// mutex dataMutex[2];
mutex dataMutex[3];
mutex writeMutex;
condition_variable condWrite;

////////////////////////////////////////////////////
// DATA STRUCTURES

struct structStreamingTimingInformation
{
  unsigned int interval;
  unsigned int duration;
  unsigned int delay;
} sStreamingTime;

struct structComponentLinearAcceleration
{
  //Big Endian
  float az;
  float ay;
  float ax;
};

struct structComponentRawAcceleration
{
  //Big Endian
  float r_az;
  float r_ay;
  float r_ax;
};

struct structComponentQuaternion
{
  //Big Endian
  float qw;
  float qz;
  float qy;
  float qx;
};

struct structComponentRawGyro
{
  // Big Endian
  float gz;
  float gy;
  float gx;
};

struct structComponentRawMag
{
  // Big Endian
  float mz;
  float my;
  float mx;
};

struct structEulerAngles
{
  //Big Endian
  float roll;
  float yaw;
  float pitch;
};

struct structComponentSensorData
{
  //Big Endian
  float mx;
  float my;
  float mz;

  float ax;
  float ay;
  float az;

  float gx;
  float gy;
  float gz;
};

struct structStreamingData
{
  //Big Endian
//  structComponentRawMag mag;
  structComponentRawAcceleration acc;
  structComponentRawGyro gyro;
  structComponentLinearAcceleration lAcc;
  //structEulerAngles eulerAngles;
  structComponentQuaternion q;
};

union unionStreamingData
{
  structStreamingData sData;
  uint8_t vData[sizeof(structStreamingData)];
} uStreamingDataIMU;


inline void reconstructIMUPacket(uint8_t* dataIMUPacket, structComponentQuaternion &structQuat, structComponentLinearAcceleration &structAcce, structComponentRawGyro &structGyro, structComponentRawAcceleration &structRAcc)
{
  uint8_t *pointer;

  ////////////////////////////////////////

  pointer = (uint8_t*)&structQuat.qx;
  pointer[3] = dataIMUPacket[0];
  pointer[2] = dataIMUPacket[1];
  pointer[1] = dataIMUPacket[2];
  pointer[0] = dataIMUPacket[3];

  pointer = (uint8_t*)&structQuat.qy;
  pointer[3] = dataIMUPacket[4];
  pointer[2] = dataIMUPacket[5];
  pointer[1] = dataIMUPacket[6];
  pointer[0] = dataIMUPacket[7];

  pointer = (uint8_t*)&structQuat.qz;
  pointer[3] = dataIMUPacket[8];
  pointer[2] = dataIMUPacket[9];
  pointer[1] = dataIMUPacket[10];
  pointer[0] = dataIMUPacket[11];

  pointer = (uint8_t*)&structQuat.qw;
  pointer[3] = dataIMUPacket[12];
  pointer[2] = dataIMUPacket[13];
  pointer[1] = dataIMUPacket[14];
  pointer[0] = dataIMUPacket[15];

  ////////////

  pointer = (uint8_t*)&structAcce.ax;
  pointer[3] = dataIMUPacket[16];
  pointer[2] = dataIMUPacket[17];
  pointer[1] = dataIMUPacket[18];
  pointer[0] = dataIMUPacket[19];

  pointer = (uint8_t*)&structAcce.ay;
  pointer[3] = dataIMUPacket[20];
  pointer[2] = dataIMUPacket[21];
  pointer[1] = dataIMUPacket[22];
  pointer[0] = dataIMUPacket[23];

  pointer = (uint8_t*)&structAcce.az;
  pointer[3] = dataIMUPacket[24];
  pointer[2] = dataIMUPacket[25];
  pointer[1] = dataIMUPacket[26];
  pointer[0] = dataIMUPacket[27];

  ////////////

  pointer = (uint8_t*)&structGyro.gx;
  pointer[3] = dataIMUPacket[28];
  pointer[2] = dataIMUPacket[29];
  pointer[1] = dataIMUPacket[30];
  pointer[0] = dataIMUPacket[31];

  pointer = (uint8_t*)&structGyro.gy;
  pointer[3] = dataIMUPacket[32];
  pointer[2] = dataIMUPacket[33];
  pointer[1] = dataIMUPacket[34];
  pointer[0] = dataIMUPacket[35];

  pointer = (uint8_t*)&structGyro.gz;
  pointer[3] = dataIMUPacket[36];
  pointer[2] = dataIMUPacket[37];
  pointer[1] = dataIMUPacket[38];
  pointer[0] = dataIMUPacket[39];

  ////////////

  pointer = (uint8_t*)&structRAcc.r_ax;
  pointer[3] = dataIMUPacket[40];
  pointer[2] = dataIMUPacket[41];
  pointer[1] = dataIMUPacket[42];
  pointer[0] = dataIMUPacket[43];

  pointer = (uint8_t*)&structRAcc.r_ay;
  pointer[3] = dataIMUPacket[44];
  pointer[2] = dataIMUPacket[45];
  pointer[1] = dataIMUPacket[46];
  pointer[0] = dataIMUPacket[47];

  pointer = (uint8_t*)&structRAcc.r_az;
  pointer[3] = dataIMUPacket[48];
  pointer[2] = dataIMUPacket[49];
  pointer[1] = dataIMUPacket[50];
  pointer[0] = dataIMUPacket[51];
}


inline void constructLogPacket(uint8_t* dataPacket, uint32_t currentTime, 
                        structComponentQuaternion structQuat, 
                        structComponentLinearAcceleration structAcce, 
                        structComponentRawGyro structGyro, 
                        structComponentRawAcceleration structRAcc,
                        uint16_t p0, uint16_t p1, uint16_t p2, uint16_t p3,
                        uint16_t p4, uint16_t p5, uint16_t p6, uint16_t p7,
                        uint32_t syncTime, uint8_t syncTrigger)
{
  uint8_t *pointer;
  int16_t val;

  // HEADER
  ////////////////////////////////////////

  dataPacket[0] = 0x01;
  dataPacket[1] = 0x02;
  dataPacket[2] = 0x03;

  // TIMESTAMP
  ////////////////////////////////////////

  pointer = (uint8_t*)&currentTime;
  dataPacket[3] = pointer[3];
  dataPacket[4] = pointer[2];
  dataPacket[5] = pointer[1];
  dataPacket[6] = pointer[0];

  // QUATERNION
  ////////////////////////////////////////

  val = int16_t(structQuat.qw * 5000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[7] = pointer[1];
  dataPacket[8] = pointer[0];

  val = int16_t(structQuat.qx * 5000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[9] = pointer[1];
  dataPacket[10] = pointer[0];

  val = int16_t(structQuat.qy * 5000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[11] = pointer[1];
  dataPacket[12] = pointer[0];

  val = int16_t(structQuat.qz * 5000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[13] = pointer[1];
  dataPacket[14] = pointer[0];
  
  // LINEAR ACCELERATION
  ////////////////////////////////////////

  val = int16_t(structAcce.ax * 1000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[15] = pointer[1];
  dataPacket[16] = pointer[0];

  val = int16_t(structAcce.ay * 1000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[17] = pointer[1];
  dataPacket[18] = pointer[0];

  val = int16_t(structAcce.az * 1000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[19] = pointer[1];
  dataPacket[20] = pointer[0];

  // ANGULAR VELOCITY
  ////////////////////////////////////////

  val = int16_t(structGyro.gx * 900.0f);
  pointer = (uint8_t*)&val;
  dataPacket[21] = pointer[1];
  dataPacket[22] = pointer[0];

  val = int16_t(structGyro.gy * 900.0f);
  pointer = (uint8_t*)&val;
  dataPacket[23] = pointer[1];
  dataPacket[24] = pointer[0];

  val = int16_t(structGyro.gz * 900.0f);
  pointer = (uint8_t*)&val;
  dataPacket[25] = pointer[1];
  dataPacket[26] = pointer[0];

  // ACCELERATION
  ////////////////////////////////////////

  val = int16_t(structRAcc.r_ax * 1000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[27] = pointer[1];
  dataPacket[28] = pointer[0];

  val = int16_t(structRAcc.r_ay * 1000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[29] = pointer[1];
  dataPacket[30] = pointer[0];

  val = int16_t(structRAcc.r_az * 1000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[31] = pointer[1];
  dataPacket[32] = pointer[0];

  // PRESSURE SENSORS
  ////////////////////////////////////////

  pointer = (uint8_t*)&p0;
  dataPacket[33] = pointer[1];
  dataPacket[34] = pointer[0];

  pointer = (uint8_t*)&p1;
  dataPacket[35] = pointer[1];
  dataPacket[36] = pointer[0];

  pointer = (uint8_t*)&p2;
  dataPacket[37] = pointer[1];
  dataPacket[38] = pointer[0];

  pointer = (uint8_t*)&p3;
  dataPacket[39] = pointer[1];
  dataPacket[40] = pointer[0];

  pointer = (uint8_t*)&p4;
  dataPacket[41] = pointer[1];
  dataPacket[42] = pointer[0];

  pointer = (uint8_t*)&p5;
  dataPacket[43] = pointer[1];
  dataPacket[44] = pointer[0];

  pointer = (uint8_t*)&p6;
  dataPacket[45] = pointer[1];
  dataPacket[46] = pointer[0];

  pointer = (uint8_t*)&p7;
  dataPacket[47] = pointer[1];
  dataPacket[48] = pointer[0];

  // SYNC SIGNAL
  ////////////////////////////////////////

  dataPacket[49] = syncTrigger;

  pointer = (uint8_t*)&syncTime;
  dataPacket[50] = pointer[3];
  dataPacket[51] = pointer[2];
  dataPacket[52] = pointer[1];
  dataPacket[53] = pointer[0];

  // TAIL
  ////////////////////////////////////////

  dataPacket[54] = 0x04;
  dataPacket[55] = 0x05;
  dataPacket[56] = 0x06;
}



inline void reconstructBinaryPacketBinary_test(uint8_t* recvbuffer, float &ax, float &ay, float &az)
{
  uint8_t *pointer;

  //recvbuffer[0];
  //recvbuffer[1];
  //recvbuffer[2];

  pointer = (uint8_t*)&ax;
  pointer[3] = recvbuffer[0];
  pointer[2] = recvbuffer[1];
  pointer[1] = recvbuffer[2];
  pointer[0] = recvbuffer[3];

  pointer = (uint8_t*)&ay;
  pointer[3] = recvbuffer[4];
  pointer[2] = recvbuffer[5];
  pointer[1] = recvbuffer[6];
  pointer[0] = recvbuffer[7];

  pointer = (uint8_t*)&az;
  pointer[3] = recvbuffer[8];
  pointer[2] = recvbuffer[9];
  pointer[1] = recvbuffer[10];
  pointer[0] = recvbuffer[11];
}

// union unionComponentSensorData
// {
//   structComponentSensorData sData;
//   uint8_t vData[sizeof(structComponentSensorData)];
// } uCompSensData;

uint8_t calcCRC256(uint8_t* dataPacket, uint8_t nByte)
{
  uint16_t checksum = 0;
  for (uint8_t i = 1; i < nByte; i++)
  {
    checksum += dataPacket[i];
  }
  return (checksum % 256);
}

// int waitByteCountFromSerial(HardwareSerial& serial, unsigned int bytecount)
// {
//   while (serial.available() < bytecount)
//   {
//     delayMicroseconds(DELAY_WAIT_SERIAL_YEI);
//   }
// }


void createLogPacket_v01(uint8_t* dataPacket, uint64_t currentTime)
{
  uint8_t *pointer;

  dataPacket[0] = 0x01;
  dataPacket[1] = 0x02;
  dataPacket[2] = 0x03;

  pointer = (uint8_t*)&currentTime;
  dataPacket[3] = pointer[7];
  dataPacket[4] = pointer[6];
  dataPacket[5] = pointer[5];
  dataPacket[6] = pointer[4];
  dataPacket[7] = pointer[3];
  dataPacket[8] = pointer[2];
  dataPacket[9] = pointer[1];
  dataPacket[10] = pointer[0];

  dataPacket[11] = 0x04;
  dataPacket[12] = 0x05;
  dataPacket[13] = 0x06;
}



////////////////////////////////////////////////////
// YEI FUNCTIONS

// void YEIwriteCommand(HardwareSerial& serial, uint8_t cmd, uint8_t value)
inline void YEIwriteCommandValue(int serial, uint8_t cmd, uint8_t value)
{
	// uint8_t YEIdataPacket[4];
  YEIdataPacket[0] = START_NO_RESP_HEADER;
  // YEIdataPacket[0] = START_RESP_HEADER;
  YEIdataPacket[1] = cmd;
  YEIdataPacket[2] = value;
  YEIdataPacket[3] = calcCRC256(YEIdataPacket, 3);
  write(serial, YEIdataPacket, 4);
  delay(YEI_DELAY_AFTER_COMMAND);
}

inline void YEIwriteCommandNoDelay(int serial, uint8_t cmd)
{
	// uint8_t YEIdataPacket[3];
  YEIdataPacket[0] = START_NO_RESP_HEADER;
  // YEIdataPacket[0] = START_RESP_HEADER;
  YEIdataPacket[1] = cmd;
  YEIdataPacket[2] = calcCRC256(YEIdataPacket, 2);
  write(serial, YEIdataPacket, 3);
}

inline void YEIwriteCommand(int serial, uint8_t cmd)
{
  YEIwriteCommandNoDelay(serial, cmd);
  delay(YEI_DELAY_AFTER_COMMAND);
}

inline void YEIsettingsHeader(int serial)
{
	// uint8_t YEIdataPacket[7];
  // Settings Header
  YEIdataPacket[0] = START_NO_RESP_HEADER;
  // YEIdataPacket[0] = START_RESP_HEADER;
  YEIdataPacket[1] = CMD_RESPONSE_HEADER_BITFIELD;
  YEIdataPacket[2] = 0x00;
  YEIdataPacket[3] = 0x00;
  YEIdataPacket[4] = 0x00;
  YEIdataPacket[5] = 0x00;
  YEIdataPacket[6] = calcCRC256(YEIdataPacket, 6);
  write(serial, YEIdataPacket, 7);
  delay(YEI_DELAY_AFTER_COMMAND);
}

// inline void YEIsettingsHeader(int serial)
// {
//   // Settings Header
//   // YEIdataPacket[0] = START_NO_RESP_HEADER;
//   YEIdataPacket[0] = START_RESP_HEADER;
//   YEIdataPacket[1] = CMD_RESPONSE_HEADER_BITFIELD;
//   YEIdataPacket[2] = 64;
//   YEIdataPacket[3] = calcCRC256(YEIdataPacket, 3);
//   write(serial, YEIdataPacket, 4);
//   delay(YEI_DELAY_AFTER_COMMAND);
// }


inline void YEIsetStreamingMode(int serial, uint8_t slot1, uint8_t slot2, uint8_t slot3, uint8_t slot4, uint8_t slot5, uint8_t slot6, uint8_t slot7, uint8_t slot8)
{
	// uint8_t YEIdataPacket[11];
  // Setting Streaming Mode
  YEIdataPacket[0] = START_NO_RESP_HEADER;
  // YEIdataPacket[0] = START_RESP_HEADER;
  YEIdataPacket[1] = CMD_SET_STREAMING_SLOT;

  YEIdataPacket[2] = slot1; //1st slot
  YEIdataPacket[3] = slot2; //2nd slot
  YEIdataPacket[4] = slot3; //3rd slot
  YEIdataPacket[5] = slot4; //4th slot
  YEIdataPacket[6] = slot5; //5th slot
  YEIdataPacket[7] = slot6; //6th slot
  YEIdataPacket[8] = slot7; //7th slot
  YEIdataPacket[9] = slot8; //8th slot

  YEIdataPacket[10] = calcCRC256(YEIdataPacket, 10);
  write(serial, YEIdataPacket, 11);

  delay(YEI_DELAY_AFTER_COMMAND);
}

inline void YEIsetStreamingTime(int serial)
{
	// uint8_t YEIdataPacket[15];
  // Set Streaming Time
  YEIdataPacket[0] = START_NO_RESP_HEADER;
  // YEIdataPacket[0] = START_RESP_HEADER;
  YEIdataPacket[1] = CMD_SET_STREAMING_TIMING;

  uint8_t *pointer = (uint8_t*)&sStreamingTime.interval;
  YEIdataPacket[2] = pointer[3];
  YEIdataPacket[3] = pointer[2];
  YEIdataPacket[4] = pointer[1];
  YEIdataPacket[5] = pointer[0];

  pointer = (uint8_t*)&sStreamingTime.duration;
  YEIdataPacket[6] = pointer[3];
  YEIdataPacket[7] = pointer[2];
  YEIdataPacket[8] = pointer[1];
  YEIdataPacket[9] = pointer[0];

  pointer = (uint8_t*)&sStreamingTime.delay;
  YEIdataPacket[10] = pointer[3];
  YEIdataPacket[11] = pointer[2];
  YEIdataPacket[12] = pointer[1];
  YEIdataPacket[13] = pointer[0];

  YEIdataPacket[14] = calcCRC256(YEIdataPacket, 14);
  write(serial, YEIdataPacket, 15);
}

// Polling mode
inline void YEIgetStreamingBatch(unionStreamingData& uStreamingDataIMU)
{ 
  int nPacketStreamingData = sizeof(uStreamingDataIMU);
  int bIMU = 0;

  YEIwriteCommandNoDelay(IMU, CMD_GET_STREAMING_BATCH);

  if(serialDataAvail(IMU)) {
    cout << "IMU AVAILABLE\n\n\n\n\n\n\n\n\n\n" << endl;
    // read(IMU, &uStreamingDataIMU.vData, sizeof(uStreamingDataIMU.vData));
    read(IMU, &uStreamingDataIMU.vData, 26);
    // delayMicroseconds(DELAY_WAIT_SERIAL_YEI);
  } 
  else {
    cout << "IMU NOT AVAILABLE\n\n\n\n\n\n\n\n\n" << endl;
  }
  

  // while (!(bIMU >= nPacketStreamingData))
  // {
  //     uStreamingDataIMU1.vData[nPacketStreamingData - bIMU - 1] = read();
  //     bIMU++;
  // }
}

uint64_t getMicrosTimeStamp() 
{
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

////////////////////////////////////////////////////
// FSR FUNCTIONS

// unsigned int readADC(int ADCModule)
// {
//   unsigned int val;
//   uint8_t ADCBuffer[2];
//   ADCBuffer[0] = 0;
//   ADCBuffer[1] = 0;
//   read(ADCModule, &ADCBuffer, 2);
//   val = ADCBuffer[0] << 8 | ADCBuffer[1];
//   return val;
// }

// uint16_t voltageToPressure(uint16_t num, uint16_t RawOutput)
// {
//   uint16_t val;
//   if (RawOutput <= segmentpoint)
//   {
//     val = coef1[num - 1] * RawOutput * 4; // FACTOR OF 4 (10-BIT TO 12-BIT RES)
//   } 
//   else 
//   {
//     val = (coef2[num - 1] * (RawOutput - segmentpoint) + coef1[num - 1] * segmentpoint) * 4; // FACTOR OF 4 (10-BIT TO 12-BIT RES)
//   }
//   return ((uint16_t) val);
// }

// uint16_t readPressure(uint8_t num, int ADCModule)
// {
//   uint16_t pressVal;
//   uint16_t rawVal;

//   digitalWrite(PIN_ENABLE, LOW);
//   switch (num) {
//     case 1:
//       digitalWrite(PIN_A, LOW);
//       digitalWrite(PIN_B, HIGH);
//       digitalWrite(PIN_C, LOW);
//       break;
//     case 2:
//       digitalWrite(PIN_A, LOW);
//       digitalWrite(PIN_B, HIGH);
//       digitalWrite(PIN_C, HIGH);
//       break;
//     case 3:
//       digitalWrite(PIN_A, HIGH);
//       digitalWrite(PIN_B, HIGH);
//       digitalWrite(PIN_C, LOW);
//       break;
//     case 4:
//       digitalWrite(PIN_A, HIGH);
//       digitalWrite(PIN_B, LOW);
//       digitalWrite(PIN_C, HIGH);
//       break;
//     case 5:
//       digitalWrite(PIN_A, LOW);
//       digitalWrite(PIN_B, LOW);
//       digitalWrite(PIN_C, HIGH);
//       break;
//     case 6:
//       digitalWrite(PIN_A, HIGH);
//       digitalWrite(PIN_B, HIGH);
//       digitalWrite(PIN_C, HIGH);
//       break;
//     case 7:
//       digitalWrite(PIN_A, HIGH);
//       digitalWrite(PIN_B, LOW);
//       digitalWrite(PIN_C, LOW);
//       break;
//     case 8:
//       digitalWrite(PIN_A, LOW);
//       digitalWrite(PIN_B, LOW);
//       digitalWrite(PIN_C, LOW);
//       break;
//   }
//   // this_thread::sleep_for(chrono::microseconds(2));
//   // delayMicroseconds(2);
//   rawVal = readADC(ADCModule);
//   // rawVal = wiringPiI2CReadReg16(ADCModule, 0x01);
//   pressVal = voltageToPressure(num, rawVal);
//   return ((uint16_t) pressVal);
// }

vector<float> axData; 
vector<float> ayData; 
vector<float> azData; 

void getAccelVectorData(float ax, float ay, float az) // vector<float>& axVector, vector<float>& ayVector, vector<float>& azVector)
{
  axData.push_back(ax);
  ayData.push_back(ay);
  azData.push_back(az);
}

float accelVectorIntegral(vector<float> ay) 
{

  if (ay.size() < 2) {
    return 0; 
  }

  int deltaX =  ay.size() / 20;
  int sumOfPoints = ay[0];

  for (int i = 1; i < ay.size()-1; i++) {
    sumOfPoints += (2*ay[i]);
  }

  sumOfPoints += ay[ay.size()-1];

  float integAccel = 0.5 * deltaX * (sumOfPoints);
  return integAccel
}


////////////////////////////////////////////////////
// MAIN CODE

int main(int argc, char* argv[])
{
 	
  wiringPiSetupGpio() ;
	pinMode (CS, OUTPUT) ;
	digitalWrite(CS,HIGH);

   	cout << "Initializing SPI" << endl ;
	unsigned char SPIbuff[3];
	// char dummy;
	// uint8_t adc_l8;
	// uint8_t adc_h4;

	// int SPIreturn;

	int adc_channel0;
	int adc_channel1;
	// char dataOut;
	// Configure the interface.
	// CHANNEL insicates chip select,
	// 500000 indicates bus speed, change depending on bus speed

	// int fd = wiringPiSPISetup(SPI_CHANNEL, 500000);


  int fd = wiringPiSPISetupMode(SPI_CHANNEL, 1000000, 0);
  if (fd == -1) {
      std::cout << "Failed to init SPI communication.\n";
      return -1;
  }
  std::cout << "SPI communication successfully setup.\n";	

  this_thread::sleep_for(chrono::milliseconds(500));
  
  char strSession[N_STR];

	printf("\nRaspberry Pi for SonicSole: \n\n");
	printf("UART0 IMU and Multi-thread Data Logging\n\n");

 	// INPUT ARGUMENTS
 	if (argc>2)
	{
		printf("Error on input argument!\n");
		return -1;
  	}
  	else if (argc == 1)
  	{
    	// No session name, simple functioning
    	sprintf(strSession, "%s", "");
  	}
  	else if (argc == 2)
  	{
    	// Simple functioning
    	sprintf(strSession, "%s", argv[1]);
    	printf("Session Name: %s\n", strSession);
  	}


  	// INITIALIZING UART1
  	printf("Initializing UART0...\n\n");
    if ((IMU = serialOpen ("/dev/ttyS0", 115200)) < 0)
  	// if ((IMU = serialOpen ("/dev/ttyS0", 460800)) < 0)
  	// if ((IMU = serialOpen ("/dev/ttyS1", 921600)) < 0)
  	// if ((IMU = serialOpen ("/dev/ttyS1", 230400)) < 0)
  	{
  		fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
  		return 1 ;
  	}
  	printf("UART1 initialized successfully!\n\n");


  	// INITIALIZING GPIO (Use wPi pins, not BCM)
  	printf("Initializing GPIO...\n\n");
  	if (wiringPiSetupGpio () == -1)
  	{
    	fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
    	return 1 ;
  	}
  	printf("GPIO initialized successfully!\n\n");
    // pinMode(PIN_ENABLE, OUTPUT);
    // pinMode(PIN_A, OUTPUT);
    // pinMode(PIN_B, OUTPUT);
    // pinMode(PIN_C, OUTPUT);


  	// INITIALIZING I2C
  	// printf("Initializing I2C...\n\n");
  	// // ADCModule = open("/dev/i2c-1", O_RDWR);
  	// // ioctl(ADCModule, I2C_SLAVE, ADCAddress);
   //  ADCModule = wiringPiI2CSetup(ADCAddress);
   //  printf("I2C initialized successfully!\n\n");


    structComponentQuaternion dataQuat;
    structComponentLinearAcceleration dataAcce;
    structComponentRawGyro dataGyro;
    structComponentRawAcceleration dataRAcc;

      // IMU DATA PACKET
    uint8_t IMU_PACKET_LENGTH = 52;
    uint8_t dataIMUPacket[IMU_PACKET_LENGTH];



	// // BLINK
	// for (int i = 0; i < 20; i++)
	// {
 //  	led_state = !led_state;
 //  	digitalWrite(debug_LED, led_state);
 //  	delay(75);
	// }
	// led_state = TRUE;
	// digitalWrite(debug_LED, led_state);


	// CONFIGURING IMU
	printf("Configuring IMU...\n\n");
	YEIsettingsHeader(IMU);
	YEIwriteCommandNoDelay(IMU, CMD_STOP_STREAMING);
  this_thread::sleep_for(chrono::milliseconds(1000));
	YEIwriteCommandValue(IMU, CMD_SET_ACCELEROMETER_RANGE, ACCELEROMETER_RANGE_8G);
	YEIwriteCommandValue(IMU, CMD_SET_GYROSCOPE_RANGE, GYROSCOPE_RANGE_2000);
	YEIwriteCommandValue(IMU, CMD_SET_COMPASS_RANGE, COMPASS_RANGE_1_3);
	YEIwriteCommandValue(IMU, CMD_SET_CALIBRATION_MODE, CALIBRATION_MODE_BIAS_SCALE);
	YEIwriteCommandValue(IMU, CMD_SET_AXIS_DIRECTIONS,AXIS_XR_YF_ZU);
	YEIwriteCommandValue(IMU, CMD_SET_REFERENCE_VECTOR_MODE, REFERENCE_VECTOR_MULTI_REFERENCE_MODE);
	YEIwriteCommandValue(IMU, CMD_SET_COMPASS_ENABLE, FALSE);
	YEIwriteCommandValue(IMU, CMD_SET_FILTER_MODE, FILTER_KALMAN);
	YEIwriteCommandNoDelay(IMU, CMD_BEGIN_GYROSCOPE_AUTOCALIBRATION);
  this_thread::sleep_for(chrono::milliseconds(500));
	YEIwriteCommandNoDelay(IMU, CMD_RESET_FILTER);
  this_thread::sleep_for(chrono::milliseconds(500));
	// YEIsetStreamingMode(IMU, READ_TARED_ORIENTATION_AS_QUATERNION, READ_CORRECTED_ACCELEROMETER_VECTOR, READ_CORRECTED_GYROSCOPE_VECTOR, READ_CORRECTED_ACCELEROMETER_VECTOR, NO_SLOT, NO_SLOT, NO_SLOT, NO_SLOT);
  YEIsetStreamingMode(IMU, READ_TARED_ORIENTATION_AS_QUATERNION, READ_CORRECTED_LINEAR_ACCELERATION, READ_CORRECTED_GYROSCOPE_VECTOR, READ_CORRECTED_ACCELEROMETER_VECTOR, NO_SLOT, NO_SLOT, NO_SLOT, NO_SLOT);
  // YEIsetStreamingMode(IMU, READ_CORRECTED_ACCELEROMETER_VECTOR, NO_SLOT, NO_SLOT, NO_SLOT, NO_SLOT, NO_SLOT, NO_SLOT, NO_SLOT);

	// sStreamingTime.interval = 2000; //10000; //[us]
	// sStreamingTime.duration = 0xFFFFFFFF;
	// sStreamingTime.delay = 0;	//[us]
	// YEIsetStreamingTime(IMU);
	YEIwriteCommandNoDelay(IMU, CMD_TARE_WITH_CURRENT_ORIENTATION);
	printf("IMU configured successfully!\n");

	bool recordState = TRUE;


	printf("Here\n");


	// INITIALIZE OFSTREAM FILE
	ofstream dataFile;
	string fileName = "testlog_RPi.dat";


	// TIME VARIABLES
	struct timeval tv;
	uint64_t timestampStart= getMicrosTimeStamp();
	uint32_t currentTime;
  uint32_t currentTimeSecs;
	uint8_t syncTrigger = 0;
	uint32_t syncTime = 0;

	uint16_t intervalWait = 3000;
	uint32_t headTime;
	uint32_t deltaTime;

  // // DUMMY PRESSURE PACKET
  // for (int j = 1; j < 8; j++)
  // {
  //   p[j] = 0x00;
  // }


  
	cout << "Start infinite loop...\n\n\n" ;

	// INIFINITE LOOP
	while(recordState)
	{
    // ADC channels
    // Channel 0
		digitalWrite(CS,LOW);
		SPIbuff[0] = 1;
		SPIbuff[1] = 160;
		SPIbuff[2] = 0;
		wiringPiSPIDataRW(SPI_CHANNEL,SPIbuff,3);
		adc_channel0 = SPIbuff[1] << 8 | SPIbuff[2];
		digitalWrite(CS, HIGH);

		// Channel 1
		digitalWrite(CS,LOW);
		SPIbuff[0] = 1;
		SPIbuff[1] = 224;
		SPIbuff[2] = 0;
		wiringPiSPIDataRW(SPI_CHANNEL,SPIbuff,3);
		adc_channel1 = SPIbuff[1] << 8 | SPIbuff[2];
		digitalWrite(CS, HIGH);

		printf("Forefoot sensor : %d \n", adc_channel0);
		printf("Hindfoot sensor : %d \n", adc_channel1);
		printf("  \n");
		this_thread::sleep_for(chrono::milliseconds(400));
		
		// pinMode
		pinMode(3, OUTPUT);
		pinMode(20, OUTPUT);
    
    
    // IMU data
		// YEIgetStreamingBatch(uStreamingDataIMU);
		for (int i = 0 ; i < sizeof(dataIMUPacket) ; i++) dataIMUPacket[i] = 0x00;

    // {lock_guard<mutex> lck(dataMutex[currBuff]);

      // cout << "Get sensor data...\n\n";

      // FILL UP BUFFER BLOCK
      for (int i = 0; i < NUMBER_BUFFER_PACKET; i++)
      {
        // GET IMU DATA
        YEIwriteCommandNoDelay(IMU, CMD_GET_STREAMING_BATCH);
        while(serialDataAvail(IMU) < IMU_PACKET_LENGTH)
        {
          // std::cout << serialDataAvail(IMU) << std::endl;
          // If no IMU data received, do nothing
        }
      	read(IMU, dataIMUPacket, IMU_PACKET_LENGTH);
        reconstructIMUPacket(dataIMUPacket, dataQuat, dataAcce, dataGyro, dataRAcc);

        // uint64_t currentTime = getMicrosTimeStamp() - timestampStart;
        currentTime = (getMicrosTimeStamp() - timestampStart) / 1000;
        // syncTime = (getMicrosTimeStamp() - timestampStart) / 1000;
        // syncTime = 	currentTime;
        

        // GET PRESSURE DATA
        // for (int j = 0; j < 8; j++)
        // {
        // 	p[j] = readPressure(j+1, ADCModule) - offset_p[j];
        // }


        // CREATE SINGLE LOG PACKET
        // constructLogPacket(singleLogPacket, currentTime, dataQuat, dataAcce, dataGyro, dataRAcc,
        //         p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], syncTime, syncTrigger);

        // DEBUG ONLY
        // printf("Pressure values: %i , %i , %i , %i , %i , %i , %i , %i \n", p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);

        // COPY TO BUFFER BLOCK         
    	  // {lock_guard<mutex> lck(dataMutex[currBuff]);
        // if (currBuff==0)
        // {
        //   memcpy(&bufferBlock0[i * LENGTH_SINGLE_PACKET], &singleLogPacket,LENGTH_SINGLE_PACKET); 
        //   // memcpy(&bufferBlock[0][i * LENGTH_SINGLE_PACKET], &singleLogPacket,LENGTH_SINGLE_PACKET);  
        // }
        // else if (currBuff==1)
        // {
        //   memcpy(&bufferBlock1[i * LENGTH_SINGLE_PACKET], &singleLogPacket,LENGTH_SINGLE_PACKET); 
        //   // memcpy(&bufferBlock[1][i * LENGTH_SINGLE_PACKET], &singleLogPacket,LENGTH_SINGLE_PACKET);  
        // }   
        // else
        // {
        //   memcpy(&bufferBlock2[i * LENGTH_SINGLE_PACKET], &singleLogPacket,LENGTH_SINGLE_PACKET); 
        //   // memcpy(&bufferBlock[1][i * LENGTH_SINGLE_PACKET], &singleLogPacket,LENGTH_SINGLE_PACKET);  
        // }
	    	// }

        // serialFlush(IMU);
        // IMUDataReady = false;

        // // SLEEP FOR 4 MS (250 Hz)
        // this_thread::sleep_for(chrono::milliseconds(4));
      }
    // }

    // PRINT OUT SOME DEBUG DATA
    // currentTimeSecs = currentTime / 1000;
    
    //on off switch
    if (adc_channel0 > 500) {
			digitalWrite(3, HIGH);
		} else {
			digitalWrite(3, LOW);
		}
		
		// Led Channel 1
		
		if (adc_channel1 > 500) {
			digitalWrite(20, HIGH);
		} else {
			digitalWrite(20, LOW);
		}
    
    //IMU data print
    printf("Time: %i msecs \n", currentTime);
    printf("IMU Acceleration Vector: %0.2f , %0.2f , %0.2f \n\n", dataRAcc.r_ax, dataRAcc.r_ay, dataRAcc.r_az);
    
    // SLEEP FOR 4 MS (250 Hz)
    // this_thread::sleep_for(chrono::milliseconds(4));
    getAccelVectorData(dataRAcc.r_ax, dataRAcc.r_ay, dataRAcc.r_az);
	}
	cout << "End of code!"; // Although we will never get here...
	return 0;
}


//   YEIwriteCommandNoDelay(IMU, CMD_GET_STREAMING_BATCH);
	// if(serialDataAvail(IMU))
	// {
	// 	uint64_t timeRead = getMicrosTimeStamp() - timestamp_start;

	// 	read(IMU, dataIMUPacket, IMU_PACKET_LENGTH);

//     reconstructIMUPacket(dataIMUPacket, dataQuat, dataAcce, dataGyro, dataRAcc);



// reconstructBinaryPacketBinary_test(dataIMUPacket, dataAcce);
// reconstructBinaryPacketBinary_test(dataIMUPacket, ax, ay, az);
// printf("IMU Acceleration Vector: %0.2f , %0.2f , %0.2f \n", ax, ay,az);


// printf("IMU Acceleration Vector: %0.2f , %0.2f , %0.2f \n", dataAcce.ax, dataAcce.ay, dataAcce.az);

// float deltaTime = (float)(getMicrosTimeStamp() - timeRead) / 1000000.0f;
// float freq = 1/deltaTime;
// float currenttime_micros = (float)(getMicrosTimeStamp() - timestamp_start);

// printf("Time obtained!\n");
// printf("IMU Gyroscope Vector: %0.2f , %0.2f , %0.2f \n", dataGyro.gx, dataGyro.gy, dataGyro.gz);
// printf("Time: %0.3f secs \n", deltaTime);