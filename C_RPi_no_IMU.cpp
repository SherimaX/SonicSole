// NPi Sensor Read and Data Logging

#include <iostream>
#include <fstream>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <chrono>
#include <iomanip>

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

// WiringPi libraries
#include <wiringPi.h>
#include <wiringSerial.h>
#include <wiringPiI2C.h>
#include <termios.h>
// #include "wiringSerial.h"

// User-defined libraries
#include "RPi_combined_Header.h"
#include "RPi_Raj_Header.hpp"

// libraries from ADC code
#include <wiringPiSPI.h>

// libraries for UDP
#include <bits/stdc++.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

using namespace std;

// channel is the wiringPi name for the chip select (or chip enable) pin.
// Set this to 0 or 1, depending on how it's connected.
static int SPI_CHANNEL = 0;

static int CS = 17;

uint64_t getMicrosTimeStamp()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec * (uint64_t)1000000 + tv.tv_usec;
}

// UDP Send

void UDPSend(int sockfd, const int *reading, socklen_t len, struct sockaddr_in servaddr)
{
  sendto(sockfd, (const int *)reading, len,
         MSG_CONFIRM, (const struct sockaddr *)&servaddr,
         sizeof(servaddr));
}

////////////////////////////////////////////////////
// MAIN CODE

int main(int argc, char *argv[])
{

  // Define UDP Parameters
  int sockfd;
  char buffer[MAXLINE];
  int input = 1;
  const int *reading = &input;
  struct sockaddr_in servaddr;

  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
  {
    perror("socket creation failed");
    return 0;
  }

  memset(&servaddr, 0, sizeof(servaddr));

  // Filling server information
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(PORT);
  servaddr.sin_addr.s_addr = INADDR_ANY;

  int n;
  socklen_t len = 3;
  //

  wiringPiSetupGpio();
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);

  cout << "Initializing SPI" << endl;
  unsigned char SPIbuff[3];

  int adc_channel0;
  int adc_channel1;

  int num_threshold_reached = 0;
  int previous_pressure_sum;
  int current_pressure_sum = 0;
  int threshold_pressure_sum;
  double threshold_time_array[20] = {};
  double start_time = getMicrosTimeStamp();
  double current_threshold_time = 0;

  int max_pressure = 1000;
  int min_pressure = 800;

  int previous_heel_pressure;
  int current_heel_pressure = 0;
  int max_heel_pressure = 1000;
  int min_heel_pressure = 800;
  int threshold_heel_pressure;

  bool mode = true;

  // Configure the interface.
  // CHANNEL insicates chip select,
  // 500000 indicates bus speed, change depending on bus speed

  // 4int fd = wiringPiSPISetup(SPI_CHANNEL, 500000);

  digitalWrite(23, HIGH); // Turn motors on and off to show device is on
  digitalWrite(20, HIGH);
  delay(1000);
  digitalWrite(23, LOW);
  digitalWrite(20, LOW);

  int fd = wiringPiSPISetupMode(SPI_CHANNEL, 1000000, 0);
  if (fd == -1)
  {
    std::cout << "Failed to init SPI communication.\n";
    return -1;
  }
  std::cout << "SPI communication successfully setup.\n";

  this_thread::sleep_for(chrono::milliseconds(500));

  char strSession[N_STR];

  printf("\nRaspberry Pi for SonicSole: \n\n");
  printf("UART0 IMU and Multi-thread Data Logging\n\n");

  // INPUT ARGUMENTS
  if (argc > 2)
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
  if ((IMU = serialOpen("/dev/ttyS0", 115200)) < 0)
  // if ((IMU = serialOpen ("/dev/ttyS0", 460800)) < 0)
  // if ((IMU = serialOpen ("/dev/ttyS0", 921600)) < 0)
  // if ((IMU = serialOpen ("/dev/ttyS1", 230400)) < 0)
  {
    fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
    return 1;
  }
  printf("UART1 initialized successfully!\n\n");

  // INITIALIZING GPIO (Use wPi pins, not BCM)
  printf("Initializing GPIO...\n\n");
  if (wiringPiSetupGpio() == -1)
  {
    fprintf(stdout, "Unable to start wiringPi: %s\n", strerror(errno));
    return 1;
  }
  printf("GPIO initialized successfully!\n\n");

  structComponentQuaternion dataQuat;
  structComponentLinearAcceleration dataAcce;
  structComponentRawGyro dataGyro;
  structComponentRawAcceleration dataRAcc;

  // IMU DATA PACKET
  uint8_t IMU_PACKET_LENGTH = 52;
  uint8_t dataIMUPacket[IMU_PACKET_LENGTH];

  // CONFIGURING IMU
  printf("Configuring IMU...\n\n");
  YEIsettingsHeader(IMU);
  YEIwriteCommandNoDelay(IMU, CMD_STOP_STREAMING);
  this_thread::sleep_for(chrono::milliseconds(1000));
  YEIwriteCommandValue(IMU, CMD_SET_ACCELEROMETER_RANGE, ACCELEROMETER_RANGE_8G);
  YEIwriteCommandValue(IMU, CMD_SET_GYROSCOPE_RANGE, GYROSCOPE_RANGE_2000);
  YEIwriteCommandValue(IMU, CMD_SET_COMPASS_RANGE, COMPASS_RANGE_1_3);
  YEIwriteCommandValue(IMU, CMD_SET_CALIBRATION_MODE, CALIBRATION_MODE_BIAS_SCALE);
  YEIwriteCommandValue(IMU, CMD_SET_AXIS_DIRECTIONS, AXIS_XR_YF_ZU);
  YEIwriteCommandValue(IMU, CMD_SET_REFERENCE_VECTOR_MODE, REFERENCE_VECTOR_MULTI_REFERENCE_MODE);
  YEIwriteCommandValue(IMU, CMD_SET_COMPASS_ENABLE, FALSE);
  YEIwriteCommandValue(IMU, CMD_SET_FILTER_MODE, FILTER_KALMAN);
  YEIwriteCommandNoDelay(IMU, CMD_BEGIN_GYROSCOPE_AUTOCALIBRATION);
  this_thread::sleep_for(chrono::milliseconds(500));
  YEIwriteCommandNoDelay(IMU, CMD_RESET_FILTER);
  this_thread::sleep_for(chrono::milliseconds(500));
  YEIsetStreamingMode(IMU, READ_TARED_ORIENTATION_AS_QUATERNION, READ_CORRECTED_LINEAR_ACCELERATION, READ_CORRECTED_GYROSCOPE_VECTOR, READ_CORRECTED_ACCELEROMETER_VECTOR, NO_SLOT, NO_SLOT, NO_SLOT, NO_SLOT);

  YEIwriteCommandNoDelay(IMU, CMD_TARE_WITH_CURRENT_ORIENTATION);
  printf("IMU configured successfully!\n\n");

  bool recordState = TRUE;

  // printf("Here\n\n"); // Troubleshooting

  digitalWrite(23, HIGH); // Turn motors on and off to show device is calibrated
  digitalWrite(20, HIGH);
  delay(1000);
  digitalWrite(23, LOW);
  digitalWrite(20, LOW);

  // TIME VARIABLES
  struct timeval tv;
  uint64_t timestampStart = getMicrosTimeStamp();
  uint64_t currentTime;
  uint64_t currentTimeSecs;

  uint64_t previousHeelStrikeTime = getMicrosTimeStamp();
  uint64_t timeBetwwenHeelStrikeSecs;

  uint8_t syncTrigger = 0;
  uint32_t syncTime = 0;

  uint16_t intervalWait = 3000;
  uint32_t headTime;
  uint32_t deltaTime;
  uint32_t cycle = 0;

  // INITIALIZE OFSTREAM FILE
  ofstream dataFile;
  // string fileName = "log_RPi_" + to_string(timestampStart) + ".csv"; //NEW FILE EACH RUN
  string fileName = "log_RPi.csv";

  bool inGaitCycle = false;

  // pinMode
  pinMode(23, OUTPUT);
  pinMode(20, OUTPUT);

  // OPEN FILE
  dataFile.open(fileName);

  // Original File Open
  if (dataFile.is_open())
  {

    printf("File opened\n\n");
  }
  else
  {

    printf("File not opened \n");
    return 0;
  }

  cout << "Start infinite loop...\n\n\n";

  // INIFINITE LOOP
  while (TRUE)
  {

    currentTime = (getMicrosTimeStamp() - timestampStart);

    // ADC channels
    // Channel 0
    digitalWrite(CS, LOW);
    SPIbuff[0] = 1;
    SPIbuff[1] = 160;
    SPIbuff[2] = 0;
    wiringPiSPIDataRW(SPI_CHANNEL, SPIbuff, 3);
    adc_channel0 = SPIbuff[1] << 8 | SPIbuff[2];
    digitalWrite(CS, HIGH);
    // Channel 1
    digitalWrite(CS, LOW);
    SPIbuff[0] = 1;
    SPIbuff[1] = 224;
    SPIbuff[2] = 0;
    wiringPiSPIDataRW(SPI_CHANNEL, SPIbuff, 3);
    adc_channel1 = SPIbuff[1] << 8 | SPIbuff[2];
    digitalWrite(CS, HIGH);

    int collectmotor;

    // IMU data print

    if ((cycle % 100) == 0)
    {

      // cout << cycle + ", " + currentTimeSecs + "seconds" + "/n";
      printf("cycle: %i , time: %0.3f seconds \n", cycle, currentTime);
      printf("IMU Raw Acceleration Vector: %0.2f , %0.2f , %0.2f \n", dataRAcc.r_ax, dataRAcc.r_ay, dataRAcc.r_az);
      // printf("IMU Acceleration Vector: %0.2f , %0.2f , %0.2f \n", dataAcce.ax, dataAcce.ay, dataAcce.az);
      // printf("IMU Gyroscope Vector: %0.2f , %0.2f , %0.2f \n", dataGyro.gx, dataGyro.gy, dataGyro.gz);
      // printf("IMU Quaternion Vector: %0.2f , %0.2f , %0.2f, %0.2f \n", dataQuat.qw, dataQuat.qx, dataQuat.qy, dataQuat.qz);
      printf("Forefoot sensor : %d \n", adc_channel1);
      printf("Hindfoot sensor : %d \n", adc_channel0);
      printf("\n");

      ofstream writer("PressureSensorReadings", ios_base::app);
      writer << "ForeFoot Sensor: " << adc_channel1 << "   "
             << "HindFoot Sensor: " << adc_channel0 << endl;
      writer.close();
      // dataFile.close();
    }

    // Record Max/Min Pressure Sum
    previous_pressure_sum = current_pressure_sum;
    current_pressure_sum = adc_channel0 + adc_channel1;

    if (current_pressure_sum < min_pressure)
    {
      min_pressure = current_pressure_sum;
    }

    else if (current_pressure_sum > max_pressure)
    {
      max_pressure = current_pressure_sum;
    }

    // Record Max/Min Heel Pressure
    previous_heel_pressure = current_heel_pressure;
    current_heel_pressure = adc_channel0;

    if (current_pressure_sum < min_heel_pressure)
    {
      min_heel_pressure = current_heel_pressure;
    }

    else if (current_pressure_sum > max_heel_pressure)
    {
      max_heel_pressure = current_heel_pressure;
    }

    threshold_pressure_sum = (max_pressure - min_pressure) * 0.1 + min_pressure;
    threshold_heel_pressure = (max_heel_pressure - min_heel_pressure) * 0.5 + min_heel_pressure;

    inGaitCycle = false;

/*
    if (current_heel_pressure > threshold_heel_pressure && previous_heel_pressure < threshold_heel_pressure)
    {
      // switch mode

      current_threshold_time = (getMicrosTimeStamp() - start_time) / 1000000;
      // system("scripts/UDP.sh");
      UDPSend(sockfd, reading, len, servaddr);

      if (current_threshold_time <= 1)
      {
        // switch mode
        mode = !mode;
        digitalWrite(23, HIGH); // Turn motors on and off to show device is on
        digitalWrite(20, HIGH);
        delay(500);
        digitalWrite(23, LOW);
        digitalWrite(20, LOW);
        cout << "Mode Switched" << endl;
      }

      start_time = getMicrosTimeStamp();
      // save current threshold time to an array
      threshold_time_array[num_threshold_reached % 20] = current_threshold_time;
      // take the average of the array
      double mean_time = 0;
      for (double t : threshold_time_array)
        mean_time += t;
      mean_time /= 20;
      cout << "mean time: " << mean_time << endl;

      cout << min_pressure << endl;
      cout << max_pressure << endl;
    }*/

    if (currentTime > 600000000)
    {

      cout << "ten minute stop\n";
      // dataFile.close();
      return 0;
    }

    // if (start_up == 1) {

    // put ios::app if you want it to add instead of overwrite
    // dataFile.open(fileName);
    // start_up = 2;

    //}


    dataFile << cycle << "," << currentTime << "," << dataRAcc.r_ax << "," << dataRAcc.r_ay << "," << dataRAcc.r_az
             << "," << dataAcce.ax << "," << dataAcce.ay << "," << dataAcce.az << "," << dataGyro.gx << "," << dataGyro.gy << "," << dataGyro.gz
             << "," << dataQuat.qw << "," << dataQuat.qx << "," << dataQuat.qy << "," << dataQuat.qz << "," << adc_channel0 << "," << adc_channel1 << "," << collectmotor << "\n";

    // this_thread::sleep_for(chrono::milliseconds(2));
    cycle++;
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
