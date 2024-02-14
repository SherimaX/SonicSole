// // This is a helpful paper for starting out with serial on Windows
// // http://www.robbayer.com/files/serial-win.pdf
// // The next one is a bit more complicated and not used in the examples uses overlapped io
// // http://msdn.microsoft.com/en-us/library/ms810467.aspx
// #include <Windows.h>
// #include <stdio.h>

// // Start Bytes, indicate the start of a command packet
// #define TSS_START_BYTE 0xf7
// #define TSS_WIRELESS_START_BYTE 0xf8
// #define TSS_RESPONSE_HEADER_START_BYTE 0xf9
// #define TSS_WIRELESS_RESPONSE_HEADER_START_BYTE 0xfa


// // Response Header bit options 
// // Details and descriptions are in the YEI 3-Space Manual under "Wired Response Header"
// #define TSS_RH_SUCCESS_FAILURE 0x01
// #define TSS_RH_TIMESTAMP 0x02
// #define TSS_RH_COMMAND_ECHO 0x04
// #define TSS_RH_CHECKSUM 0x08
// #define TSS_RH_LOGICAL_ID 0x10
// #define TSS_RH_SERIAL_NUMBER 0x20
// #define TSS_RH_DATA_LENGTH 0x40


// // The 3-Space sensors are Big Endian and x86 is Little Endian
// // So the bytes need be swapped around, this function can convert from and
// // to big endian
// void endian_swap_16(unsigned short * x)
// {
//     *x = (*x>>8) |
//         (*x<<8);
// }

// void endian_swap_32(unsigned int * x)
// {
//     *x = (*x>>24) |
//         ((*x<<8) & 0x00FF0000) |
//         ((*x>>8) & 0x0000FF00) |
//          (*x<<24);
// }

// // This is a convenience function to calculate the last byte in the packet
// // Commands without parameters can use the same number as the command
// // \param command_bytes The address of the array to sum, this does not include the start byte
// // \param num_bytes The number of bytes in the array Data + 1 for wired. Data + 2 for wireless
// // \return checksum
// unsigned char createChecksum(unsigned char * command_bytes, const unsigned int num_bytes)
// {
// 	unsigned int chkSum = 0;
// 	unsigned int i;
// 	for (i = 0; i < num_bytes; i++){
// 		chkSum += command_bytes[i];
// 	}
// 	return (unsigned char)(chkSum % 256);
// }

// // This creates the comport handle and does initial setup like baudrates and timeouts
// // \param comport The path to the port, comports higher than 8 require \\\\.\\ prepended to open
// // \return com_handle If the open or configuaration fails, INVALID_HANDLE_VALUE is returned
// HANDLE openAndSetupComPort(const TCHAR* comport)
// {
// 	HANDLE com_handle;
// 	DCB dcb_serial_params = {0};
// 	COMMTIMEOUTS timeouts = {0};
// 	DWORD com_error;
// 	COMSTAT comstat;

// 	com_handle = CreateFile(comport, 
// 							GENERIC_READ | GENERIC_WRITE, 
// 							0, 
// 							0, 
// 							OPEN_EXISTING,
// 							FILE_ATTRIBUTE_NORMAL,
// 							0);
// 	if (com_handle == INVALID_HANDLE_VALUE){
// 		printf ("Error opening port\n");
// 		return INVALID_HANDLE_VALUE;
// 	}

// 	// Setting up the baud rate
// 	dcb_serial_params.DCBlength = sizeof(dcb_serial_params);
// 	if (!GetCommState(com_handle, &dcb_serial_params)){
// 		printf ("Failed to get the previous state of the serial port\n");
// 		CloseHandle(com_handle);
// 		return INVALID_HANDLE_VALUE;
// 	}
// 	dcb_serial_params.BaudRate = 115200; // default baud rate for 3-Space Sensors
// 	dcb_serial_params.ByteSize = 8;
// 	dcb_serial_params.StopBits = ONESTOPBIT;
// 	dcb_serial_params.Parity = NOPARITY;
// 	if(!SetCommState(com_handle, &dcb_serial_params)){
// 		printf("Failed to set the serial port's state\n");
// 	}
// 	// Setting the timeouts, tweaking these can improve reliability
// 	// Bluetooth may need longer timeouts with some adapters
//     timeouts.ReadIntervalTimeout = -1;
//     timeouts.ReadTotalTimeoutConstant = 100;
//     timeouts.ReadTotalTimeoutMultiplier = 10;
//     timeouts.WriteTotalTimeoutConstant = 100;
//     timeouts.WriteTotalTimeoutMultiplier = 10;
// 	if(!SetCommTimeouts(com_handle, &timeouts)){
// 		printf( "Failed to set the timeouts\n" );
// 		CloseHandle(com_handle);
// 		return INVALID_HANDLE_VALUE;
// 	}

// 	// Flush out any data that was on the line from a previous session
// 	Sleep(300);
// 	ClearCommError(com_handle, &com_error, &comstat);
// 	if(comstat.cbInQue != 0){
// 		PurgeComm(com_handle,PURGE_RXCLEAR|PURGE_TXCLEAR);
// 	}

// 	return com_handle;
// }


#include "yei_threespace_basic_utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>

// For a full list of streamable commands refer to the "Wired Streaming Mode" section in the 
// 3-Space Manual of your sensor
#define TSS_GET_TARED_ORIENTATION_AS_QUATERNION 0x00
#define TSS_GET_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE 0x29
#define TSS_GET_BUTTON_STATE 0xfa
#define TSS_NULL 0xff // No command use to fill the empty slots in "set stream slots"
#define TSS_SET_STREAMING_SLOTS 0x50
#define TSS_GET_STREAMING_BATCH 0x54

#pragma pack(push,1)
typedef struct Batch_Data{
    float quaternion[4]; // xyzw
    float linear_acceleration[3]; // xyz
	unsigned char button_state; // bit 0 - button 0, bit 1 - button 1
} Batch_Data;
#pragma pack(pop)

int openAndSetupComPort(const char* device){
	int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
	if(fd < 0){
		printf("Error opening %s: ", device);
		return -1;
	}

	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if(tcgetattr(fd, &tty) != 0){
		printf("Error from tcgetattr: ");
		return -1;
	}

	cfsetospeed(&tty, B9600);
	cfsetispeed(&tty, B9600);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
	                                // no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
	                                // enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= 0;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if(tcsetattr(fd, TCSANOW, &tty) != 0){
		printf("Error from tcsetattr: ");
		return -1;
	}
	return fd;
}

void endian_swap_32(unsigned int *x){
	*x = (*x>>24) | ((*x<<8) & 0x00FF0000) | ((*x>>8) & 0x0000FF00) | (*x<<24);
}

int main() {
	int com_handle;
	ssize_t bytes_written;
	ssize_t bytes_read;
	unsigned char write_slot_bytes[11]; // start byte, command, data(8), checksum
	unsigned char write_batch_bytes[3]; // start byte, command, checksum
	Batch_Data batch_data={0};
	int i;

	com_handle = openAndSetupComPort("/dev/ttyUSB0"); // Adjust as necessary
	if(com_handle < 0){
		printf("comport open failed\n");
		return 1;
	}

	printf("TSS_SET_STREAMING_SLOTS\n");
	write_slot_bytes[0]= TSS_START_BYTE;
	write_slot_bytes[1]= TSS_SET_STREAMING_SLOTS;
	write_slot_bytes[2]= TSS_GET_TARED_ORIENTATION_AS_QUATERNION;
	write_slot_bytes[3]= TSS_GET_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE;
	write_slot_bytes[4]= TSS_GET_BUTTON_STATE;
	for(i = 5; i <= 9; i++) write_slot_bytes[i] = TSS_NULL;
	write_slot_bytes[10]= createChecksum(&write_slot_bytes[1], 8+1);

	if(write(com_handle, write_slot_bytes, sizeof(write_slot_bytes)) != sizeof(write_slot_bytes)){
		printf("Error writing to port\n");
		return 2;
	}

	printf("TSS_GET_STREAMING_BATCH\n");
	write_batch_bytes[0]= TSS_START_BYTE;
	write_batch_bytes[1]= TSS_GET_STREAMING_BATCH;
	write_batch_bytes[2] = createChecksum(&write_batch_bytes[1], 1); // Calculate checksum for the command

	if(write(com_handle, write_batch_bytes, sizeof(write_batch_bytes)) != sizeof(write_batch_bytes)){
		printf("Error writing to port\n");
		return 3;
	}

	// Read the batch data
	if(read(com_handle, &batch_data, sizeof(batch_data)) != sizeof(batch_data)){
		printf("Error reading from port\n");
		return 4;
	}

	// Convert the endianess of the received data for proper interpretation
	for(i = 0; i < sizeof(batch_data.quaternion) / sizeof(float); i++){
		endian_swap_32((unsigned int *)&batch_data.quaternion[i]);
	}
	for(i = 0; i < sizeof(batch_data.linear_acceleration) / sizeof(float); i++){
		endian_swap_32((unsigned int *)&batch_data.linear_acceleration[i]);
	}

	// Print the received data
	printf("Quaternion:   % 8.5f, % 8.5f, % 8.5f, % 8.5f\n",
		batch_data.quaternion[0],
		batch_data.quaternion[1],
		batch_data.quaternion[2],
		batch_data.quaternion[3]);

	printf("Linear Accel: % 8.5f, % 8.5f, % 8.5f\n",
		batch_data.linear_acceleration[0],
		batch_data.linear_acceleration[1],
		batch_data.linear_acceleration[2]);

	printf("Button State:  %u\n", batch_data.button_state);

	// Close the serial port
	close(com_handle);

	printf("Finished, press Enter to continue");
	getchar(); // Wait for user input before closing
	return 0;
}
