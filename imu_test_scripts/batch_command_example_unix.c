#include "yei_threespace_basic_utils.h"
#include <stdio.h>
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <sys/types.h>
#include <sys/stat.h>

#define TSS_GET_TARED_ORIENTATION_AS_QUATERNION 0x00
#define TSS_GET_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE 0x29
#define TSS_GET_BUTTON_STATE 0xfa
#define TSS_NULL 0xff
#define TSS_SET_STREAMING_SLOTS 0x50
#define TSS_GET_STREAMING_BATCH 0x54

#pragma pack(push,1)
typedef struct Batch_Data {
    float quaternion[4];
    float linear_acceleration[3];
    unsigned char button_state;
} Batch_Data;
#pragma pack(pop)

int openAndSetupComPort(const char* portname) {
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)B115200);
    cfsetispeed(&tty, (speed_t)B115200);

    tty.c_cflag |= (CLOCAL | CREAD);    // ignore modem controls
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         // 8-bit characters
    tty.c_cflag &= ~PARENB;     // no parity bit
    tty.c_cflag &= ~CSTOPB;     // only need 1 stop bit
    tty.c_cflag &= ~CRTSCTS;    // no hardware flowcontrol

    // setup for non-canonical mode
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    // fetch bytes as they become available
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return fd;
}

int main() {
    int com_fd;
    ssize_t bytes_written;
    ssize_t bytes_read;
    unsigned char write_slot_bytes[11];
    unsigned char write_batch_bytes[3];
    Batch_Data batch_data={0};
    int i;

    com_fd = openAndSetupComPort("/dev/ttyUSB0"); // Adjust as necessary
    if (com_fd < 0) {
        printf("comport open failed\n");
        return 1;
    }

    // The rest of your main function goes here, with WriteFile replaced by write and ReadFile by read.
    // Remember to use com_fd where you previously used com_handle.
    // Also, replace CloseHandle(com_handle) with close(com_fd).

    // Close the serial port
    close(com_fd);
    printf("Finished, press Enter to continue");
    getchar();
    return 0;
}
