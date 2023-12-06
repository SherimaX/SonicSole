#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#define PORT 25000

int main(int argc, char* argv[]) {
    sendFlexSensorData(103);
    sendFlexSensorData(26);
    sendFlexSensorData(1743);
    sendFlexSensorData(302);
    sendFlexSensorData(592);
    sendFlexSensorData(0);

    return 0;
}

static void sendFlexSensorData(int flexSensorData) {
    int sockfd;
    struct sockaddr_in serverAddr;

    // UDP Socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        std::cerr << "Error creating socket" << std::endl;
        return;
    }

    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(PORT);
    serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1"); // localhost

    /*
    if (sendto(sockfd, &flexData, sizeof(flexData), 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) == -1) {
        std::cerr << "Error sending data" << std::endl;
    } else {
        std::cout << "Flex sensor data sent successfully!" << std::endl;
    }
    */

    try {
        UDPSend(sockfd, &flexSensorData, sizeof(flexSensorData), serverAddr);
        // sendto(sockfd, &flexData, sizeof(flexData), 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    }
    catch (...) {
        std:cout << "Error: UDPSend cannot send data" << endl;
    }

    std::cout << "Data sent to UDP" << endl;
    close(sockfd);
}

static void UDPSend(int sockfd, const int *reading, socklen_t len, struct sockaddr_in servaddr) {
    sendto(sockfd, (const int *)reading, len,
           MSG_CONFIRM, (const struct sockaddr *) &servaddr,
           sizeof(servaddr));
}