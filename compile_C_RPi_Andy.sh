g++ -o C_RPi_Andy C_RPi_Andy.cpp SonicSole.cpp -lwiringPi -lpthread -std=c++17
#g++ -o C_RPi_Andy C_RPi_Andy.cpp SonicSole.cpp -I./IMUAPI -L./IMUAPI -lthreespace_api -lwiringPi -lpthread -std=c++17