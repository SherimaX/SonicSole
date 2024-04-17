// #include <iostream>
#include "SonicSole.h"

using namespace std; 

int main(int argc, char* argv[])
{
    SonicSole* sole = new SonicSole();
    cout << "SonicSole Class Initialized" << endl;

    sole->motorVibrate();
    cout << "Motor Vibrated" << endl;

    int cycle = 0;

    while (true) {
      uint64_t time = sole->getRunningTime();
      cout << "time: " << time << endl;
      cout << "Cycle: " << cycle << endl;
      cycle++;

      sole->updateCurrentTime();

      sole->updatePressure();

      //if (sole->getRunningTime() < tEnd) {
      // if (cycle % 30 == 0) { // every 10 cycles is a second
      //   sole->detectHeelThreshold();
      // }

      if (sole->thresholdCross >= 3) {
        sole->switchMode();
      }

      // sole->detectModeChange();

      if (sole->getMode()) {    // when getMode is true, soundMode is active, if false than vibMode
          sole->runSoundMode();
      } else {
          sole->runVibrateMode();
      }

      //sole->sendFlexSensorData(sole->getCurrHeelPressure());

      //if (sole->getMode() && sole->detectHeelThreshold()) {
      if (sole->detectHeelThreshold()) {
        sole->sendFlexSensorData(1);
        // cout << "Data sent to UDP" << endl;
      }

      //if () {
        
      //}
      sole->readIMU();
      
      sole->toCSV();
      cout << "\n";
      delay(100);
      // if (sole->getRunningTime() > MAX_RUN_TIME) { 
      //    cout << sole->getRunningTime() << endl;
      //    cout << MAX_RUN_TIME << endl;
      //    return 0;
      // }
    }
    return 0;
}