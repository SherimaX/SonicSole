#include <iostream>
#include "SonicSole.h"
using namespace std;
////////////////////////////////////////////////////
// MAIN CODE

int main(int argc, char* argv[])
{
    SonicSole* sole = new SonicSole();
    cout << "SonicSole Class Initialized" << endl;

    sole->motorVibrate();
    cout << "Motor Vibrated" << endl;

    int cycle = 0;

    while (true) {
      uint64_t time = sole->getCurrentTime();
      cout << "time: " << time << endl;
      cout << "Cycle: " << cycle << endl;
      cycle++;

      sole->updateCurrentTime();

      sole->updatePressure();

      sole->detectModeChange();

      if (sole->getMode()) {
          sole->runSoundMode();
      } else {
          sole->runVibrateMode();
      }
      

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