#include <iostream>
#include "SonicSole.h"
// #include "boost/asio.hpp"
// #include <json/json.h>



using namespace std; 

// struct soleData {
//   int sole-> currForePressure;
//   int sole-> currHeelPressure;
// }

// std::string createJsonPayload(const SoleData& data){
//   root["currForePressure"] = data.currForePressure;

//   JSON::StreamWriterBuilder writer;
//   return Json::writeString(writer, root);
// }

int main(int argc, char* argv[])
{
    SonicSole* sole = new SonicSole();
    cout << "SonicSole Class Initialized" << endl;


    int cycle = 0;
    vector<int> thresholdTimes; 

    while (true) {
      int time = sole->getRunningTime();
      // int time = 0;
      cout << "\ntime: " << time << endl;
      // printf("time: %0.3f seconds \n", time);
      cout << "Cycle: " << cycle << endl;
      cycle++;

      sole->updateCurrentTime();
      sole->updatePressure();

    // {
    //   if (sole->detectHeelThreshold()) {
    //     sole->updateThresholdCounter();
    //     thresholdTimes.push_back(time);
    //   }

    //   /* 
    //    * places time values in thresholdTimes vector whenever the threshold is crossed
    //    * if threshold has been crossed 3 times, runs a check to see if the threshold
    //    * was crossed in sucession (within 3 seconds)
    //    * if it has, mode switches and the vector clears
    //    */ 
    //   if (sole->thresholdCross >= 3) {
    //     bool modeSwitch = false;
    //     for (size_t i = 2; i < thresholdTimes.size(); i++) {
    //         if (thresholdTimes[i] - thresholdTimes[i - 2] <= 3) { // within 3 seconds
    //             modeSwitch = true;
    //             break;
    //         }
    //     }

    //     if (modeSwitch) {
    //         sole->switchMode();
    //         thresholdTimes.clear(); // Clear the threshold times after mode switch
    //     }
    //   }
      
      
    //   if (sole->getMode()) {    // when getMode is true, soundMode is active, if false than vibMode
    //     sole->runSoundMode();
    //   } else {
    //     sole->runVibrateMode();
    //   }

    //   if (sole->detectHeelThreshold()) {
    //     //sole->timeArr.push_back(cycle);
    //     sole->sendFlexSensorData(1);
    //     // cout << "Data sent to UDP" << endl;
    //   }
      
    // }


      cout << "\nFore Pressure: " << sole->currForePressure << endl;
      cout << "Heel Pressure: " << sole->currHeelPressure << endl;

      cout << "\n" << endl;

      sole->readIMU();

      sole->sendFlexSensorData((int)sole->currForePressure, 20000);      
      sole->sendFlexSensorData((int)sole->currHeelPressure, 21000);



      // sole->readIMU();
      // sole->toCSV();
      // cout << "\n";



      delay(100);
      // if (sole->getRunningTime() > MAX_RUN_TIME) { 
      //    cout << sole->getRunningTime() << endl;
      //    cout << MAX_RUN_TIME << endl;
      //    return 0;
      // }
    }
    return 0;
}