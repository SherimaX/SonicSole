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

    sole->openCSVFile("accel_data.csv");

    structComponentQuaternion *dataQuat;
    structComponentLinearAcceleration *dataAcce;
    structComponentRawGyro *dataGyro;
    structComponentRawAcceleration *dataRAcc;

    int cycle = 0;
    vector<int> thresholdTimes; 
    vector<float> axData; 
    vector<float> ayData; 
    vector<float> azData;

    while (true) {
      int time = sole->getRunningTime();
      // int time = 0;
      cout << "\ntime: " << time << endl;
      // printf("time: %0.3f seconds \n", time);
      cout << "Cycle: " << cycle << endl;
      cycle++;

      sole->updateCurrentTime();
      sole->updatePressure();
      // sole->readIMU();

      // // sole->getAccelVectorData(sole->ax, sole->ay, sole->az, axData, ayData, azData);
      // sole->getAccelVectorData(sole->az, azData);

      // sole->toCSV(sole->az);

      // debugging
      // cout << "DEBUG /// dataAcce.ay: " << sole->az << endl;
      // cout << "DEBUG /// axData size: " << axData.size() << ", ayData size: " << ayData.size() << ", azData size: " << azData.size() << endl;
      // if (!axData.empty()) {
      //       cout << "DEBUG /// Latest axData: " << axData.back() << endl;
      // }

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
      
      
    //   if (sole->getMode()) {
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

      sole->sendFlexSensorData((int)sole->currForePressure, 20000);      
      sole->sendFlexSensorData((int)sole->currHeelPressure, 21000);

      delay(100);
      // if (sole->getRunningTime() > MAX_RUN_TIME) { 
      //    cout << sole->getRunningTime() << endl;
      //    cout << MAX_RUN_TIME << endl;
      //    return 0;
      // }
    }
    return 0;
}