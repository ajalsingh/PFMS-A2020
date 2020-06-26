// Automotive Radar
#include "radar.h"

#include <iostream>
#include <limits>
#include <vector>
#include <time.h>
#include <chrono>
#include <thread>
#include <iomanip>
#include <pthread.h>

//_____________________________________________________________________________ Function Declarations

void getFixedParams(Radar &radar);
void configureRadar(Radar &radar);
void getCoupledParams(Radar &radar);
void initTargets(Radar &radar);
void pollRadar(Radar &radar);

//_____________________________________________________________________________ Global Variables

// Becomes true when continuous quering of sensor required
bool continuous_query = 0;

//_____________________________________________________________________________ Main

int main () {

    Radar radar;
    getFixedParams(radar);
    configureRadar(radar);
    getCoupledParams(radar);

    initTargets(radar);
    pollRadar(radar);

    continuous_query = 1;
    configureRadar(radar);
    getCoupledParams(radar);

    pollRadar(radar);
    return 0;
}

//_____________________________________________________________________________ Functions

void getFixedParams(Radar &radar){
    std::cout<<"\nAUTOMOTIVE RADAR"<<std::endl;
    std::cout<<"\nFIXED PARAMETERS:"<<std::endl;
    std::cout<<"Model                       : "<<radar.getModel()<<std::endl;
    std::cout<<"Output                      : "<<radar.getOutput()<<std::endl;
    std::cout<<"Field of View (Degrees)     : "<<radar.getFieldOfViewDeg()<<std::endl;
    std::cout<<"Min Distance ( metres)      : "<<radar.getMinDist()<<std::endl;
    std::cout<<"Max Number of Targets       : "<<radar.getMaxTargets()<<"\n"<<std::endl;
    //std::this_thread::sleep_for (std::chrono::milliseconds(1200));
}


/*  @brief User selects a configuration to configure the sensor
 */
void configureRadar(Radar &radar){
    int input;
    std::cout<<"\n____________________________________________________"<<std::endl;
    std::cout<<"CONFIGURATION OPTIONS\n     Scanning Time (ms):"<<"    Max Distance (m):"<<std::endl;
    std::cout<<"1)           100          "<<"          80";
    if (continuous_query == 0){
        std::cout<<"        (default)"<<std::endl;
    }
    else{
        std::cout<<std::endl;
    }
    std::cout<<"2)           200          "<<"          20\n"<<std::endl;
    //std::this_thread::sleep_for (std::chrono::milliseconds(1200));
    std::cout<<"Choose initialisation option (1 or 2): ";
    std::cin>>input;
    std::cout<<"____________________________________________________"<<std::endl;
    if (input != 1 && input != 2) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout<<"\nInvalid response. Existing configuration selected. \n "<<std::endl;
    }
    if (input == 2){
        std::cout<<"\nInitialisation Complete \n "<<std::endl;
        radar.setMaxDist(20);
        radar.setScanTime(200);

    }
    if (input == 1){
        std::cout<<"\nInitialisation Complete \n "<<std::endl;
        radar.setMaxDist(80);
        radar.setScanTime(100);

    }

}

void getCoupledParams(Radar &radar){
    std::cout<<"\nVARIABLE COUPLED PARAMETERS:"<<std::endl;
    std::cout<<"Scanning Time (ms)          : "<<radar.getScanTime()<<std::endl;
    std::cout<<"Max Distance (m)            : "<<radar.getMaxDist()<<std::endl;
    std::this_thread::sleep_for (std::chrono::milliseconds(700));
}

void initTargets(Radar &radar){
    radar.setData();
}

void pollRadar(Radar &radar){
    radar.getData();
    if (continuous_query == 0){
        for (auto start = std::chrono::steady_clock::now(), now = start; now < start + std::chrono::seconds(5); now = std::chrono::steady_clock::now()){
            std::cout<<"\n____________________________________________________"<<std::endl;
            std::cout<<"Data age: "<<radar.getQueryTime()<<" sec"<<"     Config age: "<<radar.getConfigTime()<<" sec"<<std::endl;
            std::cout<<"\nTargets: "<<radar.getData().at(0)<<"     Sample: "<<radar.getData().at(1)<<std::endl;
            std::cout<<"____________________________________________________"<<std::endl;
            for (int i = 2; i < radar.getData().size(); i+=2){
                std::cout<<std::setw(2)<<i/2<<") "<<" range: "<<std::setw(8)<<radar.getData().at(i)<<"     bearing: "<<radar.getData().at(i+1)<<std::endl;
            }
            radar.poll();
        }
    }
    else{
        while(1){
            std::cout<<"\n____________________________________________________"<<std::endl;
            std::cout<<"Data age: "<<radar.getQueryTime()<<" sec"<<"     Config age: "<<radar.getConfigTime()<<" sec"<<std::endl;
            std::cout<<"\nTargets: "<<radar.getData().at(0)<<"     Sample: "<<radar.getData().at(1)<<std::endl;
            std::cout<<"____________________________________________________"<<std::endl;
            for (int i = 2; i < radar.getData().size(); i+=2){
                std::cout<<std::setw(2)<<i/2<<") "<<" range: "<<std::setw(8)<<radar.getData().at(i)<<"     bearing: "<<radar.getData().at(i+1)<<std::endl;
            }
            radar.poll();
            std::cout<<"Config age: "<<radar.getConfigTime()<<std::endl;
            std::cout<<"Config age: "<<radar.getConfigTime()<<std::endl;
        }
    }
}

