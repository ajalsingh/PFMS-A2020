/**
 * @brief main
 * @file main.cpp
 * @author  Ajal Singh
 * @version 1.0
 * @date    April 2020
 */

#include "laser.h"
#include "sonar.h"
#include "rangerfusion.h"
#include "cell.h"

#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <random>

using std::cout;
using std::cin;
using std::endl;

//_____________________________________________________________________________ Function Declarations
/**
 * @brief Prints Fixed Params for Rangers on console
 * @param rangers - vector of pointers
 */
void getFixedParams(const std::vector<Ranger*> rangers);

/**
 * @brief Prints Raw data to console
 * @param raw Data - vector of vector doubles
 */
void getRawData(const std::vector<std::vector<double>> raw);

/**
 * @brief Configures ranger params according to type and user's specifications
 * @param rangers - vector of pointers
 */
void rangerConfiguration(std::vector<Ranger*> &rangers);

/**
 * @brief Creates number of cells specified by user.
 *          \n Randomly set centre location on grid based on ranger max distance
 * @param cells - vector of pointers
 * @param rangers - vector of pointers
 */
void cellConfiguration(std::vector<Cell*> &cells, std::vector<Ranger*> &rangers);

/**
 * @brief Prints coordinates of cell centre
 * @param cells - vector of pointers
 */
void printCellLocations(const std::vector<Cell*> cells);

//_____________________________________________________________________________ Global Variables

int seed = std::chrono::system_clock::now().time_since_epoch().count(); //!< Holds the seed used in random number generation
int fusion_count =1;                                                    //!< The number of fusions that have occured

//_____________________________________________________________________________ Main

/**
 * @brief Main Function creates lasers, sonar and RangerFusion objects and stores them in corresponding vectors
 *          \n Allows the user to configure rangers and cells and displays params/cell locations to console.
 *          \n Passes cells and rangers to fusion class for processing.
 *          \n Get updates cell state data and continuously print States to console
 *          \n Ranger data is continuously passed to fusion class and cell state is continuously returned
 * @note User must manually terminate program.
 */
int main(){
    Laser laser;
    Sonar sonar1, sonar2;
    RangerFusion fusion;

    std::vector<Ranger*> rangers;
    std::vector<RangerInterface*> ranger_interface;
    std::vector<Cell*> cells;
    std::vector<std::vector<double>> raw;

    rangers = {&laser, &sonar1, &sonar2};
    ranger_interface = {&laser, &sonar1, &sonar2};
    getFixedParams(rangers);
    rangerConfiguration(rangers);
    fusion.setRangers(ranger_interface);
    getFixedParams(rangers);

    cellConfiguration(cells, rangers);
    fusion.setCells(cells);
    printCellLocations(cells);

//    raw = fusion.getRawRangeData();
//    getRawData(raw);

    cout<<"\n    ";
    for (int i=0;i<cells.size();i++){
        if (i<10){
            cout<<std::setw(1)<<" C"<<i+1<<std::setw(1)<<" ";
        }
        else{
            cout<<std::setw(1)<<"C"<<i+1<<std::setw(1)<<" ";
        }
    }
    cout<<endl;

    while(fusion_count >= 1){
        fusion.setRangers(ranger_interface);
        raw = fusion.getRawRangeData();
        fusion.grabAndFuseData();
        cells = fusion.getCells();


        cout<<std::setw(2)<<fusion_count<<":  ";
        for (auto cell : cells){
            cout<<std::setw(2)<<cell->getState()<<std::setw(2)<<" ";
        }
        cout<<endl;
        fusion_count++;
        std::this_thread::sleep_for (std::chrono::seconds(1));
    }
    return 0;
}

//_____________________________________________________________________________ Functions

void getFixedParams(const std::vector<Ranger*> rangers){
    cout<<"\nRanger"<<endl;
    cout<<"\nFIXED PARAMETERS:"<<endl;
    for (auto ranger : rangers){
        cout<<"\nSensor Model"<<std::setw(19)<<": "<<ranger->getModel()<<endl;
        cout<<"Field of View "<<std::setw(17)<<"(Degrees) : "<<ranger->getFieldOfView()<<endl;
        if ( ranger->getSensingMethod() == POINT){
            cout<<"Angular Resolution (Degrees) : "<<ranger->getAngularResolution()<<endl;
        }
        cout<<"Min Distance "<<std::setw(18)<<" (Metres)  : "<<ranger->getMinRange()<<endl;
        cout<<"Max Distance "<<std::setw(18)<<" (Metres)  : "<<ranger->getMaxRange()<<endl;
        cout<<"Offset "<<std::setw(24)<<"(degrees) : "<<ranger->getOffset()<<endl;
    }
}

void getRawData(const std::vector<std::vector<double>> raw){
    int sonar = 1, laser = 1;

    for (int i=0;i<raw.size();i++){
        if (raw[i].size() > 1){         //Size is greater than 1, therefore must be laser
            cout<<"\nLaser #"<<laser<<" range (m): "<<endl;
            laser++;
        }
        else{                           //if not laser then sonar
            cout<<"\nSonar #"<<sonar<<" range (m): "<<endl;
            sonar++;
        }

        for (int j=0;j<raw[i].size();j++){
            cout<<raw[i][j]<<endl;
        }
    }
}

void rangerConfiguration(std::vector<Ranger*> &rangers){
    int input;
    int i=1;
    std::string sensor;

    for (auto ranger : rangers){
        const SensingMethod sensing_method = ranger->getSensingMethod();
        if (sensing_method == POINT){
            sensor = "Laser";

            cout<<"\n____________________________________________________"<<endl;
            cout<<"CONFIGURATION OPTIONS\n Laser Angular Resolution (degrees):"<<endl;
            cout<<"1)  10  (default)"<<endl;
            cout<<"2)  30"<<endl;
            cout<<"Choose initialisation option (1 or 2): ";
            cin>>input;
            if (input != 1 && input != 2) {
                cin.clear();
                cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                cout<<"\nInvalid response. Angular Resolution is: "<<ranger->getAngularResolution()<<endl;
            }
            if (input == 2){
                ranger->setAngularResolution(30);
                cout<<"\nLaser Angular Resolution set to: "<<ranger->getAngularResolution()<<endl;
            }
            if (input == 1){
                ranger->setAngularResolution(10);
                cout<<"\nLaser Angular Resolution set to: "<<ranger->getAngularResolution()<<endl;
            }

            cout<<"\n____________________________________________________"<<endl;
            cout<<"CONFIGURATION OPTIONS\nLaser Offset (degrees anti-clockwise):"<<endl;
            cout<<"Enter offset value between -180 and 180: ";
            cin>>input;
            if (input < -180 || input > 180) {  //must be within offset limits
                cin.clear();
                cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                cout<<"\nInvalid response. Laser Offset set to: "<<ranger->getOffset()<<endl;
            }
            else{
                ranger->setOffset(input);
                cout<<"\n"<<sensor<<" Offset set to: "<<ranger->getOffset()<<endl;
            }

        }
        else{
            sensor = "Sonar";

            cout<<"\n____________________________________________________"<<endl;
            cout<<"CONFIGURATION OPTIONS\nSonar"<<i<<" Offset (degrees anti-clockwise):"<<endl;
            cout<<"Enter offset value between -180 and 180: ";
            cin>>input;
            if (input < -180 || input > 180) {
                cin.clear();
                cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                cout<<"\nInvalid response. Sonar"<<i<<" Offset set to: "<<ranger->getOffset()<<endl;
            }
            else{
                ranger->setOffset(input);
                cout<<"\n"<<sensor<<i<<" Offset set to: "<<ranger->getOffset()<<endl;
            }
            i++;
        }
    }
}

void cellConfiguration(std::vector<Cell*> &cells, std::vector<Ranger*> &rangers){
    int input = 0;
    double max_range;

    //obtain max ranger range
    for (auto ranger : rangers){
        if (ranger->getMaxRange() > max_range){
            max_range = ranger->getMaxRange();
        }
    }

    std::default_random_engine gen(seed);
    std::uniform_real_distribution<double> distX(-max_range, max_range);
    std::uniform_real_distribution<double> distY(0.0, max_range);

    cout<<"\n____________________________________________________"<<endl;
    cout<<"CONFIGURATION OPTIONS\nChoose number of cells: ";
    cin>>input;
    if (input <= 0 || input > 100) {
        cin.clear();
        cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        if (input > 100){
            input = 100;
        }
        else{
            input = 1;
        }

        cout<<"\nInvalid response. "<<input<<" cell/s created"<<endl;
    }
    cells.reserve(input);

    for (int i=0;i<input;i++){
        cells.push_back(new Cell());
    }
    cout<<"\nYou have created "<<cells.size()<<" cell/s"<<endl;

    //creation of cells
    for (auto cell : cells){
        cell->setCentre(distX(gen), distY(gen));
        cell->setSide(1);
    }
}

void printCellLocations(const std::vector<Cell*> cells){
    for (int c=0;c<cells.size();c++){
        double x,y;
        cells[c]->getCentre(x,y);
        cout<<"Cell "<<std::setw(2)<<c+1<<":   ("<<x<<", "<<y<<")"<<endl;
    }
}
