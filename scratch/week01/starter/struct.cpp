#include <iostream>

static const int max_size=10;

struct Sensor{
    int num_samples;
    double data[max_size];
}sensor;

int main () {
    sensor.num_samples = 20;
    for (int i=0;i<sensor.num_samples;i++){
        sensor.data[i]=i;
    }

    std::cout << "[" << sensor.num_samples << "]" << std::endl;

    for(int i=0;i<sensor.num_samples;i++){
        std::cout << sensor.data[i] << std::endl;
    }
    return 0;
}
