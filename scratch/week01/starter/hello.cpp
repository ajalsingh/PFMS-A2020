#include <iostream>

bool sq(double &num)
{
    bool positive = false;
    it (num>0){
        positive = true;
    }
    num*=num;
    return positive;
}

double sqcube(double &num, double &sq, double &cube){
    bool positive = false;
    if (num>0){
        positive = true;
    }
    sq = num*num;
    cube = num*sq;
    num++;
}

int main () {
    std::cout << "Hello Class" << "Hello everyone" << std::endl;
    ex05(5);
    return 0;
}
