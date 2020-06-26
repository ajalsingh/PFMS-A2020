#include <iostream>
#include <thread>
#include <vector>
#include <mutex>

//int shared_int (0);
std::mutex mu;

void increment (int &shared_int) {
   for (int i=0; i<10000000; ++i) {
       //RAII
       std::lock_guard<std::mutex> guard(mu);   //whenever guard goes out of scope, mutex will always be unlocked (with or without exception)
       shared_int++;
   }
};

int main ()
{
    int shared_int (0);
    std::thread th1(increment, std::ref(shared_int));
    std::thread th2(increment, std::ref(shared_int));
    th1.join();
    th2.join();
    std::cout << "Final value: " << shared_int << std::endl;
    return 0;
}
