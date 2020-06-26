#include <iostream>
#include <thread>
#include <vector>
#include <mutex>

int shared_int (0);

// Class mutex is a synchronization primitive that is used to protect shared data from simultaneous access.
// A mutex can be locked and unlocked. Once a mutex is locked, current thread owns mutex until it is not unlocked.
// It means that no other thread can execute any instructions from the block of code surrounded by mutex until
// thread that owns mutex unlocks it.
std::mutex m;

void increment ()
{
    m.lock();
    for (int i=0; i<10000000; ++i)
    {
        shared_int++;
    }
    m.unlock();
};

int main ()
{
    std::thread th1(increment);
    std::thread th2(increment);
    th1.join();
    th2.join();
    std::cout << "Final value: " << shared_int << std::endl;
    return 0;
}
