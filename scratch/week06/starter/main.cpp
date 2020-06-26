#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <queue>

struct st{
    std::string name;
    std::vector<double> vec;
    std::mutex mu;


};

void incrementNum(int &num, std::mutex &numMutex) {
    while (true) {
    //Use mutex 
    }
}

void printNum(int &num, std::mutex &numMutex) {
    while (true) {
    //Use mutex 
    }
}

void populateQueue(std::queue<int> &queue){
    for (int i=0;i<500000;i++){
//        if (i/2 == 0){
//            queue.push(1);
//        }
//        else{
//            queue.push(-1);
//        }
        queue.push(1);
        queue.push(-1);
    }
}

void total(std::queue<int> &num, int &total, std::mutex &mtx){
//    mtx.lock();
    std::lock_guard<std::mutex> guard(mtx);
    while (!num.empty()){
//        std::lock_guard<std::mutex> guard(mtx);
        total +=num.front();
        num.pop();
        mtx.unlock();
        mtx.lock();
    }
//    mtx.unlock();
}

int main ()
{
    int num = 0;
    // We will use this mutex to synchonise access to num
    std::mutex numMutex;
    std::queue<int> queue;
    int total1, total2;

    populateQueue(queue);
    // Create the threads, how do we create the thread? What are the parameters
    std::thread pop_thread1(total, std::ref(queue), std::ref(total1), std::ref(numMutex));
    std::thread pop_thread2(total, std::ref(queue), std::ref(total2), std::ref(numMutex));
    //std::thread print_thread();

    // Wait for the threads to finish (they wont)
    pop_thread1.join();
    pop_thread2.join();

    std::cout<<total1<<std::endl;
    std::cout<<total2<<std::endl;
    //inc_thread.join();
    //print_thread.join();

    return 0;
}



