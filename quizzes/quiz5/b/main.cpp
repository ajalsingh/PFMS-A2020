#include <thread> //For accessing the standard C++ thread library
#include <chrono> //Used for creating thread delays
#include <iostream>
#include <queue>
#include <mutex>
#include <condition_variable>

//Define a new data structure called 'TDataBuffer' and instansiate a global variable
//of this structure called 'sequence'.
struct TDataBuffer {
  std::mutex mu;
  std::queue<long> buffer;
} ;

std::condition_variable cv;
bool ready = false;

void fibonacci(TDataBuffer &sequence) {
  static long a = 0;
  static long b = 1;

  std::unique_lock<std::mutex> lock(sequence.mu);
  sequence.buffer.push(a);
  sequence.mu.unlock();
  while(true) {
    std::unique_lock<std::mutex> lock(sequence.mu);
    sequence.buffer.push(b);
    ready=true;
    cv.notify_one();
    sequence.mu.unlock();
    
    b = a + b;
    a = b - a;

    std::this_thread::sleep_for(std::chrono::milliseconds(750));
    
  }
}

void printToTerminal(TDataBuffer &sequence) {
  while(true) {
    std::unique_lock<std::mutex> guard(sequence.mu);
    while(!ready){
      cv.wait(guard);
    }

    if(sequence.buffer.empty()) {
      std::cout << "<buffer was empty>" << std::endl;
    } else {
      // std::unique_lock<std::mutex> guard(sequence.mu);
      long next = sequence.buffer.front();
      sequence.buffer.pop();
      // sequence.mu.unlock();
      std::cout << "Next in sequence: " << next << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ready = false;
  }
}


int main (void) {
  TDataBuffer sequence;
  //Create the threads
  std::thread producer(fibonacci, std::ref(sequence));
  std::thread consumer(printToTerminal, std::ref(sequence));
  producer.join();
  consumer.join();
  return 0;
}
