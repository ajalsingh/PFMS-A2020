#include <thread> //For accessing the standard C++ thread library
#include <chrono> //Used for creating thread delays
#include <iostream>
#include <queue>
#include <mutex>
#include <condition_variable>

//Define a new data structure called 'TDataBuffer' and instansiate a global variable
//of this structure called 'sequence'.
struct TDataBuffer
{
  //Task 2
  std::mutex mu;
  std::condition_variable cond;
  std::queue<long> buffer;
};

void fibonacci(TDataBuffer &sequence)
{
  static long a = 0;
  static long b = 1;

  sequence.buffer.push(a);
  while (true)
  {
    //Task 3
    std::unique_lock<std::mutex> lock(sequence.mu);
    sequence.buffer.push(b);
    lock.unlock();
    //Task 4
    sequence.cond.notify_one();
    b = a + b;
    a = b - a;

    std::this_thread::sleep_for(std::chrono::milliseconds(750));
  }
}

void printToTerminal(TDataBuffer &sequence)
{
  while (true)
  {
    //Task 3
    std::unique_lock<std::mutex> lock(sequence.mu); //Check if should be here?
    //Task 4
    sequence.cond.wait(lock);
    long next = sequence.buffer.front();
    sequence.buffer.pop();
    lock.unlock();

    std::cout << "Next in sequence: " << next << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
}

int main(void)
{

  TDataBuffer sequence;
  //Create the threads
  //Task 5
  std::thread producer(fibonacci, std::ref(sequence));
  std::thread consumer(printToTerminal, std::ref(sequence));

  //Task 1
  producer.join();
  consumer.join();

  return 0;
}
