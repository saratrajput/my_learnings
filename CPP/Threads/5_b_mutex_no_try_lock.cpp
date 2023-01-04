#include <iostream>
#include <thread>
#include <mutex>
using namespace std;


int counter = 0;
std::mutex mtx;


void increaseTheCounterFor100000Time()
{
    for (int i=0; i<100000; i++)
    {
        // if (mtx.try_lock())
        // {
        mtx.lock();
        ++counter;
        mtx.unlock();
        // }
    }
}


int main()
{
    std::thread t1(increaseTheCounterFor100000Time);
    std::thread t2(increaseTheCounterFor100000Time);

    t1.join();
    t2.join();

    cout << " counter could increase upto : " << counter << endl;

    return 0;
}
