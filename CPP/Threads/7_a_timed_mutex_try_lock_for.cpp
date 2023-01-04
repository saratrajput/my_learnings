#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>


using namespace std;


int my_amount = 0;
std::timed_mutex m;


void increment(int i)
{
    if(m.try_lock_for(std::chrono::seconds(1)))
    {
        ++my_amount;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        cout << "Thread " << i << " entered" << endl;
        m.unlock();
    }
    else
    {
        cout << "Thread " << i << " couldn't enter." << endl;
    }
}


int main()
{
    std::thread t1(increment, 1);
    std::thread t2(increment, 2);

    t1.join();
    t2.join();

    cout << my_amount << endl;
    return 0;
}
