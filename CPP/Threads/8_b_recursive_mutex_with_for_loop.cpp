#include <iostream>
#include <thread>
#include <mutex>
using namespace std;


std::recursive_mutex m1;

int main()
{
    for(int i = 0; i < 5; i++)
    {
        m1.lock();
        std::cout << "locked " << i << endl;
    }
    for(int i = 0; i < 5; i++)
    {
        m1.unlock();
        std::cout << "unlocked " << i << endl;
    }

    return 0;
}
