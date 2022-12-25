#include <iostream>
#include <thread>
#include <chrono>
#include <algorithm>

using namespace std;
using namespace std::chrono;

class Base
{
public:
    static void run(int x)
    {
        while (x-- > 0)
        {
            cout << x << endl;
        }
    }
};

int main()
{
    std::thread t(&Base::run, 10);
    t.join();
    return 0;
}
