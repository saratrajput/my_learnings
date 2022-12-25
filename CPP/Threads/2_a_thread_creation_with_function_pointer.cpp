#include <iostream>
#include <thread>
#include <chrono>
#include <algorithm>

using namespace std;
using namespace std::chrono;

void fun(int x)
{
    while (x-- > 0)
    {
        cout << x << endl;
    }
}

int main()
{
    std::thread t(fun, 10);
    t.join();
    return 0;
}
