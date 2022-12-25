#include <iostream>
#include <thread>
#include <chrono>
#include <algorithm>

using namespace std;
using namespace std::chrono;

int main()
{
    // Lambda function.
    // auto fun = [](int x)
    // {
    //     while (x-- > 0)
    //     {
    //         cout << x << endl;
    //     }
    // };
    // std::thread t(fun, 10);

    // We can directly inject lambda at thread creation time.
    std::thread t([](int x)
                  {
        while (x-- > 0)
        {
            cout << x << endl;
        } },
                  10);

    t.join();
    return 0;
}
