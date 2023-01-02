#include <iostream>
#include <chrono>
#include <thread>
using namespace std;


void run(int count)
{
    while(count-- > 0)
    {
        cout << "CPP Nuts" << endl;
    }
    // Simulating some task.
    std::this_thread::sleep_for(chrono::seconds(3));
}

int main()
{
    std::thread t1(run, 10);
    cout << "main()" << endl;
    t1.join();

    // Some code here.

    // t1.join();  // Double join will throw system error.

    // Check if thread is joinable.
    if (t1.joinable())
    {
        t1.join();
    }

    cout << "main() after" << endl;
    return 0;
}
