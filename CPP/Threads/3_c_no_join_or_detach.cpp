#include <iostream>
#include <chrono>
#include <thread>
using namespace std;


void run(int count)
{
    while(count-- > 0)
    {
        cout << count << "CPP Nuts" << endl;
    }
    // Simulating some task.
    std::this_thread::sleep_for(chrono::seconds(3));
    cout << "Thread finished." << endl;
}

int main()
{
    std::thread t1(run, 10);
    cout << "main()" << endl;

    // No join or detach will throw an error.

    cout << "main() after" << endl;
    return 0;
}
