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

    t1.detach();
    // t1.detach();  // Double detach will throw an error.

    cout << "main() after" << endl;
    // std::this_thread::sleep_for(chrono::seconds(3));
    return 0;
}
