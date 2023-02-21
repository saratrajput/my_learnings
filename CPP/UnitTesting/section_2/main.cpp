#include <iostream>
#include "library_code.hpp"


int main(int argc, char **argv)
{
    int s = sum(2, 3);
    std::cout << "Actual application\n";
    std::cout << "sum = " << s << "\n";
    return 0;
}
