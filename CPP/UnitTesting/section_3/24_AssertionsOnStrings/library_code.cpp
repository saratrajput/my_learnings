#include "library_code.hpp"
#include <cctype>
#include <cstring>

void to_upper(char *input_string)
{
    for (unsigned i = 0; i < strlen(input_string); i++)
    {
        input_string[i] = toupper(input_string[i]); // a -> A, b -> B
    }
}