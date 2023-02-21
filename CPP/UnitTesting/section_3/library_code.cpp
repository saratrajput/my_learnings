#include "library_code.hpp"

#include <algorithm>

bool is_positive(int x)
{
    return x >= 0;
}

int count_positives(std::vector<int> const& input_vector)
{
    return std::count_if(input_vector.begin(), input_vector.end(), is_positive);
}
