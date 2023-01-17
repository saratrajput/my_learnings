#include <gtest/gtest.h>
#include "library_code.hpp"


TEST(TestSuiteSample, TestSample)
{
    int result = sum(2, 4);
    ASSERT_EQ(6, result);
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
