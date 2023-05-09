#include <iostream>
#include <gtest/gtest.h>
#include "library_code.hpp"


TEST(TestCountPositives, BasicTest)
{
    // Arrange
    std::vector<int> input_vector{1, -2, 3, -4, 5, -6, -7};

    // Act
    int count = count_positives(input_vector);

    // Assert
    ASSERT_EQ(3, count);
}

TEST(TestCountPositives, EmptyVectorTest)
{
    // Arrange
    std::vector<int> input_vector{};

    // Act
    int count = count_positives(input_vector);

    // Assert
    ASSERT_EQ(0, count);
}

TEST(TestCountPositives, AllNegativesTest)
{
    // Arrange
    std::vector<int> input_vector{-1, -2, -3};

    // Act
    int count = count_positives(input_vector);

    // Assert
    ASSERT_EQ(0, count);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
