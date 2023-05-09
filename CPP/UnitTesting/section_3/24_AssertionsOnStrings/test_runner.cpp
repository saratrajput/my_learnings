#include <iostream>
#include <string>
#include <gtest/gtest.h>
#include "library_code.hpp"

TEST(ToUpperTest, BasicTest)
{
    // Arrange
    char input_string[] = "Hello World";

    // Act
    to_upper(input_string);

    // Assert
    ASSERT_STREQ("HELLO WORLD", input_string);
    // ASSERT_STRCASEEQ("Hello WORLD", input_string);

    // or the other example:
    // std::string str(input_string);
    // ASSERT_EQ("HELLO_WORLD", str);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}