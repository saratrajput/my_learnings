# C++ Unit Testing: Google Test and Google Mock

This directory contains my notes and code from the Udemy course [C++ Unit Testing: Google Test and Google Mock](https://www.udemy.com/share/102T6y3@TP21dlAgOPbMiil2jCoMmsnSQsp4MJxKeX07GffAc_BOJtmsGzh6ESUIq4w-QDlb/).

## Notes
* ```add_library``` command:
    * STATIC: ```add_library(sumLibrary STATIC library_code.cpp)```
        * The library code will get copied into the executable when you are building.
    * SHARED: ```add_library(sumLibrary STATIC library_code.cpp)```
        * The library code will not get copied into the executable when you are building, but that will mean that you need the library available at runtime.

* ```FetchContent_Declare()```
    * First parameter doesn't have to correspond to the actual name of the dependency. For example, either ```googletest``` or ```gtest``` is fine.

## Section 3 - Unit Testing Basics

### What is a Unit Test?

* It is automated testing and regression testing. With a long project you cannot check every line of code manually every time.

* They are used to check one unit of code i.e. one class or one function.

### Unit Test Characteristics

* They have to be able to execute independently.
    * Don't use global variables which are visible in other tests.
* They have to run quickly (in the order of milliseconds).
* They are an isolated entity i.e. they don't rely on any external input, such as waiting for the user to hit the key or networks, etc..

### Types of Testing

* Unit Testing: Basic building blocks of software testing.

* Integration Testing: Verifies the combined functionality of all the modules put together.

* System Testing: End-to-end testing, where the entire application workflow, all the specifications and all the requirements of the app are tested.
    * It is a form of black-box testing, i.e. you don't have access to the code.

> Unit-testing is a form of white-box testing or glass-box testing.

All these fall under the functional testing i.e. they test if the software works correctly.

Other types of testing:
* Stress and load testing: which test your software in extreme conditions.
* Acceptance testing: which tests the user-friendliness of the software.

### Unit Test Structure

The basic structure of a unit test is divided into three parts:
* Arrange
    * Test Setup: where you set all the inputs, preconditions and create the objects that you need for the test.
* Act: Call the method or function that you want to test.
* Assert: Check that the results are correct.

Sometimes you see an extra section where some code cleanup is done. This is not good practice, because if the test fails, that last part may not get executed.

#### Advantages of the Unit Test Structure
* The code under test is clearly separated from the setup steps and from the result.
* Makes code smells obvious, for example, **Act** mixed with **Assert**, or tests that try to do many different things at once.

> Convention in unit-testing: Expected value is the first followed by the actual value: ```ASSERT_EQ(<expected_value>, <result_from_act>);```

* Test Suite: A collection of related tests.

### Assertions

Assertions can have two possible outcomes, **Success** or **Failure**. In case of **Failure**, we have two possibilities, **Fatal** and **Non-fatal**.

* Fatal failure means that if the condition is not satisfied, then the test stops executing.
    * Start with ```ASSERT_*```.
* Non-fatal failure means that the test continues, even though it has failed.
    * Start with ```EXPECT_*```.

### 22. Assertions on Strings

* Numbers in hex (0x..) are a hint that we are looking at pointers, not strings.

* Don't compare strings like ```ASSERT_EQ("HELLO WORLD", <output_of_capitalize_function>);```.
* The right way to do it, is using ```strcmp()```.

* Assertions on C-strings (char*). Wchar_t* is also accepted.

* For ```std::string```, use ```ASSERT_EQ```.

* For C-strings, use ```ASSERT_STREQ(x, y);``` or ```EXPECT_STREQ(x, y);```.
  * They compare the contents, and not the pointers.

* To test that two strings have different contents:
  ```
  ASSERT_STRNE(x, y);
  EXPECT_STRNE(x, y);
  ```

* Ignore the **case**:
  ```
  // Same content, ignore case
  ASSERT_STRCASEEQ(x, y);
  EXPECT_STRCASEEQ(x, y);
  // Different content, ignore case
  ASSERT_STRCASENE(x, y);
  EXPECT_STRCASENE(x, y);
  ```

### 25. Assertions on Exceptions

|Fatal|Non-Fatal|What it Tests|
|ASSERT_THROW(statement, exception)| EXPECT_THROW(statement, exception)| Throws exception of an **exact type**|
|ASSERT_ANY_THROW(statement)| EXPECT_ANY_THROW(statement)| Throws an exception of **any** type|
|ASSERT_NO_THROW(statement)| EXPECT_NO_THROW(statement)| Throws no exception|

### 27. Summary

* Unit tests are useful for regression testing, meaning that when you make a change, they help prevent the introduction of new bugs.
* Unit tests are short, independent and fast.
* Unit testing is part of functional testing, i.e. the correctness of the code is checked. Individual functions/methods are usually tested.
* A unit test is divided into three parts: Arrange(test setup), Act (call the method), Assert(check the result).
* An assertion is where the test condition is checked. They are fatal (ASSERT) or non-fatal (EXPECT).
* There are special assertions for strings.
* Assertions can be used to check if an exception was thrown, or what type of exception was thrown.

## Section 4 - Fixtures: Remove Redundant Code

### 29. Introduction to Test Fixtures

* Test Fixture: It's a class derived from the test class in the testing namespace.
  * Setup: Overridable function which is called **before** each test.
  * Teardown: Overridable function which is called **after** each test.
  * Syntax for tests using test fixture: ```TEST_F(TestSuiteName, Test)```