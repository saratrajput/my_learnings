# Unit Testing and Test Driven Development in Python

* Unit Testing: Testing at the function level.
* Component Testing: Testing is at the library and compiled binary level.
* System Testing: Tests the external interfaces of a system which is a collection of sub-systems.
* Performance Testing: Testing done at sub-system and system levels to verify timing and resource usages are acceptable.
* Phases of test-driven-development: Red, Green, Refactor.
* Unit tests are written before the production code.
* Don't write all the tests or production code at once.
* Tests and production code are both written together in small bits of functionality.

### Benefits of TDD
* Gives you the confidence to change the code.
* Gives you immediate feedback.
* Documents what the code is doing.
* Drives good object oriented design.

### TDD
* Create by Kent Beck.
* Part of Extreme Programming software development process.

### TDD Workflow
* Red phase: Write a failing unit test.
* Green phase: Write just enough production code to make that test pass.
* Refactor phase: Refactor the unit test and the production code to make it clean.

### Uncle Bob's Laws of TDD
* You may not write any production code until you have written a failing unit test.
* You may not write more of a unit test than is sufficient to fail, and not compiling is failing.
* You may not write more production code than is sufficient to pass the currently failing unit test.

## PyTest
* PyTest is a python unit testing framework.
* It provides the ability to create Tests, Test Modules, and Test Fixtures.
* Uses the built-in Python assert statement.
* Has command line parameters to help filter which tests are executed and in what order.

### Creating a Test
* Tests are python functions with "test" at the beginning of the function name.
* Tests do verification of values using the standard python assert statement.
* Similar tests can be grouped together by including them in the same module or class.

### Test Discovery
* Pytest will automatically discover tests when you execute based on a standard naming convention.
* Test functions should include "test" at the beginning of the function name.
* Classes with tests in them should have "Test" at the beginning of the class name and not have an "__init__" method.
* Filenames of test modules should start or end with "test".

## XUnit Style Setup and Teardown
* XUnit style setup/teardown functions will execute code before and after:
    * Test Modules
    * Test Functions
    * Test Classes
    * Test Methods in Test Classes

* Use ```-s``` argument so that pytest doesn't capture the console output. You can see the print statements this way.
```
pytest -v -s test_file_setup_and_teardown.py
```

## Test Fixtures
* Test Fixtures allow for re-use of setup and teardown code across tests.
* The "pytest.fixture" decorator is applied to functions that are decorators.
* Individual unit tests can specify which fixtures they want executed.
* The autouse parameter can be set to true to automatically execute a fixture before each test.

### Test Fixture Teardown
* Test Fixtures can each have their own optional teardown code which is called after a fixture goes out of scope.
* There are two methods for specifying teardown code. The "yield" keyword and the request-context object's "addfinalizer" method.

### Test Fixture Teardown - Yield
* When the "yield" keyword is used the code after the yield is executed after the fixture goes out of scope.
* The "yield" keyword is a replacement for the return keyword so any return values are also specified in the yield statement.

### Test Fixture Teardown - addfinalizer
* With the addfinalizer method a teardown method is defined added via the request-context's addfinalizer method.
* Multiple finalization functions can be specified.

### Test Fixtures Scope
* Test Fixtures can have the following four different scopes which specify how often the fixture will be called:
    * Function: Run the fixture once for each test.
    * Class: Run the fixture once for each class of tests.
    * Module: Run once when the module goes in scope.
    * Session: The fixture is run when pytest starts.

### Test Fixture Return Objects and Params
* Test Fixtures can optionally return data which can be used in the test.
* The optional "params" array argument in the fixture decorator can be used to specify the data returned to the test.
* When a "params" argument is specified then the test will be called one time with each value specified.

## Assert Statements and Exceptions
* Pytest allows the use of the built in python assert statement for performing verifications in a unit test.
* Comparison on all of the python data types can be performed using the standard comparison operators: <, >, <=, >=, ==, and !=
* Pytest expands on the message returned from assert failures to provide more context in the test results.

### Comparing Floating Point Values
* Validating floating point values can sometimes be difficult as internally the value is a binary fractions (i.e. 1/3 is internally 0.333333...)
* Because of this some floating point comparisons that would be expected to pass fail.
* The pytest "approx" function can be used to verify that two floating point values are "approximately" equivalent to each other with a default tolerance of 1e-6.

### Verifying Exceptions
* In some cases 