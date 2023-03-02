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
```
def test_IntAssert():
    assert 1 == 1

def test_StrAssert():
    assert "str" == "str"

def test_floatAssert():
    assert 1.0 == 1.0

def test_arrayAssert():
    assert[1, 2, 3] == [1, 2, 3]

def test_dictAssert():
    assert{"1":1} == {"1":1}
```

### Comparing Floating Point Values
* Validating floating point values can sometimes be difficult as internally the value is a binary fractions (i.e. 1/3 is internally 0.333333...)
* Because of this some floating point comparisons that would be expected to pass fail.
* The pytest "approx" function can be used to verify that two floating point values are "approximately" equivalent to each other with a default tolerance of 1e-6.

```
# Failing test
def test_BadFloatCompare():
    assert(0.1 + 0.2) == 0.3

# Passing test
def test_GoodFloatCompare():
    val = 0.1 + 0.2
    assert val == approx(0.3)
```

### Verifying Exceptions
* In some cases we want to verify that a function throws an exception under certain conditions.
* Pytest provides the "raises" helper to perform this verification using the "with" keyword.
* If the specified exception is not raised in the code block specified after the "raises" line then the test fails.

```
def test_Exception():
    with raises(ValueError):
        raise ValueError
```

### PyTest Command Line Arguments

#### Specifying What Tests Should Run
* By default PyTest will automatically discover and run all tests in all properly named modules from the current working directory and sub-directories.
* There are several command line arguments for controlling which discovered tests actually are executed.
    * moduleName - Simply specify the module name to run only the tests in that module.
    * DirectoryName/ - Runs any tests found in the specified directory.
    * -k "expression" - Matches tests found that match the evaluatable expression in the string. The string values can include module, class and function names (i.e. "TestClass and TestFunction").
    * -m "expression" - Matches tests found that have a "pytest.mark" decorator that matches the specified expression.

#### Additional Useful Command Line Arguments
* -v: Report in verbose mode.
* -q: Run in quiet mode (can be helpful when running hundreds or thousands of tests at once).
* -s: Don't capture console output(show pring statements on the console).
* --ignore: Ignore the specified path when discovering tests.
* --maxfail: Stop after the specified number of failures.

## Unit Test Isolation with Dummies, Fakes, Stubs, Spies, and Mocks

### What are Test Doubles?
* Almost all code depends (i.e. collaborates) with other parts of the system.
* Those other parts of the system are not always easy to replicate in the unit test environment or would make tests slow if used directly.
* Test doubles are objects that are used in unit tests as replacements to the real production system collaborators.

### Types of Test Doubles
* Dummy - Objects that can be passed around as necessary but do not have any type of test implementation and should never be used.
* Fake - These object generally have a simplified functional implementation of a particular interface that is adequate for testing but not for production.
* Stub - These objects provide implementations with canned answers that are suitable for the test.
* Spies - These objects provide implementations that record the values that were passed in so they can be used by the test.
* Mocks - These objects are pre-programmed to expect specific calls and parameters and can throw exceptions when necessary.

### Mock Frameworks
* Most mock frameworks provide easy ways for automatically creating any of these types of test doubles at runtime.
* They provide a fast means for creating mocking expectations for your tests.
* They can be much more efficient than implementing custom mock object of your own creation.
* Creating mock objects by hand can be tedious and error prone.

### unittest.mock
* Python Mocking Framework.
* Built-in to Python version 3.3 and newer.
* Needs to be installed for older versions of Python with the command "pip install mock".

### unittest.mock - Mock Class
* unittest.mock provides the Mock class which can be used as a fake, stub, spy, or true mock for all your tests.
* The Mock class has many initialization parameters for controlling its behaviour.
* Once it has been called a Mock object has many built-in functions for verifying how it was used.

Example
```
def test_Foo():
    bar = Mock()
    functionThatUsesBar(bar)
    bar.assert_called_once()
```

### Mock - Initialization
* Mock provides many initialization parameters which can be used to control the mock objects behaviour.
* The 'spec' parameter specifies the interface that Mock object is implementing.
* The 'side_effect' parameters specifies a function that should be called when the mock is called.
* The 'return_value' parameter specifies the return value when the Mock is called.

Example
```
def test_Foo():
    bar = Mock(spec=SpecClass)
    bar2 = Mock(side_effect=barFunc)
    bar3 = Mock(return_value=1)
```

### Mock - Verification
* Mock provides many built-in functions for verifying how it was used such as the following asserts:
    * assert_called - Assert the mock was called.
    * assert_called_once - Assert the mock was called once.
    * assert_called_with - Assert the last call to the mock was with the specified parameters.
    * assert_called_once_with - Assert the mock was called once with the specified parameters.
    * assert_any_call - Assert the mock was ever called with the specified parameters.
    * assert_not_called - Assert the mock was not called.

### Mock - Additional Verification
* Mock provides these additional built-in attributes for verification:
    * assert_has_calls - Assert the mock was called with the list of calls.
    * called - A boolean value including if the mock was every called.
    * call_count - An integer value representing the number of times the mock object was called.
    * call_args - The arguments the mock was last called with.
    * call_args_list - A list containing the arguments that were used for each call to the mock.

### unittest.mock - MagicMock Class
* unittest.mock also provides the MagicMock class.
* MagicMock is derived from Mock and provides a default implementation of many of the default "magic" methods defined for objects in Python (i.e. ```__str__```).
* The following magic methods are not implemented by default in MagicMock: ```__getattr__, __setattr__, __init__, __new__, __prepare__, __instantcheck__, __subclasscheck__```, and ```__del__```.
* I will used MagicMock in all of the examples and I use it by default in practice as it can simplify test setup.

### PyTest Monkeypatch Test Fixture
* PyTest provides the monkeypatch test fixture to allow a test to dynamically replace:
    * module and class attributes.
    * Dictionary entries.
    * Environment variables.

```
def callIt():
    print("Hello world.")

def test_patch(monkeypatch):
    monkeypatch(callIt, Mock())
    callIt()
    callIt.assert_called_once()
```

## TDD Best Practices

### Always Do the Next Simplest Test Case
* Doing the next simplest test case allows you to gradually increase the complexity of your code.
* If you jump into the complex test cases too quickly you will find yourself stuck writing a lot of functionality all at once.
* Beyond just slowing you down, this can also lead to bad design decisions.

### Use Descriptive Test Names
* Code is read 1000 times more than it's written. Make it clear and readable!
* Unit tests are the best documentation for how your code works. Make them easy to understand.
* Test suites should name the class or function under test and the test names should describe the functionality being tested.

### Keep Test Fast
* One of the biggest benefits of TDD is the fast feedback on how your changes have affected things.
* This goes away if your unit tests take more than a few seconds to build and run.
* To help your test stay fast try to:
    * Keep console output to a minimum. This slows things down and can clutter up the testing framework output.
    * Mock out any slow collaborators with test doubles that are fast.

### Use Code Coverage Tools
* Once you have all your test cases covered and you think you're done run your unit test through a code coverage tool.
* This can help you identify any test cases you may have missed (i.e. negative test cases).
* You should have a goal of 100% code coverage in functions with real logic in them (i.e. not simple getters/setters).

### Run Your Tests Multiple Times and In Random Order
* Running your tests many times will help ensure that you don't have any flaky tests that fail intermittently.
* Running your tests in random order ensures that your tests don't have any dependencies between each other.
* Use the pytest-random-order plugin to randomize the order that the tests are executed pytest-repeat plugin to repeat one or more tests a specific number of times.

### Use a Static Code Analysis Tool
* Pylint is an excellent open source static code analyis tool that will find erros in your code that you may have missed in your testing.
* Pylint can verify your python code meets your team's coding standard (or the PEP8 standard by default).
* Pylint can also detect duplicated code and can generate UML diagrams from it's analyis of the code.

### Test Behaviour Rather Than Implementation
* When writing your tests try to test the behaviour rather than the implementation.
* When your test is written to verify the behaviour rather than the implementation then the implementation can change without affecting your test.
* This is not always possible as some implementations use collaborators that need to be mocked out.
* In addition, some testing is specifically to verify that the implementation is calling and handling responses from collaborators correctly (i.e. database and network calls).

### Testing Implementation Example
```
def addDays(theDate, days):
    return theDate.timedelta(days=days)

def test_addDaysImplementation(monkeypatch):
    mock_delta = MagicMock(return_value=datetime.timedelta(days=1))
    monkeypatch(datetime.timedelta, mock_delta)
    addDays(datetime.datetime(2020, 1, 1), 1)
    mock_delta.assert_called_once_with(days=1)
```

*Testing Behaviour Instead*

```
def addDays(theDate, days):
    return theDate.timedelta(days=days)

def test_addDaysImplementation(monkeypatch):
    result = addDays(datetime.datetime(2020, 1, 1), 1)
    assert result == datetime.datetime(2020, 1, 2)
```

## Recommended Readings
* Kent Beck - Test Driven Development: By Example
* Robert Martin - Clean Code: A Handbook of Agile Software Craftsmanship
* Michael Feathers - Working Effectively with Legacy Code
* Watch Robert Martin's "Clean Code" video series: https://cleancoders.com

## Appendix
* Test for image processing algorithm to see if the same resulting output is produced
every time: [link](https://github.com/danielgatis/rembg/blob/main/tests/test_remove.py)

```
pip install imagehash==4.2.1
from imagehash import average_hash

actual_hash = average_hash(Image.open(BytesIO(actual)))
expected_hash = average_hash(Image.open(BytesIO(expected)))

assert actual_hash == expected_hash
```
