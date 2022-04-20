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
