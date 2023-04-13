# Programming with Julia

* [Course link](https://www.udemy.com/course/programming-with-julia/)
* [Github](https://github.com/ilkerarslan/JuliaCourseCodes.git)

## Section 1: Introduction

### 1. Introduction

#### Advantages of Julia
* Julia is an excellent language for studying, learning, and teaching programming, especially in numerical computing, scientific programming, data science, machine learning, and deep learning.
* Julia is unique because it is written in Julia, and all libraries and operations are written in Julia.
* Julia's syntax is similar to that of R, MATLAB, and Python, making it easy to learn.
* Julia's speed competes with that of C or C++.
* Julia solves the "two language problem" by offering the simplicity of high-level languages like Python, R, or MATLAB, along with the speed of low-level languages like C or C++.
* Julia makes it possible to develop models and deploy them using the same language, cutting project times significantly.

#### Comparison with other Programming Languages

* Unlike R and Python, Julia libraries are written in Julia, and using them is similar to using Julia.
* Julia is a great programming language for statistics, math, data science, machine learning, deep learning, and scientific computing.

### 2. History of Julia

* Julia was first developed by a group of four researchers at M.I.T.: Jeff Bezanson, Stephan Karpinsky, Viral Shah, and Alan Edelman.
* The main problem they wanted to solve was the "two-language problem" and lift the barriers between researchers and I.T. departments.
* Julia is an open-source project that was launched in 2012.
* Julia's creators wanted to create a programming language that had the best features of other languages, such as the speed of C, the flexibility of Python, R, and MATLAB, and was simple to learn and both interactive and compiled.
* Julia is used in many scientific projects, ranging from financial modeling to pharmaceutical modeling, and from critical infrastructure optimization to real-time energy forecasting.
* The creators of Julia also established a company named Julia Computing, which helps companies adapt to Julia and accelerate their business with it.

### 3. Why Julia?

* Julia solves the two-language problem for analytics projects, where code is first developed in a dynamic language like Python or R and then refactored in a lower-level language like C# or Java for production.
* Julia is a modern language designed for numerical computing, data science, machine learning, and AI, and can be integrated with production platforms easily.
* Julia is faster than Python and R, which are useful for front-end user interfaces, but slow for runtime speed.
* Julia is faster than C++ for runtime speed, but more readable than C++ for coding.
* Julia is a just-in-time compiler language that converts code to machine code for execution on the CPU runtime, making it faster than interpreted languages.
* Julia is easy to learn and write and has a growing package ecosystem, with currently over 6800 packages available.

## Section 2: Starting with Julia

### 5. Installing Julia in Windows

* Go to julialang.org
* Click on the "download" button on the main page.
* Go to the download page and select the version you want to download (stable or long term support).
* Choose the appropriate version for your operating system and click on the 64-bit installer.
* After the download is complete, run the installer file.
* Click "next" on each step of the installer.
* Select "add Julia to PATH" and click "next".
* Complete the installation process.
* Julia is now installed and ready to use.

### 6. Installing Julia in Linux

* Go to the **julialang.org** website.
* Navigate to the download page.
* Select the 64-bit version of Linux and download it.
* Locate the downloaded file in the Downloads folder.
* Extract the file either using a file extractor or through the terminal with the command ```tar zxvf [filename]```.
* Move the extracted folder to another folder with any desired name (e.g., "julia").
    ```
    sudo mkdir -p /julia/
    sudo mv julia-*/ /julia/
    ```
* Add the Julia folder to PATH by creating a symbolic link to Julia. Use the following commands in the terminal:
    ```
    # (optional - to remove any previous installations)
    sudo rm /usr/local/bin/julia
    # Create symbolic link.
    sudo ln -s ~/[folder name]/bin/julia /usr/local/bin/julia"
    ```
* Reopen the terminal and type ```julia``` to verify that it is installed successfully.

### Installing Julia in Mac

* Find the processor type for your Mac.
  ```
  uname -m
  ```
  * If the output is arm64, then you have a Mac with an M-series processor, which means you are running macOS ARM.
  * If the output is x86_64, then you have a Mac with an Intel processor, which means you are running macOS x86.
* Go to the **julialang.org** website.
* Navigate to the download page.
* Download the ```.dmg``` file based on the processor type in Step-1.
* Double-click on the downloaded file.
* Drag the downloaded file into the ```Applications``` folder.
* Create symbolic link.
  ```
  sudo ln -s /Applications/Julia-1.8.app/Contents/Resources/julia/bin/julia julia
  ```

### 7. Julia REPL

* Julia REPL is an interactive command line prompt for writing small Julia expressions or code pieces.
* To start Julia REPL, type "Julia" in the command line or click on the Julia shortcut.
* Different keys can be used to access different prompts in Julia REPL, such as the help screen or shell screen.
* The package manager in Julia REPL allows for managing packages, adding, removing, or updating them.
* Code can also be written in a file with a .jl extension and called from REPL using the include function.
* Julia REPL is not suitable for longer and more complex programs, data analysis, or data visualization, for which code editors or IDEs are needed.

#### Installing Packages in Julia REPL

* To install a package, launch Julia REPL by typing ```julia``` in the command line, and press ```Enter```.

* Press ```]```, to enter the **pkg** mode.
    * This will add ```pkg>``` prompt to your current prompt.

* To install a package:
```
add VirtualEnv
```

* To exit the ```pkg>``` prompt, simply press the **Backspace** or **Del** key on your keyboard.

#### Using a virtual environment in Julia

* Enter Julia REPL.

* Enter ```pkg>``` prompt, by pressing the ```]``` key.

* Install ```VirtualEnv``` package with:
    ```
    add VirtualEnv
    ```

* Activate a new environment.
    ```
    activate .
    ```
    This will create a ```Project.toml``` file, in the directory where the Julia REPL was launched.
    > If the file is not created, you can create a new file yourself with the following content.
    ```
    name = "LearningJulia"
    version = "0.1.0"
    ```

* Activate an existing environment.
```
using Pkg
Pkg.activate(".")
```

#### Using Jupyter Notebook in Julia

* Install the ```IJulia``` package with, ```add IJulia```, in the ```pkg>``` prompt.

* Exit the ```pkg>``` prompt by pressing the ```Backspace``` or ```Del``` key.

* Load the ```IJulia``` package: ```using IJulia```.

* Launch the Jupyter notebook with the notebook() function: ```notebook()```.
    * To launch the Jupyter notebook in the current directory: ```notebook(dir=".")```.

* Inside the Jupyter notebook interface, click on ```New``` and select the ```Julia x.x.x``` kernel.

### 8. Julia Editor and IDEs

* Visual Studio Code is a favorite editor for coding in Julia.
* Jupiter and Pluto notebooks are also available for coding in Julia.
* To use Jupiter notebooks, install the IJulia package and launch Jupyter from the command line or Anaconda Navigator.
* To use Pluto notebooks, install the Pluto package and launch Pluto from the Julia prompt.
* Visual Studio Code can be downloaded and the Julia extension can be installed from the extensions menu.
* Environments can be created for each project to manage package dependencies.
* Packages can be added to an environment using the package manager and the manifest and project files will be updated.
* Code can be executed in VS Code using keyboard shortcuts or the play icon in the top right corner.

## Section 3: Variables, Data Types and Operations

> In VSCode, press ```Shift + Enter``` to execute the code in the line in the Julia REPL.

### 10. Variables

#### Variables

* Variables are used to store values for later use.
* A variable points to the address in memory where a value is stored.
* Simple examples of variable assignment and operations are demonstrated in VSCode and Julia REPL.
* Variables can have different types and their type can be checked using the ```typeof``` function.
* Variables can be declared with a specific data type using ```variable_name::data_type```, e.g., ```x::Int64```.
    * The above type declaration is not yet supported for global variables. It's usually used in function definitions.
    * Purposes of type declaration:
        * Multiple dispatch.
        * Human readability.
        * Catch errors.
* Changing data types during runtime may be harmful for performance, although it's possible to do so.
* It's important to use descriptive names for variables, especially in longer programs.
* Certain keywords are reserved and cannot be used as variable names.
* Unicode characters, subscripts, and superscripts can be used in variable names, but it is recommended to use descriptive names instead.
* Multiple variables can be assigned values simultaneously.
* Values can be swapped between variables using ```a, b = b, a```.

#### Comments

* Comments are used to provide explanations and notes that are not evaluated by the program.
* Comments start with ```#``` and can be single-line or multi-line using ```#=``` and ```=#```.

#### Constants

* Constants are variables whose values cannot be changed after initialization.
* Constants are declared using the keyword const with uppercase names.
* Changing the data type of a constant value results in an error, but changing its value with the same data type results in a warning.

### 11. Type Hierarchy in Julia

#### Introduction to Types in Julia:

* Types are tags on values that restrict the range of potential values that can be stored at that location.
* Understanding types in Julia is very important.
#### Type Hierarchy:

* Julia has a type hierarchy where all types are in that hierarchy.
* The top of this hierarchy is the type Any.
* All other types are subtypes of Any and Any is a super type of all other types.
* Below Any, there is a long pyramid of type hierarchies.
* The bottom of the hierarchy is the type Union.
* Every type is a subtype of another type in between Any and Union.
* Any has 580 subtypes.
#### Abstract Types and Concrete Types:

* Red types that have subtypes are called abstract types and cannot be instantiated.
* Blue types do not have subtypes and are concrete types that can be instantiated.
* Abstract types allow us to define functions using the group of types we want.
* Concrete types can be used to create variables.
#### Getting Subtypes and Supertypes in Julia:

* To get subtypes and supertypes in Julia, we can use the subtypes and supertype functions.
* The <: operator is used to check whether a type is a subtype of another type.
* Type hierarchy in Julia can be applied to predefined types and user-defined types.

### 12. Numerical Data Types: Integers and Floating-Point Numbers

#### Integer and Floating Point Numbers:

* Integer and floating point numbers are the most widely used basic data types in programming.
* Julia provides a wide range of concrete, integer and floating point numbers.
* Integer types are bool, signed, and unsigned integers where the latter two are abstract types.
* The maximum and minimum values of integer and unsigned integer types can be calculated using formulas.
* The sizeof function can be used to check the size of a variable.
* Numeric values can be separated by underscores for readability purposes.
* Conversion functions can be used to convert between types.
#### Special Symbols in Julia:

* Inf represents infinity and can be used in calculations.
* NaN stands for a value that is not equal to any floating point number.
* It is recommended to check whether the result of a calculation is NaN or infinity using the isnan or isinf functions.
* Epsilon values give the distance between one point and the next floating point.
* Epsilon values are not the same for all points and get larger for large values.
#### NaN and Infinity:

* Inf represents a value greater than all finite floating point values.
* -Inf represents a value less than all finite floating point values.
* NaN stands for a value that is not equal to any floating point number.
* Division by zero is normally undefined in math, but it's Inf in Julia.
#### Bool Type:

* Bool stands for data that can only get false or true values.
* True is the integer 1 and false is the integer 0.
* The equal equal operator can be used to check whether true is equal to 1 and false is equal to 0.

### 13. Numerical Data Types: Complex and Rational Numbers

* Complex numbers are defined as a+bi where a is the real part and bi is the imaginary part.
* In Julia, the imaginary part or square root of -1 is denoted as im.
* We can use the complex function to create a complex number using two real numbers.
* We can use complex numbers in mathematical operations in Julia.
* The conjugate of a complex number a+bi is a-bi.
* Rational data types are defined using the // operator.
* To check whether an object is of a given type, we can use the isa function.

### 14. Character and String Types

* In Julia, there are two data types for text variables: char and string.
* Char type represents a single character and is defined using single quotes. String type is defined using double quotes.
* Unicode characters can also be defined as chars.
* Strings are arrays of characters and are one-indexed in Julia.
* Indexing can be done using square brackets and begin/end keywords.
* Substrings can be obtained using the colon operator and intervals.
* Unicode characters may have multiple code units, which affects indexing and length.
* Strings can be concatenated using the string function or the asterisk operator.
* The asterisk operator is used instead of the plus sign for concatenation because addition is commutative while multiplication is not always commutative.
* The power sign can be used to repeat a string multiple times.
* Interpolation using the dollar sign allows variables and operations to be used within a string.

### 15. Primitive Types and Composite Types

* Primitive types in Julia are concrete types whose data consists of plain old bits.
* Built-in primitive types in Julia include integers, floating point numbers, bool, and char data types.
* Composite types are user-defined types composed of single fields, defined using the struct keyword.
* Structs are immutable by default, but can be made mutable by adding the word "mutable" in front of struct.
* Constructors are used to create instances of structs.
* Union types are used when a variable can be in either one of a union of data types.

### 16. Parametric Types

* Julia has a powerful parametric type system, where types can take parameters using the syntax T.
* This means that data types can be flexible, as long as they have the same type.
* ParRectangle is a type object in Julia, which contains all instances with different data types.
* It is possible to limit the data types a parametric type may have using the <: operator.
* Parametric abstract types can also be defined, and the types T they can have can be limited.
* An example is provided from Julia documentation, where a struct is a subtype of real, and its fields are subtypes of integer.
* A new shape can be defined with four fields, two of them being a subtype of abstract string and the other two being a number type.
* Name and color fields should be of the same type, and x coordinate and y coordinate should also be of the same type.

### 17. Basic Operations

* Julia programming language has basic arithmetic operations like all other programming languages.
* In Julia, we can access the latest evaluated value using the keyboard and ants can be used as a variable in operations.
* Julia uses symbolic mathematics, which means that we don't need to write the multiplication sign between the variables.
* There is also an integer division in Julia, which we can get by typing backslash the and then tapping.
* Julia uses Unicode operators for mathematical and comparison operations. For example, for less than or equal to we just type backslash L Q and for greater than or equal to we type backslash GQ.
* There are numerical comparisons, Boolean operators, bitwise operators, and updating operators in Julia.
* Julia also has built-in mathematical functions like square or cubic growth, remainder, and logarithm function.
* We can use arithmetic operators together with an equal sign to update the value of a variable.
* The floating point automatic may cause bugs and unexpected behaviors. So, in some cases, we may want to check whether two values are close enough using the is approx function in Julia.
* We need to be careful about the floating point automatic because most numbers cannot be stored with a finite number of bits.

## Section 4: Data Structures

* Fundamental data types discussed: strings, integers, floating point numbers.
* User-defined types/structs introduced.
* Data structures are used to store and manipulate multiple data points effectively.
* Examples of data structures in Julia: tables, sets, collections.
* Immutable and mutable data structures were discussed.
* Immutable data structures, such as tables, cannot be modified once created.
* Different data structures may contain different types of data.
* Example of data structure usage: storing student grades and names together.

### 21. Tuples

* Tuples are fixed size sequences of values separated by commas.
* Tuples can have values of different types.
* Tuples are constructed in parentheses, but it's not a necessity.
* Once created, a tuple cannot be modified.
* Tuples can be accessed using square brackets and are one-indexed.
* Named tuples can also be created and accessed using dot connector.
* Tuples with all the same type of elements are called homogeneous tuples.
* Homogeneous tuples are composed of n elements of the same type.
* Tuples can be defined using the function ntuple.
* The in function can be used to check if an element is in a tuple.
* The Unicode equivalent of in can also be used.
* The opposite of the in function is notin.

### 22. Dictionaries

#### Creating and Modifying Dictionaries

* Key-value pairs can be provided as tables.
* Dictionaries are immutable, but new elements can be added or modified.
* Elements in a dictionary are not defined by order, but their name.
* Elements of a dictionary can be accessed using the key names.
* Length, keys, and values functions can be used to get the number of key-value pairs, keys, and values in the dictionary respectively.
* In operator and has key function can be used to check if a key exists in the dictionary.
* Get and getkey functions can be used to get values for keys, with a default value if the key does not exist.
* Delete function can be used to remove a key-value pair from a dictionary.
* Merge function can be used to combine two dictionaries.
* The keys and values in a dictionary do not have to be of the same type.
#### Functions for Dictionaries

* The get function with an exclamation mark returns the value for the given key if it exists, otherwise creates the key with the given value.
* The merge with function can be used to merge two dictionaries by adding values with the same keys in the merge with function.

### 23. Ranges

#### Range Type

* Range type is useful for working with a range of numbers.
* Range is created using the colon operator.
* The collect function converts a range to an array and returns its contents.
* Using ranges increases efficiency.
* There are many range types in Julia.
* Range is composed of integers or real numbers and can be decreasing or increasing.
#### Range Function

* Range function is used to create ranges.
* The range function has four arguments: start, stop, length, and step.
* The function can be called with any three of these arguments.
* The line range function is used for linear rate intervals.

### 24. Arrays

#### I. Introduction

* Sequences of any type of data can be made.
* Arrays are mutable, enclosed with square brackets.
* One-dimensional arrays are vectors, two-dimensional arrays are matrices.
#### II. Creating Arrays

* Using square brackets.
* Data types of the elements can be different.
* Elements can be of the same type.
* Data type can be limited.
* Arrays may have another array as an element.
#### III. Accessing and Modifying Arrays

* Accessing elements by index numbers.
* Slicing.
* Copying an array.
* Changing elements of an array.
* Assigning values to an interval of indices.
* Checking if an element is in an array.
* Checking if an array is a subset of another array.
* Special functions for arrays (typeof, length, push!, append!, deleteat, pop, first, insert, split).

### 25. Vectors and Matrices

* A vector is an ordered finite list of elements; it is a one-dimensional array also called a column vector. A matrix is a rectangular array of elements; it can be thought of as horizontally stacked column vectors.

* To create a vector, elements are separated by commas. To create a matrix, rows are separated by semicolons or pressing enter, and columns are separated by commas.

* To access elements in a matrix, the index for each dimension is provided in square brackets.

* Concatenating arrays can be done vertically or horizontally. If vertical, the sizes of the arrays must match in the second dimension (for matrices) or length (for vectors). If horizontal, the sizes must match in the first dimension (for matrices) or length (for vectors).

* Empty arrays can be created using the undef or nothing keyword, with the type and dimensions specified. Alternatively, an array of a specific data type can be created using the fill function, or an array of random numbers can be created using the rand function.

* Arrays can be reshaped using the reshape function.

* Special functions like zeros, ones, and fill can be used to create arrays with specific values. The similar function can be used to create a new array with the same data type as an existing array.

### 26. Multidimensional Arrays

#### Introduction:

* Scalar values, 1D and 2D arrays are widely used data structures.
* Multidimensional arrays are used for storing and processing large volumes of data.
* Multidimensional arrays can have as many dimensions as needed, depending on the context.
* Examples of 3D arrays are discussed.
#### Creating Multidimensional Arrays:

* To create a multidimensional array, specify the data type and optionally, the number of dimensions.
* Entries are provided in curly brackets and dimensions are separated by commas.
* Arrays can be initialized with specific values or left undefined.
* Arrays can be reshaped into multidimensional arrays.
#### Indexing Multidimensional Arrays:

* Indexing in multidimensional arrays requires one index number for each dimension.
* For arrays in memory, elements are stored sequentially.
* One number can be used to index multidimensional arrays in memory.
#### Concatenating Matrices:

* Matrices can be concatenated horizontally or vertically using semicolons.
* Double semicolons concatenate columns in the same manner.
#### Applications of Multidimensional Arrays:

* Multidimensional arrays are used for image data, with each layer representing red, green, or blue values of each pixel.

### 27. Broadcasting and Dot Syntax

* Broadcasting is a feature of R, Python, and Julia that allows for element-wise binary operations on arrays of different sizes.
* Julia has a broadcast function that automatically performs broadcasting.
* Broadcasting can be used with operators by putting a dot before the operator.
* Broadcasting can be used with functions by putting a dot after the function name.
* Broadcasting is suggested for ease of programming and readability, but it does not speed up runtime relative to loops in Julia.
* Broadcasting can be used to filter arrays for required conditions by using a Boolean array of the same size as the original array.
* Broadcasting for filtering arrays involves applying the condition to the array and then selecting the values that satisfy the condition.
* Broadcasting can be used to select elements from an array, such as the first and last elements of a vector.
* To combine conditions for filtering, bitwise operators and dots are used.
* A random matrix can be filtered for rows where the values in the first column are divisible by three and the values in the second column are divisible by four.

### 28. Sets

* Sets in Julia are similar to sets in mathematics, where they contain only unique elements without any specific order.
* The set function can be used to convert an array or tuple into a set, which eliminates any duplicated elements.
* Sets can be more efficient in cases where order is not important, such as keeping track of words in a document.
* Julia has functions like union, intersection, and setdiff that can be used with sets.
* The symdiff function should only be used with sets as arguments to avoid incorrect results.
* The issubset function can also be used to check if one set is a subset of another.

### 29. Basic Linear Algebra

* Linear algebra is an important part of programming languages like Julia.
* The linear algebra package is required for most of the operations.
* Basic linear algebra functions include trace, determinant, transpose, matrix multiplication, and matrix division.
* The inverse of a matrix is found using the inv function, and the identity matrix is represented with capital I.
* Matrix division is done using the backslash operator.

## Section 5: Conditionals and Loops

* Three ways of completing tasks in programming.
* Handling tasks in order, based on conditions, and until conditions are met.
* Using if statements to handle conditions.
* Using loops to execute tasks repeatedly.
* Nesting loops in programming.

### 33. Compound Expressions

* Compound expressions or begin and blocks are used to execute multiple expressions in order.
* Julia has two ways to perform compound expressions, using begin and end keywords or using semicolons between the expressions in parentheses.
* Return value of a begin and block is the result of the last sub-expression.
* Variables introduced in begin and blocks are accessible from outside the block.
* Begin and blocks are useful for keeping together a list of statements or phrases which don't have values themselves and for returning only the final exploration.
* When using notebooks for Julia, multiple lines of code should be enclosed in a begin and block to avoid errors.

### 34. Conditional Evaluation

#### Handling tasks based on conditions

* Programs can execute tasks based on predefined conditions.
* Example: Converting student scores to letter grades.
* Programs use the if keyword to handle conditions.
* Programs execute commands only if the condition is true.
* Programs can use if-else statements to execute different commands based on the condition.
* Programs can use nested if-else statements for multiple conditions.
* Programs can use a compact form of nested if-else statements using the if keyword.
#### Handling complex conditions

* Programs can use logical operators such as and and or to handle complex conditions.
* Programs can use a specific Julia syntax to check if a value is within a range.
#### Short conditional statements

* Programs can use a short conditional statement called the ternary operator.
* The ternary operator takes three arguments and is used to simplify short conditional statements.

### 35. Short Circuit Evaluation

* We can use the and and or operators in Julia to short circuit simple if conditions.
* The and operator returns true only if both conditions are true, and the or operator returns true if at least one of the conditions is true.
* If a statement is written instead of the second condition, and it is executed, then it is considered true.
* Short circuit evaluation can be used to shortcut if conditions and validate data.
* The ternary operator is also available in Julia for short if conditions in one line.

### 36. For Loops

* Repeated evaluation or looping is the final phase of executing tasks in a computer program.
* There are two kinds of loops in Julia: for loops and while loops.
* For loops are used to perform a task for a given number of times or for each element in a collection.
* The general syntax for a for loop in Julia is for iterator in range statement end.
* The iterator takes the values of the next element in the range and the statement is executed.
* It's possible to use any iterable collection as the range, and we can use functions like size or length.
* We can also use the enumerate function to get both the index and value of an element.
* Nested for loops can be used to iterate through multiple collections.
* In Julia, it's possible to write nested loops in one line.
* We can also iterate through strings, files, and key-value tuples of dictionaries.
* While loops are used when we don't know how many times a task needs to be repeated or when the condition for stopping is more complex than a fixed number of iterations.

### 37. While Loops

* Loops are used to perform repetitive tasks as long as some conditions are met.
* There are two types of loops in Julia: for loops and while loops.
* For loops are used to perform a task for a given number of times or for each element in a collection.
* For loops use an iterator that iterates through the range or collection and performs given tasks.
* For loops can use any iterable collection as the range and can also use functions like size or length.
* Enumerate function can be used to get both the index of an element and the element itself.
* Nested for loops can be used to iterate over a matrix or multiple collections at the same time using the zip function.
* While loops are used to repeat a task as long as a condition holds.
* While loops require an initial state, a condition check, execution of requested tasks, and updating of state.
* Global and local variables should be denoted clearly.
* Examples of loops include iterating over an array, reading lines from a file, and modifying the calculation of sum and average of user inputs.

### 38. Continue and Break

* Loops are used to repeat a task as long as some conditions are met.
* Two kinds of loops in Julia are for loops and while loops.
* For loops are used to perform a task for a given number of times or for each element in a collection. Syntax: for iterator in range do statement end
* While loops are used to repeat a task as long as a condition holds. Syntax: while condition do statement end
* To exit a loop, the break statement is used. To skip an iteration and continue to the next one, the continue statement is used.
* Loops can be used with if statements to execute certain tasks based on certain conditions.
* Nested loops can be used to perform tasks on matrices and collections.
* The zip function can be used to iterate over multiple collections at the same time.
* Good practice to denote global and local variables clearly.

### 39. Comprehensions

* Loops are used to perform tasks repeatedly based on conditions in Julia.
* There are two types of loops in Julia: for loops and while loops.
* Nested for loops can be used to perform tasks in two dimensions.
* While loops are used when the number of iterations is not known in advance.
* Break and continue statements can be used to stop or skip iterations in loops based on specific conditions.
* Comprehensions are a shorter and simpler way of constructing a follow-up loop to create an array. Comprehensions can also use if conditions and multiple iterations, and result in generator objects which are more memory-efficient than arrays.

## Section 6: Functions

#### Introduction:

* Repeating routine jobs in programming is time-consuming.
* Automating repetitive tasks through coding is a better option.
* Functions are used to automate tasks in programming.
#### Functions in Julia:

* Julia functions are similar to mathematical functions.
* Julia functions may change the global state of the program.
* There are two ways of defining functions in Julia.
* Arguments in Julia are passed by reference.
* Passing behavior in Julia is called pass by sharing behavior.
#### Defining functions:

* The first way to define a function in Julia is using the "function" keyword.
* The second way to define a function is using a one-line expression.
* A simple example of a one-line expression function is provided.
#### Automating tasks:

* Automating tasks through coding can save time and effort.
* If a task is repeated more than twice, it is better to write code to automate it.
#### Comprehensions:

* Comprehensions are used to create arrays at the end of a for loop.
* Comprehensions can use multiple iterations, if statements, and nested loops.
* Generators provide memory efficiency and faster performance for large amounts of data.
#### Loops:

* Loops are used to repeat a task as long as a condition holds.
* While loops are used when the length of the process is not known in advance.
* If statements are used in loops to stop or skip iterations based on certain conditions.
* Examples of for loops, while loops, and if statements in Julia are provided.

### 44. Map, Reduce and Filter

#### Benefits of Automating Tasks:

* Automating repetitive tasks saves time.
* Writing a piece of code to automate tasks is better than repeating the task multiple times.
* Functions are the best way to automate tasks.
* If a task is repeated more than twice, it's better to write a function.

#### Functions in Julia:

* Functions in programming and mathematics are similar.
* Two ways to define functions in Julia: using the function keyboard and mathematical function definition.
* Arguments in Julia are passed by reference, except for numbers and characters.
* Argument passing behavior in Julia is sometimes called pass by sharing behavior.

#### Higher Order Functions:

* Using higher order functions simplifies the code and increases readability.
* Map function maps a function onto all the elements in an array.
* Reduce function reduces all the elements in an array to a single value.
* Filter function selects some elements based on given conditions.
* Most operations on arrays can be expressed as a combination of map, reduce, and filter operations.

#### Anonymous Functions:

* Anonymous functions are unnamed functions.
* Anonymous functions can be used with higher order functions like map and reduce.
* Dual keyword creates an anonymous function with argument X and passes it as the first argument to map.

### 45. Variable Number of Arguments

* Variadic functions are functions that can take a variable number of arguments.
* These functions are defined using the ellipsis operator, which is three dots.
* We can define a variadic function using a recursive structure and add behavior for when there is only one argument.
* The ellipsis operator is also used for two other tasks: combining many arguments into one single argument (slurping) and splitting one argument into many arguments (splitting).
* We can use splitting in a function to convert a single argument into separate arguments.

### 46. Optional Arguments

* Default values can be assigned to some or all of the arguments in a function.
* If the user doesn't enter a value for the input, the default value is used.
* A function with two arguments A and B is defined to calculate A to the power of B, and if B is not entered, it calculates the square of A.
* In a function defined to find the minimum number in a sequence of numbers, an error occurs when only one input is provided.
* Default values cannot be provided for positional arguments, only for optional arguments.

### 47. Keyword Arguments

* Default values can be assigned for some or all of the arguments in a function to be used when the user doesn't enter a value for the input.
* We can only provide default values for positional arguments and not for keyword arguments.
* Julia has both positional and keyword arguments. Keyword arguments are separated from positional arguments with a semicolon.
* When using positional arguments, the arguments are taken in the order they are defined and cannot be used with their names.
* Keyword arguments can be used in any order and positional and keyword arguments can be separated with a semicolon but it's not preferred.

### 48. Composite Functions

* Functions can be chained by using the output of one function as input to another function.
* Piping functions is a common practice in programming, and it involves using the pipe operator to send the output of one function to the input of another function.
* Piping functions is more intuitive and prone to fewer mistakes.
* The pipe operator can be used with any number of functions.
* Broadcasting and piping operators can be combined in the same line.
* Root mean square error is a common function used in statistical modelling, and it can be calculated using the pipe operator.

### 49. Mutating Functions

* Julia functions can modify or imitate their arguments.
* Functions that modify their arguments are represented with an exclamation mark at the end of the function name.
* The built-in sort function has two versions: one mutating and one non-mutating.
* Mutating functions change their input argument, while non-mutating functions do not.
* Similarly, imitating functions do not change their input argument.
* The behavior of functions with respect to their input arguments should be considered when choosing which function to use.

## Section 7: Methods

* Functions in Julia have methods, which define their behavior depending on the combination of argument types and counts.
* A method is the definition of one possible behavior for a function.
* Julia uses multiple dispatch to choose which method to implement for a function based on the order, types, and counts of all arguments.
* Multiple dispatch is one of the most important features of Julia.

* Method: Definition of a behavior for a function.
* Dispatch: Choice of which method to implement.
* Single Dispatch: Method is chosen based on the first argument.
* Multiple Dispatch: Method is chosen based on all of the arguments.

### 53. Multiple Dispatch

* Multiple dispatch is a feature of Julia that allows a function to have different behaviors for different types and counts of arguments.
* All built-in functions in Julia use multiple dispatch.
* We can define our own methods for a function, and the most specific method for the argument types is called.
* If there is not a unique, most specific method that can be applied, an error is raised due to method ambiguity.
* To avoid method ambiguity, it's good practice to define the intersection case before the others.
* Optional arguments are tied to the function, not to a specific method, and can be used together with multiple dispatch.
* Methods are dispatched based only on positional arguments, and then keyword arguments are processed.

### 54. Parametric Methods

* Parametric type system can be used in multiple dispatch to provide more flexible and generic functions.
* Types are important in Julia and can be checked and limited using the type() and <: operators.
* Parametric types can be used anywhere applicable.
* Parametric methods can be defined for different types.
* Multiple dispatch can also be used to modify base functions.
* Varg notation can be used to restrict the number of arguments a function can take.

### 55. Function Like Objects

#### Callable Objects or Functors:

* Objects can be made callable by adding methods to them.
* This is also called a functional-like object or a functor.
* For example, we can define a struct to hold coefficients of a model, and define a function on this struct instead of a function name.
* The struct instance can then be called like a function with appropriate arguments to calculate the model result.
#### Using a Struct as a Function in Julia:

* A model result can be calculated using a struct with coefficients and exponents.
* A function can be defined on this struct, which takes exponents as inputs and calculates the model result.
* An instance of the model struct can be called as a function with appropriate arguments to calculate the model result.
* This approach is useful in creating machine learning or deep learning models in Julia.

### 56. Constructors

* Constructors are functions or methods used to create instances of composite types or structs.
* There are two types of constructors in Julia: outer constructors and inner constructors.
* Accessing fields of structs can be done using dot notation.
* Mutability provides flexibility, but immutable types run faster.
* Creating abstract types requires creating concrete subtypes.
* We can use keyword arguments and default fields in constructors to create instances of structs.
* Inner constructors are declared inside types and use a special local function called new to create objects.
* We can construct empty structs using inner constructors, but accessing their elements can result in errors.
* The course struct can be used to enroll students, and a condition can be set to determine if the course is opened.

### 57. Neural Networks Demo

* Constructors are used to create instances of composite types (structs).
* There are two types of constructors in Julia: outer constructors and inner constructors.
* Inner constructors are declared inside the type and have access to a special local function called the new function.
* Julia allows the use of keyword arguments in constructors, which can define default field values.
* Julia allows the creation of abstract types and subtypes of abstract types.
* Julia can be used to create a simple neural network using structures and functions.
* The number of layers in a neural network may be very large, so a better way to perform calculations automatically is necessary.
* A network object can be defined to group layers together and perform calculations on notes with one call.

## Section 8: Modules and Packages

* Keeping everything in one file can cause problems, such as losing track of the code and forgetting to update related parts.
* Separating design and implementation can help in organizing the code and avoiding complications.
* Modules allow us to store variables, constants, types, and functions separately from their implementation, and can increase productivity.
* A module is defined with the "module" keyword, and we can make objects accessible from other files using the "export" keyword.
* We can access modules and their contents using the "using", "import", or explicit calling with the colon operator, and we can include file contents with the "include" command.

### 59. Writing and Using Modules

* Check the modules currently loaded with: ```LOAD_PATH```.
* Check the current directory path with: ```pwd()```.
* Add the current path to the ```LOAD_PATH```.
  ```
  push!(LOAD_PATH, pwd())
  ```

#### Introduction to module:

* Putting everything together in one file causes complications.
* Separating design and implementation can make code manageable.
* Encapsulating types and functions in one place makes them usable from other files.
* Using modules efficiently can increase productivity by 10 times.
#### Creating modules and their contents:

* Three new files are created in a folder named "modules": bank module, customer module, and transactions module.
* The bank module defines a bank struct, a new bank with 1000 of equity, and a default function.
* The customer module defines a customer struct, a new customer with 50 cash, and a default function.
* The transactions module defines a deposit function, a withdrawal function, and uses the customer and bank types.
* Modules are defined with the "module" keyword and can be given the same name as the file.
* Objects in modules are made accessible to other files using the "export" keyword.
* Three keywords to access modules and their contents are "using", "import", and "include".

#### Using modules efficiently:

* Separating design and implementation makes code manageable.
* Encapsulating types and functions in one place makes them usable from other files.
* Using modules efficiently can increase productivity by 10 times.

#### Using auto modules:

* Exported functions can be accessed using a dot connector after the module name.
* Using import accesses contents only after the dot connector.
* Objects or types in modules can also be accessed using the colon operator after using or import.

#### Using include function:

* The include function can be used instead of Launchpad.
* Using include requires accessing modules with the using keyword and putting a dot in front of the module name if the module is part of the main environment.
* Deliberate errors can be used to learn more about module commands.


### 60. Revise Package

* In order to reload a module which has been updated, you can either restart the Julia REPL, or use the ```Revise``` package.
* Install ```Revise``` package with:
  ```Julia
  // Press ']' key
  add Revise
  ```
* And load it with: ```using Revise```.

* Keeping everything together in one file can cause problems, such as difficulty in tracking changes and finding what you need.
* Modules help in separating design and implementation, storing variables, constants, types, and functions separately from their implementation.
* Modules are defined using the module keyword, and objects can be made accessible from other files using the export keyword.
* Three keywords are used to access modules and their contents: using, import, and include.
* Using the Colon operator after using or import allows access to objects and types in a module directly.
* Using revised packages can save time and effort by avoiding the need to restart the Julia REPL and reload modules.
* It is important to use the appropriate commands when working with modules to avoid errors and increase productivity.

### 61. Package Development

* Enter the Julia REPL: ```julia```
* Change directory to Desktop: ```cd("Desktop")``` OR in the present directory.
* Create ```MyPackage```
```
// Press the ']' key to enter pkg mode
generate MyPackage
```

* Encapsulating code in modules is helpful for better code management and reuse.
* Creating packages enables others to use your code and contributes to the Julia community.
* Two ways of creating a package: using the package manager and the packaging templates package.
* The packaging templates package provides an easier and more elegant way of creating packages.
* The package folder contains the module file and the test directory for unit tests.
* Unit tests are important to ensure package functionality.
* Registering a Julia package is simple and can be done through the registration GitHub app or Julia Hub.
* The package management and ecosystem in Julia are underrated features.


#### Another way of creating packages.

* Enter the Julia REPL: ```julia```
* Add ```PkgTemplates```.
```
// Press the ']' key to enter pkg mode
add PkgTemplates

julia> using PkgTemplates

// Interactive
julia> Template(;interactive=true)("NewPackage")
Template keywords to customize:
[press: d=done, a=all, n=none]
   [X] user
   [ ] authors
   [X] dir
   [ ] host
   [ ] julia
 > [ ] plugins
Enter value for 'user' (required): pattarsuraj
Enter value for 'dir' (default: ~/.julia/dev): section_8/new_package
[ Info: Running prehooks
[ Info: Running hooks
...

// Commandline
julia> t = Template(; user="pattarsuraj")
julia> t("BankSim")
```

#### Creating a Julia package repository and Testing

* Create a repository with ```.jl``` extension on GitHub.
> Repo Link: github.com/saratrajput/BankSim.jl

* Copy the modules defined in ```BankModule.jl```, ```CustomerModule.jl``` and ```Transactions.jl``` to ```BankSim.jl``` and add some tests in the ```test/runtests.jl``` file.

* To run the tests:
  * Go to the directory where the ```BankSim.jl``` repo is located.
  * Activate Julia REPL: ```julia```.
  ```
  julia> push!(LOAD_PATH, pwd())
  julia> using BankSim
  // Go to pkg mode.
  julia> ]
  // Activate the BankSim project.
  (@v1.8) pkg> activate .
  // Run the tests
  (BankSim) pkg> test
  ```

#### Registering a Julia Package

##### Method-1
* Open ```https://github.com/JuliaRegistries/Registrator.jl```.
* Install the Registrator Github App.

#### Method-2 (Easy Way)
* Open ```juliahub.com```.
* Click on the ```Register Packages``` link.
* Fill the form.

## Section 9: Metaprogramming

* Meta programming is the technique of writing code that generates new code.
* Code is represented as data in Julia, which is inherited from Lisp.
* Abstract syntax trees (ASTs) represent the code in Julia.
* Meta programming can be more concise, automate boilerplate code, and produce new code for better performance.
* The example of measuring the runtime of a function demonstrates how meta programming can be used.
* The built-in macro in Julia is easier to use than manually writing a function to measure runtime.

### 63. Symbols and Expressions

#### Building Blocks of Code:

* Literals and variables are basic building blocks in Julia and are called symbols.
* Symbols point to different types of objects and are used to access these objects.
* Colon operator is used to represent a symbol in Julia.
* Expression is an object made up of symbols and literals that represents an evaluated Julia code.
* Colon operator is also used to create an expression in Julia.
* Expressions can be evaluated using assigned values of variables.
* Metadata parse function and court and block can be used to define expressions that are more than one line.
#### Symbols and Expressions:

* Symbol is a data type and is a subtype of any.
* Symbol method and eval function can be used to create and evaluate symbols respectively.
* Expr is a data type and is a subtype of any that represents an expression.
* Different types of expressions exist in Julia such as function call, if, while, and for loops.
* PR method can be used to manually create any expression.
#### Using Symbols and Expressions:

* Interpolation using the dollar sign can be used to substitute a variable value in an expression.
* Court not can be used to get the symbol of a symbol or assign a symbol to a variable.
* The interpolated part of an expression is evaluated when the expression is constructed or passed.
#### Meta-Programming and Macros:

* Meta-programming involves writing code that generates new code.
* Macros are a type of meta-programming that can automate boilerplate code and generate code automatically.
* Meta-programming can produce new code rather than executing it, which may improve performance.
* Three main cases where using meta-programming is preferred: more concise and clearer code, automate boilerplate code, and performance improvement.
* Macros can be written in Julia using the macro keyword.

### 64. Writing Macros

#### Introduction to macros:

* Macros take expressions, symbols, or literal values as inputs and create new expressions as output.
* Macros replace one piece of code with another piece of code.
* Macros can create expressions automatically.

#### Defining and using macros:

* A macro definition is similar to a function definition, but with the keyword "macro" instead of "function."
* Macros are invoked with the edit sign in front of the macro name and can take arguments with or without parentheses.
* Macros behave differently from functions when it comes to handling arguments as expressions or data.
* Macros can access the current environment when called.
#### Accessing the environment with macros:

* Macros can access the environment where they are called, which can lead to unexpected behavior.
* The escape function can be used to prevent macros from accessing the environment where they are called.
#### Macro capabilities:

* Macros can be used to chain multiple functions with the pipe operator.
* Function calls can be visualized as abstract syntax trees.
* Macros can use abstract syntax trees to write complex expressions automatically.

## Section 10 - Streams and Networking

#### I. Simple Input and Output from/to a User

* Almost always, programs interact with the outside world, such as users, files, servers, etc.
* Simple input and output functions can be used to interact with users.
* Examples of input functions in Julia are readline() and readlines().
* Examples of output functions in Julia are print(), println(), and printf().
#### II. File Input and Output

* Julia provides various functions to read from and write to files.
* Examples of file input functions in Julia are read(), readlines(), and readline().
* Examples of file output functions in Julia are write() and writedlm().
#### III. Communication between a Server and Client

* In Julia, communication between a server and client can be done using the Sockets module.
* The Sockets module provides functions to create and connect to a socket.
* Once a connection is established, messages can be sent and received between the server and client using read() and write() functions.

### 66. Basic Input and Output

#### Standard Input and Output

* Standard Input and Output are the most basic inputs and outputs.
* They are global variables which refer to the standard input and output streams.
* The data type used is teletype, which is shown as T, t, y in Julia.
* We can read standard input from the terminal using the readline() function.
* There is a read() and a write() function to read from and write to standard output, respectively.
* write() function returns the number of characters in the string provided.
* To suppress this, we can put a semicolon at the end.
* There are also functions like readline() and eachline() which allow us to read multiple lines.
#### Reading Data from and Writing Data to Files

* We can use the open() function to open a file and read its contents.
* To write to a file, we can use the open() function with the second argument set to write.
* There are various modes available for reading and writing files, such as "r", "w", "a", etc.
* We can also use the do block to read and write files.
#### Communication between a Server and Client

* HTTP.jl package is used for communication between a server and client.
* The package provides functions to make HTTP requests and handle responses.
* We can use the HTTP.request() function to make HTTP requests.
* The response is returned as a HTTP.Response object which can be handled accordingly.

### 67. File Input and Output

#### Introduction to Standard Input and Output

* Standard input and output are the most basic inputs and outputs.
* They are represented by global variables: stdin for standard input and stdout for standard output.
* Teletype (T, t, y) is the data type for standard input and output in Julia.
* Reading from standard input using readline() and read(stdin, n).
* Writing to standard output using println() and print().
#### Reading and Writing Files

* Julia uses the open() function to read and write files.
* open() returns an object of type IOStream or IOBuffer.
* Reading a file using read(), readlines(), or eachline().
* Writing to a file using write(), writedlm(), or print().
* Opening a file in different modes: read, write, and append.
* Using escape characters like \n for a new line.
* Creating a directory, creating files in it, and writing to those files.
#### Other Topics

* Using anonymous functions with do and end.
* Other types of files such as CSV and Excel files will be covered in a separate course.

### 68. TCP/IP

* In the terminal, start the host with:
  ```
  netcat localhost 8000
  OR
  ncat localhost 8000
  ```

#### TCP/IP Communication Protocol

* Used to send information between devices on a network like the Internet.
* TCP stands for Transmission Control Protocol and IP stands for Internet Protocol.
* IP obtains the address of the target computer to which the information will be sent.
* TCP fulfills the task of communication rules, correct information transmission, and package reassembly.
#### Socket

* Defined with a port number.
* Specifies the end points of communicating programs.
#### AndKit and nmap

* Networking tools for reading and writing data across networks from the command line.
* Used to create clients and make them communicate with the host.
#### Network Connection in Julia

* Use the Sockets package.
* Listen function creates a server on the local host and waits for incoming connections for the specified port.
#### Server and Client Communication

* Redline command used to read messages from the client.
* Infinite while loop used to read messages automatically.
* Async macro used to deal with each connection in a different core routine.
#### Parallel and Asynchronous Programming

To complete multiple operations without waiting for each other.
#### Benefits of Network Programming in Julia

* Easier than other languages because no need to deal with callbacks.

## Section 11: Parallel Programming

#### Introduction to Parallel Programming:

* Julia supports four categories of concurrent or parallel programming: asynchronous tasks or core routines, multi-threading, distributed computing, and GPU computing.
* The aim of parallel programming is to decrease latency and increase throughput.
#### Concurrency:

* Concurrency works on a single thread and gives the illusion of parallel programming.
* Dividing tasks into subtasks and using them properly can decrease latency significantly.
#### Parallelism:

* In parallelism, multiple processes run at different threads simultaneously.
* Improvement in parallel programming is distributed computing.
#### Distributed Computing:

* Distributed computing is running a system as a single one on separate memory spaces.
* These memory spaces can be on the same computer or on multiple computers, connected by a local or a wide area network.
#### GPU Computing:

* A GPU consists of lots of cores, each of which can handle a limited set of instructions.
* GPU computing is much more efficient for repetitive tasks which can be paralyzed extensively, such as image processing, machine learning, and deep learning.

### 70. Asynchronous Programming

* Asynchronous programming is not really parallel programming, but it can increase the performance of tasks that don't require CPU resources.
* The async macro in Julia runs tasks concurrently, allowing for more efficient use of resources.
* Tasks in Julia have a lifecycle consisting of four phases: create, start, run, and finish.
* Channels in Julia are used for communication between tasks and operate on a first-in-first-out basis.
* Channels can be used to send output from a task to another task for consumption.
* The take function is used to consume data from a channel, and the output function is used to produce and write data to a channel.
* If a channel is empty, take calls will be blocked, and if a channel is full, put calls will be blocked.

### 71. Multi-Threaded Programming

* To start Julia with ```4 threads```.
  * In the terminal.
  ```
  bash> export JULIA_NUM_THREADS=4
  ```

  * Start Julia.
  ```
  bash> julia
  julia> Threads.nthreads()
  ```
  OR

  * Start Julia with 4 threads directly.
  ```
  bash> julia --threads 4
  ```

* Check thread id.
```
julia> Threads.threadid()
```

* You can also edit Julia thread settings in VSCode.
  * Open Settings: ```Ctrl+Shift+P```.
  * Search for ```julia.AdditionalArgs```.
  * Click on ```Edit in settings.json```.
  * Edit the ```settings.json``` file.
  ```
      ...
      ],
      "julia.symbolCacheDownload": true,
      "julia.enableTelemetry": true,
      "julia.NumThreads": 1,
      "julia.additionalArgs": [

      ]
  }
  ```
  * To set the maximum number of threads, set the ```julia.NumThreads``` value to ```"auto"```.

* Default number of threads is 1 in Julia.
* Number of threads can be changed from command line or using the Julian threads environment variable.
* Changing number of threads should be done before starting the Julia session.
* Threads macro is used for multithreading.
* Data race issue can occur when multiple threads write to the same memory location concurrently.
* Atomic arrays and operations can be used to avoid data race issue.
* Using threads macro in a for loop can send each iteration to another thread.
* Tasks and channels can be used to send tasks to the available thread.
* Using tasks and channels with threads can make the runtime faster.
* Vtime macro from the BenchmarkTools package can be used to run the code multiple times and provide better prediction.
* Parallelism example was demonstrated using vector means with 36 vectors of length 1 million.
* Using threads macro for parallelism made the runtime much faster.
* Using tasks and channels with threads for parallelism was even faster when vectors had different lengths.
