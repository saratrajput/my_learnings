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

Introduction to Types in Julia:

Types are tags on values that restrict the range of potential values that can be stored at that location
Understanding types in Julia is very important
Type Hierarchy:

Julia has a type hierarchy where all types are in that hierarchy
The top of this hierarchy is the type Any
All other types are subtypes of Any and Any is a super type of all other types
Below Any, there is a long pyramid of type hierarchies
The bottom of the hierarchy is the type Union
Every type is a subtype of another type in between Any and Union
Any has 580 subtypes
Abstract Types and Concrete Types:

Red types that have subtypes are called abstract types and cannot be instantiated
Blue types do not have subtypes and are concrete types that can be instantiated
Abstract types allow us to define functions using the group of types we want
Concrete types can be used to create variables
Getting Subtypes and Supertypes in Julia:

To get subtypes and supertypes in Julia, we can use the subtypes and supertype functions
The <: operator is used to check whether a type is a subtype of another type
Type hierarchy in Julia can be applied to predefined types and user-defined types

### 12. Numerical Data Types: Integers and Floating-Point Numbers

Integer and Floating Point Numbers:

Integer and floating point numbers are the most widely used basic data types in programming
Julia provides a wide range of concrete, integer and floating point numbers
Integer types are bool, signed, and unsigned integers where the latter two are abstract types
The maximum and minimum values of integer and unsigned integer types can be calculated using formulas
The sizeof function can be used to check the size of a variable
Numeric values can be separated by underscores for readability purposes
Conversion functions can be used to convert between types
Special Symbols in Julia:

Inf represents infinity and can be used in calculations
NaN stands for a value that is not equal to any floating point number
It is recommended to check whether the result of a calculation is NaN or infinity using the isnan or isinf functions
Epsilon values give the distance between one point and the next floating point
Epsilon values are not the same for all points and get larger for large values
NaN and Infinity:

Inf represents a value greater than all finite floating point values
-Inf represents a value less than all finite floating point values
NaN stands for a value that is not equal to any floating point number
Division by zero is normally undefined in math, but it's Inf in Julia
Bool Type:

Bool stands for data that can only get false or true values
True is the integer 1 and false is the integer 0
The equal equal operator can be used to check whether true is equal to 1 and false is equal to 0

### 13. Numerical Data Types: Complex and Rational Numbers

Complex numbers are defined as a+bi where a is the real part and bi is the imaginary part.
In Julia, the imaginary part or square root of -1 is denoted as im.
We can use the complex function to create a complex number using two real numbers.
We can use complex numbers in mathematical operations in Julia.
The conjugate of a complex number a+bi is a-bi.
Rational data types are defined using the // operator.
To check whether an object is of a given type, we can use the isa function.

### 14. Character and String Types

In Julia, there are two data types for text variables: char and string.
Char type represents a single character and is defined using single quotes. String type is defined using double quotes.
Unicode characters can also be defined as chars.
Strings are arrays of characters and are one-indexed in Julia.
Indexing can be done using square brackets and begin/end keywords.
Substrings can be obtained using the colon operator and intervals.
Unicode characters may have multiple code units, which affects indexing and length.
Strings can be concatenated using the string function or the asterisk operator.
The asterisk operator is used instead of the plus sign for concatenation because addition is commutative while multiplication is not always commutative.
The power sign can be used to repeat a string multiple times.
Interpolation using the dollar sign allows variables and operations to be used within a string.

### 15. Primitive Types and Composite Types

Primitive types in Julia are concrete types whose data consists of plain old bits.
Built-in primitive types in Julia include integers, floating point numbers, bool, and char data types.
Composite types are user-defined types composed of single fields, defined using the struct keyword.
Structs are immutable by default, but can be made mutable by adding the word "mutable" in front of struct.
Constructors are used to create instances of structs.
Union types are used when a variable can be in either one of a union of data types.

### 16. Parametric Types

Julia has a powerful parametric type system, where types can take parameters using the syntax T.
This means that data types can be flexible, as long as they have the same type.
ParRectangle is a type object in Julia, which contains all instances with different data types.
It is possible to limit the data types a parametric type may have using the <: operator.
Parametric abstract types can also be defined, and the types T they can have can be limited.
An example is provided from Julia documentation, where a struct is a subtype of real, and its fields are subtypes of integer.
A new shape can be defined with four fields, two of them being a subtype of abstract string and the other two being a number type.
Name and color fields should be of the same type, and x coordinate and y coordinate should also be of the same type.


## Section 8: Modules and Packages

### 59. Writing and Using Modules

* Check the modules currently loaded with: ```LOAD_PATH```.
* Check the current directory path with: ```pwd()```.
* Add the current path to the ```LOAD_PATH```.
  ```
  push!(LOAD_PATH, pwd())
  ```

### 60. Revise Package

* In order to reload a module which has been updated, you can either restart the Julia REPL, or use the ```Revise``` package.
* Install ```Revise``` package with:
  ```Julia
  // Press ']' key
  add Revise
  ```
* And load it with: ```using Revise```.

### 61. Package Development

* Enter the Julia REPL: ```julia```
* Change directory to Desktop: ```cd("Desktop")``` OR in the present directory.
* Create ```MyPackage```
```
// Press the ']' key to enter pkg mode
generate MyPackage
```

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

### Section 10 - Streams and Networking

#### 68. TCP/IP

* In the terminal, start the host with:
  ```
  netcat localhost 8000
  OR
  ncat localhost 8000
  ```

#### 71. Multi-Threaded Programming

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
