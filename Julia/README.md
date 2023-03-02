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

### 7. Julia REPL

* Julia REPL is an interactive command line prompt for writing small Julia expressions or code pieces.
* To start Julia REPL, type "Julia" in the command line or click on the Julia shortcut.
* Different keys can be used to access different prompts in Julia REPL, such as the help screen or shell screen.
* The package manager in Julia REPL allows for managing packages, adding, removing, or updating them.
* Code can also be written in a file with a .jl extension and called from REPL using the include function.
* Julia REPL is not suitable for longer and more complex programs, data analysis, or data visualization, for which code editors or IDEs are needed.

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
