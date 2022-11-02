# Learn Rust by Building Real Applications

These are my notes from the Udemy course [Learn Rust by Building Real World Applications](https://www.udemy.com/course/rust-fundamentals/learn/lecture/20712358#overview).

[Original Github Repo](https://github.com/gavadinov/Learn-Rust-by-Building-Real-Applications)
## Introduction

### Advantages About Rust
* Memory safe
* No Null
* No Exceptions
* Modern package manager
* No Data Races

### Installation

* Open https://doc.rust-lang.org/stable/book/

* For Linux

```
curl --proto '=https' --tlsv1.2 https://sh.rustup.rs -sSf | sh
```

---

For fish shell, add the following line to ```~/.config/fish/config.fish``` file.
```
# Add path to RUST
set PATH /home/sp/.cargo/bin $PATH
```
---

#### Setup Development Environment

* Create a new package
```
cargo new example
```

* Build
```
cd example/
cargo build
```

* Run
```
cargo run
```

* Install cargo-expand
```
cargo install cargo-expand
cargo expand
# If errors, install nightly toolchain
rustup install nightly
rustup default nightly

cargo expand
```

---
**NOTES:**

Central repository for crates: crates.io
---

## Memory Management
* The Stack
* The Heap
* Pointers
* Smart Pointers

### The Stack
* It's a special region of the process memory that stores variables created by each function.

* For every function call a new stack frame is allocated on top of the current one.

* The size of every variable on the stack has to be known at compile time.

* When a function exits it's stack frame is released.

### The Heap
* It's a region of the process memory that is NOT automatically managed.

* It has no size restrictions.

* It's accessible by any function, anywhere in the program.

* Heap allocations are expensive and we should avoid them when possible.

* When we forget to free heap allocated memory we have a Memory Leak.

We have to deallocate the memory manually the memory which we have allocated in the heap. Otherwise, there'll be a memory leak.

### Smart Pointers

## Building a Command Line Application
* Functions
* Basic Data Types
* Standard Library
* Memory Ownership

## Ownership
### Ownership Rules
- Each value in Rust is owned by a variable.
- When the owner goes out of scope, the value will be deallocated.
- There can only be ONE owner at a time.

## Debugging
* Executable files are stored in **target/debug/**.
```
./mars_calc
```

* Debugger
```
gdb mars_calc
```

## HTTP/1.1
* L7 Protocol
* Sent over TCP
* Message based

### Strings
* Rust has two types of strings.
    * String is a datatype stored on heap, and you have access to that location.
    * &str is aslice type. That means it is just reference to an already present String somewhere in the heap.

```
// String
let string = String::from("127.0.0.1:8080");
let string_slice = &string[10..];  // Everything after the 10th byte.
let string_borrow: &str = &string;
let string_literal = "1234";

dbg!(&string);
dbg!(string_slice);
dbg!(string_borrow);
dbg!(string_literal);
```

### The Option Enum
#### Null values
* Most languages have the concept of ```null``` values.
* Null represents the absence of a value.
* Rust doesn't have null values.
* Rust has a different way of representing the absence of a value.
* It uses the ```Option Enum``` to achieve this.
```
pub enum Option<T>
{
    None,
    Some(T),
}
```

* So to declare an optional string, we can use:
```
query_string: Option<String>
```

### Organising Our Code into Modules
* To encapsulate our code into modules, add the code inside a ```mod {}``` (module).
* Add the ```pub``` keyword to the struct definition or function definitions to make them public. By default everything is private.
* Use the ```super``` keyword to refer to the parent module.
* To define namespaces,
```
use server::Server;
```
* To separate the modules in different files, we don't need to use the ```mod {}``` definition.

### Listening for TCP Connections

* ```std::net```
Networking primitives for TCP/UDP communication.

* ```std::net::TcpListener```
A TCP socket server, listening for connections.

### The Result Enum
Result Enum is in the core of error handling in Rust.

Rust groups error in two categories:
* Recoverable:
    * Eg: FileNotFound Error
    * In most cases, you would not like your program to crash if the file does not exist.
* Unrecoverable:
    * Eg: Result of a bug, like trying to access after an array has ended.

Most languages don't distinguish between these two kinds of errors and handles both the same way using ```Exceptions```.

Rust does not have ```Exceptions```. It handles errors with the Result enum.

```
pub enum Result<T, E>
{
    /// Contains the success value
    Ok(T),
    /// Contains the error value
    Err(E),
}
```

```unwrap```
When it is used, it looks at the **Result**, and if it is ```Ok```, it will return the value that is wrapped by ```Ok```.
If it is ```No```, it will terminate the program and log the reason on the screen.

* Command to simulate the port ```8080``` is occupied.
```
netcat -k -l 8080
```

### Loops
* Rust has a simpler syntax for ```while```.

```
while true
{
    <body of the loop>
}
```

Rust:
```
loop
{
    <body of the loop>
}
```

* To continue the rest of the code: ```continue;```.
* To break out of the loop: ```break;```.

* To break out of the outer loop from inside the inner loop, one can use **labels**.

```
'outer: loop
{
    loop
    {
        break 'outer;
    }
}
```

### Tuples
* Useful for grouping together different types together which are immutable.

### The Match Expression
* Rust has a special control flow operator called match.
* It allows you to compare a value against a series of patterns and then execute code based on which pattern matches.
* When matching on ```enums```, the code will not compile unless we have covered all the possible variants.

* You can use an ```_``` (underscore), for unwrapping unneeded variables.
    * It can also be used as a pattern, where we don't want to match all patterns.
    * Similar to **default** case in other languages.

### Arrays
