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
**NOTE TO SELF**
You need to use **bash** when working with Rust. **Fish** is not setup yet.
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