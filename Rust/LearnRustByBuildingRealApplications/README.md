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
* Rust has a special control flow operator called ```match```.
* It allows you to compare a value against a series of patterns and then execute code based on which pattern matches.
* When matching on ```enums```, the code will not compile unless we have covered all the possible variants.

* You can use an ```_``` (underscore), for unwrapping unneeded variables.
    * It can also be used as a pattern, where we don't want to match all patterns.
    * Similar to **default** case in other languages.
    * It is important to note that the code will not compile unless we have covered all the possible variants of the enum that we are matching.
    * The match operator allows for more powerful and convenient handling of errors in Rust compared to if-else statements.

### Arrays

* To use the ```read()``` method, we need to include ```Read``` with ```use std::io::Read;```.
* Trait: Something like an interface.
  * It defines the signature of the read method, but it has no body.
* ```read``` takes a mutable reference to self.

* Array
  * Like a tuple it's a compound type.
  * But unlike a tuple, it is a collection of values with the same type.
* To define an array:
```
let a = [1, 2, 3, 4];
```
* The concrete type of an array is the type of the values in it, and the number of values it contains.
* The compiler needs to know exactly how big an array is so it can allocate enough memory on the stack for it.

* To define a function using an array, it needs to know the size of an array and the type.
```
fn arr(a: [u8; 5]) {}
```
* To overcome the problem of knowing the size of an array, we can use references:
```
let a = [1, 2, 3, 4];

fn arr(a: &[u8]) {}

arr(&a);
```
* We can use slices:
```
let a = [1, 2, 2, 3, 4, 3, 4];

arr(&a[1..3]);
```

* To create an array of a given size with default element:
    * Where 0 is the value and 1024 is the size.
```
let mut buffer = [0; 1024];
```

* In C, when we create an array of a given size, and don't give the default value for every element, it occupies some random address in the memory. This can cause a problem as we don't know the value that address contains. Rust overcomes this by insisting on providing a default value.

### Logging the Incoming Requests to the Console

* ```String::from_utf8()```
  * Expects a parameter, which is a buffer containing bytes.
  * The bytes need to be ```utf8```.
  * If the bytes contain an invalid ```utf8``` byte, then the whole operation can fail.

* ```String::from_utf8_lossy()```
  * This function never fails.
  * Converts a slice of bytes to a string, including invalid characters.
  * Replaces any invalid byte, with a "?" character.

* To test the operation:
  * Launch the server with: ```cargo build; cargo run```
  * From another terminal, connect to the port with ```netcat``` command.
  > To install netcat on Mac: ```brew install netcat```.
  ```
  echo "TEST" | netcat 127.0.0.1 8080
  ```
  * If it's successful, the server receives the message, and logs it to the screen.
  * To test the invalid character logging: ```echo "\xFF TEST" | netcat 127.0.0.1 8080```
  * You can also open the address ```127.0.0.1:8080``` on a browser to send a request to the server.

### Traits and Type Conversions

* In Rust, a trait is a way to define a set of behaviors (or methods) that a type can implement.
* It's similar to an interface in other programming languages.
* A trait acts as a blueprint for a set of methods that types can choose to implement.
* This allows for a way of generic programming in Rust.
* In Python, you would use inheritance, where a class can inherit the properties and methods of another class, while in Rust, you can use traits to define a set of behaviors that a type can implement.

```
trait Summable {
    fn sum(&self) -> i32;
}
```

This trait defines a single method called sum(), which takes no arguments and returns a value of type i32. Any struct that wants to implement this trait must also have a method called sum() with the same signature (takes no arguments and returns an i32).

Here's an example of a struct implementing the trait:
```
struct MyStruct {
    value: i32,
}

impl Summable for MyStruct {
    fn sum(&self) -> i32 {
        self.value
    }
}
```

In this example, the struct MyStruct has an implementation of the sum() method that is defined by the Summable trait. This means that MyStruct is now considered "Summable", and we can use it in a generic context where a Summable type is required.

Now we can create a function that takes a generic parameter T that is bound by the Summable trait and call the sum method on that parameter, knowing that it will always be available because the trait bound guarantees it.

```
fn print_sum<T: Summable>(item: &T) {
    println!("Sum: {}", item.sum());
}

let s = MyStruct { value: 10 };
print_sum(&s);
```

This will print "Sum: 10"

It's important to note that in Rust, the trait implementation is done on the struct level and not on the instance level, so any instance of the struct will have the same trait implementation.

Traits are useful when you want to:

* Write generic code that works with multiple types.
* Write code that can work with multiple types that share a common behavior.
* Reuse code across multiple types.
* Organize and structure your code.
* Write unit tests that cover all the possible behaviors of a type.

#### Type Conversions

* Traits can be used to define conversions between types in Rust. The most common way to do this is through the use of the From and Into traits.

* The From trait is used to define a conversion from one type to another. The trait is defined like this:

```
trait From<T> {
    fn from(T) -> Self;
}
```
This trait defines a single method called from(), which takes a value of type T and returns a value of the implementing type (Self).

The Into trait is similar to the From trait, but it's defined in the opposite direction. It is defined like this:

```
trait Into<T> {
    fn into(self) -> T;
}
```
It defines a single method called into(), which takes a value of the implementing type (Self) and returns a value of type T.

Here's an example of how you could use these traits to define conversions between two types:

```
struct MyStruct {
    value: i32,
}

impl From<i32> for MyStruct {
    fn from(value: i32) -> MyStruct {
        MyStruct { value }
    }
}

impl Into<i32> for MyStruct {
    fn into(self) -> i32 {
        self.value
    }
}
```
With these implementations, you can now use the From and Into traits to convert between MyStruct and i32 types using the from() and into() methods, respectively.

```
let s = MyStruct::from(10);
let v: i32 = s.into();
```
You can also use the From and Into trait as a bound in a function that takes a generic T, where T is bound by the trait, this allows the function to convert any T into a specific type.

```
fn convert_into_mystruct<T: Into<MyStruct>>(t: T) -> MyStruct {
    t.into()
}

let v = 10;
let s = convert_into_mystruct(v);
```
In this example, the convert_into_mystruct function takes a generic parameter T that is bound by the Into<MyStruct> trait, this guarantees that the function can convert any T into a MyStruct.

It's important to note that the From and Into traits are automatically implemented for some types, such as primitive types. This means that you don't always need to define them explicitly.

> Use ```unimplemented!()``` macro for anything that is not yet implemented. Similar to ```pass``` in Python.

#### Using Traits to extend the functionality of built-in Types

```
trait Encrypt{
    fn ecrypt(&self) -> Self;
}

impl Encrypt for String{
    fn encrypt(&self) -> Self{
        unimplemented!()
    }
}

impl Encrypt for &[u8]{
    fn encrypt(&self) -> Self{
        unimplemented!()
    }
}

# Use it on a string.
let string = String::from("asd");
string.encrypt();
# Use encrpyt() on an array.
let buf = &[u8];
buf.encrypt();
```

### Custom Errors

* For logging using Debug trait:
```
println!("Received a request: {:?}", String::from_utf8_lossy(&buffer));
```

### Advanced Error Handling

* ```str::from_utf8(buf);```
    * If the conversion is successful, it gives us a string, otherwise we get a ```Utf8Error```.

* ```or``` method returns the ```res: Result<T, F>``` if the result is an error, otherwise it returns the ```Ok``` value.

```let request = str::from_utf8(buf).or(Err(ParseError::InvalidEncoding))?;```
* ```?``` operator, looks at the result, and if it's ```Ok``` it returns the value that ```Ok``` wraps, otherwise it returns the error.
    * If the error is not given, e.g.: ```let request = str::from_utf8(buf)?;```, then it tries to convert the ```Utf8Error``` into a ```ParseError```, by looking for a ```From``` trait.
    * If the From trait is not implemented, you'll need to implement it.

### Iterating Over Strings

* ```Option```
  * We
