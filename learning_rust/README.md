# Learning RUST

Contains code snippets for learning Rust from this [tutorial](https://stevedonovan.github.io/rust-gentle-intro/1-basics.html).

## [Installation on Linux](https://doc.rust-lang.org/book/ch01-01-installation.html)
* Install Rust from terminal
```
$ curl https://sh.rustup.rs -sSf | sh
```

* Add Rust to your path
```
$ source $HOME/.cargo/env
```
Or add it your *~/.bashrc*:
```
$ echo export PATH="$HOME/.cargo/bin:$PATH" >> ~/.bashrc
```


## Compiling Rust
* Compile *.rs* files in linux with:
```
$ rustc main.rs
```
