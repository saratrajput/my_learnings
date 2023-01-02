# CPP Threads Tutorial

## Introduction to Thread in C++

In every application there is a default thread which is main(), inside this we create other threads.
A thread is also known as lightweight process. Idea is achieve parallelism by dividing
a process into multiple threads.

For example:
- The browser has multiple tabs that can be different threads.
- MS Word must be using multiple threads, one thread to format the text, another thread
  to process inputs (spell checker), etc..
- Visual Studio code editor would be using threading for auto completing the code. (Intellisense).

### Ways to create threads in C++
1. Function Pointers.
2. Lambda Functions.
3. Functors (Function object).
4. Member Functions.
5. Static Member Functions.

* Compile with:
```
g++ -std=c++11 -pthread 1_a_thread_introduction.cpp
```

## ```join()```, ```detach()``` and ```joinable()```

### ```join()```
- Once a thread is started, we wait for this thread to finish by calling join() function on
  thread object.
- Double join will result into program termination.
- If needed we should check thread is joinable before joining. (using joinable() function.)

### ```detach()```
- This is used to detach newly created thread from the parent thread.
- Always check before detaching a thread that it is joinable otherwise we may end
  up double detaching and double detach() will result into program termination.
- If we have detached thread and main function is returning then the detached thread execution
  is suspended.

> Either join() or detach() should be called on the thread object, otherwise during
  thread object's destructor it will terminate the program. Because inside destructor it checks
  if thread is still joinable? If yes, then it terminates the program.

## Mutex in C++ Threading

* Mutex: Mutual Exclusion.

* Race Condition.
  * Race condition is a situation where two or more threads/ process happened to change a common data at the same time.
  * If there is a race condition then we have to protect it and the protected section is called critical section/region.

* Mutex.
  * Mutex is used to avoid race condition.
  * We use ```lock()```, ```unlock()``` on mutex to avoid race condition.

### ```try_lock()```

* ```try_lock()``` tries to lock the mutex.
* Returns **immediately**.
* On successful lock acquisition returns true otherwise returns false.
* If ```try_lock()``` is called again by the same thread which owns the mutex, the behaviour is undefined.
* It is a **dead lock** situation with undefined behaviour.
* If you want to be able to lock the same mutex by same thread **more than one time**, then go for ```recursive_mutex```.

#### Types of ```try_lock()``` functions.

1. std::try_lock
2. std::mutex::try_lock
3. std::shared_lock::try_lock
4. std::timed_mutex::try_lock
5. std::unique_lock::try_lock
6. std::shared_mutex::try_lock
7. std::recursive_mutex::try_lock
8. std::shared_timed_mutex::try_lock
9. std::recursive_timed_mutex::try_lock
