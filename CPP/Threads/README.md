# CPP Threads Tutorial

## 1. Introduction to Thread in C++

In every application there is a default thread which is main(), inside this we create other threads.
A thread is also known as lightweight process. Idea is achieve parallelism by dividing
a process into multiple threads.

For example:
- The browser has multiple tabs that can be different threads.
- MS Word must be using multiple threads, one thread to format the text, another thread
  to process inputs (spell checker), etc..
- Visual Studio code editor would be using threading for auto completing the code. (Intellisense).

### 2. Ways to create threads in C++
1. Function Pointers.
2. Lambda Functions.
3. Functors (Function object).
4. Member Functions.
5. Static Member Functions.

* Compile with:
```
g++ -std=c++11 -pthread 1_a_thread_introduction.cpp
```

## 3. ```join()```, ```detach()``` and ```joinable()```

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

## 4. Mutex in C++ Threading

* Mutex: Mutual Exclusion.

* Race Condition.
  * Race condition is a situation where two or more threads/ process happened to change a common data at the same time.
  * If there is a race condition then we have to protect it and the protected section is called critical section/region.

* Mutex.
  * Mutex is used to avoid race condition.
  * We use ```lock()```, ```unlock()``` on mutex to avoid race condition.

### 5. ```try_lock()```

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

### 6. ```std::try_lock()```

1. ```std::try_lock()``` tries to lock all the lockable objects passed in it one by one in given order.
```
std::try_lock(m1, m2, m3, m4, ..., mn);
```

2. On success this function returns **-1** otherwise it will return **0-based** mutex index number which it could not lock.

3. If it fails to lock any of the mutex then it will **release** all the mutex it locked before.

4. If a call to ```try_lock``` results in an exception, unlock is called for any locked objects before re-throwing.

5. The actual use of ```std::try_lock()``` function is, it can try to lock multiple mutex objects at the same time.


### 7. Timed Mutex

1. ```std::timed_mutex``` is blocked till ```timeout_time``` or the lock is acquired and returns true if success otherwise false.

2. Member function:
    * ```lock```
    * ```try_lock```
    * ```try_lock_for```    ---\ These two functions are different from mutex.
    * ```try_lock_until```  ---/
    * ```unlock```

* ```try_lock_for()```: Waits until specified timeout_duration has elapsed or the lock is acquired, whichever comes first. On successful lock acquisition returns true, otherwise returns false.

### 8. Recursive Mutex

* It is the same as mutex but, same thread can lock one mutex multiple times using ```recursive_mutex```.
* If thread T1 first call lock/try_lock on recursive mutex m1, then m1 is locked by T1, now as T1 is running in recursion T1 can call lock/try_lock any number of times, there is no issue.
* But if T1 has acquired 10 times lock/try_lock on mutex m1 then thread T1 will have to unlock it 10 times. Otherwise no other thread will be able to lock mutex m1. It means recursive_mutex keeps count how many times it was locked so that many times it should be unlocked.
* How many time we can lock recursive_mutex is not defined, but when that number reaches and if if were calling lock() it will return std::system_error OR if we were calling try_lock() then it will return false.

1. It is similar to mutex but has extra facility that it can be locked multiple times by the same thread.
2. If we can avoid recursive_mutex then we should because it brings overhead to the system.
3. It can be used in loops also.
