#include <sys/file.h>

int main() {
    
    // ... get file descriptor here

    // Acquire non-blocking exclusive lock
    if(flock(fd, LOCK_EX | LOCK_NB) == -1) {
        throw std::runtime_error("Serial port with file descriptor " + 
            std::to_string(fd) + " is already locked by another process.");
    }

    // ... read/write to serial port here
}

