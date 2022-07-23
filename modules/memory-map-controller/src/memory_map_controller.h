#ifndef MM_CONTROLLER_MEMORY_MAP_CONTROLLER_H
#define MM_CONTROLLER_MEMORY_MAP_CONTROLLER_H

#include <vector>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>      /* Definition of MADV_* constants */
#include <sys/syscall.h>   /* Definition of SYS_* constants */
#include <sys/uio.h>       /* Definition of struct iovec type */
#include <unistd.h>
#include <map>

class memory_map_controller {
public:
    /**
     * Creates and memory-maps a new temporary file of size n * sizeof(Type).
     * @tparam Type element type.
     * @param n number of elements.
     * @return Begin address of memory-mapped region.
     */
    template <typename Type>
    Type* falloc(unsigned long n){
        return falloc<Type>(n, 0);
    }

    /**
     * Creates and memory-maps a new temporary file of size n * sizeof(T). Advises kernel of expected behaviour.
     * @tparam T element type.
     * @param n number of elements.
     * @param advice advise to kernel.
     * @return Begin address of memory-mapped region.
     */
    template <typename T>
    T* falloc(unsigned long n, int advice) {
        FILE* temporary_file = std::tmpfile();
        int file_descriptor = temporary_file->_fileno;
        if (file_descriptor < 0) {
            perror("Could not create temporary file for falloc");
            exit(1);
        }
        long bytes = n * sizeof(T);
        ::ftruncate(file_descriptor, bytes);
        T* begin = (T*) ::mmap64(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_PRIVATE, file_descriptor, 0);

        if ((unsigned char*) begin == MAP_FAILED) {
            perror("Could not memory map file");
            exit(1);
        }
        ::madvise(begin, bytes, advice);
        files_.insert({begin, {temporary_file, bytes}});
        return begin;
    }

    /**
     * Unmaps a previously memory-mapped region of memory.
     * Underlying file is automatically deleted upon program termination.
     * @param address
     */
    void free(void* address);

    /**
     * Unmaps all regions mapped by this controller.
     */
    void purge();
private:
    std::map<void*, std::pair<FILE*, unsigned long>> files_;
};

#endif //MM_CONTROLLER_MEMORY_MAP_CONTROLLER_H
