#include <sys/syscall.h>
#include <tuple>
#include <stdio.h>
#include <unistd.h>
#include <linux/perf_event.h>
#include <linux/hw_breakpoint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <cstdint>
#include <chrono>

// Instructions, Page Faults, Cache References, Cache Misses, Branches, Branch Mispredictions, Microseconds
typedef std::tuple<uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t> process_data;

class snitch {
private:
    void store_data();
    int instructions_file_descriptor_;
    int page_faults_file_descriptor_;
    int branch_file_descriptor_;
    int branch_mispredictions_file_descriptor_;
    int cache_references_file_descriptor_;
    int cache_misses_file_descriptor_;
    std::chrono::_V2::steady_clock::time_point start_;

    uint64_t instructions_count_;
    uint64_t page_faults_count_;
    uint64_t branches_;
    uint64_t branch_misses_;
    uint64_t cache_references_;    
    uint64_t cache_misses_;
public:
    void start();
    void stop();
    void start_time();
    long stop_time();
    process_data read_data();
};
