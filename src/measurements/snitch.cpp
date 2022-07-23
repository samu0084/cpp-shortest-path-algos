#include "snitch.h"

/**
 * @brief Opens a perf event
 * Further documentation @ https://web.eece.maine.edu/~vweaver/projects/perf_events/perf_event_open.html
 * @param hw_event 
 * @param pid specifies the process to monitor, 0 = calling process, -1 = all proccesses.
 * @param cpu specifies which CPU to monitor, -1 = all CPUs.
 * @param group_file_descriptor specifies the file descriptor of the leader of the event group.
 * "An event group is scheduled onto the CPU as a unit: it will be put onto the CPU only if all of the events in the group can be put onto the CPU. 
 * This means that the values of the member events can be meaningfully compared---added, divided (to get ratios), and so on---with each other, since they have counted events for the same set of executed instructions."
 * @param flags 
 * @return int file_descriptor of created 
 */
static int perf_event_open(struct perf_event_attr *hw_event, pid_t pid,
                       int cpu, int group_fd, unsigned long flags){
    int ret;
    ret = syscall(__NR_perf_event_open, hw_event, pid, cpu,
                   group_fd, flags);
    return ret;
}


/**
 * @brief Create a page faults monitoring perf event and returns the file descriptor.
 * 
 * @param group_file_descriptor file descriptor of group leader. (-1 to intialise new group).
 * @return int file descriptor.
 */
static int create_page_faults_monitor(int group_file_descriptor) {
    struct perf_event_attr pe_attr_page_faults;
    memset(&pe_attr_page_faults, 0, sizeof(pe_attr_page_faults));
    pe_attr_page_faults.size = sizeof(pe_attr_page_faults);
    pe_attr_page_faults.type =   PERF_TYPE_SOFTWARE;
    pe_attr_page_faults.config = PERF_COUNT_SW_PAGE_FAULTS;
    pe_attr_page_faults.disabled = 1;
    pe_attr_page_faults.exclude_kernel = 1;


    int page_faults_file_descriptor = perf_event_open(&pe_attr_page_faults, 0, -1, group_file_descriptor, 0);
    if (page_faults_file_descriptor == -1) {
        throw "Perf_event_open failed for page faults";
    }
    return page_faults_file_descriptor;
}
/**
 * @brief Create a branch monitoring perf event and returns the file descriptor.
 * 
 * @param group_file_descriptor file descriptor of group leader. (-1 to intialise new group).
 * @return int file descriptor.
 */
static int create_branch_monitor(int group_file_descriptor) {
    struct perf_event_attr pe_attr_branch;
    memset(&pe_attr_branch, 0, sizeof(pe_attr_branch));
    pe_attr_branch.size = sizeof(pe_attr_branch);
    pe_attr_branch.type =   PERF_TYPE_HARDWARE;
    pe_attr_branch.config = PERF_COUNT_HW_BRANCH_INSTRUCTIONS;
    pe_attr_branch.disabled = 1;
    pe_attr_branch.exclude_kernel = 1;


    int branch_file_descriptor = perf_event_open(&pe_attr_branch, 0, -1, group_file_descriptor, 0);
    if (branch_file_descriptor == -1) {
        throw "Perf_event_open failed for retired branches";
    }
    return branch_file_descriptor;
}
/**
 * @brief Create an instruction monitoring perf event and returns the file descriptor.
 * 
 * @param group_file_descriptor file descriptor of group leader. (-1 to intialise new group).
 * @return long file descriptor.
 */
static int create_instructions_monitor(int group_file_descriptor) {
    struct perf_event_attr pe_attr_instructions;
    memset(&pe_attr_instructions, 0, sizeof(pe_attr_instructions));
    pe_attr_instructions.size = sizeof(pe_attr_instructions);
    pe_attr_instructions.type =   PERF_TYPE_HARDWARE;
    pe_attr_instructions.config = PERF_COUNT_HW_INSTRUCTIONS;
    pe_attr_instructions.disabled = 1;
    pe_attr_instructions.exclude_kernel = 1;
    pe_attr_instructions.exclude_hv = 1;


    int instructions_file_descriptor = perf_event_open(&pe_attr_instructions, 0, -1, group_file_descriptor, 0);
    if (instructions_file_descriptor == -1) {
        throw "Perf_event_open failed for instructions";
    }
    return instructions_file_descriptor;
}

/**
 * @brief Create a branch mispredictions monitoring perf event and returns the file descriptor.
 * 
 * @param group_file_descriptor file descriptor of group leader. (-1 to intialise new group).
 * @return long file descriptor.
 */
static int create_branch_mispredictions_monitor(int group_file_descriptor) {
    struct perf_event_attr pe_attr_branch_mispredictions;
    memset(&pe_attr_branch_mispredictions, 0, sizeof(pe_attr_branch_mispredictions));
    pe_attr_branch_mispredictions.size = sizeof(pe_attr_branch_mispredictions);
    pe_attr_branch_mispredictions.type =   PERF_TYPE_HARDWARE;
    pe_attr_branch_mispredictions.config = PERF_COUNT_HW_BRANCH_MISSES;
    pe_attr_branch_mispredictions.disabled = 1;
    pe_attr_branch_mispredictions.exclude_kernel = 1;


    int file_descriptor = perf_event_open(&pe_attr_branch_mispredictions, 0, -1, group_file_descriptor, 0);
    if (file_descriptor == -1) {
        throw "Perf_event_open failed for branch mispredictions";
    }
    return file_descriptor;
}
/**
 * @brief Create a cache misses monitoring perf event and returns the file descriptor.
 * 
 * @param group_file_descriptor file descriptor of group leader. (-1 to intialise new group).
 * @return long file descriptor.
 */
static int create_cache_misses_monitor(int group_file_descriptor) {
    struct perf_event_attr pe_attr_cache_misses;
    memset(&pe_attr_cache_misses, 0, sizeof(pe_attr_cache_misses));
    pe_attr_cache_misses.size = sizeof(pe_attr_cache_misses);
    pe_attr_cache_misses.type =   PERF_TYPE_HARDWARE;
    pe_attr_cache_misses.config = PERF_COUNT_HW_CACHE_MISSES;
    pe_attr_cache_misses.disabled = 1;
    pe_attr_cache_misses.exclude_kernel = 1;


    int file_descriptor = perf_event_open(&pe_attr_cache_misses, 0, -1, group_file_descriptor, 0);
    if (file_descriptor == -1) {
        throw "Perf_event_open failed for cache misses";
    }
    return file_descriptor;
}
/**
 * @brief Create a cache references monitoring perf event and returns the file descriptor.
 * 
 * @param group_file_descriptor file descriptor of group leader. (-1 to intialise new group).
 * @return long file descriptor.
 */
static int create_cache_references_monitor(int group_file_descriptor) {
    struct perf_event_attr pe_attr_cache_ref;
    memset(&pe_attr_cache_ref, 0, sizeof(pe_attr_cache_ref));
    pe_attr_cache_ref.size = sizeof(pe_attr_cache_ref);
    pe_attr_cache_ref.type =   PERF_TYPE_HARDWARE;
    pe_attr_cache_ref.config = PERF_COUNT_HW_CACHE_REFERENCES;
    pe_attr_cache_ref.disabled = 1;
    pe_attr_cache_ref.exclude_kernel = 1;


    int file_descriptor = perf_event_open(&pe_attr_cache_ref, 0, -1, group_file_descriptor, 0);
    if (file_descriptor == -1) {
        throw "Perf_event_open failed for cache references";
    }
    return file_descriptor;
}
void snitch::start() {
    instructions_file_descriptor_ = (create_instructions_monitor(-1));
    page_faults_file_descriptor_ = (create_page_faults_monitor(instructions_file_descriptor_));
    branch_file_descriptor_ = (create_branch_monitor(instructions_file_descriptor_));
    branch_mispredictions_file_descriptor_ = (create_branch_mispredictions_monitor(instructions_file_descriptor_));
    cache_references_file_descriptor_ = (create_cache_references_monitor(instructions_file_descriptor_));
    cache_misses_file_descriptor_ = (create_cache_misses_monitor(instructions_file_descriptor_));
    ioctl(instructions_file_descriptor_, PERF_EVENT_IOC_RESET, PERF_IOC_FLAG_GROUP);
    ioctl(instructions_file_descriptor_, PERF_EVENT_IOC_ENABLE, PERF_IOC_FLAG_GROUP); 
}


void snitch::stop() {
    ioctl(instructions_file_descriptor_, PERF_EVENT_IOC_DISABLE, PERF_IOC_FLAG_GROUP);
    store_data();
    close(instructions_file_descriptor_);
    close(page_faults_file_descriptor_);
    close(branch_file_descriptor_);
    close(branch_mispredictions_file_descriptor_);
    close(cache_references_file_descriptor_);
    close(cache_misses_file_descriptor_);
}

void snitch::start_time() {
    start_ = std::chrono::steady_clock::now();
}

long snitch::stop_time() {
    auto stop_time = std::chrono::steady_clock::now();
    auto duration_microsec = std::chrono::duration_cast<std::chrono::microseconds>(stop_time - start_).count();
    return duration_microsec;
}

void snitch::store_data() {
    long success = 0;

    success = read(instructions_file_descriptor_, &instructions_count_, sizeof(instructions_count_));

    if (!success)
        throw "Error when reading instructions";

    success = read(page_faults_file_descriptor_, &page_faults_count_, sizeof(page_faults_count_));

    if (!success)
        throw "Error when reading page faults";    


    success = read(branch_file_descriptor_, &branches_, sizeof(branches_));

    if (!success)
        throw "Error when reading branch mispredictions";

    success = read(branch_mispredictions_file_descriptor_, &branch_misses_, sizeof(branch_misses_));

    if (!success)
        throw "Error when reading branch mispredictions";


    success = read(cache_references_file_descriptor_, &cache_references_, sizeof(cache_references_));

    if (!success)
        throw "Error when reading cache references";


    success = read(cache_misses_file_descriptor_, &cache_misses_, sizeof(cache_misses_));

    if (!success)
        throw "Error when reading cache misses";
}

process_data snitch::read_data() {
    return {instructions_count_, page_faults_count_, cache_references_, cache_misses_, branches_, branch_misses_};
}