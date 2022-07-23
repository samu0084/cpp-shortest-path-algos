#include "memory_map_controller.h"

void memory_map_controller::free(void* address) {
    if (!files_.contains(address)) {
        perror("Address not registered by controller");
        exit(1);
    }
    unsigned long bytes = files_[address].second;
    if (munmap(address, bytes) < 0) {
        perror("Could not unmap file");
        exit(1);
    }
}

void memory_map_controller::purge() {
    for (auto mapping : files_) {
        free(mapping.first);
    }
}