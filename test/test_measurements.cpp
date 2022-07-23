#include "gtest/gtest.h"
#include "../src/measurements/snitch.cpp"

void print_line() {
    std::cout << "Hello World!\n";
}

void add(int &a, int &b) {
    a += b;
}

TEST(snitch, smoke_test) {
    snitch snitch;
    snitch.start();
    print_line();
    snitch.stop();
    auto data = snitch.read_data();
    std::cout << "Instructions " << std::get<0>(data) << "\n";
}

TEST(snitch, none_are_negative) {
    int a = 1;
    int b = 2;
    snitch snitch;
    snitch.start();
    print_line();
    snitch.stop();
    auto data = snitch.read_data();
    
    EXPECT_GE(std::get<0>(data), 0);
    EXPECT_GE(std::get<1>(data), 0);
    EXPECT_GE(std::get<2>(data), 0);
    EXPECT_GE(std::get<3>(data), 0);
    EXPECT_GE(std::get<4>(data), 0);
}