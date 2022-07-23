# D-ary Post-order Heap

C++ library implementing the post-order heap as described by:

Nicholas J. A. Harvey and Kevin C. Zatloukal. 
The post-order heap.
In Proc. Third International Conference on Fun with Algorithms
(FUN), 2004.
http://people.csail.mit.edu/nickh/Publications/PostOrderHeap/FUN04-PostOrderHeap.pdf

This implementation is adapted to support heaps of any degree.

## Usage

The interface, and template, is designed to mimic that of the [C++ standard priority_queue](https://en.cppreference.com/w/cpp/container/priority_queue), with the exception of the added template parameter degree.
```cpp
template <
          int degree,
          class T, 
          class Container = std::vector<T>, 
          class Compare = std::less<typename Container::value_type>
> class postorder_heap
```
#### Template parameters
  1. **degree** - Integer specifying the degree of the constructed post-order heap.
  2. **T** - Element type, must equal Container::value_type.
  3. **Container** - The type of the container. Must satisfy the requirements of SequenceContainer, and its iterator must satisfy the requirements of LegacyRandomAccessIterator.
  4. **Compare** - A compare type providing strict weak ordering. ***NOTE***: The heap outputs smallest elements first, unlike std::priority_queue. That is, the front/root of the heap contains the "first" element according to the weak ordering imposed by Compare.

### Construction

Construction of heaps can happen using the following three kinds of constructors:

**The explicit constructor**

```cpp
    auto comparator = [](int left, int right) {
        return left < right;
    };
    postorder_heap<3, int, std::vector<int>, decltype(comparator)> heap(comparator);
```

All necessary parameters are supplied. Creates a post-order heap with the specified degree (here 3), and a custom comparator.

**The implicit constructor**

```cpp
    post_order_heap<3, int> heap;
```
Creates a post-order heap with the specified degree (here 3) using std::less<> as the comparator.

### Push, Poll, Pop & Top

```cpp
    postorder_heap<2, int, std::vector<int>, std::less<>> heap{};
    
    heap.push(7);
    heap.push(33);
    heap.push(13);
    heap.push(42);
    
    int poll = heap.poll(); // poll = 7
    int top = heap.top(); // top = 13
    heap.pop();
    top = top(); // top = 33;
```
