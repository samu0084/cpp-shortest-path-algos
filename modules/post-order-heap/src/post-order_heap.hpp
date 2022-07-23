//
// Created by nicolaj on 3/27/22.
//
// Inspired by:
// Nicholas J. A. Harvey and Kevin C. Zatloukal.
// The post-order heap.
// In Proc. Third International Conference on Fun with Algorithms
// (FUN), 2004.
// http://people.csail.mit.edu/nickh/Publications/PostOrderHeap/FUN04-PostOrderHeap.pdf
//
// Adapted by Nicolaj Kuno Bræmhøj for variable degree.
// Contact: nicolaj (at) braemhoej (dot) me
//

#ifndef POSTORDER_HEAP_LIBRARY_HPP
#define POSTORDER_HEAP_LIBRARY_HPP

#include <vector>
#include <functional>

template <int degree, class T, class Container = std::vector<T>, class Compare = std::less<typename Container::value_type>> 
class postorder_heap {
public:
    typedef typename Container::size_type size_type;
    typedef typename Container::value_type value_type;
    typedef typename Container::const_reference const_reference;
    /**
     * Default constructor, degree = 2 and standard '<' comparator is used.
     */
    explicit postorder_heap();
    /**
     * Constructor with custom comparator
     * @param comparator
     * @param degree
     */
    postorder_heap(Compare comparator);

    /**
     * Pushes and element to the back of the container and ensures that post-order heap-ordering is preserved.
     * @param element to insert
     */
    void push(const value_type &element);
    /**
     * Removes the top element.
     */
    void pop();
    /**
     * Returns, and remove, the top element.
     * @return top element.
     */
    T poll();
    /**
     * Returns the top element.
     * @return top element
     */
    const_reference top();
    /**
     * Checks whether the underlying container is empty.
     * @return
     */
    bool empty();
    /**
     * Returns the size of the underlying container.
     * @return
     */
    size_type size();

    /**
     * @brief Sets the capacity of the underlying container.
     * 
     * @param new_cap the number of elements to reserve space for in the container.
     */
    void reserve(size_type new_cap);

    /**
     * @brief Clears the underlying container.
     * 
     */
    void clear();

    /**
     * @brief Operator for setting the underlying container.
     * 
     * @param container 
     */
    void operator=(Container container);
private:
    // The underlying container
    Container container_;
    // Comparator used for comparisons
    Compare comparator_;
    // Sizes of the implicit heaps.
    std::vector<int> sizes_;
    // Index of last located most prioritised root.
    int prioritised_root_index_;
    // Size of the sub-heap containing the prioritised root.
    int prioritised_root_size_;
    // Has previously located root index been invalidated?
    bool prioritised_root_invalidated_;
    /**
     *
     * @param index
     * @param size
     */
    void heapify(int index, int size_of_subtree);
};

template <int degree, class T, class Container, class Compare> 
postorder_heap<degree, T, Container, Compare> ::postorder_heap() : postorder_heap(Compare()) // Constructor delegation
{ 

}

template <int degree, class T, class Container, class Compare> 
postorder_heap<degree, T, Container, Compare> ::postorder_heap(Compare comparator) :
    container_(Container()),
    comparator_(comparator),
    sizes_(std::vector<int>()),
    prioritised_root_invalidated_(true)
{ 

}

template <int degree, class T, class Container, class Compare> 
void postorder_heap<degree, T, Container, Compare> ::heapify(int root, int size) {
    T initial_root = container_[root];
    while (size > 1) {
        size /= degree;
        int right_child_index = (root - 1);
        int left_child_index = right_child_index - ((degree - 1) * size);
        int prioritised_child_index = left_child_index;
        T prioritised_child = container_[left_child_index];
        for (int i = 1; i < degree; i++) {
            int child_index = left_child_index + (i * size);
            const_reference child = container_[child_index];
            if (comparator_(child, prioritised_child)) {
                prioritised_child_index = child_index;
                prioritised_child = child;
            }
        }
        if (comparator_(initial_root, prioritised_child))
            break;
        container_[root] = std::move(prioritised_child);
        root = prioritised_child_index;
    }
    container_[root] = std::move(initial_root);
}

template <int degree, class T, class Container, class Compare> 
void postorder_heap<degree, T, Container, Compare>::push(const value_type &element) {
    container_.push_back(element);
    bool merge_trees = true;

    // Determine if trees should be merged.
    if (sizes_.size() >= degree) {
        for (int offset = 1; offset < degree; offset++) {
            int tree_index = sizes_.size() - offset;
            merge_trees &= sizes_[tree_index] == sizes_[tree_index - 1];
        }
    } else {
        merge_trees = false;
    }

    // Do (or don't) merge
    if (merge_trees) {
        int size_of_subtree = 1 + (degree * sizes_.back());
        int root_of_subtree = container_.size() - 1;
        for (int index = 0; index < degree; index++)
            sizes_.pop_back();
        sizes_.push_back(size_of_subtree);
        heapify(root_of_subtree, size_of_subtree);
    } else {
        sizes_.push_back(1);
    }
    prioritised_root_invalidated_ = true;
}

template <int degree, class T, class Container, class Compare> 
void postorder_heap<degree, T, Container, Compare>::pop() 
{
    if (prioritised_root_invalidated_) {
        top();
    }

    // Split rightmost heap at root.
    int size_of_subtree = sizes_.back() / degree;
    sizes_.pop_back();

    // If rightmost heap > 1.
    if (size_of_subtree) {
        for (int index = 0; index < degree; index++)
            sizes_.push_back(size_of_subtree);
    }

    // If identified top is not root of rightmost heap...
    if (prioritised_root_index_ < container_.size() - 1) {
        container_[prioritised_root_index_] = std::move(container_.back());
        heapify(prioritised_root_index_, prioritised_root_size_);
    }

    container_.pop_back();
    prioritised_root_invalidated_ = true;
}

template <int degree, class T, class Container, class Compare> 
postorder_heap<degree, T, Container, Compare>::const_reference postorder_heap<degree, T, Container, Compare>:: top() 
{   
    prioritised_root_index_ = sizes_.front() - 1;
    prioritised_root_size_ = sizes_.front();
    int cursor = -1;
    for (int size : sizes_) {
        cursor += size;
        if (comparator_(container_[cursor], container_[prioritised_root_index_])) {
            prioritised_root_index_ = cursor;
            prioritised_root_size_ = size;
        }
    }
    prioritised_root_invalidated_ = false;
    return container_[prioritised_root_index_];
}

template <int degree, class T, class Container, class Compare> 
T postorder_heap<degree, T, Container, Compare>::poll() 
{
    T top = top();
    pop();
    return std::move(top);
}

template <int degree, class T, class Container, class Compare> 
bool postorder_heap<degree, T, Container, Compare>::empty()
{
    return container_.empty();
}

template <int degree, class T, class Container, class Compare> 
postorder_heap<degree, T, Container, Compare>::size_type postorder_heap<degree, T, Container, Compare>::size() 
{
    return container_.size();
}

template <int degree, class T, class Container, class Compare> 
void postorder_heap<degree, T, Container, Compare>::reserve(size_type new_cap) 
{
    container_.reserve(new_cap);
}

template <int degree, class T, class Container, class Compare> 
void postorder_heap<degree, T, Container, Compare>::clear() 
{
    container_.clear();
}

template <int degree, class T, class Container, class Compare> 
void postorder_heap<degree, T, Container, Compare>::operator=(Container container) 
{
    container_ = container;
}

#endif //DARYPOSTORDERHEAP_LIBRARY_HPP