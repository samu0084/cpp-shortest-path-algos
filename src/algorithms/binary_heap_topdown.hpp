#include <vector>
#include <functional>
#include <stdexcept>

template <class T, class Container = std::vector<T>, class Compare = std::less<typename Container::value_type>> 
class binary_heap_topdown {
    typedef typename Container::size_type size_type;
    typedef typename Container::value_type value_type;
    typedef typename Container::const_reference const_reference;

public:
    /**
     * @brief Construct a new dary heap object
     * 
     */
    binary_heap_topdown() = default;
    
    binary_heap_topdown(Compare comparator) : comparator_(comparator) { }

    value_type top() {
        if (empty())
            throw std::out_of_range("Container is empty");
        return container_[0];
    }

    bool empty() {
        return container_.empty();
    }

    void reserve(size_t number) {
        container_.reserve(number);
    }

    void clear() {
        container_.clear();
    }

    Container& container() {
        return container_;
    }

    void push(T item) {
        int number_of_elements = container_.size();
        int index = container_.size();
        int cursor = index;
        container_.push_back(item);
        int level = 0;
        while (index > 0) {
            index = parent(index);
            level++;
        }
        while (index < cursor && comparator_(container_[index], item)) {
            index = ((cursor + 1) >> level) - 1;
            level--; 
        }
        for (int parent_index = parent(cursor); cursor != index; parent_index = parent(cursor)) {
            container_[cursor] = container_[parent_index];
            cursor = parent_index;
        }
        container_[index] = item;
    }

    void pop() {
        if (empty())
            throw std::out_of_range("Container is empty");
        if (container_.size() > 1)
            std::swap(container_[0], container_.back());
        container_.pop_back();
        if (!empty())
            heapify(0);
    }

private:
    // The underlying container
    Container container_;
    // Comparator used for comparisons
    Compare comparator_;

    inline int parent(int index) {
        return (index - 1) / 2;
    }
    inline int leftChild(int index) {
        return (2 * index) + 1;
    }
    inline int rightChild(int index) {
        return (2 * index) + 2;
    }
    void heapify(int index) {
        T initial_root = container_[index];
        while (true) {
            int smallest_child_index = leftChild(index);
            if (smallest_child_index >= container_.size()) 
                break;
            int right_child_index = rightChild(index);
            if (right_child_index < container_.size() && comparator_(container_[right_child_index], container_[smallest_child_index]))
                smallest_child_index = right_child_index;
            if (comparator_(initial_root, container_[smallest_child_index]))
                break;
            container_[index] = std::move(container_[smallest_child_index]);
            index = smallest_child_index;
        }
        container_[index] = std::move(initial_root);
    }
};