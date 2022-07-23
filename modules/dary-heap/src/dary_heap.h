#include <vector>
#include <functional>

template <int degree, class T, class Container = std::vector<T>, class Compare = std::less<typename Container::value_type>> 
class dary_heap {

    typedef typename Container::size_type size_type;
    typedef typename Container::value_type value_type;
    typedef typename Container::const_reference const_reference;

public:
    /**
     * @brief Construct a new dary heap object
     * 
     */
    dary_heap() = default;
    
    dary_heap(Compare comparator) : comparator_(comparator) { }

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
        container_.push_back(item);
        int index = number_of_elements;
        while (index > 0) {
            int parent_index = parent(index);
            if (!comparator_(item, container_[parent_index]))
                break;
            std::swap(container_[index], container_[parent_index]);
            index = parent_index;
        }
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

    int parent(int index) {
        return (index - 1) / degree;
    }
    int firstChild(int index) {
        return (degree * index) + 1;
    }
    int lastChild(int index) {
        return (degree * index) + degree;
    }
    void heapify(int index) {
        int first_child_index = firstChild(index);
        int last_child_index = lastChild(index);
        T old_root = std::move(container_[index]);
        while (first_child_index < container_.size()) {
            int smallest_child_index = first_child_index;
            T smallest_child = container_[smallest_child_index];
            for (int child_index = first_child_index; child_index < container_.size() && child_index <= last_child_index; child_index++) {
                T child = container_[child_index];
                if (comparator_(child, smallest_child)) {
                    std::swap(child, smallest_child);
                    smallest_child_index = child_index;
                }
            }
            std::swap(container_[index], container_[smallest_child_index]);
            index = smallest_child_index;
            first_child_index = firstChild(index);
            last_child_index = lastChild(index);
        }
        while (index > 0) {
            int parent_index = parent(index);
            if (!(old_root < container_[parent_index]))
                break;
            std::swap(container_[index], container_[parent_index]);
            index = parent_index;
        }
        container_[index] = std::move(old_root);
    }
};