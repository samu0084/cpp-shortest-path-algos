#include <vector>
#ifndef ATB_FAST_VECTOR_HPP
#define ATB_FAST_VECTOR_HPP

template<typename T>
class fast_vector {
public:
    typedef typename std::vector<T>::value_type value_type;
    typedef typename std::vector<T>::reference reference;
    typedef typename std::vector<T>::const_reference const_reference;
    fast_vector();
    fast_vector(T init);
    fast_vector(size_t size, T init);

    void reset();

    constexpr void clear();
    constexpr void shrink_to_fit();

    constexpr reference operator[](size_t pos);
    constexpr const_reference operator[](size_t pos) const;

    template< typename... Args >
    constexpr reference emplace_back( Args&&... args );
private:
    bool is_certified(size_t pos);
    void write(size_t pos, T val);

    std::vector<T> principal_;
    std::vector<size_t> certification_index_;
    std::vector<size_t> certification_;
    T init_;
    int count_;
};

template<typename T>
fast_vector<T>::fast_vector() 
{

}

template<typename T>
fast_vector<T>::fast_vector(T init) 
: init_(init),
count_(0) 
{

}

template<typename T>
fast_vector<T>::fast_vector(size_t size, T init) 
: init_(init),
principal_(size, init_),
certification_(size),
certification_index_(size),
count_(0) 
{

}

template<typename T>
bool fast_vector<T>::is_certified(size_t pos) {
    bool certification_index_within_bounds = 0 <= certification_index_[pos] && certification_index_[pos] < count_;
    bool is_certified = certification_[certification_index_[pos]] == pos;
    return certification_index_within_bounds && is_certified;
}

template<typename T>
void fast_vector<T>::write(size_t pos, T val) {
    principal_[pos] = val;
    if (!is_certified(pos)) {
        certification_[count_] = pos;
        certification_index_[pos] = count_;
        count_++;
    }
}

template<typename T>
void fast_vector<T>::reset() {
    count_ = 0;
}

template<typename T>
constexpr void fast_vector<T>::clear() {
    principal_.clear();
    certification_.clear();
    certification_index_.clear();
}

template<typename T>
constexpr void fast_vector<T>::shrink_to_fit() {
    principal_.shrink_to_fit();
    certification_.shrink_to_fit();
    certification_index_.shrink_to_fit();
}

template<typename T>
constexpr fast_vector<T>::reference fast_vector<T>::operator[](size_t pos) {
    if (!is_certified(pos))
        write(pos, init_);
    return principal_[pos];
}

template<typename T>
constexpr fast_vector<T>::const_reference fast_vector<T>::operator[](size_t pos) const {
    if (!is_certified(pos))
        write(pos, init_);
    return principal_[pos];
}

template<typename T>
template<typename... Args >
constexpr fast_vector<T>::reference fast_vector<T>::emplace_back( Args&&... args ) {
    size_t pos = principal_.size();
    principal_.emplace_back(args...);
    certification_.push_back(-1);
    certification_[count_] = pos;
    certification_index_.push_back(count_);
    count_++;
}
#endif