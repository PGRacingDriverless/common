#ifndef PGR_VECTOR_HPP
#define PGR_VECTOR_HPP

#include <vector>

/*
Prolog:
    If the C++ Standards Committee ever adds a virtual destructor to std::vector, the essence of something like this will become unnecessary.
*/

template <typename T>
class vector
{
private:
    std::vector<T> vector_;

public:
    // Member functions
    vector() = default;
    explicit vector(typename std::vector<T>::size_type count)
    {
        vector_ = std::vector<T>(count);
    }
    vector(typename std::vector<T>::size_type count, const T &value)
    {
        vector_ = std::vector<T>(count, value);
    }
    vector(const vector &other)
    {
        vector_ = std::vector<T>(other.vector_);
    }
    vector(vector &&other)
    {
        vector_ = std::move(other.vector_);
    }
    vector(std::initializer_list<T> init)
    {
        vector_ = std::vector<T>(init);
    }

    virtual ~vector() = default;

    vector<T> &operator=(const vector<T> &other)
    {
        vector_ = other.vector_;
        return *this;
    }
    vector<T> &operator=(vector<T> &&other) noexcept
    {
        vector_ = std::move(other.vector_);
        return *this;
    }
    vector<T> &operator=(std::initializer_list<T> ilist)
    {
        vector_ = ilist;
        return *this;
    }

    void assign(typename std::vector<T>::size_type count, const T &value)
    {
        vector_.assign(count, value);
    }
    template <class InputIt>
    void assign(InputIt first, InputIt last)
    {
        vector_.assign(first, last);
    }
    void assign(std::initializer_list<T> ilist)
    {
        vector_.assign(ilist);
    }

    // Element access
    typename std::vector<T>::reference at(typename std::vector<T>::size_type pos)
    {
        return vector_.at(pos);
    }
    typename std::vector<T>::const_reference at(typename std::vector<T>::size_type pos) const
    {
        return vector_.at(pos);
    }

    typename std::vector<T>::reference operator[](typename std::vector<T>::size_type pos)
    {
        return vector_[pos];
    }
    typename std::vector<T>::const_reference operator[](typename std::vector<T>::size_type pos) const
    {
        return vector_[pos];
    }

    typename std::vector<T>::reference front()
    {
        return vector_.front();
    }
    typename std::vector<T>::const_reference front() const
    {
        return vector_.front();
    }

    typename std::vector<T>::reference back()
    {
        return vector_.back();
    }
    typename std::vector<T>::const_reference back() const
    {
        return vector_.back();
    }

    T *data()
    {
        return vector_.data();
    }
    const T *data() const
    {
        return vector_.data();
    }

    // Iterators
    typename std::vector<T>::iterator begin() noexcept
    {
        return vector_.begin();
    }
    typename std::vector<T>::const_iterator begin() const noexcept
    {
        return vector_.begin();
    }
    typename std::vector<T>::const_iterator cbegin() const noexcept
    {
        return vector_.cbegin();
    }

    typename std::vector<T>::iterator end() noexcept
    {
        return vector_.end();
    }
    typename std::vector<T>::const_iterator end() const noexcept
    {
        return vector_.end();
    }
    typename std::vector<T>::const_iterator cend() const noexcept
    {
        return vector_.cend();
    }

    typename std::vector<T>::reverse_iterator rbegin() noexcept
    {
        return vector_.rbegin();
    }
    typename std::vector<T>::const_reverse_iterator rbegin() const noexcept
    {
        return vector_.rbegin();
    }
    typename std::vector<T>::const_reverse_iterator crbegin() const noexcept
    {
        return vector_.crbegin();
    }

    typename std::vector<T>::reverse_iterator rend() noexcept
    {
        return vector_.rend();
    }
    typename std::vector<T>::const_reverse_iterator rend() const noexcept
    {
        return vector_.rend();
    }
    typename std::vector<T>::const_reverse_iterator crend() const noexcept
    {
        return vector_.crend();
    }

    // Capacity
    bool empty() const noexcept
    {
        return vector_.empty();
    }

    typename std::vector<T>::size_type size() const noexcept
    {
        return vector_.size();
    }

    typename std::vector<T>::size_type max_size() const noexcept
    {
        return vector_.max_size();
    }

    void reserve(typename std::vector<T>::size_type new_cap)
    {
        vector_.reserve(new_cap);
    }

    typename std::vector<T>::size_type capacity() const noexcept
    {
        return vector_.capacity();
    }

    // Modifiers
    void clear() noexcept
    {
        vector_.clear();
    }

    typename std::vector<T>::iterator insert(typename std::vector<T>::const_iterator pos,
                                             const T &value)
    {
        return vector_.insert(pos, value);
    }
    typename std::vector<T>::iterator insert(typename std::vector<T>::const_iterator pos,
                                             T &&value)
    {
        return vector_.insert(pos, std::move(value));
    }
    typename std::vector<T>::iterator insert(typename std::vector<T>::const_iterator pos,
                                             typename std::vector<T>::size_type count,
                                             const T &value)
    {
        return vector_.insert(pos, count, value);
    }
    template <class InputIt>
    typename std::vector<T>::iterator insert(typename std::vector<T>::const_iterator pos,
                                             InputIt first,
                                             InputIt last)
    {
        return vector_.insert(pos, first, last);
    }
    typename std::vector<T>::iterator insert(typename std::vector<T>::const_iterator pos,
                                             std::initializer_list<T> ilist)
    {
        return vector_.insert(pos, ilist);
    }

    template <class... Args>
    typename std::vector<T>::iterator emplace(typename std::vector<T>::const_iterator pos, Args &&...args)
    {
        return vector_.emplace(pos, std::forward<Args>(args)...);
    }

    typename std::vector<T>::iterator erase(typename std::vector<T>::const_iterator pos)
    {
        return vector_.erase(pos);
    }
    typename std::vector<T>::const_iterator erase(typename std::vector<T>::const_iterator first, typename std::vector<T>::const_iterator last)
    {
        return vector_.erase(first, last);
    }

    void push_back(const T &value)
    {
        vector_.push_back(value);
    }
    void push_back(T &&value)
    {
        vector_.push_back(std::move(value));
    }

    template <class... Args>
    void emplace_back(Args &&...args)
    {
        vector_.emplace_back(std::forward<Args>(args)...);
    }

    void pop_back()
    {
        vector_.pop_back();
    }

    void resize(typename std::vector<T>::size_type count)
    {
        vector_.resize(count);
    }
    void resize(typename std::vector<T>::size_type count, const T &value)
    {
        vector_.resize(count, value);
    }
};

#endif