#ifndef PGR_VECTOR_HPP
#define PGR_VECTOR_HPP

#include <vector>

/*
    Prolog:
        Jak kiedyś komisja CPP doda wirtualny destruktor do std::vector to istota takiego czegoś będzie zbędna
*/
namespace pgr
{
    template <typename T>
    class vector
    {
    private:
        std::vector<T> vector_;

    public:
        virtual ~vector() = default;

        // Element access
        typename std::vector<T>::reference operator[](typename std::vector<T>::size_type pos)
        {
            return vector_[pos];
        }
        typename std::vector<T>::const_reference operator[](typename std::vector<T>::size_type pos) const
        {
            return vector_[pos];
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

        // Modifiers
        void clear() noexcept
        {
            vector_.clear();
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
    };
};

#endif