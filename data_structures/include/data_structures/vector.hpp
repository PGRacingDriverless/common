#ifndef PGR_VECTOR_HPP
#define PGR_VECTOR_HPP

#include <vector>

namespace pgr
{
    template <typename T>
    class vector
    {
    private:
        std::vector<T> vector_;

    public:
    
        virtual ~vector() = default;
    };
};

#endif