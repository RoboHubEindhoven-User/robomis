#ifndef VECTOR_UTILS_H
#define VECTOR_UTILS_H

#include<vector>
#include <stddef.h>

template <typename T>
void remove(std::vector<T>& vec, size_t pos)
{
    typename std::vector<T>::iterator it = vec.begin();
    std::advance(it, pos);
    vec.erase(it);
}

#endif // VECTOR_UTILS_H
