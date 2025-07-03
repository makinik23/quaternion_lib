#include "quaternionlib/Quaternion.hpp"
#include <cmath>

namespace quaternionlib {

template <std::floating_point T>
Quaternion<T>::Quaternion(T x_, T y_, T z_, T w_)
    : x(x_), y(y_), z(z_), w(w_) {}

template <std::floating_point T>
T Quaternion<T>::Norm() const {
    return std::sqrt(x * x + y * y + z * z + w * w);
}

template <std::floating_point T>
Quaternion<T> Quaternion<T>::Normalize() const {
    T n = Norm();
    return Quaternion<T>{x / n, y / n, z / n, w / n};
}

template <std::floating_point T>
Quaternion<T> Quaternion<T>::Conjugate() const {
    return Quaternion<T>{-x, -y, -z, w};
}

// Wymagana instancjacja template
template class Quaternion<float>;
template class Quaternion<double>;

} // namespace quaternionlib
