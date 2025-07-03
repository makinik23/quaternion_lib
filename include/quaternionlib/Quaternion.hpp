#ifndef QUATERNIONLIB_QUATERNION_HPP
#define QUATERNIONLIB_QUATERNION_HPP

#include <concepts>
#include <ostream>
#include <type_traits>

namespace quaternionlib {

template <typename T>
concept FloatingPoint = std::is_floating_point_v<T>;

template <FloatingPoint T>
class Quaternion {
public:
    T x{}, y{}, z{}, w{1};

    Quaternion() = default;
    Quaternion(T x, T y, T z, T w);

    [[nodiscard]] T Norm() const;
    Quaternion<T> Normalize() const;
    Quaternion<T> Conjugate() const;

    friend std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
        return os << "Quaternion(" << q.x << ", " << q.y << ", "
                  << q.z << ", " << q.w << ")";
    }
};

}

#endif
