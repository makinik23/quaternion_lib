#ifndef QUATERNIONLIB_QUATERNION_HPP
#define QUATERNIONLIB_QUATERNION_HPP

#include <concepts>
#include <ostream>
#include <type_traits>
#include <cmath>

namespace quaternionlib
{

    template <typename T>
    static inline constexpr auto is_floating_point_v = std::is_floating_point<T>::value;

    template <typename T>
    concept FloatingPoint = std::is_floating_point_v<T> and requires(T v)
    {
        {v + v};
        {v - v};
        {v * v};
        {- v};
    };

    template <FloatingPoint T>
    class Quaternion
    {
    public:
        constexpr Quaternion() noexcept;
        constexpr Quaternion(T x, T y, T z, T w) noexcept;

        [[nodiscard]] constexpr T X() const noexcept;
        [[nodiscard]] constexpr T Y() const noexcept;
        [[nodiscard]] constexpr T Z() const noexcept;
        [[nodiscard]] constexpr T W() const noexcept;

        [[nodiscard]] constexpr T Norm() const noexcept;
        [[nodiscard]] constexpr Quaternion Normalize() const noexcept;
        [[nodiscard]] constexpr Quaternion Conjugate() const noexcept;

        friend std::ostream& operator<<(std::ostream& os, const Quaternion& q)
        {
            return os << "Quaternion(" << q._x << ", " << q._y << ", "
                    << q._z << ", " << q._w << ")";
        }

    private:
        T _x{}, _y{}, _z{}, _w{1};
    };


    template <FloatingPoint T>
    constexpr Quaternion<T>::Quaternion() noexcept = default;

    template <FloatingPoint T>
    constexpr Quaternion<T>::Quaternion(T x, T y, T z, T w) noexcept
        : _x(x), _y(y), _z(z), _w(w) {}

    template <FloatingPoint T>
    constexpr T Quaternion<T>::X() const noexcept 
    {
        return _x;
    }

    template <FloatingPoint T>
    constexpr T Quaternion<T>::Y() const noexcept
    {
        return _y;
    }

    template <FloatingPoint T>
    constexpr T Quaternion<T>::Z() const noexcept
    {
        return _z;
    }

    template <FloatingPoint T>
    constexpr T Quaternion<T>::W() const noexcept
    {
        return _w;
    }

    template <FloatingPoint T>
    constexpr T Quaternion<T>::Norm() const noexcept
    {
        return std::sqrt(_x * _x + _y * _y + _z * _z + _w * _w);
    }

    template <FloatingPoint T>
    constexpr Quaternion<T> Quaternion<T>::Normalize() const noexcept
    {
        T n = Norm();
        return Quaternion{_x / n, _y / n, _z / n, _w / n};
    }

    template <FloatingPoint T>
    constexpr Quaternion<T> Quaternion<T>::Conjugate() const noexcept
    {
        return Quaternion{-_x, -_y, -_z, _w};
    }

} // namespace quaternionlib

#endif // QUATERNIONLIB_QUATERNION_HPP
