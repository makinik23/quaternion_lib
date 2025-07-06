#ifndef QUATERNIONLIB_QUATERNION_HPP
#define QUATERNIONLIB_QUATERNION_HPP

#include <concepts>
#include <ostream>
#include <type_traits>
#include <cmath>

namespace quaternionlib
{
    namespace concepts
    {
        template <typename T, typename S> // TODO: check if one of them is a scalar
        static inline constexpr auto is_scalar_multiplicable_v =
            requires(T t, S s)
            {
                {t * s} -> std::same_as<T>;
                {s * t} -> std::same_as<T>;
            };

        template <typename T>
        static inline constexpr auto is_floating_point_v = std::is_floating_point<T>::value and
            requires(T v)
            {
                {v + v} -> std::same_as<T>;
                {v - v} -> std::same_as<T>;
                {v * v} -> std::same_as<T>;
                {-v} -> std::same_as<T>;
            };

        // template <typename T>
        // static inline constexpr auto is_unary_negatable_v =
        //     requires(T t)
        //     {
        //         {-t} -> std::same_as<T>;
        //     };

        template <typename T, typename S>
        static inline constexpr auto is_multipliable_v =
            requires(T t, S s)
            {
                {t * s} -> std::same_as<T>;
            };
        
        template <typename T> // TODO: necessary???
        static inline constexpr auto is_quaternion_compatible_v =
            requires(T t, T s)
            {
                {t + s} -> std::same_as<T>;
                {t - s} -> std::same_as<T>;
                {t == s} -> std::same_as<bool>;
            };

        template <typename From_, typename To_>
        static inline constexpr auto is_convertible_v = std::is_convertible_v<From_, To_>;

        template <typename T>
        concept FloatingPoint = is_floating_point_v<T>;
        
        template <typename T, typename S>
        concept ScalarMultiplicable = is_scalar_multiplicable_v<T, S>;

        // template <typename T>
        // concept UnaryNegatable = is_unary_negatable_v<T>;

        template <typename T, typename S>
        concept Multipliable = is_multipliable_v<T, S>;

        template <typename T>
        concept QuaternionCompatible = is_quaternion_compatible_v<T>;
    } // namespace concepts

    template <concepts::FloatingPoint T>
    class Quaternion final
    {
    public:
        // Ideas: maybe add a constructor that allocates all internal values with one
        constexpr Quaternion() noexcept;

        explicit constexpr Quaternion(T x, T y, T z, T w) noexcept;

        constexpr Quaternion(T x, T y, T z) noexcept; // add explicit or not???

        template <concepts::FloatingPoint U>
            requires concepts::is_convertible_v<U, T>
        explicit constexpr Quaternion(const Quaternion<U>& other) noexcept;

        explicit constexpr Quaternion(std::initializer_list<T> values) noexcept;

        constexpr Quaternion(const Quaternion& other) noexcept = default;

        constexpr Quaternion(Quaternion&& other) noexcept = default;

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

        // template <concepts::FloatingPoint U>
        //     requires (concepts::QuaternionCompatible<U, T>)
        // constexpr auto operator+(const Quaternion<U>& rhs) const noexcept
        //     -> Quaternion<T>;

    private:
        T _x{}, _y{}, _z{}, _w{static_cast<T>(1)}; // cos moze byc nie tak
    };

    template <concepts::FloatingPoint T>
    constexpr Quaternion<T>::Quaternion() noexcept = default;

    template <concepts::FloatingPoint T>
    constexpr Quaternion<T>::Quaternion(T x, T y, T z, T w) noexcept
        : _x(x), _y(y), _z(z), _w(w) {}

    template <concepts::FloatingPoint T>
    constexpr Quaternion<T>::Quaternion(T x, T y, T z) noexcept
        : _x(x), _y(y), _z(z), _w(static_cast<T>(1)) {}

    template <concepts::FloatingPoint T>
    template <concepts::FloatingPoint U>
        requires concepts::is_convertible_v<U, T>
    constexpr Quaternion<T>::Quaternion(const Quaternion<U>& other) noexcept
        : _x(static_cast<T>(other.X())),
          _y(static_cast<T>(other.Y())),
          _z(static_cast<T>(other.Z())),
          _w(static_cast<T>(other.W())) {}

          

    template <concepts::FloatingPoint T>
    constexpr Quaternion<T>::Quaternion(std::initializer_list<T> values) noexcept
    {
        auto it = values.begin();
        _x = (it != values.end()) ? *it++ : T{};
        _y = (it != values.end()) ? *it++ : T{};
        _z = (it != values.end()) ? *it++ : T{};
        _w = (it != values.end()) ? *it++ : T{1};
    }
            
    template <concepts::FloatingPoint T>
    constexpr T Quaternion<T>::X() const noexcept 
    {
        return _x;
    }

    template <concepts::FloatingPoint T>
    constexpr T Quaternion<T>::Y() const noexcept
    {
        return _y;
    }

    template <concepts::FloatingPoint T>
    constexpr T Quaternion<T>::Z() const noexcept
    {
        return _z;
    }

    template <concepts::FloatingPoint T>
    constexpr T Quaternion<T>::W() const noexcept
    {
        return _w;
    }

    template <concepts::FloatingPoint T>
    constexpr T Quaternion<T>::Norm() const noexcept
    {
        return std::sqrt(_x * _x + _y * _y + _z * _z + _w * _w);
    }

    template <concepts::FloatingPoint T>
    constexpr Quaternion<T> Quaternion<T>::Normalize() const noexcept
    {
        T n = Norm();
        return Quaternion{_x / n, _y / n, _z / n, _w / n};
    }

    template <concepts::FloatingPoint T>
    constexpr Quaternion<T> Quaternion<T>::Conjugate() const noexcept
    {
        return Quaternion{-_x, -_y, -_z, _w};
    }

    // template <concepts::QuaternionCompatible T>
    // template <concepts::FloatingPoint T>
    // template <concepts::FloatingPoint U>
    //     requires (concepts::QuaternionCompatible<U, T>)
    // constexpr auto Quaternion<T>::operator+(const Quaternion<U>& rhs) const noexcept -> Quaternion<T>
    // {
    //     return Quaternion<T>{
    //         static_cast<T>(_x + rhs.X()),
    //         static_cast<T>(_y + rhs.Y()),
    //         static_cast<T>(_z + rhs.Z()),
    //         static_cast<T>(_w + rhs.W())
    //     };
    // }

} // namespace quaternionlib

#endif // QUATERNIONLIB_QUATERNION_HPP
