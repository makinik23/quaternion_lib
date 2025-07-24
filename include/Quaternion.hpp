#ifndef QUATERNIONLIB_QUATERNION_HPP
#define QUATERNIONLIB_QUATERNION_HPP

#include <cassert>
#include <cmath>
#include <concepts>
#include <ostream>
#include <type_traits>
#include <utility>

namespace quaternionlib
{
    template <typename T>
    static inline constexpr T EPSILON = std::numeric_limits<T>::epsilon();

    namespace details
    {
        template <typename T>
        static inline constexpr auto is_arithmetic_v = std::is_arithmetic<T>::value;

        template <typename From_, typename To_>
        static inline constexpr auto is_convertible_v = std::is_convertible_v<From_, To_>;

        template <typename T>
        concept Arithmetic = is_arithmetic_v<T> and requires(T v) {
            { v + v } -> std::same_as<T>;
            { v - v } -> std::same_as<T>;
            { v * v } -> std::same_as<T>;
            { v / v } -> std::same_as<T>;
            { -v } -> std::same_as<T>;
        };

        template <typename T, typename U>
        concept Scalar = std::is_scalar_v<U> and requires(T a_type, U b_type) {
            { a_type * b_type };
            { a_type / b_type };
        };

        template <typename From_, typename To_>
        concept QuaternionConvertible = is_convertible_v<From_, To_>;
    } // namespace details

    template <details::Arithmetic T>
    class Quaternion final
    {
    public:
        using value_type = T;

        constexpr Quaternion() noexcept = default;

        constexpr ~Quaternion() noexcept = default;

        explicit constexpr Quaternion(const T& x, const T& y, const T& z, const T& w) noexcept;

        constexpr Quaternion(const T& x, const T& y, const T& z) noexcept;

        constexpr Quaternion(std::initializer_list<T> values);
        constexpr auto operator=(std::initializer_list<T> values) -> Quaternion<T>&;

        constexpr Quaternion(const Quaternion<T>& other) noexcept;
        constexpr auto operator=(const Quaternion<T>& other) noexcept -> Quaternion<T>&;

        constexpr Quaternion(Quaternion<T>&& other) noexcept;
        constexpr auto operator=(Quaternion<T>&& other) noexcept -> Quaternion<T>&;

        template <details::Arithmetic U>
        requires details::QuaternionConvertible<U, T>
        explicit constexpr Quaternion(const Quaternion<U>& other) noexcept;

        template <details::Arithmetic U>
        requires details::QuaternionConvertible<U, T>
        constexpr auto operator=(const Quaternion<U>& other) noexcept -> Quaternion<T>&;

        template <details::Arithmetic U>
        requires details::QuaternionConvertible<U, T>
        explicit constexpr Quaternion(Quaternion<U>&& other) noexcept;

        template <details::Arithmetic U>
        requires details::QuaternionConvertible<U, T>
        constexpr auto operator=(Quaternion<U>&& other) noexcept -> Quaternion<T>&;

        [[nodiscard]] constexpr auto X() const noexcept -> T;
        [[nodiscard]] constexpr auto Y() const noexcept -> T;
        [[nodiscard]] constexpr auto Z() const noexcept -> T;
        [[nodiscard]] constexpr auto W() const noexcept -> T;

        constexpr auto Zero() noexcept -> void;

        [[nodiscard]] constexpr auto Norm() const noexcept -> T;
        [[nodiscard]] constexpr auto SquaredNorm() const noexcept -> T;
        constexpr auto Normalize() noexcept -> void;
        [[nodiscard]] constexpr auto Normalized() const noexcept -> Quaternion<T>;
        [[nodiscard]] constexpr auto IsNormalized() const noexcept -> bool;
        constexpr auto Conjugate() noexcept -> void;
        [[nodiscard]] constexpr auto Conjugated() const noexcept -> Quaternion<T>;
        constexpr auto Inverse() noexcept -> void;
        [[nodiscard]] constexpr auto Inversed() const noexcept -> Quaternion<T>;

        template <details::Arithmetic U>
        friend constexpr auto operator<<(std::ostream&, const Quaternion<U>&) -> std::ostream&;

        template <details::Arithmetic U>
        requires details::QuaternionConvertible<T, U>
        explicit constexpr operator Quaternion<U>() const noexcept;

        template <details::Arithmetic U>
        requires details::QuaternionConvertible<U, T>
        constexpr auto operator+=(const Quaternion<U>& other) noexcept -> Quaternion<T>&;

        template <details::Arithmetic U>
        requires details::QuaternionConvertible<U, T>
        constexpr auto operator-=(const Quaternion<U>& other) noexcept -> Quaternion<T>&;

        template <details::Arithmetic U>
        requires details::QuaternionConvertible<U, T>
        constexpr auto operator*=(const Quaternion<U>& other) noexcept -> Quaternion<T>&;

        template <details::Scalar<T> U>
        requires details::QuaternionConvertible<U, T>
        constexpr auto operator*=(const U& scalar) noexcept -> Quaternion<T>&;

        template <details::Scalar<T> U>
        requires details::QuaternionConvertible<U, T>
        constexpr auto operator/=(const U& scalar) noexcept -> Quaternion<T>&;

        constexpr auto operator-() const noexcept -> Quaternion<T>;

    private:
        T _x{}, _y{}, _z{}, _w{};
    };

    template <details::Arithmetic T>
    constexpr Quaternion<T>::Quaternion(const T& x, const T& y, const T& z, const T& w) noexcept
        : _x(x), _y(y), _z(z), _w(w)
    {
    }

    template <details::Arithmetic T>
    constexpr Quaternion<T>::Quaternion(const T& x, const T& y, const T& z) noexcept
        : _x(x), _y(y), _z(z), _w(static_cast<T>(1))
    {
    }

    template <details::Arithmetic T>
    constexpr Quaternion<T>::Quaternion(std::initializer_list<T> values)
    {
        if (values.size() != 3 && values.size() != 4) [[unlikely]]
        {
            throw std::invalid_argument("Quaternion requires at least 3 values (x, y, z).");
        }

        else [[likely]]
        {
            auto it = values.begin();
            _x = (it != values.end()) ? *it++ : T{};
            _y = (it != values.end()) ? *it++ : T{};
            _z = (it != values.end()) ? *it++ : T{};
            _w = (it != values.end()) ? *it++ : T{1};
        }
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::operator=(std::initializer_list<T> values) -> Quaternion<T>&
    {
        if (values.size() != 3 && values.size() != 4) [[unlikely]]
        {
            throw std::invalid_argument("Quaternion requires at least 3 values (x, y, z).");
        }

        else [[likely]]
        {
            auto it = values.begin();
            _x = (it != values.end()) ? *it++ : T{};
            _y = (it != values.end()) ? *it++ : T{};
            _z = (it != values.end()) ? *it++ : T{};
            _w = (it != values.end()) ? *it++ : T{1};

            return *this;
        }
    }

    template <details::Arithmetic T>
    constexpr Quaternion<T>::Quaternion(const Quaternion<T>& other) noexcept
        : _x(other._x), _y(other._y), _z(other._z), _w(other._w)
    {
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::operator=(const Quaternion<T>& other) noexcept -> Quaternion<T>&
    {
        if (this != &other)
        {
            _x = other._x;
            _y = other._y;
            _z = other._z;
            _w = other._w;
        }

        return *this;
    }

    template <details::Arithmetic T>
    constexpr Quaternion<T>::Quaternion(Quaternion<T>&& other) noexcept
        : _x{std::exchange(other._x, T{})},
          _y{std::exchange(other._y, T{})},
          _z{std::exchange(other._z, T{})},
          _w{std::exchange(other._w, T{})}
    {
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::operator=(Quaternion<T>&& other) noexcept -> Quaternion<T>&
    {
        if (this != &other)
        {
            _x = std::exchange(other._x, T{});
            _y = std::exchange(other._y, T{});
            _z = std::exchange(other._z, T{});
            _w = std::exchange(other._w, T{});
        }

        return *this;
    }

    template <details::Arithmetic T>
    template <details::Arithmetic U>
    requires details::QuaternionConvertible<U, T>
    constexpr Quaternion<T>::Quaternion(const Quaternion<U>& other) noexcept
        : _x(static_cast<T>(other.X())),
          _y(static_cast<T>(other.Y())),
          _z(static_cast<T>(other.Z())),
          _w(static_cast<T>(other.W()))
    {
    }

    template <details::Arithmetic T>
    template <details::Arithmetic U>
    requires details::QuaternionConvertible<U, T>
    constexpr auto Quaternion<T>::operator=(const Quaternion<U>& other) noexcept -> Quaternion<T>&
    {
        _x = static_cast<T>(other.X());
        _y = static_cast<T>(other.Y());
        _z = static_cast<T>(other.Z());
        _w = static_cast<T>(other.W());

        return *this;
    }

    template <details::Arithmetic T>
    template <details::Arithmetic U>
    requires details::QuaternionConvertible<U, T>
    constexpr Quaternion<T>::Quaternion([[maybe_unused]] Quaternion<U>&& other) noexcept
        : _x(static_cast<T>(other.X())),
          _y(static_cast<T>(other.Y())),
          _z(static_cast<T>(other.Z())),
          _w(static_cast<T>(other.W()))
    {
        other.Zero();
    }

    template <details::Arithmetic T>
    template <details::Arithmetic U>
    requires details::QuaternionConvertible<U, T>
    constexpr auto Quaternion<T>::operator=([[maybe_unused]] Quaternion<U>&& other) noexcept
        -> Quaternion<T>&
    {
        *this = static_cast<Quaternion<T>>(std::move(other));
        other.Zero();

        return *this;
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::X() const noexcept -> T
    {
        return _x;
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::Y() const noexcept -> T
    {
        return _y;
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::Z() const noexcept -> T
    {
        return _z;
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::W() const noexcept -> T
    {
        return _w;
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::Zero() noexcept -> void
    {
        _x = T{};
        _y = T{};
        _z = T{};
        _w = T{};
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::Norm() const noexcept -> T
    {
        return std::sqrt((_x * _x) + (_y * _y) + (_z * _z) + (_w * _w));
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::SquaredNorm() const noexcept -> T
    {
        return (_x * _x) + (_y * _y) + (_z * _z) + (_w * _w);
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::Normalize() noexcept -> void
    {
        const T n = Norm();

        _w /= n;
        _x /= n;
        _y /= n;
        _z /= n;
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::Normalized() const noexcept -> Quaternion<T>
    {
        const T n = Norm();

        return Quaternion{_x / n, _y / n, _z / n, _w / n};
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::IsNormalized() const noexcept -> bool
    {
        return std::abs(Quaternion<T>::SquaredNorm() - static_cast<T>(1)) <= EPSILON<T>;
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::Conjugate() noexcept -> void
    {
        _x *= -1;
        _y *= -1;
        _z *= -1;
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::Conjugated() const noexcept -> Quaternion<T>
    {
        return Quaternion{-_x, -_y, -_z, _w};
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::Inverse() noexcept -> void
    {
        *this = Conjugated() / SquaredNorm();
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::Inversed() const noexcept -> Quaternion<T>
    {
        return Conjugated() / SquaredNorm();
    }

    template <details::Arithmetic T>
    template <details::Arithmetic U>
    requires details::QuaternionConvertible<T, U>
    constexpr Quaternion<T>::operator Quaternion<U>() const noexcept
    {
        return Quaternion<U>{static_cast<U>(_x), static_cast<U>(_y), static_cast<U>(_z),
                             static_cast<U>(_w)};
    }

    template <details::Arithmetic T>
    template <details::Arithmetic U>
    requires details::QuaternionConvertible<U, T>
    constexpr auto Quaternion<T>::operator+=(const Quaternion<U>& other) noexcept -> Quaternion<T>&
    {
        _x += static_cast<T>(other.X());
        _y += static_cast<T>(other.Y());
        _z += static_cast<T>(other.Z());
        _w += static_cast<T>(other.W());

        return *this;
    }

    template <details::Arithmetic T>
    template <details::Arithmetic U>
    requires details::QuaternionConvertible<U, T>
    constexpr auto Quaternion<T>::operator-=(const Quaternion<U>& other) noexcept -> Quaternion<T>&
    {
        _x -= static_cast<T>(other.X());
        _y -= static_cast<T>(other.Y());
        _z -= static_cast<T>(other.Z());
        _w -= static_cast<T>(other.W());

        return *this;
    }

    template <details::Arithmetic T>
    constexpr auto operator<<(std::ostream& os, const Quaternion<T>& q) -> std::ostream&
    {
        return os << "Quaternion(" << q._x << ", " << q._y << ", " << q._z << ", " << q._w << ")";
    }

    template <details::Arithmetic T, details::Arithmetic U>
    requires details::QuaternionConvertible<T, U>
    [[nodiscard]] constexpr auto operator==(const Quaternion<T>& lhs,
                                            const Quaternion<U>& rhs) noexcept -> bool
    {
        return lhs.W() == rhs.W() && lhs.X() == rhs.X() && lhs.Y() == rhs.Y() && lhs.Z() == rhs.Z();
    }

    template <details::Arithmetic T, details::Arithmetic U>
    requires details::QuaternionConvertible<T, U>
    [[nodiscard]] constexpr auto operator==(const Quaternion<T>& q,
                                            const std::initializer_list<U>& list) noexcept -> bool
    {
        if (list.size() != 4)
        {
            return false;
        }

        auto it = list.begin();

        const auto x = *it++;
        const auto y = *it++;
        const auto z = *it++;
        const auto w = *it;

        return q.X() == x && q.Y() == y && q.Z() == z && q.W() == w;
    }

    template <details::Arithmetic T, details::Arithmetic U>
    requires details::QuaternionConvertible<T, U>
    [[nodiscard]] constexpr auto operator!=(const Quaternion<T>& lhs,
                                            const Quaternion<U>& rhs) noexcept -> bool
    {
        return !(lhs == rhs);
    }

    template <details::Arithmetic T, details::Arithmetic U>
    requires details::QuaternionConvertible<T, U>
    [[nodiscard]] constexpr auto IsApproxEqual(const Quaternion<T>& lhs,
                                               const Quaternion<U>& rhs) noexcept -> bool
    {
        using V = std::common_type_t<T, U>;

        return std::abs(lhs.W() - rhs.W()) <= EPSILON<V> && std::abs(lhs.X() - rhs.X()) <= EPSILON<V> &&
               std::abs(lhs.Y() - rhs.Y()) <= EPSILON<V> && std::abs(lhs.Z() - rhs.Z()) <= EPSILON<V>;
    }

    template <details::Arithmetic T, details::Arithmetic U>
    constexpr auto operator+(const Quaternion<T>& lhs, const Quaternion<U>& rhs)
        -> Quaternion<std::common_type_t<T, U>>
    {
        auto tmp = static_cast<Quaternion<std::common_type_t<T, U>>>(lhs);
        tmp += rhs;

        return tmp;
    }

    template <details::Arithmetic T, details::Arithmetic U>
    constexpr auto operator-(const Quaternion<T>& lhs, const Quaternion<U>& rhs)
        -> Quaternion<std::common_type_t<T, U>>
    {
        auto tmp = static_cast<Quaternion<std::common_type_t<T, U>>>(lhs);
        tmp -= rhs;

        return tmp;
    }

    template <details::Arithmetic T>
    template <details::Arithmetic U>
    requires details::QuaternionConvertible<U, T>
    constexpr auto Quaternion<T>::operator*=(const Quaternion<U>& other) noexcept -> Quaternion<T>&
    {
        const T x1 = _x;
        const T y1 = _y;
        const T z1 = _z;
        const T w1 = _w;

        const T x2 = static_cast<T>(other.X());
        const T y2 = static_cast<T>(other.Y());
        const T z2 = static_cast<T>(other.Z());
        const T w2 = static_cast<T>(other.W());

        _x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
        _y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
        _z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
        _w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;

        return *this;
    }

    template <details::Arithmetic T>
    template <details::Scalar<T> U>
    requires details::QuaternionConvertible<U, T>
    constexpr auto Quaternion<T>::operator*=(const U& scalar) noexcept -> Quaternion<T>&
    {
        _w *= static_cast<T>(scalar);
        _x *= static_cast<T>(scalar);
        _y *= static_cast<T>(scalar);
        _z *= static_cast<T>(scalar);

        return *this;
    }

    template <details::Arithmetic T>
    template <details::Scalar<T> U>
    requires details::QuaternionConvertible<U, T>
    constexpr auto Quaternion<T>::operator/=(const U& scalar) noexcept
        -> Quaternion<T>& // TODO u dumb if 0
    {
        assert(scalar != 0);

        _w /= static_cast<T>(scalar);
        _x /= static_cast<T>(scalar);
        _y /= static_cast<T>(scalar);
        _z /= static_cast<T>(scalar);

        return *this;
    }

    template <details::Arithmetic T, details::Scalar<T> U>
    requires details::QuaternionConvertible<U, T>
    constexpr auto operator*(const Quaternion<T>& lhs, const U& rhs)
        -> Quaternion<std::common_type_t<T, U>>
    {
        auto tmp = static_cast<Quaternion<std::common_type_t<T, U>>>(lhs);
        tmp *= rhs;

        return tmp;
    }

    template <details::Arithmetic T, details::Scalar<T> U>
    requires details::QuaternionConvertible<U, T>
    constexpr auto operator*(const U& lhs, const Quaternion<T>& rhs)
        -> Quaternion<std::common_type_t<T, U>>
    {
        auto tmp = static_cast<Quaternion<std::common_type_t<T, U>>>(rhs);
        tmp *= lhs;

        return tmp;
    }

    template <details::Arithmetic T, details::Scalar<T> U>
    requires details::QuaternionConvertible<U, T>
    constexpr auto operator/(const Quaternion<T>& lhs, const U& rhs)
        -> Quaternion<std::common_type_t<T, U>>
    {
        if (rhs == 0)
        {
            throw std::domain_error("One must not divide by 0");
        }

        auto tmp = static_cast<Quaternion<std::common_type_t<T, U>>>(lhs);
        tmp /= rhs;

        return tmp;
    }

    template <details::Arithmetic T, details::Arithmetic U>
    requires details::QuaternionConvertible<U, T>
    constexpr auto operator*(const Quaternion<T>& lhs, const Quaternion<U>& rhs)
        -> Quaternion<std::common_type_t<T, U>>
    {
        auto tmp = static_cast<Quaternion<std::common_type_t<T, U>>>(lhs);
        tmp *= rhs;

        return tmp;
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::operator-() const noexcept -> Quaternion<T>
    {
        return Quaternion{-_x, -_y, -_z, -_w};
    }

} // namespace quaternionlib

#endif // QUATERNIONLIB_QUATERNION_HPP
