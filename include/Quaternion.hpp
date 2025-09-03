#ifndef QUATERNIONLIB_QUATERNION_HPP
#define QUATERNIONLIB_QUATERNION_HPP

#include <cassert>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <expected>
#include <ostream>
#include <type_traits>
#include <utility>

namespace quaternionlib
{
    template <typename T>
    static inline constexpr T EPSILON = std::numeric_limits<T>::epsilon();

    namespace error
    {
        enum class QuaternionError : std::uint8_t
        {
            DivisionByZero = 0,
            InvalidQuaternionSize = 1,
            InvalidInterpolationTime = 2
        };
    }

    namespace details
    {
        template <typename ResultType>
        using Result = std::expected<ResultType, error::QuaternionError>;

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
            { v == v } -> std::same_as<bool>;
            { v != v } -> std::same_as<bool>;
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

        constexpr Quaternion(const Quaternion<T>& other) noexcept = default;
        constexpr auto operator=(const Quaternion<T>& other) noexcept -> Quaternion<T>& = default;

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

        [[nodiscard]] constexpr auto X() noexcept -> T&;
        [[nodiscard]] constexpr auto X() const noexcept -> const T&;

        [[nodiscard]] constexpr auto Y() noexcept -> T&;
        [[nodiscard]] constexpr auto Y() const noexcept -> const T&;

        [[nodiscard]] constexpr auto Z() noexcept -> T&;
        [[nodiscard]] constexpr auto Z() const noexcept -> const T&;

        [[nodiscard]] constexpr auto W() noexcept -> T&;
        [[nodiscard]] constexpr auto W() const noexcept -> const T&;

        [[nodiscard]] constexpr auto ScalarPart() const noexcept -> T;
        [[nodiscard]] constexpr auto VectorPart() const noexcept -> std::array<T, 3>;

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
        constexpr auto operator/=(const U& scalar) -> Quaternion<T>&;

        constexpr auto operator-() const noexcept -> Quaternion<T>;

        constexpr auto operator[](std::size_t index) -> T&;
        constexpr auto operator[](std::size_t index) const -> const T&;

    private:
        std::array<T, 4> _data{};
    };

    template <details::Arithmetic T>
    constexpr Quaternion<T>::Quaternion(const T& x, const T& y, const T& z, const T& w) noexcept
        : _data{x, y, z, w}
    {
    }

    template <details::Arithmetic T>
    constexpr Quaternion<T>::Quaternion(const T& x, const T& y, const T& z) noexcept
        : _data{x, y, z, T{1}}
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

            _data[0] = (it != values.end()) ? *it++ : T{};
            _data[1] = (it != values.end()) ? *it++ : T{};
            _data[2] = (it != values.end()) ? *it++ : T{};
            _data[3] = (it != values.end()) ? *it++ : T{1};
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

            _data[0] = (it != values.end()) ? *it++ : T{};
            _data[1] = (it != values.end()) ? *it++ : T{};
            _data[2] = (it != values.end()) ? *it++ : T{};
            _data[3] = (it != values.end()) ? *it++ : T{1};

            return *this;
        }
    }

    template <details::Arithmetic T>
    constexpr Quaternion<T>::Quaternion(Quaternion<T>&& other) noexcept
        : _data{std::exchange(other._data[0], T{}), std::exchange(other._data[1], T{}),
                std::exchange(other._data[2], T{}), std::exchange(other._data[3], T{})}
    {
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::operator=(Quaternion<T>&& other) noexcept -> Quaternion<T>&
    {
        if (this != &other)
        {
            _data[0] = std::exchange(other._data[0], T{});
            _data[1] = std::exchange(other._data[1], T{});
            _data[2] = std::exchange(other._data[2], T{});
            _data[3] = std::exchange(other._data[3], T{});
        }

        return *this;
    }

    template <details::Arithmetic T>
    template <details::Arithmetic U>
    requires details::QuaternionConvertible<U, T>
    constexpr Quaternion<T>::Quaternion(const Quaternion<U>& other) noexcept
        : _data{static_cast<T>(other.X()), static_cast<T>(other.Y()),
                static_cast<T>(other.Z()), static_cast<T>(other.W())}
    {
    }

    template <details::Arithmetic T>
    template <details::Arithmetic U>
    requires details::QuaternionConvertible<U, T>
    constexpr auto Quaternion<T>::operator=(const Quaternion<U>& other) noexcept -> Quaternion<T>&
    {
        _data[0] = static_cast<T>(other.X());
        _data[1] = static_cast<T>(other.Y());
        _data[2] = static_cast<T>(other.Z());
        _data[3] = static_cast<T>(other.W());

        return *this;
    }

    template <details::Arithmetic T>
    template <details::Arithmetic U>
    requires details::QuaternionConvertible<U, T>
    constexpr Quaternion<T>::Quaternion(Quaternion<U>&& other) noexcept
        : _data{std::move(static_cast<T>(other.X())), std::move(static_cast<T>(other.Y())),
                std::move(static_cast<T>(other.Z())), std::move(static_cast<T>(other.W()))}
    {
        other.Zero();
    }

    template <details::Arithmetic T>
    template <details::Arithmetic U>
    requires details::QuaternionConvertible<U, T>
    constexpr auto Quaternion<T>::operator=(Quaternion<U>&& other) noexcept -> Quaternion<T>&
    {
        _data[0] = std::move(static_cast<T>(other.X()));
        _data[1] = std::move(static_cast<T>(other.Y()));
        _data[2] = std::move(static_cast<T>(other.Z()));
        _data[3] = std::move(static_cast<T>(other.W()));

        other.Zero();

        return *this;
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::X() noexcept -> T&
    {
        return _data[0];
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::X() const noexcept -> const T&
    {
        return _data[0];
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::Y() noexcept -> T&
    {
        return _data[1];
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::Y() const noexcept -> const T&
    {
        return _data[1];
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::Z() noexcept -> T&
    {
        return _data[2];
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::Z() const noexcept -> const T&
    {
        return _data[2];
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::W() noexcept -> T&
    {
        return _data[3];
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::W() const noexcept -> const T&
    {
        return _data[3];
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::ScalarPart() const noexcept -> T
    {
        return _data[3];
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::VectorPart() const noexcept -> std::array<T, 3>
    {
        return { _data[0], _data[1], _data[2] };
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::Zero() noexcept -> void
    {
        _data[0] = T{};
        _data[1] = T{};
        _data[2] = T{};
        _data[3] = T{};
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::Norm() const noexcept -> T
    {
        return std::sqrt((_data[0] * _data[0]) + (_data[1] * _data[1]) + (_data[2] * _data[2]) + (_data[3] * _data[3]));
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::SquaredNorm() const noexcept -> T
    {
        return (_data[0] * _data[0]) + (_data[1] * _data[1]) + (_data[2] * _data[2]) + (_data[3] * _data[3]);
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::Normalize() noexcept -> void
    {
        const T n = Norm();

        _data[3] /= n;
        _data[0] /= n;
        _data[1] /= n;
        _data[2] /= n;
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::Normalized() const noexcept -> Quaternion<T>
    {
        const T n = Norm();

        return Quaternion{_data[0] / n, _data[1] / n, _data[2] / n, _data[3] / n};
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::IsNormalized() const noexcept -> bool
    {
        return std::abs(Quaternion<T>::SquaredNorm() - static_cast<T>(1)) <= EPSILON<T>;
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::Conjugate() noexcept -> void
    {
        _data[0] *= -1;
        _data[1] *= -1;
        _data[2] *= -1;
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::Conjugated() const noexcept -> Quaternion<T>
    {
        return Quaternion{-_data[0], -_data[1], -_data[2], _data[3]};
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
        return Quaternion<U>{static_cast<U>(_data[0]), static_cast<U>(_data[1]), static_cast<U>(_data[2]),
                             static_cast<U>(_data[3])};
    }

    template <details::Arithmetic T>
    template <details::Arithmetic U>
    requires details::QuaternionConvertible<U, T>
    constexpr auto Quaternion<T>::operator+=(const Quaternion<U>& other) noexcept -> Quaternion<T>&
    {
        _data[0] += static_cast<T>(other.X());
        _data[1] += static_cast<T>(other.Y());
        _data[2] += static_cast<T>(other.Z());
        _data[3] += static_cast<T>(other.W());

        return *this;
    }

    template <details::Arithmetic T>
    template <details::Arithmetic U>
    requires details::QuaternionConvertible<U, T>
    constexpr auto Quaternion<T>::operator-=(const Quaternion<U>& other) noexcept -> Quaternion<T>&
    {
        _data[0] -= static_cast<T>(other.X());
        _data[1] -= static_cast<T>(other.Y());
        _data[2] -= static_cast<T>(other.Z());
        _data[3] -= static_cast<T>(other.W());

        return *this;
    }

    template <details::Arithmetic T>
    constexpr auto operator<<(std::ostream& os, const Quaternion<T>& q) -> std::ostream&
    {
        return os << "Quaternion(" << q._data[0] << ", " << q._data[1] << ", " << q._data[2] << ", " << q._data[3] << ")";
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

        constexpr auto x = *it++;
        constexpr auto y = *it++;
        constexpr auto z = *it++;
        constexpr auto w = *it;

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
    [[nodiscard]] constexpr auto
    IsApproxEqual(const Quaternion<T>& lhs, const Quaternion<U>& rhs,
                  const bool epsilon = EPSILON<std::common_type_t<T, U>>) noexcept -> bool
    {
        return std::abs(lhs.W() - rhs.W()) <= epsilon && std::abs(lhs.X() - rhs.X()) <= epsilon &&
               std::abs(lhs.Y() - rhs.Y()) <= epsilon && std::abs(lhs.Z() - rhs.Z()) <= epsilon;
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
        const T x1 = _data[0];
        const T y1 = _data[1];
        const T z1 = _data[2];
        const T w1 = _data[3];

        const T x2 = static_cast<T>(other.X());
        const T y2 = static_cast<T>(other.Y());
        const T z2 = static_cast<T>(other.Z());
        const T w2 = static_cast<T>(other.W());

        _data[0] = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
        _data[1] = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
        _data[2] = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
        _data[3] = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;

        return *this;
    }

    template <details::Arithmetic T>
    template <details::Scalar<T> U>
    requires details::QuaternionConvertible<U, T>
    constexpr auto Quaternion<T>::operator*=(const U& scalar) noexcept -> Quaternion<T>&
    {
        _data[3] *= static_cast<T>(scalar);
        _data[0] *= static_cast<T>(scalar);
        _data[1] *= static_cast<T>(scalar);
        _data[2] *= static_cast<T>(scalar);

        return *this;
    }

    template <details::Arithmetic T>
    template <details::Scalar<T> U>
    requires details::QuaternionConvertible<U, T>
    constexpr auto Quaternion<T>::operator/=(const U& scalar) -> Quaternion<T>&
    {
        if (scalar == 0)
        {
            throw std::domain_error("One must not divide by 0");
        }

        _data[3] /= static_cast<T>(scalar);
        _data[0] /= static_cast<T>(scalar);
        _data[1] /= static_cast<T>(scalar);
        _data[2] /= static_cast<T>(scalar);

        return *this;
    }

    template <details::Arithmetic T, details::Scalar<T> U>
    requires details::QuaternionConvertible<U, T>
    constexpr auto operator*(const Quaternion<T>& lhs, const U& rhs) noexcept
        -> Quaternion<std::common_type_t<T, U>>
    {
        auto tmp = static_cast<Quaternion<std::common_type_t<T, U>>>(lhs);
        tmp *= rhs;

        return tmp;
    }

    template <details::Arithmetic T, details::Scalar<T> U>
    requires details::QuaternionConvertible<U, T>
    constexpr auto operator*(const U& lhs, const Quaternion<T>& rhs) noexcept
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
    constexpr auto operator*(const Quaternion<T>& lhs, const Quaternion<U>& rhs) noexcept
        -> Quaternion<std::common_type_t<T, U>>
    {
        auto tmp = static_cast<Quaternion<std::common_type_t<T, U>>>(lhs);
        tmp *= rhs;

        return tmp;
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::operator-() const noexcept -> Quaternion<T>
    {
        return Quaternion{-_data[0], -_data[1], -_data[2], -_data[3]};
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::operator[](std::size_t index) -> T&
    {
        switch (index)
        {
        case 0:
            return _data[0];
        case 1:
            return _data[1];
        case 2:
            return _data[2];
        case 3:
            return _data[3];
        default:
            throw std::out_of_range("Index out of bounds for Quaternion access.");
        }
    }

    template <details::Arithmetic T>
    constexpr auto Quaternion<T>::operator[](std::size_t index) const -> const T&
    {
        switch (index)
        {
        case 0:
            return _data[0];
        case 1:
            return _data[1];
        case 2:
            return _data[2];
        case 3:
            return _data[3];
        default:
            throw std::out_of_range("Index out of bounds for Quaternion access.");
        }
    }

    template <details::Arithmetic T>
    constexpr auto swap(Quaternion<T>& lhs, Quaternion<T>& rhs) noexcept -> void
    {
        using std::swap;

        swap(lhs.X(), rhs.X());
        swap(lhs.Y(), rhs.Y());
        swap(lhs.Z(), rhs.Z());
        swap(lhs.W(), rhs.W());
    }

    template <details::Arithmetic T, details::Arithmetic U>
    [[nodiscard]] constexpr auto Dot(const Quaternion<T>& lhs, const Quaternion<U>& rhs) noexcept
        -> std::common_type_t<T, U>
    {
        using V = std::common_type_t<T, U>;
        return static_cast<V>(lhs.W()) * static_cast<V>(rhs.W()) +
               static_cast<V>(lhs.X()) * static_cast<V>(rhs.X()) +
               static_cast<V>(lhs.Y()) * static_cast<V>(rhs.Y()) +
               static_cast<V>(lhs.Z()) * static_cast<V>(rhs.Z());
    }

    template <details::Arithmetic T>
    [[nodiscard]] constexpr auto Lerp(const Quaternion<T>& lhs, const Quaternion<T>& rhs,
                                      const T t) noexcept -> details::Result<Quaternion<T>>
    {
        if (t < 0 || t > 1)
        {
            return std::unexpected(error::QuaternionError::InvalidInterpolationTime);
        }

        return Quaternion<T>{(static_cast<T>(1) - t) * lhs.X() + t * rhs.X(),
                             (static_cast<T>(1) - t) * lhs.Y() + t * rhs.Y(),
                             (static_cast<T>(1) - t) * lhs.Z() + t * rhs.Z(),
                             (static_cast<T>(1) - t) * lhs.W() + t * rhs.W()}
            .Normalized();
    }

    template <details::Arithmetic T>
    [[nodiscard]] constexpr auto Slerp(const Quaternion<T>& q1, const Quaternion<T>& q2,
                                       const T t) noexcept -> details::Result<Quaternion<T>>
    {
        if (t < static_cast<T>(0) || t > static_cast<T>(1))
        {
            return std::unexpected(error::QuaternionError::InvalidInterpolationTime);
        }

        auto q1n = q1.Normalized();
        auto q2n = q2.Normalized();

        T dot = Dot(q1n, q2n);

        if (dot < static_cast<T>(0))
        {
            q2n = -q2n;
            dot = -dot;
        }

        constexpr auto threshold = 0.9995;

        if (dot > threshold)
        {
            return Lerp(q1n, q2n, t).value().Normalized();
        }

        const T theta = std::acos(dot);
        const T sin_theta = std::sin(theta);

        const T a = std::sin((1 - t) * theta) / sin_theta;
        const T b = std::sin(t * theta) / sin_theta;

        return (q1n * a + q2n * b).Normalized();
    }

    template <details::Arithmetic T>
    [[nodiscard]] constexpr auto AngleBetween(const Quaternion<T>& q1,
                                              const Quaternion<T>& q2) noexcept -> T
    {
        const T dot = Dot(q1.Normalized(), q2.Normalized());

        return std::acos(dot);
    }
} // namespace quaternionlib

#endif // QUATERNIONLIB_QUATERNION_HPP
