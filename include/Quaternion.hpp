#ifndef QUATERNIONLIB_QUATERNION_HPP
#define QUATERNIONLIB_QUATERNION_HPP

#include <concepts>
#include <ostream>
#include <type_traits>
#include <cmath>
#include <utility>

namespace quaternionlib
{
    namespace concepts
    {
        template <typename T, typename S> // TODO: check if one of them is a scalar, add concept scalar
        static inline constexpr auto is_scalar_multiplicable_v =
            requires(T t, S s)
            {
                { t * s } -> std::same_as<T>;
                { s * t } -> std::same_as<T>;
            };

        template <typename T>
        static inline constexpr auto is_floating_point_v = std::is_floating_point<T>::value;

        template <typename T, typename S>
        static inline constexpr auto is_multipliable_v =
            requires(T t, S s)
            {
                { t * s } -> std::same_as<T>;
            };
        
        template <typename T> // TODO: necessary??? or to be repaired
        static inline constexpr auto is_quaternion_compatible_v =
            requires(T t, T s)
            {
                { t + s } -> std::same_as<T>;
                { t - s } -> std::same_as<T>;
                { t == s } -> std::same_as<bool>;
            };

        template <typename From_, typename To_>
        static inline constexpr auto is_convertible_v = std::is_convertible_v<From_, To_>;

        template <typename T>
        concept FloatingPoint = is_floating_point_v<T> and requires(T v)
        {
            { v + v } -> std::same_as<T>;
            { v - v } -> std::same_as<T>;
            { v * v } -> std::same_as<T>;
            { v / v} -> std::same_as<T>;
            { -v } -> std::same_as<T>;
        };
        
        template <typename T, typename S>
        concept ScalarMultiplicable = is_scalar_multiplicable_v<T, S>;

        template <typename From_, typename To_>
        concept QuaternionConvertible = is_convertible_v<From_, To_>;

        template <typename T, typename S>
        concept Multipliable = is_multipliable_v<T, S>;

        template <typename T>
        concept QuaternionCompatible = is_quaternion_compatible_v<T>;
    } // namespace concepts

    template <concepts::FloatingPoint T>
    struct AngleAxis // TODO optionaly convert quaternion to angle-axis
    {
        T angle{};
        T axis_x{}, axis_y{}, axis_z{};

        constexpr AngleAxis() noexcept = default;

        explicit constexpr AngleAxis(T angle, T x, T y, T z) noexcept
            : angle(angle), axis_x(x), axis_y(y), axis_z(z) {}
    };

    template <concepts::FloatingPoint T>
    struct Matrix3x3 // TODO change to std::array if it is more efficient, or Maciek's lib
    {                // TODO optionaly convert quaternion to matrix
        T m[3][3]{};

        constexpr Matrix3x3() noexcept = default;

        constexpr Matrix3x3(
            T m00, T m01, T m02,
            T m10, T m11, T m12,
            T m20, T m21, T m22) noexcept
            : m{{m00, m01, m02},
                {m10, m11, m12},
                {m20, m21, m22}}
        {}

        constexpr T* operator[](std::size_t row) noexcept { return m[row]; }
        constexpr const T* operator[](std::size_t row) const noexcept { return m[row]; }
    };
    
    template <concepts::FloatingPoint T>
    class Quaternion final
    {
    public:
        using value_type = T;

        constexpr Quaternion() noexcept = default;

        constexpr ~Quaternion() noexcept = default;

        explicit constexpr Quaternion(const T& x, const T& y, const T& z, const T& w) noexcept; // const& ?

        constexpr Quaternion(const T& x, const T& y, const T& z) noexcept; // add explicit or not???

        // hicpp-explicit-conversions
        constexpr Quaternion(std::initializer_list<T> values);
        constexpr auto operator=(std::initializer_list<T> values) -> Quaternion<T>&;

        constexpr Quaternion(const Quaternion<T>& other) noexcept;
        constexpr auto operator=(const Quaternion<T>& other) noexcept -> Quaternion<T>&;

        constexpr Quaternion(Quaternion<T>&& other) noexcept;
        constexpr auto operator=(Quaternion<T>&& other) noexcept -> Quaternion<T>&;

        template <concepts::FloatingPoint U>
            requires concepts::QuaternionConvertible<U, T>
        explicit constexpr Quaternion(const Quaternion<U>& other) noexcept;

        template <concepts::FloatingPoint U>
            requires concepts::QuaternionConvertible<U, T>
        constexpr auto operator=(const Quaternion<U>& other) noexcept -> Quaternion<T>&;

        template <concepts::FloatingPoint U>
            requires concepts::QuaternionConvertible<U, T>
        explicit constexpr Quaternion(Quaternion<U>&& other) noexcept;

        template <concepts::FloatingPoint U>
            requires concepts::QuaternionConvertible<U, T>
        constexpr auto operator=(Quaternion<U>&& other) noexcept -> Quaternion<T>&;

        explicit constexpr Quaternion(const AngleAxis<T>& angleAxis) noexcept; // TODO operator overload

        explicit constexpr Quaternion(const Matrix3x3<T>& matrix) noexcept; // TODO operator overload

        [[nodiscard]] constexpr auto X() const noexcept -> T;
        [[nodiscard]] constexpr auto Y() const noexcept -> T;
        [[nodiscard]] constexpr auto Z() const noexcept -> T;
        [[nodiscard]] constexpr auto W() const noexcept -> T;

        constexpr auto Zero() noexcept -> void;

        [[nodiscard]] constexpr auto Norm() const noexcept -> T;
        [[nodiscard]] constexpr auto SquaredNorm() const noexcept -> T;
        [[nodiscard]] constexpr auto Normalize() const noexcept -> Quaternion<T>;
        [[nodiscard]] constexpr auto IsNormalized(T epsilon = T(1e-6)) const noexcept -> bool;
        [[nodiscard]] constexpr auto Conjugate() const noexcept -> Quaternion<T>;

        template <concepts::FloatingPoint U>
        friend constexpr auto operator<<(std::ostream&, const Quaternion<U>&) -> std::ostream&;

        template <concepts::FloatingPoint U>
            requires concepts::QuaternionConvertible<T, U>
        explicit constexpr operator Quaternion<U>() const noexcept;

        template <concepts::FloatingPoint U>
            requires concepts::QuaternionConvertible<U, T>
        constexpr auto operator+=(const Quaternion<U>& other) noexcept -> Quaternion<T>&;

        template <concepts::FloatingPoint U>
            requires concepts::QuaternionConvertible<U, T>   
        constexpr auto operator-=(const Quaternion<U>& other) noexcept -> Quaternion<T>&;

    private:
        T _x{}, _y{}, _z{}, _w{1}; // cos moze byc nie tak
    
    protected:
        constexpr T& XRef() noexcept { return _x; }
    };

    template <concepts::FloatingPoint T>
    constexpr Quaternion<T>::Quaternion(const T& x, const T& y, const T& z, const T& w) noexcept
        : _x(x), _y(y), _z(z), _w(w) {}

    template <concepts::FloatingPoint T>
    constexpr Quaternion<T>::Quaternion(const T& x, const T& y, const T& z) noexcept
        : _x(x), _y(y), _z(z), _w(static_cast<T>(1)) {}

    template <concepts::FloatingPoint T>
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

    template <concepts::FloatingPoint T>
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

    template <concepts::FloatingPoint T>
    constexpr Quaternion<T>::Quaternion(const Quaternion<T>& other) noexcept
        : _x(other._x),
          _y(other._y),
          _z(other._z),
          _w(other._w) {}
    
    template <concepts::FloatingPoint T>
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

    template <concepts::FloatingPoint T>
    constexpr Quaternion<T>::Quaternion(Quaternion<T>&& other) noexcept
        : _x{std::exchange(other._x, T{})},
          _y{std::exchange(other._y, T{})},
          _z{std::exchange(other._z, T{})},
          _w{std::exchange(other._w, T{})} {}

    template <concepts::FloatingPoint T>
    constexpr auto Quaternion<T>::operator=(Quaternion<T>&& other) noexcept -> Quaternion<T>& // TODO here changes as well
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

    template <concepts::FloatingPoint T>
    template <concepts::FloatingPoint U>
        requires concepts::QuaternionConvertible<U, T>
    constexpr Quaternion<T>::Quaternion(const Quaternion<U>& other) noexcept
        : _x(static_cast<T>(other.X())),
          _y(static_cast<T>(other.Y())),
          _z(static_cast<T>(other.Z())),
          _w(static_cast<T>(other.W())) {}

    template <concepts::FloatingPoint T>
    template <concepts::FloatingPoint U>
        requires concepts::QuaternionConvertible<U, T>
    constexpr auto Quaternion<T>::operator=(const Quaternion<U>& other) noexcept -> Quaternion<T>&
    {
        _x = static_cast<T>(other.X());
        _y = static_cast<T>(other.Y());
        _z = static_cast<T>(other.Z());
        _w = static_cast<T>(other.W());

        return *this;
    }

    template <concepts::FloatingPoint T>
    template <concepts::FloatingPoint U>
        requires concepts::QuaternionConvertible<U, T>
    constexpr Quaternion<T>::Quaternion(Quaternion<U>&& other) noexcept
        : _x{static_cast<T>(std::move(other.X()))},
          _y{static_cast<T>(std::move(other.Y()))},
          _z{static_cast<T>(std::move(other.Z()))},
          _w{static_cast<T>(std::move(other.W()))}
    {
        other.Zero();
    }

    template <concepts::FloatingPoint T>
    template <concepts::FloatingPoint U>
        requires concepts::QuaternionConvertible<U, T>
    constexpr auto Quaternion<T>::operator=(Quaternion<U>&& other) noexcept -> Quaternion<T>&
    {
        _x = static_cast<T>(std::move(other.X()));
        _y = static_cast<T>(std::move(other.Y()));
        _z = static_cast<T>(std::move(other.Z()));
        _w = static_cast<T>(std::move(other.W()));

        other.Zero();
        return *this;
    }

    template<concepts::FloatingPoint T>
    constexpr Quaternion<T>::Quaternion(const AngleAxis<T>& aa) noexcept
        : _x{std::sin(aa.angle / 2) * aa.axis_x},
          _y{std::sin(aa.angle / 2) * aa.axis_y},
          _z{std::sin(aa.angle / 2) * aa.axis_z},
          _w{std::cos(aa.angle / 2)} {}

    template <concepts::FloatingPoint T>
    constexpr Quaternion<T>::Quaternion(const Matrix3x3<T>& m) noexcept
    {
        const T trace = m[0][0] + m[1][1] + m[2][2];

        if (trace > T(0))
        {
            const T s = std::sqrt(trace + T(1)) * T(2); // s = 4 * w
            _w = T(0.25) * s;
            _x = (m[2][1] - m[1][2]) / s;
            _y = (m[0][2] - m[2][0]) / s;
            _z = (m[1][0] - m[0][1]) / s;
        }
        else if (m[0][0] > m[1][1] && m[0][0] > m[2][2])
        {
            const T s = std::sqrt(T(1) + m[0][0] - m[1][1] - m[2][2]) * T(2); // s = 4 * x
            _w = (m[2][1] - m[1][2]) / s;
            _x = T(0.25) * s;
            _y = (m[0][1] + m[1][0]) / s;
            _z = (m[0][2] + m[2][0]) / s;
        }
        else if (m[1][1] > m[2][2])
        {
            const T s = std::sqrt(T(1) + m[1][1] - m[0][0] - m[2][2]) * T(2); // s = 4 * y
            _w = (m[0][2] - m[2][0]) / s;
            _x = (m[0][1] + m[1][0]) / s;
            _y = T(0.25) * s;
            _z = (m[1][2] + m[2][1]) / s;
        }
        else
        {
            const T s = std::sqrt(T(1) + m[2][2] - m[0][0] - m[1][1]) * T(2); // s = 4 * z
            _w = (m[1][0] - m[0][1]) / s;
            _x = (m[0][2] + m[2][0]) / s;
            _y = (m[1][2] + m[2][1]) / s;
            _z = T(0.25) * s;
        }
    }

    template <concepts::FloatingPoint T>
    constexpr auto Quaternion<T>::X() const noexcept -> T
    {
        return _x;
    }

    template <concepts::FloatingPoint T>
    constexpr auto Quaternion<T>::Y() const noexcept -> T
    {
        return _y;
    }

    template <concepts::FloatingPoint T>
    constexpr auto Quaternion<T>::Z() const noexcept -> T
    {
        return _z;
    }

    template <concepts::FloatingPoint T>
    constexpr auto Quaternion<T>::W() const noexcept -> T
    {
        return _w;
    }

    template <concepts::FloatingPoint T>
    constexpr auto Quaternion<T>::Zero() noexcept -> void
    {
        _x = T{};
        _y = T{};
        _z = T{};
        _w = T{};
    }

    template <concepts::FloatingPoint T>
    constexpr auto Quaternion<T>::Norm() const noexcept -> T
    {
        return std::sqrt(_x * _x + _y * _y + _z * _z + _w * _w);
    }

    template <concepts::FloatingPoint T>
    constexpr auto Quaternion<T>::SquaredNorm() const noexcept -> T
    {
        return _x * _x + _y * _y + _z * _z + _w * _w;
    }

    template <concepts::FloatingPoint T>
    constexpr auto Quaternion<T>::Normalize() const noexcept -> Quaternion<T>
    {
        const T n = Norm();
        return Quaternion{_x / n, _y / n, _z / n, _w / n};
    }

    template <concepts::FloatingPoint T>
    constexpr auto Quaternion<T>::IsNormalized(T epsilon) const noexcept -> bool
    {
        // change to epsilon = std::numeric_limits<T>
        return std::abs(Quaternion<T>::SquaredNorm() - T(1)) <= epsilon;
    }

    template <concepts::FloatingPoint T>
    constexpr auto Quaternion<T>::Conjugate() const noexcept -> Quaternion<T>
    {
        return Quaternion{-_x, -_y, -_z, _w};
    }

    template <concepts::FloatingPoint T>
    template <concepts::FloatingPoint U>
        requires concepts::QuaternionConvertible<T, U>
    constexpr Quaternion<T>::operator Quaternion<U>() const noexcept
    {
        return Quaternion<U>
        {
            static_cast<U>(_x),
            static_cast<U>(_y),
            static_cast<U>(_z),
            static_cast<U>(_w)
        };
    }

    template <concepts::FloatingPoint T>
    template <concepts::FloatingPoint U>
        requires concepts::QuaternionConvertible<U, T>
    constexpr auto Quaternion<T>::operator+=(const Quaternion<U>& other) noexcept -> Quaternion<T>&
    {
        _x += static_cast<T>(other.X());
        _y += static_cast<T>(other.Y());
        _z += static_cast<T>(other.Z());
        _w += static_cast<T>(other.W());

        return *this;
    }

    template <concepts::FloatingPoint T>
    template <concepts::FloatingPoint U>
        requires concepts::QuaternionConvertible<U, T>
    constexpr auto Quaternion<T>::operator-=(const Quaternion<U>& other) noexcept -> Quaternion<T>&
    {
        _x -= static_cast<T>(other.X());
        _y -= static_cast<T>(other.Y());
        _z -= static_cast<T>(other.Z());
        _w -= static_cast<T>(other.W());

        return *this;
    }

    template <concepts::FloatingPoint T>
    constexpr auto operator<<(std::ostream& os, const Quaternion<T>& q) -> std::ostream&
    {
        return os << "Quaternion(" << q._x << ", " << q._y << ", "
                  << q._z << ", " << q._w << ")";
    }

    template <concepts::FloatingPoint T, concepts::FloatingPoint U>
        requires concepts::is_convertible_v<T, U>
    [[nodiscard]] constexpr inline auto operator==(const Quaternion<T>& lhs, const Quaternion<U>& rhs) noexcept -> bool
    {
        return lhs.W() == rhs.W() &&
               lhs.X() == rhs.X() &&
               lhs.Y() == rhs.Y() &&
               lhs.Z() == rhs.Z();
    }

    template <concepts::FloatingPoint T, concepts::FloatingPoint U>
        requires concepts::is_convertible_v<T, U>
    [[nodiscard]] constexpr inline auto operator!=(const Quaternion<T>& lhs, const Quaternion<U>& rhs) noexcept -> bool
    {
        return !(lhs == rhs);
    }

    template <concepts::FloatingPoint T> // TODO probably to trash or change accordingly to == operator
    [[nodiscard]] constexpr auto IsApproxEqual(const Quaternion<T>& a,
                                const Quaternion<T>& b,
                                T epsilon = T(1e-6)) noexcept -> bool
    {
        return std::abs(a.W() - b.W()) <= epsilon &&
               std::abs(a.X() - b.X()) <= epsilon &&
               std::abs(a.Y() - b.Y()) <= epsilon &&
               std::abs(a.Z() - b.Z()) <= epsilon;
    }

    template <concepts::FloatingPoint T, concepts::FloatingPoint U>
    constexpr auto operator+(const Quaternion<T>& lhs, const Quaternion<U>& rhs) -> Quaternion<std::common_type_t<T, U>>
    {
        auto tmp = static_cast<Quaternion<std::common_type_t<T, U>>>(lhs);
        tmp += rhs;

        return tmp;
    }

    template <concepts::FloatingPoint T, concepts::FloatingPoint U>
    constexpr auto operator-(const Quaternion<T>& lhs, const Quaternion<U>& rhs) -> Quaternion<std::common_type_t<T, U>>
    {
        auto tmp = static_cast<Quaternion<std::common_type_t<T, U>>>(lhs);
        tmp -= rhs;

        return tmp;
    }
} // namespace quaternionlib

#endif // QUATERNIONLIB_QUATERNION_HPP
