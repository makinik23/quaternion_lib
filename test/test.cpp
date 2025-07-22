#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <Quaternion.hpp>
#include <iostream>

using Catch::Approx;

namespace
{
    static constexpr auto PI = 3.14159265;
}

TEST_CASE("Creating an object - default constructor")
{
    constexpr quaternionlib::Quaternion<double> q;

    REQUIRE(q.X() == 0.0);
    REQUIRE(q.Y() == 0.0);
    REQUIRE(q.Z() == 0.0);
    REQUIRE(q.W() == 0.0);
}

TEST_CASE("Creating an object - 4 args constructor")
{
    constexpr quaternionlib::Quaternion<double> q(1.0, 2.0, 3.0, 4.0);

    REQUIRE(q.X() == 1.0);
    REQUIRE(q.Y() == 2.0);
    REQUIRE(q.Z() == 3.0);
    REQUIRE(q.W() == 4.0);
}

TEST_CASE("Creating an object - initializer list")
{
    SECTION("Valid use")
    {
        constexpr auto initializeWithValidList = []()
        {
            return quaternionlib::Quaternion<double>{1.0, 2.0, 3.0, 4.0};
        };
    
        REQUIRE_NOTHROW(initializeWithValidList());
    
        const auto q = initializeWithValidList();
    
        REQUIRE(q.X() == 1.0);
        REQUIRE(q.Y() == 2.0);
        REQUIRE(q.Z() == 3.0);
        REQUIRE(q.W() == 4.0);
    }
    
    SECTION("Invalid use")
    {
        constexpr auto initializeWithInvalidList = []()
        {
            return quaternionlib::Quaternion<double>{1.0, 2.0, 3.0, 4.0, 5.0};
        };

        REQUIRE_THROWS_AS(initializeWithInvalidList(), std::invalid_argument);
    }
}

TEST_CASE("Operator= with std::initializer_list") // maybe add to one test "initializer_list"
{
    SECTION("Copy initializer list")
    {
        quaternionlib::Quaternion<double> q;
        const std::initializer_list<double> list{1.0, 2.0, 3.0, 4.0};
        q = list;

        REQUIRE(q.X() == 1.0);
        REQUIRE(q.Y() == 2.0);
        REQUIRE(q.Z() == 3.0);
        REQUIRE(q.W() == 4.0);    
    }
}

TEST_CASE("Copying quaternion")
{
    SECTION("Same type constructor")
    {
        const quaternionlib::Quaternion<double> q1{1.0, 2.0, 3.0, 4.0};
        const quaternionlib::Quaternion<double> q2{q1};

        REQUIRE(q2 == q1);
    }

    SECTION("Different type constructor")
    {
        const quaternionlib::Quaternion<int> q1{1, 2, 3, 4};
        const quaternionlib::Quaternion<double> q2{q1};

        REQUIRE(q2 == q1);
    }

    SECTION("Same type assignment operator")
    {
        const quaternionlib::Quaternion<double> q1{1.0, 2.0, 3.0, 4.0};
        quaternionlib::Quaternion<double> q2;
        q2 = q1;

        REQUIRE(q2 == q1);
    }

    SECTION("Different type assignment operator")
    {
        const quaternionlib::Quaternion<int> q1{1, 2, 3, 4};
        quaternionlib::Quaternion<double> q2;
        q2 = q1;

        REQUIRE(q2 == q1);
    }
}

TEST_CASE("Moving quaternion")
{
    SECTION("Same type constructor")
    {
        quaternionlib::Quaternion<double> q1{1.0, 2.0, 3.0, 4.0};
        quaternionlib::Quaternion<double> q2{std::move(q1)};

        REQUIRE(q2.X() == 1.0);
        REQUIRE(q2.Y() == 2.0);
        REQUIRE(q2.Z() == 3.0);
        REQUIRE(q2.W() == 4.0);

        REQUIRE(q1.X() == 0.0);
        REQUIRE(q1.Y() == 0.0);
        REQUIRE(q1.Z() == 0.0);
        REQUIRE(q1.W() == 0.0);
    }

    SECTION("Different type constructor")
    {
        quaternionlib::Quaternion<int> q1{1, 2, 3, 4};
        quaternionlib::Quaternion<double> q2{std::move(q1)};

        REQUIRE(q2.X() == 1.0);
        REQUIRE(q2.Y() == 2.0);
        REQUIRE(q2.Z() == 3.0);
        REQUIRE(q2.W() == 4.0);

        REQUIRE(q1.X() == 0);
        REQUIRE(q1.Y() == 0);
        REQUIRE(q1.Z() == 0);
        REQUIRE(q1.W() == 0);
    }

    SECTION("Same type assignment operator")
    {
        quaternionlib::Quaternion<double> q1{1.0, 2.0, 3.0, 4.0};
        quaternionlib::Quaternion<double> q2;
        q2 = std::move(q1);

        REQUIRE(q2.X() == 1.0);
        REQUIRE(q2.Y() == 2.0);
        REQUIRE(q2.Z() == 3.0);
        REQUIRE(q2.W() == 4.0);

        REQUIRE(q1.X() == 0.0);
        REQUIRE(q1.Y() == 0.0);
        REQUIRE(q1.Z() == 0.0);
        REQUIRE(q1.W() == 0.0);
    }

    SECTION("Different type assignment operator")
    {
        quaternionlib::Quaternion<int> q1{1, 2, 3, 4};
        quaternionlib::Quaternion<double> q2;
        q2 = std::move(q1);

        REQUIRE(q2.X() == 1.0);
        REQUIRE(q2.Y() == 2.0);
        REQUIRE(q2.Z() == 3.0);
        REQUIRE(q2.W() == 4.0);

        REQUIRE(q1.X() == 0);
        REQUIRE(q1.Y() == 0);
        REQUIRE(q1.Z() == 0);
        REQUIRE(q1.W() == 0);
    }
}

TEST_CASE("Creating an object - angle axis") // TODO
{
   
}

TEST_CASE("Creating an object - rotational matrix") // TODO
{

}

TEST_CASE("Normalizing")
{
    SECTION("Norm and Squared norm")
    {
        constexpr quaternionlib::Quaternion<double> q{1.0, 2.0, 3.0, 4.0};

        REQUIRE(q.SquaredNorm() == Approx(1.0 * 1.0 + 2.0 * 2.0 + 3.0 * 3.0 + 4.0 * 4.0));
        REQUIRE(q.Norm() == Approx(std::sqrt(1.0 * 1.0 + 2.0 * 2.0 + 3.0 * 3.0 + 4.0 * 4.0)));
    }

    SECTION("normalize and normalized")
    {
        const quaternionlib::Quaternion<double> q{2.0, 0.0, 0.0, 0.0};
        const auto norm = q.Norm();
        const auto normalized = q.Normalize();

        REQUIRE(normalized.X() == Approx(1.0));
        REQUIRE(normalized.Y() == Approx(0.0));
        REQUIRE(normalized.Z() == Approx(0.0));
        REQUIRE(normalized.W() == Approx(0.0));
        REQUIRE(q.Norm() == norm);
    }

    SECTION("IsNormalized method")
    {
        const quaternionlib::Quaternion<double> q1{1.0 / std::sqrt(2), 0.0, 0.0, 1.0 / std::sqrt(2)};
        const quaternionlib::Quaternion<double> q2{1.0, 2.0, 3.0, 4.0};

        REQUIRE(q1.IsNormalized());
        REQUIRE_FALSE(q2.IsNormalized());
    }
}

TEST_CASE("Conversion operator")
{
    constexpr quaternionlib::Quaternion<int> q{1, 2, 3, 4};
    constexpr quaternionlib::Quaternion<double> qd{static_cast<quaternionlib::Quaternion<double>>(q)};

    REQUIRE(qd.X() == 1.0);
    REQUIRE(qd.Y() == 2.0);
    REQUIRE(qd.Z() == 3.0);
    REQUIRE(qd.W() == 4.0);
}

TEST_CASE("Addition assignment") // TODO different types
{
    SECTION("Addition assignment same types")
    {
        quaternionlib::Quaternion<double> q1{1.0, 2.0, 3.0, 4.0};
        const quaternionlib::Quaternion<double> q2{0.5, -1.0, 1.0, 0.0};
        const quaternionlib::Quaternion<double> result{1.5, 1.0, 4.0, 4.0};

        q1 += q2;

        REQUIRE(q1 == result);
    }

    SECTION("Addition assignment different types")
    {
        quaternionlib::Quaternion<int> q1{4, 3, 5, 7};
        const quaternionlib::Quaternion<double> q2{5.2, 23.6, 2.8, 5.4};
        const quaternionlib::Quaternion<int> result{9, 26, 7, 12};

        q1 += q2;

        REQUIRE(q1 == result);
    }
}

TEST_CASE("Substraction assignment")
{
    SECTION("Substraction assignment same types")
    {
        quaternionlib::Quaternion<double> q1{1.0, 2.0, 3.0, 4.0};
        const quaternionlib::Quaternion<double> q2{0.5, -1.0, 1.0, 0.0};
        const quaternionlib::Quaternion<double> result{0.5, 3.0, 2.0, 4.0};

        q1 -= q2;

        REQUIRE(q1 == result);
    }

    SECTION("Substraction assignment different types")
    {
        quaternionlib::Quaternion<int> q1{4, 3, 5, 7};
        const quaternionlib::Quaternion<double> q2{2.1, 1.5, 4.6, 1.5};
        const quaternionlib::Quaternion<int> result{2, 2, 1, 6};

        q1 -= q2;

        REQUIRE(q1 == result);
    }
}

TEST_CASE("Addition")
{
    SECTION("Addition same types")
    {
        constexpr quaternionlib::Quaternion<int> q1{1, 2, 3, 4};
        constexpr quaternionlib::Quaternion<int> q2{2, 5, 3, 1};
        constexpr quaternionlib::Quaternion<int> result{3, 7, 6, 5};

        REQUIRE(q1 + q2 == result);
    }

    SECTION("Addition different types")
    {
        constexpr quaternionlib::Quaternion<int> q1{1, 3, 4, 2};
        constexpr quaternionlib::Quaternion<double> q2{2.3, 5.4, 3.8, 1.6};
        constexpr quaternionlib::Quaternion<double> result{3.3, 8.4, 7.8, 3.6};

        REQUIRE(q1 + q2 == result);
    }
}

TEST_CASE("Substraction")
{
    SECTION("Substraction same types")
    {
        constexpr quaternionlib::Quaternion<int> q1{1, 7, 3, 4};
        constexpr quaternionlib::Quaternion<int> q2{2, 5, 3, 1};
        constexpr quaternionlib::Quaternion<int> result{-1, 2, 0, 3};

        REQUIRE(q1 - q2 == result);
    }

    SECTION("Substraction different types")
    {
        constexpr quaternionlib::Quaternion<int> q1{6, 7, 4, 4};
        constexpr quaternionlib::Quaternion<double> q2{2.5, 5.5, 3.5, 1.5};
        constexpr quaternionlib::Quaternion<double> result{3.5, 1.5, 0.5, 2.5};

        REQUIRE(q1 - q2 == result);
    }
}

TEST_CASE("Scalar multiplication assignment")
{
    SECTION("Scalar multilication assignment same types")
    {
        quaternionlib::Quaternion<int> q{6, 7, 3, 4};
        const int scalar {2};
        const quaternionlib::Quaternion<int> result{12, 14, 6, 8};

        q *= scalar;

        REQUIRE(q == result);
    }

    SECTION("Scalar multiplication assignment different types")
    {
        quaternionlib::Quaternion<int> q{6, 8, 1, 3};
        const double scalar {2.5};
        quaternionlib::Quaternion<double> result{12, 16, 2, 6};

        q *= scalar;

        REQUIRE(q == result);
    }
}

TEST_CASE("Scalar division assignment")  // TODO divide by zero
{
    SECTION("Scalar division assignment same types")
    {
        quaternionlib::Quaternion<int> q{6, 8, 1, 3};
        const int scalar {2};
        const quaternionlib::Quaternion<int> result{3, 4, 0, 1};

        q /= scalar;

        REQUIRE(q == result);
    }

    SECTION("Scalar division assignment different types")
    {
        quaternionlib::Quaternion<double> q{6, 8, 1, 3};
        const int scalar {2};
        const quaternionlib::Quaternion<double> result{3, 4, 0.5, 1.5};

        q /= scalar;

        REQUIRE(q == result);
    }

    SECTION("Scalar dividing by zero")
    {

    } 
}

TEST_CASE("Scalar multiplication")
{
    SECTION("Scalar multiplication same types")
    {
        constexpr quaternionlib::Quaternion<int> q{6, 8, 1, 3};
        constexpr int scalar {2};
        constexpr quaternionlib::Quaternion<int> result{12, 16, 2, 6};

        REQUIRE(q * scalar == result);
        REQUIRE(scalar * q == result);
    }

    SECTION("Scalar multiplication different types")
    {
        constexpr quaternionlib::Quaternion<int> q{7, 5, 1, 3};
        constexpr double scalar {1.5};
        constexpr quaternionlib::Quaternion<double> result{10.5, 7.5, 1.5, 4.5};

        REQUIRE(q * scalar == result);
        REQUIRE(scalar * q == result);
    }
}

TEST_CASE("Scalar division")
{
    SECTION("Scalar division same types")
    {
        constexpr quaternionlib::Quaternion<int> q{6, 8, 4, 2};
        constexpr int scalar {2};
        constexpr quaternionlib::Quaternion<int> result{3, 4, 2, 1};

        REQUIRE(q / scalar == result);
    }

    SECTION("Scalar multiplication different types")
    {
        constexpr quaternionlib::Quaternion<int> q{8, 5, 1, 3};
        constexpr double scalar {0.8};
        constexpr quaternionlib::Quaternion<double> result{10, 6.25, 1.25, 3.75};

        REQUIRE(q / scalar == result);
    }
}
