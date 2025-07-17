#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <Quaternion.hpp>

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
    REQUIRE(q.W() == 1.0);
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
        const quaternionlib::Quaternion<float> q1{1.0f, 2.0f, 3.0f, 4.0f};
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
        const quaternionlib::Quaternion<float> q1{1.0f, 2.0f, 3.0f, 4.0f};
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
        quaternionlib::Quaternion<float> q1{1.0f, 2.0f, 3.0f, 4.0f};
        quaternionlib::Quaternion<double> q2{std::move(q1)};

        REQUIRE(q2.X() == 1.0);
        REQUIRE(q2.Y() == 2.0);
        REQUIRE(q2.Z() == 3.0);
        REQUIRE(q2.W() == 4.0);

        REQUIRE(q1.X() == 0.0f);
        REQUIRE(q1.Y() == 0.0f);
        REQUIRE(q1.Z() == 0.0f);
        REQUIRE(q1.W() == 0.0f);
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
        quaternionlib::Quaternion<float> q1{1.0f, 2.0f, 3.0f, 4.0f};
        quaternionlib::Quaternion<double> q2;
        q2 = std::move(q1);

        REQUIRE(q2.X() == 1.0);
        REQUIRE(q2.Y() == 2.0);
        REQUIRE(q2.Z() == 3.0);
        REQUIRE(q2.W() == 4.0);

        REQUIRE(q1.X() == 0.0f);
        REQUIRE(q1.Y() == 0.0f);
        REQUIRE(q1.Z() == 0.0f);
        REQUIRE(q1.W() == 0.0f);
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

TEST_CASE("Conversion operator") // Necessary???
{
    constexpr quaternionlib::Quaternion<float> qf{1.0f, 2.0f, 3.0f, 4.0f};
    constexpr quaternionlib::Quaternion<double> qd{static_cast<quaternionlib::Quaternion<double>>(qf)};

    REQUIRE(qd.X() == Approx(1.0));
    REQUIRE(qd.Y() == Approx(2.0));
    REQUIRE(qd.Z() == Approx(3.0));
    REQUIRE(qd.W() == Approx(4.0));
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

    }
}

TEST_CASE("Substraction assignment") // TODO different types
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

    }
}

TEST_CASE("Addition")
{
    SECTION("Addition same types")
    {
        quaternionlib::Quaternion<double> q1{1, 2, 3, 4};
        const quaternionlib::Quaternion<double> q2{2, 5, 3, 1};
        const quaternionlib::Quaternion<double> result{3, 7, 6, 5};

        REQUIRE(q1 + q2 == result);
    }

    SECTION("Addition different types")
    {

    }
}

TEST_CASE("Substraction")
{
    SECTION("Substraction same types")
    {
        quaternionlib::Quaternion<double> q1{6, 7, 3, 4};
        const quaternionlib::Quaternion<double> q2{2, 5, 3, 1};
        const quaternionlib::Quaternion<double> result{4, 2, 0, 3};

        REQUIRE(q1 - q2 == result);
    }

    SECTION("Substraction different types")
    {

    }
}
