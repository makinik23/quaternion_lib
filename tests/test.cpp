#include <catch2/catch_test_macros.hpp>
#include <Quaternion.hpp>

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

TEST_CASE("Creating an object - 3 args constructor")
{
    constexpr quaternionlib::Quaternion<double> q(1.0, 2.0, 3.0);

    REQUIRE(q.X() == 1.0);
    REQUIRE(q.Y() == 2.0);
    REQUIRE(q.Z() == 3.0);
    REQUIRE(q.W() == 1.0);
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

TEST_CASE("Creating an object - angle axis")
{
    quaternionlib::AngleAxis<double> angleAxis(PI / 2, 1.0, 2.0, 4.0);
    quaternionlib::Quaternion<double> q{angleAxis};
}
