#include <gtest/gtest.h>
#include <stdexcept>
#include <cmath>
#include "Quaternion.hpp"

using quaternionlib::Quaternion;
using quaternionlib::Matrix3x3;
using quaternionlib::AngleAxis;

namespace
{
    static constexpr float PI = 3.14159265358979323846f;
}

TEST(QuaternionConstructors, DefaultConstructor)
{
    Quaternion<float> q;

    EXPECT_EQ(q.X(), 0.0f);
    EXPECT_EQ(q.Y(), 0.0f);
    EXPECT_EQ(q.Z(), 0.0f);
    EXPECT_EQ(q.W(), 1.0f);
}

TEST(QuaternionConstructors, FullInitialization)
{
    Quaternion<double> q(1.0, 2.0, 3.0, 4.0);

    EXPECT_EQ(q.X(), 1.0);
    EXPECT_EQ(q.Y(), 2.0);
    EXPECT_EQ(q.Z(), 3.0);
    EXPECT_EQ(q.W(), 4.0);
}

TEST(QuaternionConstructors, VectorPartOnly)
{
    Quaternion<float> q(1.0f, 2.0f, 3.0f); // w = 1

    EXPECT_EQ(q.X(), 1.0f);
    EXPECT_EQ(q.Y(), 2.0f);
    EXPECT_EQ(q.Z(), 3.0f);
    EXPECT_EQ(q.W(), 1.0f);
}

TEST(QuaternionConstructors, InitializerList3Elements)
{
    Quaternion<float> q{1.0f, 2.0f, 3.0f};

    EXPECT_EQ(q.X(), 1.0f);
    EXPECT_EQ(q.Y(), 2.0f);
    EXPECT_EQ(q.Z(), 3.0f);
    EXPECT_EQ(q.W(), 1.0f);
}

TEST(QuaternionConstructors, InitializerList4Elements)
{
    Quaternion<float> q{1.0f, 2.0f, 3.0f, 4.0f};

    EXPECT_EQ(q.X(), 1.0f);
    EXPECT_EQ(q.Y(), 2.0f);
    EXPECT_EQ(q.Z(), 3.0f);
    EXPECT_EQ(q.W(), 4.0f);
}

TEST(QuaternionConstructors, InitializerListTooFewElements)
{
    EXPECT_THROW(Quaternion<float>({1.0f, 2.0f}), std::invalid_argument);
}

TEST(QuaternionConstructors, InitializerListEmpty)
{
    EXPECT_THROW(Quaternion<float>({}), std::invalid_argument);
}

TEST(QuaternionConstructors, CopyConvertConstructor)
{
    Quaternion<double> qd(1.0, 2.0, 3.0, 4.0);
    Quaternion<float> qf(qd);

    EXPECT_FLOAT_EQ(qf.X(), 1.0f);
    EXPECT_FLOAT_EQ(qf.Y(), 2.0f);
    EXPECT_FLOAT_EQ(qf.Z(), 3.0f);
    EXPECT_FLOAT_EQ(qf.W(), 4.0f);
}

TEST(QuaternionConstructors, MoveConvertConstructor)
{
    Quaternion<double> temp(1.1, 2.2, 3.3, 4.4);
    Quaternion<float> q(std::move(temp));

    EXPECT_FLOAT_EQ(q.X(), 1.1f);
    EXPECT_FLOAT_EQ(q.Y(), 2.2f);
    EXPECT_FLOAT_EQ(q.Z(), 3.3f);
    EXPECT_FLOAT_EQ(q.W(), 4.4f);
}

TEST(QuaternionConstructors, CopyConstructor)
{
    Quaternion<float> q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<float> q2(q1);

    EXPECT_EQ(q2.X(), 1.0f);
    EXPECT_EQ(q2.Y(), 2.0f);
    EXPECT_EQ(q2.Z(), 3.0f);
    EXPECT_EQ(q2.W(), 4.0f);
}

TEST(QuaternionConstructors, MoveConstructor)
{
    Quaternion<float> q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<float> q2(std::move(q1));

    EXPECT_EQ(q2.X(), 1.0f);
    EXPECT_EQ(q2.Y(), 2.0f);
    EXPECT_EQ(q2.Z(), 3.0f);
    EXPECT_EQ(q2.W(), 4.0f);
}

TEST(QuaternionConstructors, FromAngleAxis)
{
    constexpr float angle = PI;
    constexpr float x = 0.0f;
    constexpr float y = 0.0f;
    constexpr float z = 1.0f;

    AngleAxis<float> aa(angle, x, y, z);
    Quaternion<float> q(aa);

    EXPECT_NEAR(q.X(), std::sin(angle / 2) * x, 1e-6f);
    EXPECT_NEAR(q.Y(), std::sin(angle / 2) * y, 1e-6f);
    EXPECT_NEAR(q.Z(), std::sin(angle / 2) * z, 1e-6f);
    EXPECT_NEAR(q.W(), std::cos(angle / 2), 1e-6f);
}

TEST(QuaternionConstructors, FromMatrix3x3)
{
    Matrix3x3<float> rotZ90{
        0.0f, -1.0f, 0.0f,
        1.0f,  0.0f, 0.0f,
        0.0f,  0.0f, 1.0f
    };

    Quaternion<float> q(rotZ90);

    constexpr float sqrt2over2 = 0.70710678118f;
    EXPECT_NEAR(q.W(), sqrt2over2, 1e-5f);
    EXPECT_NEAR(q.X(), 0.0f, 1e-5f);
    EXPECT_NEAR(q.Y(), 0.0f, 1e-5f);
    EXPECT_NEAR(q.Z(), sqrt2over2, 1e-5f);
}

TEST(QuaternionOperators, CopyAssignment)
{
    Quaternion<float> q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<float> q2;
    q2 = q1;

    EXPECT_EQ(q2.X(), 1.0f);
    EXPECT_EQ(q2.Y(), 2.0f);
    EXPECT_EQ(q2.Z(), 3.0f);
    EXPECT_EQ(q2.W(), 4.0f);
}

TEST(QuaternionOperators, MoveAssignment)
{
    Quaternion<float> q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion<float> q2;
    q2 = std::move(q1);

    EXPECT_EQ(q2.X(), 1.0f);
    EXPECT_EQ(q2.Y(), 2.0f);
    EXPECT_EQ(q2.Z(), 3.0f);
    EXPECT_EQ(q2.W(), 4.0f);
}

TEST(QuaternionOperators, CopyConvertAssignment)
{
    Quaternion<double> qd(1.0, 2.0, 3.0, 4.0);
    Quaternion<float> qf;
    qf = qd;

    EXPECT_FLOAT_EQ(qf.X(), 1.0f);
    EXPECT_FLOAT_EQ(qf.Y(), 2.0f);
    EXPECT_FLOAT_EQ(qf.Z(), 3.0f);
    EXPECT_FLOAT_EQ(qf.W(), 4.0f);
}

TEST(QuaternionOperators, MoveConvertAssignment)
{
    Quaternion<double> qd(1.1, 2.2, 3.3, 4.4);
    Quaternion<float> qf;
    qf = std::move(qd);

    EXPECT_FLOAT_EQ(qf.X(), 1.1f);
    EXPECT_FLOAT_EQ(qf.Y(), 2.2f);
    EXPECT_FLOAT_EQ(qf.Z(), 3.3f);
    EXPECT_FLOAT_EQ(qf.W(), 4.4f);
}

TEST(QuaternionOperators, InitializerListAssignment)
{
    Quaternion<float> q;
    q = {1.0f, 2.0f, 3.0f, 4.0f};

    EXPECT_FLOAT_EQ(q.X(), 1.0f);
    EXPECT_FLOAT_EQ(q.Y(), 2.0f);
    EXPECT_FLOAT_EQ(q.Z(), 3.0f);
    EXPECT_FLOAT_EQ(q.W(), 4.0f);
}

TEST(QuaternionMethods, Norm) // Test test
{
    Quaternion<float> qf;
    qf = {1.0f, 2.0f, 3.0f, 4.0f};
    EXPECT_FLOAT_EQ(qf.X(), 1.0f);
    EXPECT_FLOAT_EQ(qf.Y(), 2.0f);
    EXPECT_FLOAT_EQ(qf.Z(), 3.0f);
    EXPECT_FLOAT_EQ(qf.W(), 4.0f);
}