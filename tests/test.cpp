#include <gtest/gtest.h>
#include "Quaternion.hpp"

using quaternionlib::Quaternion;

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

