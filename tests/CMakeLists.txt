# // #include <gtest/gtest.h>
# // #include <stdexcept>
# // #include <cmath>
# // #include "Quaternion.hpp"

# // using quaternionlib::Quaternion;
# // using quaternionlib::Matrix3x3;
# // using quaternionlib::AngleAxis;

# // namespace
# // {
# //     static constexpr float PI = 3.14159265358979323846f;
# // }

# // TEST(QuaternionConstructors, DefaultConstructor)
# // {
# //     Quaternion<float> q;

# //     EXPECT_EQ(q.X(), 0.0f);
# //     EXPECT_EQ(q.Y(), 0.0f);
# //     EXPECT_EQ(q.Z(), 0.0f);
# //     EXPECT_EQ(q.W(), 1.0f);
# // }

# // TEST(QuaternionConstructors, FullInitialization)
# // {
# //     Quaternion<double> q(1.0, 2.0, 3.0, 4.0);

# //     EXPECT_EQ(q.X(), 1.0);
# //     EXPECT_EQ(q.Y(), 2.0);
# //     EXPECT_EQ(q.Z(), 3.0);
# //     EXPECT_EQ(q.W(), 4.0);
# // }

# // TEST(QuaternionConstructors, VectorPartOnly)
# // {
# //     Quaternion<float> q(1.0f, 2.0f, 3.0f); // w = 1

# //     EXPECT_EQ(q.X(), 1.0f);
# //     EXPECT_EQ(q.Y(), 2.0f);
# //     EXPECT_EQ(q.Z(), 3.0f);
# //     EXPECT_EQ(q.W(), 1.0f);
# // }

# // TEST(QuaternionConstructors, InitializerList3Elements)
# // {
# //     Quaternion<float> q{1.0f, 2.0f, 3.0f};

# //     EXPECT_EQ(q.X(), 1.0f);
# //     EXPECT_EQ(q.Y(), 2.0f);
# //     EXPECT_EQ(q.Z(), 3.0f);
# //     EXPECT_EQ(q.W(), 1.0f);
# // }

# // TEST(QuaternionConstructors, InitializerList4Elements)
# // {
# //     Quaternion<float> q{1.0f, 2.0f, 3.0f, 4.0f};

# //     EXPECT_EQ(q.X(), 1.0f);
# //     EXPECT_EQ(q.Y(), 2.0f);
# //     EXPECT_EQ(q.Z(), 3.0f);
# //     EXPECT_EQ(q.W(), 4.0f);
# // }

# // TEST(QuaternionConstructors, InitializerListTooFewElements)
# // {
# //     EXPECT_THROW(Quaternion<float>({1.0f, 2.0f}), std::invalid_argument);
# // }

# // TEST(QuaternionConstructors, InitializerListEmpty)
# // {
# //     EXPECT_THROW(Quaternion<float>({}), std::invalid_argument);
# // }

# // TEST(QuaternionConstructors, CopyConvertConstructor)
# // {
# //     Quaternion<double> qd(1.0, 2.0, 3.0, 4.0);
# //     Quaternion<float> qf(qd);

# //     EXPECT_FLOAT_EQ(qf.X(), 1.0f);
# //     EXPECT_FLOAT_EQ(qf.Y(), 2.0f);
# //     EXPECT_FLOAT_EQ(qf.Z(), 3.0f);
# //     EXPECT_FLOAT_EQ(qf.W(), 4.0f);
# // }

# // TEST(QuaternionConstructors, MoveConvertConstructor)
# // {
# //     Quaternion<double> temp(1.1, 2.2, 3.3, 4.4);
# //     Quaternion<float> q(std::move(temp));

# //     EXPECT_FLOAT_EQ(q.X(), 1.1f);
# //     EXPECT_FLOAT_EQ(q.Y(), 2.2f);
# //     EXPECT_FLOAT_EQ(q.Z(), 3.3f);
# //     EXPECT_FLOAT_EQ(q.W(), 4.4f);
# // }

# // TEST(QuaternionConstructors, CopyConstructor)
# // {
# //     Quaternion<float> q1(1.0f, 2.0f, 3.0f, 4.0f);
# //     Quaternion<float> q2(q1);

# //     EXPECT_EQ(q2.X(), 1.0f);
# //     EXPECT_EQ(q2.Y(), 2.0f);
# //     EXPECT_EQ(q2.Z(), 3.0f);
# //     EXPECT_EQ(q2.W(), 4.0f);
# // }

# // TEST(QuaternionConstructors, MoveConstructor)
# // {
# //     Quaternion<float> q1(1.0f, 2.0f, 3.0f, 4.0f);
# //     Quaternion<float> q2(std::move(q1));

# //     EXPECT_EQ(q2.X(), 1.0f);
# //     EXPECT_EQ(q2.Y(), 2.0f);
# //     EXPECT_EQ(q2.Z(), 3.0f);
# //     EXPECT_EQ(q2.W(), 4.0f);
# // }

# // TEST(QuaternionConstructors, FromAngleAxis)
# // {
# //     constexpr float angle = PI;
# //     constexpr float x = 0.0f;
# //     constexpr float y = 0.0f;
# //     constexpr float z = 1.0f;

# //     AngleAxis<float> aa(angle, x, y, z);
# //     Quaternion<float> q(aa);

# //     EXPECT_NEAR(q.X(), std::sin(angle / 2) * x, 1e-6f);
# //     EXPECT_NEAR(q.Y(), std::sin(angle / 2) * y, 1e-6f);
# //     EXPECT_NEAR(q.Z(), std::sin(angle / 2) * z, 1e-6f);
# //     EXPECT_NEAR(q.W(), std::cos(angle / 2), 1e-6f);
# // }

# // TEST(QuaternionConstructors, FromMatrix3x3)
# // {
# //     Matrix3x3<float> rotZ90{
# //         0.0f, -1.0f, 0.0f,
# //         1.0f,  0.0f, 0.0f,
# //         0.0f,  0.0f, 1.0f
# //     };

# //     Quaternion<float> q(rotZ90);

# //     constexpr float sqrt2over2 = 0.70710678118f;
# //     EXPECT_NEAR(q.W(), sqrt2over2, 1e-5f);
# //     EXPECT_NEAR(q.X(), 0.0f, 1e-5f);
# //     EXPECT_NEAR(q.Y(), 0.0f, 1e-5f);
# //     EXPECT_NEAR(q.Z(), sqrt2over2, 1e-5f);
# // }

# // TEST(QuaternionOperators, CopyAssignment)
# // {
# //     Quaternion<float> q1(1.0f, 2.0f, 3.0f, 4.0f);
# //     Quaternion<float> q2;
# //     q2 = q1;

# //     EXPECT_EQ(q2.X(), 1.0f);
# //     EXPECT_EQ(q2.Y(), 2.0f);
# //     EXPECT_EQ(q2.Z(), 3.0f);
# //     EXPECT_EQ(q2.W(), 4.0f);
# // }

# // TEST(QuaternionOperators, MoveAssignment)
# // {
# //     Quaternion<float> q1(1.0f, 2.0f, 3.0f, 4.0f);
# //     Quaternion<float> q2;
# //     q2 = std::move(q1);

# //     EXPECT_EQ(q2.X(), 1.0f);
# //     EXPECT_EQ(q2.Y(), 2.0f);
# //     EXPECT_EQ(q2.Z(), 3.0f);
# //     EXPECT_EQ(q2.W(), 4.0f);
# // }

# // TEST(QuaternionOperators, CopyConvertAssignment)
# // {
# //     Quaternion<double> qd(1.0, 2.0, 3.0, 4.0);
# //     Quaternion<float> qf;
# //     qf = qd;

# //     EXPECT_FLOAT_EQ(qf.X(), 1.0f);
# //     EXPECT_FLOAT_EQ(qf.Y(), 2.0f);
# //     EXPECT_FLOAT_EQ(qf.Z(), 3.0f);
# //     EXPECT_FLOAT_EQ(qf.W(), 4.0f);
# // }

# // TEST(QuaternionOperators, MoveConvertAssignment)
# // {
# //     Quaternion<double> qd(1.1, 2.2, 3.3, 4.4);
# //     Quaternion<float> qf;
# //     qf = std::move(qd);

# //     EXPECT_FLOAT_EQ(qf.X(), 1.1f);
# //     EXPECT_FLOAT_EQ(qf.Y(), 2.2f);
# //     EXPECT_FLOAT_EQ(qf.Z(), 3.3f);
# //     EXPECT_FLOAT_EQ(qf.W(), 4.4f);
# // }

# // TEST(QuaternionOperators, InitializerListAssignment)
# // {
# //     Quaternion<float> q;
# //     q = {1.0f, 2.0f, 3.0f, 4.0f};

# //     EXPECT_FLOAT_EQ(q.X(), 1.0f);
# //     EXPECT_FLOAT_EQ(q.Y(), 2.0f);
# //     EXPECT_FLOAT_EQ(q.Z(), 3.0f);
# //     EXPECT_FLOAT_EQ(q.W(), 4.0f);
# // }

# // TEST(QuaternionTest, SquaredNorm)
# // {
# //     Quaternion<double> q(1.0, 2.0, 3.0, 4.0);
# //     EXPECT_DOUBLE_EQ(q.SquaredNorm(), 1.0*1.0 + 2.0*2.0 + 3.0*3.0 + 4.0*4.0);
# // }

# // TEST(QuaternionTest, Normalize)
# // {
# //     Quaternion<double> q(1.0, 2.0, 3.0, 4.0);
# //     auto normalized = q.Normalize();
# //     EXPECT_NEAR(normalized.SquaredNorm(), 1.0, 1e-12);
# // }

# // TEST(QuaternionTest, IsNormalized)
# // {
# //     Quaternion<double> q(1.0, 2.0, 3.0, 4.0);
# //     EXPECT_FALSE(q.IsNormalized());
# //     auto norm_q = q.Normalize();
# //     EXPECT_TRUE(norm_q.IsNormalized());
# // }

# // TEST(QuaternionTest, Conjugate)
# // {
# //     Quaternion<double> q(1.0, -2.0, 3.0, -4.0);
# //     auto conj = q.Conjugate();
# //     EXPECT_EQ(conj.X(), -q.X());
# //     EXPECT_EQ(conj.Y(), -q.Y());
# //     EXPECT_EQ(conj.Z(), -q.Z());
# //     EXPECT_EQ(conj.W(), q.W());
# // }

# // TEST(QuaternionTest, EqualityOperators)
# // {
# //     Quaternion<double> a(1.0, 2.0, 3.0, 4.0);
# //     Quaternion<double> b(1.0, 2.0, 3.0, 4.0);
# //     Quaternion<double> c(1.0, 2.0, 3.0, 5.0);
# //     EXPECT_TRUE(a == b);
# //     EXPECT_FALSE(a != b);
# //     EXPECT_TRUE(a != c);
# // }

# // TEST(QuaternionTest, IsApproxEqual)
# // {
# //     Quaternion<double> a(1.0, 2.0, 3.0, 4.0);
# //     Quaternion<double> b(1.0 + 1e-7, 2.0 - 1e-7, 3.0 + 1e-7, 4.0 - 1e-7);
# //     EXPECT_TRUE(IsApproxEqual(a, b));
# // }

# // TEST(QuaternionTest, AdditionOperator)
# // {
# //     Quaternion<float> a(1.f, 2.f, 3.f, 4.f);
# //     Quaternion<double> b(0.5, -1.0, 0.0, 1.0);
# //     auto result = a + b;
# //     using ResultT = decltype(result)::value_type;
# //     EXPECT_NEAR(result.X(), ResultT(1.5), 1e-6);
# //     EXPECT_NEAR(result.Y(), ResultT(1.0), 1e-6);
# //     EXPECT_NEAR(result.Z(), ResultT(3.0), 1e-6);
# //     EXPECT_NEAR(result.W(), ResultT(5.0), 1e-6);
# // }

# // TEST(QuaternionTest, SubtractionOperator)
# // {
# //     Quaternion<float> a(1.f, 2.f, 3.f, 4.f);
# //     Quaternion<double> b(0.5, 1.0, 1.0, 0.0);
# //     auto result = a - b;
# //     using ResultT = decltype(result)::value_type;
# //     EXPECT_NEAR(result.X(), ResultT(0.5), 1e-6);
# //     EXPECT_NEAR(result.Y(), ResultT(1.0), 1e-6);
# //     EXPECT_NEAR(result.Z(), ResultT(2.0), 1e-6);
# //     EXPECT_NEAR(result.W(), ResultT(4.0), 1e-6);
# // }

# // TEST(QuaternionTest, CompoundAddition)
# // {
# //     Quaternion<double> a(1.0, 2.0, 3.0, 4.0);
# //     Quaternion<double> b(0.5, 0.5, 0.5, 0.5);
# //     a += b;
# //     EXPECT_DOUBLE_EQ(a.X(), 1.5);
# //     EXPECT_DOUBLE_EQ(a.Y(), 2.5);
# //     EXPECT_DOUBLE_EQ(a.Z(), 3.5);
# //     EXPECT_DOUBLE_EQ(a.W(), 4.5);
# // }

# // TEST(QuaternionTest, CompoundSubtraction)
# // {
# //     Quaternion<double> a(1.0, 2.0, 3.0, 4.0);
# //     Quaternion<double> b(0.5, 1.0, 1.5, 2.0);
# //     a -= b;
# //     EXPECT_DOUBLE_EQ(a.X(), 0.5);
# //     EXPECT_DOUBLE_EQ(a.Y(), 1.0);
# //     EXPECT_DOUBLE_EQ(a.Z(), 1.5);
# //     EXPECT_DOUBLE_EQ(a.W(), 2.0);
# // }

# // TEST(QuaternionTest, StreamOperator)
# // {
# //     Quaternion<double> q(1.0, 2.0, 3.0, 4.0);
# //     std::ostringstream oss;
# //     oss << q;
# //     EXPECT_TRUE(oss.str().find("Quaternion(1") != std::string::npos);
# // }
