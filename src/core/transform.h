
/*
    pbrt source code is Copyright(c) 1998-2016
                        Matt Pharr, Greg Humphreys, and Wenzel Jakob.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_CORE_TRANSFORM_H
#define PBRT_CORE_TRANSFORM_H

// core/transform.h*
#include "pbrt.h"
#include "stringprint.h"
#include "geometry.h"
#include "quaternion.h"

namespace pbrt
{

// Matrix4x4 声明
struct Matrix4x4
{
    // Matrix4x4 公有方法
    Matrix4x4()
    {
        // 默认是单位矩阵
        m[0][0] = m[1][1] = m[2][2] = m[3][3] = 1.f;
        m[0][1] = m[0][2] = m[0][3] = 0.f;
        m[1][0] = m[1][2] = m[1][3] = 0.f;
        m[2][0] = m[2][1] = m[2][3] = 0.f;
        m[3][0] = m[3][1] = m[3][2] = 0.f;
    }

    // 构造函数
    Matrix4x4(Float mat[4][4]);
    Matrix4x4(Float t00, Float t01, Float t02, Float t03,
              Float t10, Float t11, Float t12, Float t13,
              Float t20, Float t21, Float t22, Float t23,
              Float t30, Float t31, Float t32, Float t33);

    // 判断是否相等
    bool operator==(const Matrix4x4 &m2) const
    {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                if (m[i][j] != m2.m[i][j])
                    return false;
        return true;
    }

    // 判断是否不等
    bool operator!=(const Matrix4x4 &m2) const
    {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                if (m[i][j] != m2.m[i][j])
                    return true;
        return false;
    }

    // 友元函数，转置矩阵
    friend Matrix4x4 Transpose(const Matrix4x4 &);

    // 友元函数，逆矩阵
    friend Matrix4x4 Inverse(const Matrix4x4 &);

    // 打印到文件描述符
    void Print(FILE *f) const
    {
        fprintf(f, "[ ");
        for (int i = 0; i < 4; ++i)
        {
            fprintf(f, "  [ ");
            for (int j = 0; j < 4; ++j)
            {
                fprintf(f, "%f", m[i][j]);
                if (j != 3)
                    fprintf(f, ", ");
            }
            fprintf(f, " ]\n");
        }
        fprintf(f, " ] ");
    }

    // 矩阵乘法
    static Matrix4x4 Mul(const Matrix4x4 &m1, const Matrix4x4 &m2)
    {
        Matrix4x4 r;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                r.m[i][j] = m1.m[i][0] * m2.m[0][j] +
                            m1.m[i][1] * m2.m[1][j] +
                            m1.m[i][2] * m2.m[2][j] +
                            m1.m[i][3] * m2.m[3][j];
        return r;
    }

    // 输出流
    friend std::ostream &operator<<(std::ostream &os, const Matrix4x4 &m)
    {
        // clang-format off
        os << StringPrintf("[ [ %f, %f, %f, %f ] "
                           "[ %f, %f, %f, %f ] "
                           "[ %f, %f, %f, %f ] "
                           "[ %f, %f, %f, %f ] ]",
                           m.m[0][0], m.m[0][1], m.m[0][2], m.m[0][3],
                           m.m[1][0], m.m[1][1], m.m[1][2], m.m[1][3],
                           m.m[2][0], m.m[2][1], m.m[2][2], m.m[2][3],
                           m.m[3][0], m.m[3][1], m.m[3][2], m.m[3][3]);
        // clang-format on
        return os;
    }

    Float m[4][4];
};

// Transform 声明
class Transform
{
public:
    // Transform 公有方法
    Transform() {}

    // 构造函数
    Transform(const Float mat[4][4])
    {
        m = Matrix4x4(mat[0][0], mat[0][1], mat[0][2], mat[0][3],
                      mat[1][0], mat[1][1], mat[1][2], mat[1][3],
                      mat[2][0], mat[2][1], mat[2][2], mat[2][3],
                      mat[3][0], mat[3][1], mat[3][2], mat[3][3]);
        mInv = Inverse(m);
    }

    Transform(const Matrix4x4 &m) : m(m), mInv(Inverse(m)) {}
    Transform(const Matrix4x4 &m, const Matrix4x4 &mInv) : m(m), mInv(mInv) {}

    // 输出到文件描述符
    void Print(FILE *f) const;

    // 逆变换
    friend Transform Inverse(const Transform &t)
    {
        return Transform(t.mInv, t.m);
    }

    // 转置矩阵
    friend Transform Transpose(const Transform &t)
    {
        return Transform(Transpose(t.m), Transpose(t.mInv));
    }

    // 判断是否相等
    bool operator==(const Transform &t) const
    {
        return t.m == m && t.mInv == mInv;
    }

    // 判断是否不等
    bool operator!=(const Transform &t) const
    {
        return t.m != m || t.mInv != mInv;
    }

    // 判断是否小于
    bool operator<(const Transform &t2) const
    {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
            {
                if (m.m[i][j] < t2.m.m[i][j])
                    return true;
                if (m.m[i][j] > t2.m.m[i][j])
                    return false;
            }
        return false;
    }

    // 是否是单位矩阵
    bool IsIdentity() const
    {
        return (m.m[0][0] == 1.f && m.m[0][1] == 0.f && m.m[0][2] == 0.f && m.m[0][3] == 0.f &&
                m.m[1][0] == 0.f && m.m[1][1] == 1.f && m.m[1][2] == 0.f && m.m[1][3] == 0.f &&
                m.m[2][0] == 0.f && m.m[2][1] == 0.f && m.m[2][2] == 1.f && m.m[2][3] == 0.f &&
                m.m[3][0] == 0.f && m.m[3][1] == 0.f && m.m[3][2] == 0.f && m.m[3][3] == 1.f);
    }

    // 提取变换矩阵
    const Matrix4x4 &GetMatrix() const { return m; }

    // 提取逆变换矩阵
    const Matrix4x4 &GetInverseMatrix() const { return mInv; }

    bool HasScale() const
    {
        Float la2 = (*this)(Vector3f(1, 0, 0)).LengthSquared();
        Float lb2 = (*this)(Vector3f(0, 1, 0)).LengthSquared();
        Float lc2 = (*this)(Vector3f(0, 0, 1)).LengthSquared();
#define NOT_ONE(x) ((x) < .999f || (x) > 1.001f)
        return (NOT_ONE(la2) || NOT_ONE(lb2) || NOT_ONE(lc2));
#undef NOT_ONE
    }

    // ()运算符重载，将Tranform对象作为函数调用

    // 对Point3变换
    template <typename T>
    inline Point3<T> operator()(const Point3<T> &p) const;

    template <typename T>
    inline Point3<T> operator()(const Point3<T> &pt, Vector3<T> *absError) const;

    template <typename T>
    inline Point3<T> operator()(const Point3<T> &p, const Vector3<T> &pError, Vector3<T> *pTransError) const;

    // 对Vector3变换
    template <typename T>
    inline Vector3<T> operator()(const Vector3<T> &v) const;

    template <typename T>
    inline Vector3<T> operator()(const Vector3<T> &v, Vector3<T> *vTransError) const;

    template <typename T>
    inline Vector3<T> operator()(const Vector3<T> &v, const Vector3<T> &vError, Vector3<T> *vTransError) const;

    // 对Normal3变换
    template <typename T>
    inline Normal3<T> operator()(const Normal3<T> &) const;

    // 对Ray变换
    inline Ray operator()(const Ray &r) const;

    inline Ray operator()(const Ray &r, Vector3f *oError, Vector3f *dError) const;

    inline Ray operator()(const Ray &r, const Vector3f &oErrorIn, const Vector3f &dErrorIn,
                          Vector3f *oErrorOut, Vector3f *dErrorOut) const;

    // 对RayDifferential变换
    inline RayDifferential operator()(const RayDifferential &r) const;

    // 对SurfaceInteraction变换
    SurfaceInteraction operator()(const SurfaceInteraction &si) const;

    // 对Bounds3f变换
    Bounds3f operator()(const Bounds3f &b) const;

    // *运算符重载，合并两个Transform
    Transform operator*(const Transform &t2) const;

    // 判断该变换是否包含坐标手性变换
    bool SwapsHandedness() const;

    friend std::ostream &operator<<(std::ostream &os, const Transform &t) // 输出流
    {
        os << "t=" << t.m << ", inv=" << t.mInv;
        return os;
    }

private:
    // Transform 私有数据
    Matrix4x4 m, mInv; // 存储逆矩阵是为了方便计算，法线变换时需要使用逆矩阵
    friend class AnimatedTransform;
    friend struct Quaternion;
};

// 根据不同变换生成Transform

// 平移
Transform Translate(const Vector3f &delta);

// 缩放
Transform Scale(Float x, Float y, Float z);

// 绕X轴旋转
Transform RotateX(Float theta);

// 绕Y轴旋转
Transform RotateY(Float theta);

// 绕Z轴旋转
Transform RotateZ(Float theta);

// 绕指定轴旋转
Transform Rotate(Float theta, const Vector3f &axis);

// 指向
Transform LookAt(const Point3f &pos, const Point3f &look, const Vector3f &up);

// 正交
Transform Orthographic(Float znear, Float zfar);

// 透视
Transform Perspective(Float fov, Float znear, Float zfar);

bool SolveLinearSystem2x2(const Float A[2][2], const Float B[2], Float *x0, Float *x1);

// Transform 内联函数

// ()重载，Point3变换
template <typename T>
inline Point3<T> Transform::operator()(const Point3<T> &p) const
{
    T x = p.x, y = p.y, z = p.z;
    T xp = m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z + m.m[0][3];
    T yp = m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z + m.m[1][3];
    T zp = m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z + m.m[2][3];
    T wp = m.m[3][0] * x + m.m[3][1] * y + m.m[3][2] * z + m.m[3][3];
    CHECK_NE(wp, 0);
    if (wp == 1)
        return Point3<T>(xp, yp, zp);
    else
        return Point3<T>(xp, yp, zp) / wp;
}

template <typename T>
inline Point3<T> Transform::operator()(const Point3<T> &p, Vector3<T> *pError) const
{
    T x = p.x, y = p.y, z = p.z;
    // Compute transformed coordinates from point _pt_
    T xp = m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z + m.m[0][3];
    T yp = m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z + m.m[1][3];
    T zp = m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z + m.m[2][3];
    T wp = m.m[3][0] * x + m.m[3][1] * y + m.m[3][2] * z + m.m[3][3];

    // Compute absolute error for transformed point
    T xAbsSum = (std::abs(m.m[0][0] * x) + std::abs(m.m[0][1] * y) +
                 std::abs(m.m[0][2] * z) + std::abs(m.m[0][3]));
    T yAbsSum = (std::abs(m.m[1][0] * x) + std::abs(m.m[1][1] * y) +
                 std::abs(m.m[1][2] * z) + std::abs(m.m[1][3]));
    T zAbsSum = (std::abs(m.m[2][0] * x) + std::abs(m.m[2][1] * y) +
                 std::abs(m.m[2][2] * z) + std::abs(m.m[2][3]));
    *pError = gamma(3) * Vector3<T>(xAbsSum, yAbsSum, zAbsSum);
    CHECK_NE(wp, 0);
    if (wp == 1)
        return Point3<T>(xp, yp, zp);
    else
        return Point3<T>(xp, yp, zp) / wp;
}

template <typename T>
inline Point3<T> Transform::operator()(const Point3<T> &pt, const Vector3<T> &ptError, Vector3<T> *absError) const
{
    T x = pt.x, y = pt.y, z = pt.z;
    T xp = m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z + m.m[0][3];
    T yp = m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z + m.m[1][3];
    T zp = m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z + m.m[2][3];
    T wp = m.m[3][0] * x + m.m[3][1] * y + m.m[3][2] * z + m.m[3][3];
    absError->x =
        (gamma(3) + (T)1) *
            (std::abs(m.m[0][0]) * ptError.x + std::abs(m.m[0][1]) * ptError.y +
             std::abs(m.m[0][2]) * ptError.z) +
        gamma(3) * (std::abs(m.m[0][0] * x) + std::abs(m.m[0][1] * y) +
                    std::abs(m.m[0][2] * z) + std::abs(m.m[0][3]));
    absError->y =
        (gamma(3) + (T)1) *
            (std::abs(m.m[1][0]) * ptError.x + std::abs(m.m[1][1]) * ptError.y +
             std::abs(m.m[1][2]) * ptError.z) +
        gamma(3) * (std::abs(m.m[1][0] * x) + std::abs(m.m[1][1] * y) +
                    std::abs(m.m[1][2] * z) + std::abs(m.m[1][3]));
    absError->z =
        (gamma(3) + (T)1) *
            (std::abs(m.m[2][0]) * ptError.x + std::abs(m.m[2][1]) * ptError.y +
             std::abs(m.m[2][2]) * ptError.z) +
        gamma(3) * (std::abs(m.m[2][0] * x) + std::abs(m.m[2][1] * y) +
                    std::abs(m.m[2][2] * z) + std::abs(m.m[2][3]));
    CHECK_NE(wp, 0);
    if (wp == 1.)
        return Point3<T>(xp, yp, zp);
    else
        return Point3<T>(xp, yp, zp) / wp;
}

// ()重载，Vector3变换
template <typename T>
inline Vector3<T> Transform::operator()(const Vector3<T> &v) const
{
    T x = v.x, y = v.y, z = v.z;
    return Vector3<T>(m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z,
                      m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z,
                      m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z);
}

template <typename T>
inline Vector3<T> Transform::operator()(const Vector3<T> &v, Vector3<T> *absError) const
{
    T x = v.x, y = v.y, z = v.z;
    absError->x =
        gamma(3) * (std::abs(m.m[0][0] * v.x) + std::abs(m.m[0][1] * v.y) +
                    std::abs(m.m[0][2] * v.z));
    absError->y =
        gamma(3) * (std::abs(m.m[1][0] * v.x) + std::abs(m.m[1][1] * v.y) +
                    std::abs(m.m[1][2] * v.z));
    absError->z =
        gamma(3) * (std::abs(m.m[2][0] * v.x) + std::abs(m.m[2][1] * v.y) +
                    std::abs(m.m[2][2] * v.z));
    return Vector3<T>(m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z,
                      m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z,
                      m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z);
}

template <typename T>
inline Vector3<T> Transform::operator()(const Vector3<T> &v, const Vector3<T> &vError, Vector3<T> *absError) const
{
    T x = v.x, y = v.y, z = v.z;
    absError->x =
        (gamma(3) + (T)1) *
            (std::abs(m.m[0][0]) * vError.x + std::abs(m.m[0][1]) * vError.y +
             std::abs(m.m[0][2]) * vError.z) +
        gamma(3) * (std::abs(m.m[0][0] * v.x) + std::abs(m.m[0][1] * v.y) +
                    std::abs(m.m[0][2] * v.z));
    absError->y =
        (gamma(3) + (T)1) *
            (std::abs(m.m[1][0]) * vError.x + std::abs(m.m[1][1]) * vError.y +
             std::abs(m.m[1][2]) * vError.z) +
        gamma(3) * (std::abs(m.m[1][0] * v.x) + std::abs(m.m[1][1] * v.y) +
                    std::abs(m.m[1][2] * v.z));
    absError->z =
        (gamma(3) + (T)1) *
            (std::abs(m.m[2][0]) * vError.x + std::abs(m.m[2][1]) * vError.y +
             std::abs(m.m[2][2]) * vError.z) +
        gamma(3) * (std::abs(m.m[2][0] * v.x) + std::abs(m.m[2][1] * v.y) +
                    std::abs(m.m[2][2] * v.z));
    return Vector3<T>(m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z,
                      m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z,
                      m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z);
}

// ()重载，Normal3变换
template <typename T>
inline Normal3<T> Transform::operator()(const Normal3<T> &n) const
{
    T x = n.x, y = n.y, z = n.z;
    return Normal3<T>(mInv.m[0][0] * x + mInv.m[1][0] * y + mInv.m[2][0] * z,
                      mInv.m[0][1] * x + mInv.m[1][1] * y + mInv.m[2][1] * z,
                      mInv.m[0][2] * x + mInv.m[1][2] * y + mInv.m[2][2] * z);
}

// ()重载，Ray变换
inline Ray Transform::operator()(const Ray &r) const
{
    Vector3f oError;
    Point3f o = (*this)(r.o, &oError);
    Vector3f d = (*this)(r.d);
    // Offset ray origin to edge of error bounds and compute _tMax_
    // 处理浮点数舍入问题
    Float lengthSquared = d.LengthSquared();
    Float tMax = r.tMax;
    if (lengthSquared > 0)
    {
        Float dt = Dot(Abs(d), oError) / lengthSquared;
        o += d * dt;
        tMax -= dt;
    }
    return Ray(o, d, tMax, r.time, r.medium);
}

inline Ray Transform::operator()(const Ray &r, Vector3f *oError, Vector3f *dError) const
{
    Point3f o = (*this)(r.o, oError);
    Vector3f d = (*this)(r.d, dError);
    Float tMax = r.tMax;
    Float lengthSquared = d.LengthSquared();
    if (lengthSquared > 0)
    {
        Float dt = Dot(Abs(d), *oError) / lengthSquared;
        o += d * dt;
        // tMax -= dt;
    }
    return Ray(o, d, tMax, r.time, r.medium);
}

inline Ray Transform::operator()(const Ray &r, const Vector3f &oErrorIn, const Vector3f &dErrorIn,
                                 Vector3f *oErrorOut, Vector3f *dErrorOut) const
{
    Point3f o = (*this)(r.o, oErrorIn, oErrorOut);
    Vector3f d = (*this)(r.d, dErrorIn, dErrorOut);
    Float tMax = r.tMax;
    Float lengthSquared = d.LengthSquared();
    if (lengthSquared > 0)
    {
        Float dt = Dot(Abs(d), *oErrorOut) / lengthSquared;
        o += d * dt;
        // tMax -= dt;ß
    }
    return Ray(o, d, tMax, r.time, r.medium);
}

// ()重载，RayDifferential变换
inline RayDifferential Transform::operator()(const RayDifferential &r) const
{
    Ray tr = (*this)(Ray(r));
    RayDifferential ret(tr.o, tr.d, tr.tMax, tr.time, tr.medium);
    ret.hasDifferentials = r.hasDifferentials;
    ret.rxOrigin = (*this)(r.rxOrigin);
    ret.ryOrigin = (*this)(r.ryOrigin);
    ret.rxDirection = (*this)(r.rxDirection);
    ret.ryDirection = (*this)(r.ryDirection);
    return ret;
}

// AnimatedTransform 声明
class AnimatedTransform
{
public:
    // AnimatedTransform 公有方法
    AnimatedTransform(const Transform *startTransform, Float startTime,
                      const Transform *endTransform, Float endTime);

    // 分解，将Transform分解为平移，旋转和缩放
    static void Decompose(const Matrix4x4 &m, Vector3f *T, Quaternion *R, Matrix4x4 *S);

    // 插值，按给定时间，计算变换插值，由t带回结果
    void Interpolate(Float time, Transform *t) const;

    // 变换
    Ray operator()(const Ray &r) const;
    RayDifferential operator()(const RayDifferential &r) const;
    Point3f operator()(Float time, const Point3f &p) const;
    Vector3f operator()(Float time, const Vector3f &v) const;

    // 是否包含缩放
    bool HasScale() const
    {
        return startTransform->HasScale() || endTransform->HasScale();
    }
    Bounds3f MotionBounds(const Bounds3f &b) const;
    Bounds3f BoundPointMotion(const Point3f &p) const;

private:
    // AnimatedTransform 私有数据
    const Transform *startTransform, *endTransform; // 起点和终点
    const Float startTime, endTime; // 起点时间和终点时间
    const bool actuallyAnimated;
    Vector3f T[2]; // 起点和终点的平移变换
    Quaternion R[2]; // 起点和终点的旋转变换
    Matrix4x4 S[2]; // 起点和终点的缩放变换
    bool hasRotation; // 如果不包含旋转，可以直接对矩阵插值，而不需要转换为四元数
    // 导数项
    struct DerivativeTerm
    {
        DerivativeTerm() {}
        DerivativeTerm(Float c, Float x, Float y, Float z)
            : kc(c), kx(x), ky(y), kz(z) {}
        Float kc, kx, ky, kz;
        Float Eval(const Point3f &p) const
        {
            return kc + kx * p.x + ky * p.y + kz * p.z;
        }
    };
    DerivativeTerm c1[3], c2[3], c3[3], c4[3], c5[3];
};

} // namespace pbrt

#endif // PBRT_CORE_TRANSFORM_H
