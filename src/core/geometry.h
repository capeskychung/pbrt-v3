
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

#ifndef PBRT_CORE_GEOMETRY_H
#define PBRT_CORE_GEOMETRY_H

// core/geometry.h*
#include "pbrt.h"
#include "stringprint.h"
#include <iterator>

namespace pbrt
{

// 检查溢出，除零错误等
template <typename T>
inline bool isNaN(const T x)
{
    return std::isnan(x);
}
template <>
inline bool isNaN(const int x)
{
    return false;
}

// Vector 声明
template <typename T>
class Vector2
{
public:
    // Vector2 公有方法
    Vector2() { x = y = 0; }
    Vector2(T xx, T yy) : x(xx), y(yy) { DCHECK(!HasNaNs()); }
    bool HasNaNs() const { return isNaN(x) || isNaN(y); }
    explicit Vector2(const Point2<T> &p); // 必须显示调用构造函数
    explicit Vector2(const Point3<T> &p); // 必须显示调用构造函数
#ifndef NDEBUG
    // The default versions of these are fine for release builds; for debug
    // we define them so that we can add the Assert checks.
    Vector2(const Vector2<T> &v)
    {
        DCHECK(!v.HasNaNs());
        x = v.x;
        y = v.y;
    }
    Vector2<T> &operator=(const Vector2<T> &v)
    {
        DCHECK(!v.HasNaNs());
        x = v.x;
        y = v.y;
        return *this;
    }
#endif // !NDEBUG

    Vector2<T> operator+(const Vector2<T> &v) const // 加
    {
        DCHECK(!v.HasNaNs());
        return Vector2(x + v.x, y + v.y);
    }

    Vector2<T> &operator+=(const Vector2<T> &v) // 加且赋值
    {
        DCHECK(!v.HasNaNs());
        x += v.x;
        y += v.y;
        return *this;
    }
    Vector2<T> operator-(const Vector2<T> &v) const // 减
    {
        DCHECK(!v.HasNaNs());
        return Vector2(x - v.x, y - v.y);
    }

    Vector2<T> &operator-=(const Vector2<T> &v) // 减且赋值
    {
        DCHECK(!v.HasNaNs());
        x -= v.x;
        y -= v.y;
        return *this;
    }
    bool operator==(const Vector2<T> &v) const { return x == v.x && y == v.y; } // 比较是否相等
    bool operator!=(const Vector2<T> &v) const { return x != v.x || y != v.y; } // 比较是否不相等
    template <typename U>
    Vector2<T> operator*(U f) const // 乘以
    {
        return Vector2<T>(f * x, f * y);
    }

    template <typename U>
    Vector2<T> &operator*=(U f) // 乘以且赋值
    {
        DCHECK(!isNaN(f));
        x *= f;
        y *= f;
        return *this;
    }
    template <typename U>
    Vector2<T> operator/(U f) const // 除以
    {
        CHECK_NE(f, 0);
        Float inv = (Float)1 / f;
        return Vector2<T>(x * inv, y * inv);
    }

    template <typename U>
    Vector2<T> &operator/=(U f) // 除以且赋值
    {
        CHECK_NE(f, 0);
        Float inv = (Float)1 / f;
        x *= inv;
        y *= inv;
        return *this;
    }
    Vector2<T> operator-() const { return Vector2<T>(-x, -y); } // 负号
    T operator[](int i) const
    {
        DCHECK(i >= 0 && i <= 1);
        if (i == 0)
            return x;
        return y;
    }

    T &operator[](int i) // 下标
    {
        DCHECK(i >= 0 && i <= 1);
        if (i == 0)
            return x;
        return y;
    }
    Float LengthSquared() const { return x * x + y * y; }       // 长度的平方
    Float Length() const { return std::sqrt(LengthSquared()); } // 长度

    // Vector2 公有数据
    T x, y;
};

template <typename T>
inline std::ostream &operator<<(std::ostream &os, const Vector2<T> &v) // 全局运算符重载，输出Vector2
{
    os << "[ " << v.x << ", " << v.y << " ]";
    return os;
}

template <>
inline std::ostream &operator<<(std::ostream &os, const Vector2<Float> &v)
{
    os << StringPrintf("[ %f, %f ]", v.x, v.y);
    return os;
}

template <typename T>
class Vector3
{
public:
    // Vector3 公有方法
    T operator[](int i) const
    {
        DCHECK(i >= 0 && i <= 2);
        if (i == 0)
            return x;
        if (i == 1)
            return y;
        return z;
    }
    T &operator[](int i)
    {
        DCHECK(i >= 0 && i <= 2);
        if (i == 0)
            return x;
        if (i == 1)
            return y;
        return z;
    }
    Vector3() { x = y = z = 0; }
    Vector3(T x, T y, T z) : x(x), y(y), z(z) { DCHECK(!HasNaNs()); }
    bool HasNaNs() const { return isNaN(x) || isNaN(y) || isNaN(z); }
    explicit Vector3(const Point3<T> &p);
    explicit Vector3(const Normal3<T> &n);
#ifndef NDEBUG
    // The default versions of these are fine for release builds; for debug
    // we define them so that we can add the Assert checks.
    Vector3(const Vector3<T> &v) // 拷贝构造函数
    {
        DCHECK(!v.HasNaNs());
        x = v.x;
        y = v.y;
        z = v.z;
    }

    Vector3<T> &operator=(const Vector3<T> &v) // 赋值函数
    {
        DCHECK(!v.HasNaNs());
        x = v.x;
        y = v.y;
        z = v.z;
        return *this;
    }
#endif                                              // !NDEBUG
    Vector3<T> operator+(const Vector3<T> &v) const // 加
    {
        DCHECK(!v.HasNaNs());
        return Vector3(x + v.x, y + v.y, z + v.z);
    }
    Vector3<T> &operator+=(const Vector3<T> &v) // 加且赋值
    {
        DCHECK(!v.HasNaNs());
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }
    Vector3<T> operator-(const Vector3<T> &v) const // 减
    {
        DCHECK(!v.HasNaNs());
        return Vector3(x - v.x, y - v.y, z - v.z);
    }
    Vector3<T> &operator-=(const Vector3<T> &v) // 减且赋值
    {
        DCHECK(!v.HasNaNs());
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }
    bool operator==(const Vector3<T> &v) const // 比较是否相等
    {
        return x == v.x && y == v.y && z == v.z;
    }
    bool operator!=(const Vector3<T> &v) const // 比较是否不相等
    {
        return x != v.x || y != v.y || z != v.z;
    }
    template <typename U>
    Vector3<T> operator*(U s) const // 乘以
    {
        return Vector3<T>(s * x, s * y, s * z);
    }
    template <typename U>
    Vector3<T> &operator*=(U s) // 乘以且赋值
    {
        DCHECK(!isNaN(s));
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }
    template <typename U>
    Vector3<T> operator/(U f) const // 除以
    {
        CHECK_NE(f, 0);
        Float inv = (Float)1 / f;
        return Vector3<T>(x * inv, y * inv, z * inv);
    }

    template <typename U>
    Vector3<T> &operator/=(U f) // 除以且赋值
    {
        CHECK_NE(f, 0);
        Float inv = (Float)1 / f;
        x *= inv;
        y *= inv;
        z *= inv;
        return *this;
    }
    Vector3<T> operator-() const { return Vector3<T>(-x, -y, -z); } // 负号
    Float LengthSquared() const { return x * x + y * y + z * z; }   // 长度的平方
    Float Length() const { return std::sqrt(LengthSquared()); }     // 长度

    // Vector3 公有数据
    T x, y, z;
};

template <typename T>
inline std::ostream &operator<<(std::ostream &os, const Vector3<T> &v) // 全局运算符重载，输出Vector3
{
    os << "[ " << v.x << ", " << v.y << ", " << v.z << " ]";
    return os;
}

template <>
inline std::ostream &operator<<(std::ostream &os, const Vector3<Float> &v)
{
    os << StringPrintf("[ %f, %f, %f ]", v.x, v.y, v.z);
    return os;
}

// 常用特化类型
typedef Vector2<Float> Vector2f;
typedef Vector2<int> Vector2i;
typedef Vector3<Float> Vector3f;
typedef Vector3<int> Vector3i;

// Point 声明
template <typename T>
class Point2
{
public:
    // Point2 公有方法
    explicit Point2(const Point3<T> &p) : x(p.x), y(p.y) { DCHECK(!HasNaNs()); }
    Point2() { x = y = 0; }
    Point2(T xx, T yy) : x(xx), y(yy) { DCHECK(!HasNaNs()); }

    template <typename U>
    explicit Point2(const Point2<U> &p) // 类型转换构造函数
    {
        x = (T)p.x;
        y = (T)p.y;
        DCHECK(!HasNaNs());
    }

    template <typename U>
    explicit Point2(const Vector2<U> &p) // 类型转换构造函数
    {
        x = (T)p.x;
        y = (T)p.y;
        DCHECK(!HasNaNs());
    }

    template <typename U>
    explicit operator Vector2<U>() const // 类型转换重载，可以转换为Vector2
    {
        return Vector2<U>(x, y);
    }

#ifndef NDEBUG
    Point2(const Point2<T> &p) // 拷贝构造函数
    {
        DCHECK(!p.HasNaNs());
        x = p.x;
        y = p.y;
    }

    Point2<T> &operator=(const Point2<T> &p) // 赋值函数
    {
        DCHECK(!p.HasNaNs());
        x = p.x;
        y = p.y;
        return *this;
    }
#endif                                             // !NDEBUG
    Point2<T> operator+(const Vector2<T> &v) const // 加Vector2，得到新的Point2
    {
        DCHECK(!v.HasNaNs());
        return Point2<T>(x + v.x, y + v.y);
    }

    Point2<T> &operator+=(const Vector2<T> &v) // 加Vector2且赋值，相当于移动该点
    {
        DCHECK(!v.HasNaNs());
        x += v.x;
        y += v.y;
        return *this;
    }

    Point2<T> operator-(const Vector2<T> &v) const // 减Vector2，得到新的Point2
    {
        DCHECK(!v.HasNaNs());
        return Point2<T>(x - v.x, y - v.y);
    }

    Point2<T> &operator-=(const Vector2<T> &v) // 减Vector2且赋值，相当于移动该点
    {
        DCHECK(!v.HasNaNs());
        x -= v.x;
        y -= v.y;
        return *this;
    }

    Point2<T> operator-() const { return Point2<T>(-x, -y); } // 负号

    Vector2<T> operator-(const Point2<T> &p) const // 减Point2，得到一个Vector2
    {
        DCHECK(!p.HasNaNs());
        return Vector2<T>(x - p.x, y - p.y);
    }

    Point2<T> &operator+=(const Point2<T> &p) // 加Point2且赋值
    {
        DCHECK(!p.HasNaNs());
        x += p.x;
        y += p.y;
        return *this;
    }

    Point2<T> operator+(const Point2<T> &p) const // 加Point2
    {
        DCHECK(!p.HasNaNs());
        return Point2<T>(x + p.x, y + p.y);
    }

    template <typename U>
    Point2<T> operator*(U f) const // 乘以
    {
        return Point2<T>(f * x, f * y);
    }

    template <typename U>
    Point2<T> &operator*=(U f) // 乘以并赋值
    {
        x *= f;
        y *= f;
        return *this;
    }

    template <typename U>
    Point2<T> operator/(U f) const // 除以
    {
        CHECK_NE(f, 0);
        Float inv = (Float)1 / f;
        return Point2<T>(inv * x, inv * y);
    }

    template <typename U>
    Point2<T> &operator/=(U f) // 除以并赋值
    {
        CHECK_NE(f, 0);
        Float inv = (Float)1 / f;
        x *= inv;
        y *= inv;
        return *this;
    }

    T operator[](int i) const // 下标，返回左值
    {
        DCHECK(i >= 0 && i <= 1);
        if (i == 0)
            return x;
        return y;
    }

    T &operator[](int i) // 下标，返回右值
    {
        DCHECK(i >= 0 && i <= 1);
        if (i == 0)
            return x;
        return y;
    }
    bool operator==(const Point2<T> &p) const { return x == p.x && y == p.y; } // 比较是否相等
    bool operator!=(const Point2<T> &p) const { return x != p.x || y != p.y; } // 比较是否不相等
    bool HasNaNs() const { return isNaN(x) || isNaN(y); }

    // Point2 公有数据
    T x, y;
};

template <typename T>
inline std::ostream &operator<<(std::ostream &os, const Point2<T> &v)
{
    os << "[ " << v.x << ", " << v.y << " ]";
    return os;
}

template <>
inline std::ostream &operator<<(std::ostream &os, const Point2<Float> &v)
{
    os << StringPrintf("[ %f, %f ]", v.x, v.y);
    return os;
}

template <typename T>
class Point3
{
public:
    // Point3 公有方法
    Point3() { x = y = z = 0; }
    Point3(T x, T y, T z) : x(x), y(y), z(z) { DCHECK(!HasNaNs()); }
    template <typename U>
    explicit Point3(const Point3<U> &p)
        : x((T)p.x), y((T)p.y), z((T)p.z)
    {
        DCHECK(!HasNaNs());
    }
    template <typename U>
    explicit operator Vector3<U>() const // 类型转换重载，可以转换为Vector3
    {
        return Vector3<U>(x, y, z);
    }
#ifndef NDEBUG
    Point3(const Point3<T> &p) // 拷贝构造函数
    {
        DCHECK(!p.HasNaNs());
        x = p.x;
        y = p.y;
        z = p.z;
    }

    Point3<T> &operator=(const Point3<T> &p) // 赋值函数
    {
        DCHECK(!p.HasNaNs());
        x = p.x;
        y = p.y;
        z = p.z;
        return *this;
    }
#endif                                             // !NDEBUG
    Point3<T> operator+(const Vector3<T> &v) const // 加Vector3，得到新的Point3
    {
        DCHECK(!v.HasNaNs());
        return Point3<T>(x + v.x, y + v.y, z + v.z);
    }
    Point3<T> &operator+=(const Vector3<T> &v) // 加Vector3且赋值，相当于移动该点
    {
        DCHECK(!v.HasNaNs());
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }
    Point3<T> operator-(const Vector3<T> &v) const // 减Vector3，得到新的Point3
    {
        DCHECK(!v.HasNaNs());
        return Point3<T>(x - v.x, y - v.y, z - v.z);
    }
    Point3<T> &operator-=(const Vector3<T> &v) // 减Vector3且赋值，相当于移动该点
    {
        DCHECK(!v.HasNaNs());
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    Point3<T> operator-() const { return Point3<T>(-x, -y, -z); } // 负号

    Vector3<T> operator-(const Point3<T> &p) const // 减Point3，得到一个Vector3
    {
        DCHECK(!p.HasNaNs());
        return Vector3<T>(x - p.x, y - p.y, z - p.z);
    }

    Point3<T> &operator+=(const Point3<T> &p) // 加Point3且赋值
    {
        DCHECK(!p.HasNaNs());
        x += p.x;
        y += p.y;
        z += p.z;
        return *this;
    }

    Point3<T> operator+(const Point3<T> &p) const // 加Point3
    {
        DCHECK(!p.HasNaNs());
        return Point3<T>(x + p.x, y + p.y, z + p.z);
    }

    template <typename U>
    Point3<T> operator*(U f) const // 乘以
    {
        return Point3<T>(f * x, f * y, f * z);
    }

    template <typename U>
    Point3<T> &operator*=(U f) // 乘以并赋值
    {
        x *= f;
        y *= f;
        z *= f;
        return *this;
    }
    template <typename U>
    Point3<T> operator/(U f) const // 除以
    {
        CHECK_NE(f, 0);
        Float inv = (Float)1 / f;
        return Point3<T>(inv * x, inv * y, inv * z);
    }
    template <typename U>
    Point3<T> &operator/=(U f) // 除以并赋值
    {
        CHECK_NE(f, 0);
        Float inv = (Float)1 / f;
        x *= inv;
        y *= inv;
        z *= inv;
        return *this;
    }
    T operator[](int i) const // 下标，返回左值
    {
        DCHECK(i >= 0 && i <= 2);
        if (i == 0)
            return x;
        if (i == 1)
            return y;
        return z;
    }

    T &operator[](int i) // 下标，返回右值
    {
        DCHECK(i >= 0 && i <= 2);
        if (i == 0)
            return x;
        if (i == 1)
            return y;
        return z;
    }
    bool operator==(const Point3<T> &p) const // 比较是否相等
    {
        return x == p.x && y == p.y && z == p.z;
    }
    bool operator!=(const Point3<T> &p) const // 比较是否不相等
    {
        return x != p.x || y != p.y || z != p.z;
    }
    bool HasNaNs() const { return isNaN(x) || isNaN(y) || isNaN(z); }

    // Point3 共有数据
    T x, y, z;
};

template <typename T>
inline std::ostream &operator<<(std::ostream &os, const Point3<T> &v)
{
    os << "[ " << v.x << ", " << v.y << ", " << v.z << " ]";
    return os;
}

template <>
inline std::ostream &operator<<(std::ostream &os, const Point3<Float> &v)
{
    os << StringPrintf("[ %f, %f, %f ]", v.x, v.y, v.z);
    return os;
}

// 常用特化版本
typedef Point2<Float> Point2f;
typedef Point2<int> Point2i;
typedef Point3<Float> Point3f;
typedef Point3<int> Point3i;

// Normal 声明
template <typename T>
class Normal3
{
public:
    // Normal3 公有方法
    Normal3() { x = y = z = 0; }
    Normal3(T xx, T yy, T zz) : x(xx), y(yy), z(zz) { DCHECK(!HasNaNs()); }
    Normal3<T> operator-() const { return Normal3(-x, -y, -z); } // 负号
    Normal3<T> operator+(const Normal3<T> &n) const              // 加号
    {
        DCHECK(!n.HasNaNs());
        return Normal3<T>(x + n.x, y + n.y, z + n.z);
    }

    Normal3<T> &operator+=(const Normal3<T> &n) // 加且赋值
    {
        DCHECK(!n.HasNaNs());
        x += n.x;
        y += n.y;
        z += n.z;
        return *this;
    }
    Normal3<T> operator-(const Normal3<T> &n) const // 减
    {
        DCHECK(!n.HasNaNs());
        return Normal3<T>(x - n.x, y - n.y, z - n.z);
    }

    Normal3<T> &operator-=(const Normal3<T> &n) // 减且赋值
    {
        DCHECK(!n.HasNaNs());
        x -= n.x;
        y -= n.y;
        z -= n.z;
        return *this;
    }

    bool HasNaNs() const { return isNaN(x) || isNaN(y) || isNaN(z); }

    template <typename U>
    Normal3<T> operator*(U f) const // 乘以
    {
        return Normal3<T>(f * x, f * y, f * z);
    }

    template <typename U>
    Normal3<T> &operator*=(U f) // 乘以且赋值
    {
        x *= f;
        y *= f;
        z *= f;
        return *this;
    }

    template <typename U>
    Normal3<T> operator/(U f) const // 除以
    {
        CHECK_NE(f, 0);
        Float inv = (Float)1 / f;
        return Normal3<T>(x * inv, y * inv, z * inv);
    }

    template <typename U>
    Normal3<T> &operator/=(U f) // 除以且赋值
    {
        CHECK_NE(f, 0);
        Float inv = (Float)1 / f;
        x *= inv;
        y *= inv;
        z *= inv;
        return *this;
    }
    Float LengthSquared() const { return x * x + y * y + z * z; } // 长度的平方
    Float Length() const { return std::sqrt(LengthSquared()); }   // 长度

#ifndef NDEBUG
    Normal3<T>(const Normal3<T> &n) // 拷贝构造函数
    {
        DCHECK(!n.HasNaNs());
        x = n.x;
        y = n.y;
        z = n.z;
    }

    Normal3<T> &operator=(const Normal3<T> &n) // 赋值函数
    {
        DCHECK(!n.HasNaNs());
        x = n.x;
        y = n.y;
        z = n.z;
        return *this;
    }
#endif                                                                // !NDEBUG
    explicit Normal3<T>(const Vector3<T> &v) : x(v.x), y(v.y), z(v.z) // 显示构造函数
    {
        DCHECK(!v.HasNaNs());
    }

    bool operator==(const Normal3<T> &n) const // 比较是否相等
    {
        return x == n.x && y == n.y && z == n.z;
    }

    bool operator!=(const Normal3<T> &n) const // 比较是否不相等
    {
        return x != n.x || y != n.y || z != n.z;
    }

    T operator[](int i) const // 下标，返回左值
    {
        DCHECK(i >= 0 && i <= 2);
        if (i == 0)
            return x;
        if (i == 1)
            return y;
        return z;
    }

    T &operator[](int i) // 下标，返回右值
    {
        DCHECK(i >= 0 && i <= 2);
        if (i == 0)
            return x;
        if (i == 1)
            return y;
        return z;
    }

    // Normal3 公有数据
    T x, y, z;
};

template <typename T>
inline std::ostream &operator<<(std::ostream &os, const Normal3<T> &v)
{
    os << "[ " << v.x << ", " << v.y << ", " << v.z << " ]";
    return os;
}

template <>
inline std::ostream &operator<<(std::ostream &os, const Normal3<Float> &v)
{
    os << StringPrintf("[ %f, %f, %f ]", v.x, v.y, v.z);
    return os;
}

// 常用特化类型
typedef Normal3<Float> Normal3f;

// Bounds 声明
template <typename T>
class Bounds2
{
public:
    // Bounds2 公有方法
    Bounds2()
    {
        T minNum = std::numeric_limits<T>::lowest();
        T maxNum = std::numeric_limits<T>::max();
        pMin = Point2<T>(maxNum, maxNum); // 小边界
        pMax = Point2<T>(minNum, minNum); // 大边界
    }
    explicit Bounds2(const Point2<T> &p) : pMin(p), pMax(p) {} // 显示构造函数
    Bounds2(const Point2<T> &p1, const Point2<T> &p2)          // 构造函数
    {
        pMin = Point2<T>(std::min(p1.x, p2.x), std::min(p1.y, p2.y));
        pMax = Point2<T>(std::max(p1.x, p2.x), std::max(p1.y, p2.y));
    }
    template <typename U>
    explicit operator Bounds2<U>() const // 类型转换重载
    {
        return Bounds2<U>((Point2<U>)pMin, (Point2<U>)pMax);
    }

    Vector2<T> Diagonal() const { return pMax - pMin; } // 求对角线

    T Area() const // 面积
    {
        Vector2<T> d = pMax - pMin;
        return (d.x * d.y);
    }

    int MaximumExtent() const // 哪个轴向最长
    {
        Vector2<T> diag = Diagonal();
        if (diag.x > diag.y)
            return 0;
        else
            return 1;
    }

    inline const Point2<T> &operator[](int i) const // 下标，返回左值
    {
        DCHECK(i == 0 || i == 1);
        return (i == 0) ? pMin : pMax;
    }

    inline Point2<T> &operator[](int i) // 下标，返回左值
    {
        DCHECK(i == 0 || i == 1);
        return (i == 0) ? pMin : pMax;
    }

    bool operator==(const Bounds2<T> &b) const // 比较是否相等
    {
        return b.pMin == pMin && b.pMax == pMax;
    }

    bool operator!=(const Bounds2<T> &b) const // 比较是否不相等
    {
        return b.pMin != pMin || b.pMax != pMax;
    }

    Point2<T> Lerp(const Point2f &t) const // 插值
    {
        return Point2<T>(pbrt::Lerp(t.x, pMin.x, pMax.x),
                         pbrt::Lerp(t.y, pMin.y, pMax.y));
    }

    Vector2<T> Offset(const Point2<T> &p) const // 偏移，即p点在该区域中的位置，(0,0)代表pMin，(1,1)代表pMax
    {
        Vector2<T> o = p - pMin;
        if (pMax.x > pMin.x)
            o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y)
            o.y /= pMax.y - pMin.y;
        return o;
    }

    void BoundingSphere(Point2<T> *c, Float *rad) const // 求外接圆，c为中心（圆心），rad为半径
    {
        *c = (pMin + pMax) / 2; // 区域中心
        *rad = Inside(*c, *this) ? Distance(*c, pMax) : 0;
    }
    friend std::ostream &operator<<(std::ostream &os, const Bounds2<T> &b)
    {
        os << "[ " << b.pMin << " - " << b.pMax << " ]";
        return os;
    }

    // Bounds2 公有数据
    Point2<T> pMin, pMax;
};

template <typename T>
class Bounds3
{
public:
    // Bounds3 公有方法
    Bounds3()
    {
        T minNum = std::numeric_limits<T>::lowest();
        T maxNum = std::numeric_limits<T>::max();
        pMin = Point3<T>(maxNum, maxNum, maxNum); // 小边界
        pMax = Point3<T>(minNum, minNum, minNum); // 大边界
    }
    explicit Bounds3(const Point3<T> &p) : pMin(p), pMax(p) {} // 显示构造函数
    Bounds3(const Point3<T> &p1, const Point3<T> &p2)          // 构造函数
        : pMin(std::min(p1.x, p2.x), std::min(p1.y, p2.y), std::min(p1.z, p2.z)),
          pMax(std::max(p1.x, p2.x), std::max(p1.y, p2.y), std::max(p1.z, p2.z))
    {
    }

    const Point3<T> &operator[](int i) const; // 下标，返回左值
    Point3<T> &operator[](int i);             // 下标，返回左值

    bool operator==(const Bounds3<T> &b) const // 比较是否相等
    {
        return b.pMin == pMin && b.pMax == pMax;
    }

    bool operator!=(const Bounds3<T> &b) const // 比较是否不相等
    {
        return b.pMin != pMin || b.pMax != pMax;
    }

    Point3<T> Corner(int corner) const // 长方体的各个顶点，共8个
    {
        DCHECK(corner >= 0 && corner < 8);
        return Point3<T>((*this)[(corner & 1)].x,
                         (*this)[(corner & 2) ? 1 : 0].y,
                         (*this)[(corner & 4) ? 1 : 0].z);
    }

    Vector3<T> Diagonal() const { return pMax - pMin; } // 对角线

    T SurfaceArea() const // 表面积
    {
        Vector3<T> d = Diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    T Volume() const // 体积
    {
        Vector3<T> d = Diagonal();
        return d.x * d.y * d.z;
    }

    int MaximumExtent() const // 哪个轴向最长
    {
        Vector3<T> d = Diagonal();
        if (d.x > d.y && d.x > d.z)
            return 0;
        else if (d.y > d.z)
            return 1;
        else
            return 2;
    }

    Point3<T> Lerp(const Point3f &t) const // 插值
    {
        return Point3<T>(pbrt::Lerp(t.x, pMin.x, pMax.x),
                         pbrt::Lerp(t.y, pMin.y, pMax.y),
                         pbrt::Lerp(t.z, pMin.z, pMax.z));
    }

    Vector3<T> Offset(const Point3<T> &p) const // 偏移，即p点在该长方体中的位置，(0,0,0)代表pMin，(1,1,1)代表pMax
    {
        Vector3<T> o = p - pMin;
        if (pMax.x > pMin.x)
            o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y)
            o.y /= pMax.y - pMin.y;
        if (pMax.z > pMin.z)
            o.z /= pMax.z - pMin.z;
        return o;
    }

    void BoundingSphere(Point3<T> *center, Float *radius) const // 求外接球，c为中心（球心），rad为半径
    {
        *center = (pMin + pMax) / 2;
        *radius = Inside(*center, *this) ? Distance(*center, pMax) : 0;
    }

    template <typename U>
    explicit operator Bounds3<U>() const // 类型转换重载
    {
        return Bounds3<U>((Point3<U>)pMin, (Point3<U>)pMax);
    }
    // 射线求交，返回两个交点
    bool IntersectP(const Ray &ray, Float *hitt0 = nullptr, Float *hitt1 = nullptr) const;
    // 射线求交，不返回交点信息，速度更快
    inline bool IntersectP(const Ray &ray, const Vector3f &invDir, const int dirIsNeg[3]) const;
    friend std::ostream &operator<<(std::ostream &os, const Bounds3<T> &b)
    {
        os << "[ " << b.pMin << " - " << b.pMax << " ]";
        return os;
    }

    // Bounds3 公有数据
    Point3<T> pMin, pMax;
};

// 常用特化类型
typedef Bounds2<Float> Bounds2f;
typedef Bounds2<int> Bounds2i;
typedef Bounds3<Float> Bounds3f;
typedef Bounds3<int> Bounds3i;

class Bounds2iIterator : public std::forward_iterator_tag
{
public:
    Bounds2iIterator(const Bounds2i &b, const Point2i &pt)
        : p(pt), bounds(&b) {}
    Bounds2iIterator operator++() // 前置++，返回操作之后的值
    {
        advance();
        return *this;
    }
    Bounds2iIterator operator++(int) // 后置++，接受一个不被使用int类型的参数。返回操作之前的值
    {
        Bounds2iIterator old = *this;
        advance();
        return old;
    }
    bool operator==(const Bounds2iIterator &bi) const
    {
        return p == bi.p && bounds == bi.bounds;
    }
    bool operator!=(const Bounds2iIterator &bi) const
    {
        return p != bi.p || bounds != bi.bounds;
    }

    Point2i operator*() const { return p; }

private:
    void advance()
    {
        ++p.x;
        if (p.x == bounds->pMax.x)
        {
            p.x = bounds->pMin.x;
            ++p.y;
        }
    }
    Point2i p;
    const Bounds2i *bounds;
};

// Ray 定义
class Ray
{
public:
    // Ray 公有方法
    Ray() : tMax(Infinity), time(0.f), medium(nullptr) {}
    Ray(const Point3f &o, const Vector3f &d, Float tMax = Infinity,
        Float time = 0.f, const Medium *medium = nullptr)
        : o(o), d(d), tMax(tMax), time(time), medium(medium) {}
    Point3f operator()(Float t) const { return o + d * t; }
    bool HasNaNs() const { return (o.HasNaNs() || d.HasNaNs() || isNaN(tMax)); }
    friend std::ostream &operator<<(std::ostream &os, const Ray &r)
    {
        os << "[o=" << r.o << ", d=" << r.d << ", tMax=" << r.tMax
           << ", time=" << r.time << "]";
        return os;
    }

    // Ray 公有数据
    Point3f o;            // 原点
    Vector3f d;           // 方向
    mutable Float tMax;   // 最大值
    Float time;           // 时间
    const Medium *medium; // 介质
};

class RayDifferential : public Ray
{
public:
    // RayDifferential 公有方法
    RayDifferential() { hasDifferentials = false; }
    RayDifferential(const Point3f &o, const Vector3f &d, Float tMax = Infinity,
                    Float time = 0.f, const Medium *medium = nullptr)
        : Ray(o, d, tMax, time, medium)
    {
        hasDifferentials = false;
    }
    RayDifferential(const Ray &ray) : Ray(ray) { hasDifferentials = false; }
    bool HasNaNs() const
    {
        return Ray::HasNaNs() || (hasDifferentials &&
                                  (rxOrigin.HasNaNs() || ryOrigin.HasNaNs() || rxDirection.HasNaNs() || ryDirection.HasNaNs()));
    }
    void ScaleDifferentials(Float s)
    {
        rxOrigin = o + (rxOrigin - o) * s;
        ryOrigin = o + (ryOrigin - o) * s;
        rxDirection = d + (rxDirection - d) * s;
        ryDirection = d + (ryDirection - d) * s;
    }
    friend std::ostream &operator<<(std::ostream &os, const RayDifferential &r)
    {
        os << "[ " << (Ray &)r << " has differentials: "
           << (r.hasDifferentials ? "true" : "false")
           << ", xo = " << r.rxOrigin << ", xd = " << r.rxDirection
           << ", yo = " << r.ryOrigin << ", yd = " << r.ryDirection;
        return os;
    }

    // RayDifferential 公有数据
    bool hasDifferentials;      // 标记
    Point3f rxOrigin, ryOrigin; //
    Vector3f rxDirection, ryDirection;
};

// Geometry 内联函数
template <typename T>
inline Vector3<T>::Vector3(const Point3<T> &p)
    : x(p.x), y(p.y), z(p.z)
{
    DCHECK(!HasNaNs());
}

template <typename T>
inline Vector3<T>::Vector3(const Normal3<T> &n)
    : x(n.x), y(n.y), z(n.z)
{
    DCHECK(!n.HasNaNs());
}

template <typename T, typename U>
inline Vector3<T> operator*(U s, const Vector3<T> &v) // 乘以，全局运算符重载
{
    return v * s;
}

template <typename T>
Vector3<T> Abs(const Vector3<T> &v) // 绝对值
{
    return Vector3<T>(std::abs(v.x), std::abs(v.y), std::abs(v.z));
}

template <typename T>
inline T Dot(const Vector3<T> &v1, const Vector3<T> &v2) // Vector3点乘
{
    DCHECK(!v1.HasNaNs() && !v2.HasNaNs());
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

template <typename T>
inline T AbsDot(const Vector3<T> &v1, const Vector3<T> &v2) // Vector3点乘的绝对值
{
    DCHECK(!v1.HasNaNs() && !v2.HasNaNs());
    return std::abs(Dot(v1, v2));
}

template <typename T>
inline Vector3<T> Cross(const Vector3<T> &v1, const Vector3<T> &v2) // Vector3叉乘
{
    DCHECK(!v1.HasNaNs() && !v2.HasNaNs());
    double v1x = v1.x, v1y = v1.y, v1z = v1.z;
    double v2x = v2.x, v2y = v2.y, v2z = v2.z;
    return Vector3<T>((v1y * v2z) - (v1z * v2y), (v1z * v2x) - (v1x * v2z),
                      (v1x * v2y) - (v1y * v2x));
}

template <typename T>
inline Vector3<T> Cross(const Vector3<T> &v1, const Normal3<T> &v2) // Vector3叉乘Normal3
{
    DCHECK(!v1.HasNaNs() && !v2.HasNaNs());
    double v1x = v1.x, v1y = v1.y, v1z = v1.z;
    double v2x = v2.x, v2y = v2.y, v2z = v2.z;
    return Vector3<T>((v1y * v2z) - (v1z * v2y), (v1z * v2x) - (v1x * v2z),
                      (v1x * v2y) - (v1y * v2x));
}

template <typename T>
inline Vector3<T> Cross(const Normal3<T> &v1, const Vector3<T> &v2) // Normal3叉乘Vector3
{
    DCHECK(!v1.HasNaNs() && !v2.HasNaNs());
    double v1x = v1.x, v1y = v1.y, v1z = v1.z;
    double v2x = v2.x, v2y = v2.y, v2z = v2.z;
    return Vector3<T>((v1y * v2z) - (v1z * v2y), (v1z * v2x) - (v1x * v2z),
                      (v1x * v2y) - (v1y * v2x));
}

template <typename T>
inline Vector3<T> Normalize(const Vector3<T> &v) // 归一化
{
    return v / v.Length();
}

template <typename T>
T MinComponent(const Vector3<T> &v) // 最小值
{
    return std::min(v.x, std::min(v.y, v.z));
}

template <typename T>
T MaxComponent(const Vector3<T> &v) // 最大值
{
    return std::max(v.x, std::max(v.y, v.z));
}

template <typename T>
int MaxDimension(const Vector3<T> &v) // 最大值对应的轴向
{
    return (v.x > v.y) ? ((v.x > v.z) ? 0 : 2) : ((v.y > v.z) ? 1 : 2);
}

template <typename T>
Vector3<T> Min(const Vector3<T> &p1, const Vector3<T> &p2) // 两个Vector3的最小值
{
    return Vector3<T>(std::min(p1.x, p2.x), std::min(p1.y, p2.y), std::min(p1.z, p2.z));
}

template <typename T>
Vector3<T> Max(const Vector3<T> &p1, const Vector3<T> &p2) // 两个Vector3的最大值
{
    return Vector3<T>(std::max(p1.x, p2.x), std::max(p1.y, p2.y), std::max(p1.z, p2.z));
}

template <typename T>
Vector3<T> Permute(const Vector3<T> &v, int x, int y, int z) // 重置顺序
{
    return Vector3<T>(v[x], v[y], v[z]);
}

template <typename T>
inline void CoordinateSystem(const Vector3<T> &v1, Vector3<T> *v2, Vector3<T> *v3) // 坐标系统转换
{
    if (std::abs(v1.x) > std::abs(v1.y))
        *v2 = Vector3<T>(-v1.z, 0, v1.x) / std::sqrt(v1.x * v1.x + v1.z * v1.z);
    else
        *v2 = Vector3<T>(0, v1.z, -v1.y) / std::sqrt(v1.y * v1.y + v1.z * v1.z);
    *v3 = Cross(v1, *v2);
}

template <typename T>
Vector2<T>::Vector2(const Point2<T> &p)
    : x(p.x), y(p.y)
{
    DCHECK(!HasNaNs());
}

template <typename T>
Vector2<T>::Vector2(const Point3<T> &p)
    : x(p.x), y(p.y)
{
    DCHECK(!HasNaNs());
}

template <typename T, typename U>
inline Vector2<T> operator*(U f, const Vector2<T> &v) // 乘以，全局运算符重载
{
    return v * f;
}
template <typename T>
inline Float Dot(const Vector2<T> &v1, const Vector2<T> &v2) // 点乘
{
    DCHECK(!v1.HasNaNs() && !v2.HasNaNs());
    return v1.x * v2.x + v1.y * v2.y;
}

template <typename T>
inline Float AbsDot(const Vector2<T> &v1, const Vector2<T> &v2) // 点乘后求绝对值
{
    DCHECK(!v1.HasNaNs() && !v2.HasNaNs());
    return std::abs(Dot(v1, v2));
}

template <typename T>
inline Vector2<T> Normalize(const Vector2<T> &v) // 归一化
{
    return v / v.Length();
}

template <typename T>
Vector2<T> Abs(const Vector2<T> &v) // 绝对值
{
    return Vector2<T>(std::abs(v.x), std::abs(v.y));
}

template <typename T>
inline Float Distance(const Point3<T> &p1, const Point3<T> &p2) // 距离
{
    return (p1 - p2).Length();
}

template <typename T>
inline Float DistanceSquared(const Point3<T> &p1, const Point3<T> &p2) // 距离的平方
{
    return (p1 - p2).LengthSquared();
}

template <typename T, typename U>
inline Point3<T> operator*(U f, const Point3<T> &p) // 乘以，全局运算符重载
{
    DCHECK(!p.HasNaNs());
    return p * f;
}

template <typename T>
Point3<T> Lerp(Float t, const Point3<T> &p0, const Point3<T> &p1) // 插值
{
    return (1 - t) * p0 + t * p1;
}

template <typename T>
Point3<T> Min(const Point3<T> &p1, const Point3<T> &p2) // 最小值
{
    return Point3<T>(std::min(p1.x, p2.x), std::min(p1.y, p2.y), std::min(p1.z, p2.z));
}

template <typename T>
Point3<T> Max(const Point3<T> &p1, const Point3<T> &p2) // 最大值
{
    return Point3<T>(std::max(p1.x, p2.x), std::max(p1.y, p2.y), std::max(p1.z, p2.z));
}

template <typename T>
Point3<T> Floor(const Point3<T> &p) // 向下取整
{
    return Point3<T>(std::floor(p.x), std::floor(p.y), std::floor(p.z));
}

template <typename T>
Point3<T> Ceil(const Point3<T> &p) // 向上取整
{
    return Point3<T>(std::ceil(p.x), std::ceil(p.y), std::ceil(p.z));
}

template <typename T>
Point3<T> Abs(const Point3<T> &p) // 绝对值
{
    return Point3<T>(std::abs(p.x), std::abs(p.y), std::abs(p.z));
}

template <typename T>
inline Float Distance(const Point2<T> &p1, const Point2<T> &p2) // 距离
{
    return (p1 - p2).Length();
}

template <typename T>
inline Float DistanceSquared(const Point2<T> &p1, const Point2<T> &p2) // 距离的平方
{
    return (p1 - p2).LengthSquared();
}

template <typename T, typename U>
inline Point2<T> operator*(U f, const Point2<T> &p) // 乘以，全局运算符重载
{
    DCHECK(!p.HasNaNs());
    return p * f;
}

template <typename T>
Point2<T> Floor(const Point2<T> &p) // 向下取整
{
    return Point2<T>(std::floor(p.x), std::floor(p.y));
}

template <typename T>
Point2<T> Ceil(const Point2<T> &p) // 向上取整
{
    return Point2<T>(std::ceil(p.x), std::ceil(p.y));
}

template <typename T>
Point2<T> Lerp(Float t, const Point2<T> &v0, const Point2<T> &v1) // 插值
{
    return (1 - t) * v0 + t * v1;
}

template <typename T>
Point2<T> Min(const Point2<T> &pa, const Point2<T> &pb) // 最小值
{
    return Point2<T>(std::min(pa.x, pb.x), std::min(pa.y, pb.y));
}

template <typename T>
Point2<T> Max(const Point2<T> &pa, const Point2<T> &pb) // 最大值
{
    return Point2<T>(std::max(pa.x, pb.x), std::max(pa.y, pb.y));
}

template <typename T>
Point3<T> Permute(const Point3<T> &p, int x, int y, int z) // 重置顺序
{
    return Point3<T>(p[x], p[y], p[z]);
}

template <typename T, typename U>
inline Normal3<T> operator*(U f, const Normal3<T> &n) // 乘以，全局运算符重载
{
    return Normal3<T>(f * n.x, f * n.y, f * n.z);
}

template <typename T>
inline Normal3<T> Normalize(const Normal3<T> &n) // 归一化
{
    return n / n.Length();
}

template <typename T>
inline T Dot(const Normal3<T> &n1, const Vector3<T> &v2) // Normal3点乘Vector3
{
    DCHECK(!n1.HasNaNs() && !v2.HasNaNs());
    return n1.x * v2.x + n1.y * v2.y + n1.z * v2.z;
}

template <typename T>
inline T Dot(const Vector3<T> &v1, const Normal3<T> &n2) // Vector3点乘Normal3
{
    DCHECK(!v1.HasNaNs() && !n2.HasNaNs());
    return v1.x * n2.x + v1.y * n2.y + v1.z * n2.z;
}

template <typename T>
inline T Dot(const Normal3<T> &n1, const Normal3<T> &n2) // Normal3点乘Normal3
{
    DCHECK(!n1.HasNaNs() && !n2.HasNaNs());
    return n1.x * n2.x + n1.y * n2.y + n1.z * n2.z;
}

template <typename T>
inline T AbsDot(const Normal3<T> &n1, const Vector3<T> &v2) // Normal3点乘Vector3的绝对值
{
    DCHECK(!n1.HasNaNs() && !v2.HasNaNs());
    return std::abs(n1.x * v2.x + n1.y * v2.y + n1.z * v2.z);
}

template <typename T>
inline T AbsDot(const Vector3<T> &v1, const Normal3<T> &n2) // Vector3点乘Normal3的绝对值
{
    DCHECK(!v1.HasNaNs() && !n2.HasNaNs());
    return std::abs(v1.x * n2.x + v1.y * n2.y + v1.z * n2.z);
}

template <typename T>
inline T AbsDot(const Normal3<T> &n1, const Normal3<T> &n2) // Normal3点乘Normal3的绝对值
{
    DCHECK(!n1.HasNaNs() && !n2.HasNaNs());
    return std::abs(n1.x * n2.x + n1.y * n2.y + n1.z * n2.z);
}

template <typename T>
inline Normal3<T> Faceforward(const Normal3<T> &n, const Vector3<T> &v) // 若n与v夹角大于90，则n反向
{
    return (Dot(n, v) < 0.f) ? -n : n;
}

template <typename T>
inline Normal3<T> Faceforward(const Normal3<T> &n, const Normal3<T> &n2)
{
    return (Dot(n, n2) < 0.f) ? -n : n;
}

template <typename T>
inline Vector3<T> Faceforward(const Vector3<T> &v, const Vector3<T> &v2)
{
    return (Dot(v, v2) < 0.f) ? -v : v;
}

template <typename T>
inline Vector3<T> Faceforward(const Vector3<T> &v, const Normal3<T> &n2)
{
    return (Dot(v, n2) < 0.f) ? -v : v;
}

template <typename T>
Normal3<T> Abs(const Normal3<T> &v) // 绝对值
{
    return Normal3<T>(std::abs(v.x), std::abs(v.y), std::abs(v.z));
}

template <typename T>
inline const Point3<T> &Bounds3<T>::operator[](int i) const // 下标
{
    DCHECK(i == 0 || i == 1);
    return (i == 0) ? pMin : pMax;
}

template <typename T>
inline Point3<T> &Bounds3<T>::operator[](int i) // 下标
{
    DCHECK(i == 0 || i == 1);
    return (i == 0) ? pMin : pMax;
}

template <typename T>
Bounds3<T> Union(const Bounds3<T> &b, const Point3<T> &p) // Bounds3和Point3合并，得到更大的Bounds3
{
    Bounds3<T> ret;
    ret.pMin = Min(b.pMin, p);
    ret.pMax = Max(b.pMax, p);
    return ret;
}

template <typename T>
Bounds3<T> Union(const Bounds3<T> &b1, const Bounds3<T> &b2) // Bounds3合并
{
    Bounds3<T> ret;
    ret.pMin = Min(b1.pMin, b2.pMin);
    ret.pMax = Max(b1.pMax, b2.pMax);
    return ret;
}

template <typename T>
Bounds3<T> Intersect(const Bounds3<T> &b1, const Bounds3<T> &b2) // Bounds3交集，得到更小的Bounds3
{
    // Important: assign to pMin/pMax directly and don't run the Bounds2()
    // constructor, since it takes min/max of the points passed to it.  In
    // turn, that breaks returning an invalid bound for the case where we
    // intersect non-overlapping bounds (as we'd like to happen).
    Bounds3<T> ret;
    ret.pMin = Max(b1.pMin, b2.pMin);
    ret.pMax = Min(b1.pMax, b2.pMax);
    return ret;
}

template <typename T>
bool Overlaps(const Bounds3<T> &b1, const Bounds3<T> &b2) // 检查是否重叠
{
    bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
    bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
    bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
    return (x && y && z);
}

template <typename T>
bool Inside(const Point3<T> &p, const Bounds3<T> &b) // 检查p点是否在b中
{
    return (p.x >= b.pMin.x && p.x <= b.pMax.x &&
            p.y >= b.pMin.y && p.y <= b.pMax.y &&
            p.z >= b.pMin.z && p.z <= b.pMax.z);
}

template <typename T>
bool InsideExclusive(const Point3<T> &p, const Bounds3<T> &b) // 检查p点是否在b中，前闭后开
{
    return (p.x >= b.pMin.x && p.x < b.pMax.x &&
            p.y >= b.pMin.y && p.y < b.pMax.y &&
            p.z >= b.pMin.z && p.z < b.pMax.z);
}

template <typename T, typename U>
inline Bounds3<T> Expand(const Bounds3<T> &b, U delta) // 沿边界扩展delta的宽度
{
    return Bounds3<T>(b.pMin - Vector3<T>(delta, delta, delta),
                      b.pMax + Vector3<T>(delta, delta, delta));
}

// 点到Bounds3的最小距离的平方，如果点在里面，则返回零
template <typename T, typename U>
inline Float DistanceSquared(const Point3<T> &p, const Bounds3<U> &b)
{
    Float dx = std::max({Float(0), b.pMin.x - p.x, p.x - b.pMax.x});
    Float dy = std::max({Float(0), b.pMin.y - p.y, p.y - b.pMax.y});
    Float dz = std::max({Float(0), b.pMin.z - p.z, p.z - b.pMax.z});
    return dx * dx + dy * dy + dz * dz;
}

// 点到Bounds3的最小距离，如果点在里面，则返回零
template <typename T, typename U>
inline Float Distance(const Point3<T> &p, const Bounds3<U> &b)
{
    return std::sqrt(DistanceSquared(p, b));
}

inline Bounds2iIterator begin(const Bounds2i &b)
{
    return Bounds2iIterator(b, b.pMin);
}

inline Bounds2iIterator end(const Bounds2i &b)
{
    // Normally, the ending point is at the minimum x value and one past
    // the last valid y value.
    Point2i pEnd(b.pMin.x, b.pMax.y);
    // However, if the bounds are degenerate, override the end point to
    // equal the start point so that any attempt to iterate over the bounds
    // exits out immediately.
    if (b.pMin.x >= b.pMax.x || b.pMin.y >= b.pMax.y)
        pEnd = b.pMin;
    return Bounds2iIterator(b, pEnd);
}

template <typename T>
Bounds2<T> Union(const Bounds2<T> &b, const Point2<T> &p) // Bounds2与Point2合并
{
    Bounds2<T> ret;
    ret.pMin = Min(b.pMin, p);
    ret.pMax = Max(b.pMax, p);
    return ret;
}

template <typename T>
Bounds2<T> Union(const Bounds2<T> &b, const Bounds2<T> &b2) // Bounds2合并
{
    Bounds2<T> ret;
    ret.pMin = Min(b.pMin, b2.pMin);
    ret.pMax = Max(b.pMax, b2.pMax);
    return ret;
}

template <typename T>
Bounds2<T> Intersect(const Bounds2<T> &b1, const Bounds2<T> &b2) // 求交集
{
    // Important: assign to pMin/pMax directly and don't run the Bounds2()
    // constructor, since it takes min/max of the points passed to it.  In
    // turn, that breaks returning an invalid bound for the case where we
    // intersect non-overlapping bounds (as we'd like to happen).
    Bounds2<T> ret;
    ret.pMin = Max(b1.pMin, b2.pMin);
    ret.pMax = Min(b1.pMax, b2.pMax);
    return ret;
}

template <typename T>
bool Overlaps(const Bounds2<T> &ba, const Bounds2<T> &bb) // 检查是否重叠
{
    bool x = (ba.pMax.x >= bb.pMin.x) && (ba.pMin.x <= bb.pMax.x);
    bool y = (ba.pMax.y >= bb.pMin.y) && (ba.pMin.y <= bb.pMax.y);
    return (x && y);
}

template <typename T>
bool Inside(const Point2<T> &pt, const Bounds2<T> &b) // p点是否在b中
{
    return (pt.x >= b.pMin.x && pt.x <= b.pMax.x &&
            pt.y >= b.pMin.y && pt.y <= b.pMax.y);
}

template <typename T>
bool InsideExclusive(const Point2<T> &pt, const Bounds2<T> &b) // p点是否在b中，前闭后开
{
    return (pt.x >= b.pMin.x && pt.x < b.pMax.x &&
            pt.y >= b.pMin.y && pt.y < b.pMax.y);
}

template <typename T, typename U>
Bounds2<T> Expand(const Bounds2<T> &b, U delta) // 边界扩展delta宽度
{
    return Bounds2<T>(b.pMin - Vector2<T>(delta, delta),
                      b.pMax + Vector2<T>(delta, delta));
}

// 射线求交
template <typename T>
inline bool Bounds3<T>::IntersectP(const Ray &ray, Float *hitt0, Float *hitt1) const
{
    Float t0 = 0, t1 = ray.tMax;
    for (int i = 0; i < 3; ++i)
    {
        // Update interval for _i_th bounding box slab
        Float invRayDir = 1 / ray.d[i];
        Float tNear = (pMin[i] - ray.o[i]) * invRayDir;
        Float tFar = (pMax[i] - ray.o[i]) * invRayDir;

        // Update parametric interval from slab intersection $t$ values
        if (tNear > tFar)
            std::swap(tNear, tFar);

        // Update _tFar_ to ensure robust ray--bounds intersection
        tFar *= 1 + 2 * gamma(3);
        t0 = tNear > t0 ? tNear : t0;
        t1 = tFar < t1 ? tFar : t1;
        if (t0 > t1)
            return false;
    }
    if (hitt0)
        *hitt0 = t0;
    if (hitt1)
        *hitt1 = t1;
    return true;
}

template <typename T>
inline bool Bounds3<T>::IntersectP(const Ray &ray, const Vector3f &invDir,
                                   const int dirIsNeg[3]) const
{
    const Bounds3f &bounds = *this;
    // Check for ray intersection against $x$ and $y$ slabs
    Float tMin = (bounds[dirIsNeg[0]].x - ray.o.x) * invDir.x;
    Float tMax = (bounds[1 - dirIsNeg[0]].x - ray.o.x) * invDir.x;
    Float tyMin = (bounds[dirIsNeg[1]].y - ray.o.y) * invDir.y;
    Float tyMax = (bounds[1 - dirIsNeg[1]].y - ray.o.y) * invDir.y;

    // Update _tMax_ and _tyMax_ to ensure robust bounds intersection
    tMax *= 1 + 2 * gamma(3);
    tyMax *= 1 + 2 * gamma(3);
    if (tMin > tyMax || tyMin > tMax)
        return false;
    if (tyMin > tMin)
        tMin = tyMin;
    if (tyMax < tMax)
        tMax = tyMax;

    // Check for ray intersection against $z$ slab
    Float tzMin = (bounds[dirIsNeg[2]].z - ray.o.z) * invDir.z;
    Float tzMax = (bounds[1 - dirIsNeg[2]].z - ray.o.z) * invDir.z;

    // Update _tzMax_ to ensure robust bounds intersection
    tzMax *= 1 + 2 * gamma(3);
    if (tMin > tzMax || tzMin > tMax)
        return false;
    if (tzMin > tMin)
        tMin = tzMin;
    if (tzMax < tMax)
        tMax = tzMax;
    return (tMin < ray.tMax) && (tMax > 0);
}

inline Point3f OffsetRayOrigin(const Point3f &p, const Vector3f &pError,
                               const Normal3f &n, const Vector3f &w)
{
    Float d = Dot(Abs(n), pError);
#ifdef PBRT_FLOAT_AS_DOUBLE
    // We have tons of precision; for now bump up the offset a bunch just
    // to be extra sure that we start on the right side of the surface
    // (In case of any bugs in the epsilons code...)
    d *= 1024.;
#endif
    Vector3f offset = d * Vector3f(n);
    if (Dot(w, n) < 0)
        offset = -offset;
    Point3f po = p + offset;
    // Round offset point _po_ away from _p_
    for (int i = 0; i < 3; ++i)
    {
        if (offset[i] > 0)
            po[i] = NextFloatUp(po[i]);
        else if (offset[i] < 0)
            po[i] = NextFloatDown(po[i]);
    }
    return po;
}

inline Vector3f SphericalDirection(Float sinTheta, Float cosTheta, Float phi)
{
    return Vector3f(sinTheta * std::cos(phi), sinTheta * std::sin(phi),
                    cosTheta);
}

inline Vector3f SphericalDirection(Float sinTheta, Float cosTheta, Float phi,
                                   const Vector3f &x, const Vector3f &y,
                                   const Vector3f &z)
{
    return sinTheta * std::cos(phi) * x + sinTheta * std::sin(phi) * y +
           cosTheta * z;
}

inline Float SphericalTheta(const Vector3f &v)
{
    return std::acos(Clamp(v.z, -1, 1));
}

inline Float SphericalPhi(const Vector3f &v)
{
    Float p = std::atan2(v.y, v.x);
    return (p < 0) ? (p + 2 * Pi) : p;
}

} // namespace pbrt

#endif // PBRT_CORE_GEOMETRY_H
