
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

#ifndef PBRT_CORE_SHAPE_H
#define PBRT_CORE_SHAPE_H

// core/shape.h*
#include "geometry.h"
#include "interaction.h"
#include "memory.h"
#include "pbrt.h"
#include "transform.h"

namespace pbrt
{

// Shape Declarations
// Shape 声明
class Shape
{
public:
    // Shape Interface
    // Shape 接口
    Shape(const Transform *ObjectToWorld, const Transform *WorldToObject,
          bool reverseOrientation);
    virtual ~Shape();
    virtual Bounds3f ObjectBound() const = 0; // 模型空间包围盒
    virtual Bounds3f WorldBound() const;      // 世界空间包围盒，默认实现：将模型空间包围盒转换到世界空间，再求包围盒

    // 求交，并返回交点信息，ray为世界空间表示，返回的信息也应为世界空间表示
    virtual bool Intersect(const Ray &ray, Float *tHit, SurfaceInteraction *isect,
                           bool testAlphaTexture = true) const = 0;
    // 求交，但不返回交点信息，默认实现为调用Intersect，比较慢，建议子类单独实现更高效的版本
    virtual bool IntersectP(const Ray &ray, bool testAlphaTexture = true) const
    {
        return Intersect(ray, nullptr, nullptr, testAlphaTexture);
    }
    // 区域光源使用
    virtual Float Area() const = 0;
    // Sample a point on the surface of the shape and return the PDF with
    // respect to area on the surface.
    virtual Interaction Sample(const Point2f &u, Float *pdf) const = 0;
    virtual Float Pdf(const Interaction &) const { return 1 / Area(); }

    // Sample a point on the shape given a reference point |ref| and
    // return the PDF with respect to solid angle from |ref|.
    virtual Interaction Sample(const Interaction &ref, const Point2f &u,
                               Float *pdf) const;
    virtual Float Pdf(const Interaction &ref, const Vector3f &wi) const;

    // Returns the solid angle subtended by the shape w.r.t. the reference
    // point p, given in world space. Some shapes compute this value in
    // closed-form, while the default implementation uses Monte Carlo
    // integration; the nSamples parameter determines how many samples are
    // used in this case.
    virtual Float SolidAngle(const Point3f &p, int nSamples = 512) const;

    // Shape Public Data
    // Shape 公有数据
    const Transform *ObjectToWorld, *WorldToObject; // 模型空间和世界空间的转换
    const bool reverseOrientation;
    const bool transformSwapsHandedness; // 手性转换
};

} // namespace pbrt

#endif // PBRT_CORE_SHAPE_H
