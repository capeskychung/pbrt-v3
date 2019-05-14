
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

#ifndef PBRT_SHAPES_TRIANGLE_H
#define PBRT_SHAPES_TRIANGLE_H

// shapes/triangle.h*
#include "shape.h"
#include "stats.h"
#include <map>

namespace pbrt
{

STAT_MEMORY_COUNTER("Memory/Triangle meshes", triMeshBytes);

// Triangle Declarations
// Triangle 声明
struct TriangleMesh
{
    // TriangleMesh Public Methods
    // TriangleMesh 公有方法
    TriangleMesh(const Transform &ObjectToWorld, int nTriangles,
                 const int *vertexIndices, int nVertices, const Point3f *P,
                 const Vector3f *S, const Normal3f *N, const Point2f *uv,
                 const std::shared_ptr<Texture<Float>> &alphaMask,
                 const std::shared_ptr<Texture<Float>> &shadowAlphaMask,
                 const int *faceIndices);

    // TriangleMesh Data
    // TriangleMesh 数据
    const int nTriangles, nVertices;                            // 三角形数量，顶点数量
    std::vector<int> vertexIndices;                             // 顶点索引数组，对应数组p中的索引，每连续三个索引对应一个三角形
    std::unique_ptr<Point3f[]> p;                               // 顶点位置数组，构造函数中将定点转换到世界空间，方便计算
    std::unique_ptr<Normal3f[]> n;                              // 可选，顶点法线数组
    std::unique_ptr<Vector3f[]> s;                              // 可选，顶点切线数组
    std::unique_ptr<Point2f[]> uv;                              // 可选，顶点uv数组
    std::shared_ptr<Texture<Float>> alphaMask, shadowAlphaMask; // 可选，alpha蒙版
    std::vector<int> faceIndices;                               // 面索引
};

class Triangle : public Shape
{
public:
    // Triangle Public Methods
    // Triangle 公有方法
    Triangle(const Transform *ObjectToWorld, const Transform *WorldToObject,
             bool reverseOrientation, const std::shared_ptr<TriangleMesh> &mesh,
             int triNumber)
        : Shape(ObjectToWorld, WorldToObject, reverseOrientation), mesh(mesh)
    {
        v = &mesh->vertexIndices[3 * triNumber];
        triMeshBytes += sizeof(*this);
        faceIndex = mesh->faceIndices.size() ? mesh->faceIndices[triNumber] : 0;
    }

    // 模型空间边界
    Bounds3f ObjectBound() const;
    // 世界空间边界
    Bounds3f WorldBound() const;
    
    bool Intersect(const Ray &ray, Float *tHit, SurfaceInteraction *isect,
                   bool testAlphaTexture = true) const;
    bool IntersectP(const Ray &ray, bool testAlphaTexture = true) const;
    
    Float Area() const;

    using Shape::Sample; // Bring in the other Sample() overload.
    Interaction Sample(const Point2f &u, Float *pdf) const;

    // Returns the solid angle subtended by the triangle w.r.t. the given
    // reference point p.
    Float SolidAngle(const Point3f &p, int nSamples = 0) const;

private:
    // Triangle Private Methods
    // TriangleMesh 私有方法
    void GetUVs(Point2f uv[3]) const
    {
        if (mesh->uv)
        {
            uv[0] = mesh->uv[v[0]];
            uv[1] = mesh->uv[v[1]];
            uv[2] = mesh->uv[v[2]];
        }
        else
        {
            uv[0] = Point2f(0, 0);
            uv[1] = Point2f(1, 0);
            uv[2] = Point2f(1, 1);
        }
    }

    // Triangle Private Data
    // TriangleMesh 私有数据
    std::shared_ptr<TriangleMesh> mesh; // 三角网格
    const int *v;                       // 索引数组中的指针
    int faceIndex;                      // 面索引
};

std::vector<std::shared_ptr<Shape>> CreateTriangleMesh(
    const Transform *o2w, const Transform *w2o, bool reverseOrientation,
    int nTriangles, const int *vertexIndices, int nVertices, const Point3f *p,
    const Vector3f *s, const Normal3f *n, const Point2f *uv,
    const std::shared_ptr<Texture<Float>> &alphaTexture,
    const std::shared_ptr<Texture<Float>> &shadowAlphaTexture,
    const int *faceIndices = nullptr);

std::vector<std::shared_ptr<Shape>> CreateTriangleMeshShape(
    const Transform *o2w, const Transform *w2o, bool reverseOrientation,
    const ParamSet &params,
    std::map<std::string, std::shared_ptr<Texture<Float>>> *floatTextures =
        nullptr);

bool WritePlyFile(const std::string &filename, int nTriangles,
                  const int *vertexIndices, int nVertices, const Point3f *P,
                  const Vector3f *S, const Normal3f *N, const Point2f *UV,
                  const int *faceIndices);

} // namespace pbrt

#endif // PBRT_SHAPES_TRIANGLE_H
