
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

#ifndef PBRT_CORE_SCENE_H
#define PBRT_CORE_SCENE_H

// core/scene.h*
#include "pbrt.h"
#include "geometry.h"
#include "primitive.h"
#include "light.h"

namespace pbrt
{

// Scene Declarations
// Scene 声明
class Scene
{
public:
    // Scene Public Methods
    // Scene 公有方法
    Scene(std::shared_ptr<Primitive> aggregate,
          const std::vector<std::shared_ptr<Light>> &lights)
        : lights(lights), aggregate(aggregate)
    {
        // Scene Constructor Implementation
        // Scene 构造函数实现
        worldBound = aggregate->WorldBound(); // 生成包围盒
        for (const auto &light : lights)
        {
            light->Preprocess(*this);                     // light根据场景进行预处理
            if (light->flags & (int)LightFlags::Infinite) // 讲infinite光源添加到单独的vector中
                infiniteLights.push_back(light);
        }
    }
    const Bounds3f &WorldBound() const { return worldBound; } // 获取包围盒
    // 光线与场景求交，如果相交，返回true，并用SurfaceInteraction返回距离最近的相交信息
    bool Intersect(const Ray &ray, SurfaceInteraction *isect) const;
    // 光线与场景求交，如果相交，返回true，但不计算相交信息，所以更快
    bool IntersectP(const Ray &ray) const;
    bool IntersectTr(Ray ray, Sampler &sampler, SurfaceInteraction *isect,
                     Spectrum *transmittance) const;

    // Scene Public Data
    // Scene 公有数据
    std::vector<std::shared_ptr<Light>> lights; // 光源
    // 额外再存储infinite光源，以便单独操作
    std::vector<std::shared_ptr<Light>> infiniteLights; // 无限光源

private:
    // Scene Private Data
    // Scene 私有数据
    std::shared_ptr<Primitive> aggregate; // 图元集合
    Bounds3f worldBound;                  // 包围盒
};

} // namespace pbrt

#endif // PBRT_CORE_SCENE_H
