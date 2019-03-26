
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

// core/integrator.cpp*
#include "integrator.h"
#include "camera.h"
#include "film.h"
#include "integrator.h"
#include "interaction.h"
#include "parallel.h"
#include "progressreporter.h"
#include "sampler.h"
#include "sampling.h"
#include "scene.h"
#include "stats.h"

namespace pbrt
{

STAT_COUNTER("Integrator/Camera rays traced", nCameraRays);

// Integrator 方法定义
Integrator::~Integrator() {}

// Integrator 实用函数
Spectrum UniformSampleAllLights(const Interaction &it, const Scene &scene,
                                MemoryArena &arena, Sampler &sampler,
                                const std::vector<int> &nLightSamples,
                                bool handleMedia)
{
    ProfilePhase p(Prof::DirectLighting);
    Spectrum L(0.f);
    for (size_t j = 0; j < scene.lights.size(); ++j)
    {
        // Accumulate contribution of _j_th light to _L_
        const std::shared_ptr<Light> &light = scene.lights[j];
        int nSamples = nLightSamples[j];
        const Point2f *uLightArray = sampler.Get2DArray(nSamples);
        const Point2f *uScatteringArray = sampler.Get2DArray(nSamples);
        if (!uLightArray || !uScatteringArray)
        {
            // Use a single sample for illumination from _light_
            Point2f uLight = sampler.Get2D();
            Point2f uScattering = sampler.Get2D();
            L += EstimateDirect(it, uScattering, *light, uLight, scene, sampler,
                                arena, handleMedia);
        }
        else
        {
            // Estimate direct lighting using sample arrays
            Spectrum Ld(0.f);
            for (int k = 0; k < nSamples; ++k)
                Ld += EstimateDirect(it, uScatteringArray[k], *light, uLightArray[k],
                                     scene, sampler, arena, handleMedia);
            L += Ld / nSamples;
        }
    }
    return L;
}

Spectrum UniformSampleOneLight(const Interaction &it, const Scene &scene,
                               MemoryArena &arena, Sampler &sampler,
                               bool handleMedia,
                               const Distribution1D *lightDistrib)
{
    ProfilePhase p(Prof::DirectLighting);
    // Randomly choose a single light to sample, _light_
    int nLights = int(scene.lights.size());
    if (nLights == 0)
        return Spectrum(0.f);
    int lightNum;
    Float lightPdf;
    if (lightDistrib)
    {
        lightNum = lightDistrib->SampleDiscrete(sampler.Get1D(), &lightPdf);
        if (lightPdf == 0)
            return Spectrum(0.f);
    }
    else
    {
        lightNum = std::min((int)(sampler.Get1D() * nLights), nLights - 1);
        lightPdf = Float(1) / nLights;
    }
    const std::shared_ptr<Light> &light = scene.lights[lightNum];
    Point2f uLight = sampler.Get2D();
    Point2f uScattering = sampler.Get2D();
    return EstimateDirect(it, uScattering, *light, uLight, scene, sampler, arena,
                          handleMedia) /
           lightPdf;
}

Spectrum EstimateDirect(const Interaction &it, const Point2f &uScattering,
                        const Light &light, const Point2f &uLight,
                        const Scene &scene, Sampler &sampler,
                        MemoryArena &arena, bool handleMedia, bool specular)
{
    BxDFType bsdfFlags =
        specular ? BSDF_ALL : BxDFType(BSDF_ALL & ~BSDF_SPECULAR);
    Spectrum Ld(0.f);
    // Sample light source with multiple importance sampling
    Vector3f wi;
    Float lightPdf = 0, scatteringPdf = 0;
    VisibilityTester visibility;
    Spectrum Li = light.Sample_Li(it, uLight, &wi, &lightPdf, &visibility);
    VLOG(2) << "EstimateDirect uLight:" << uLight << " -> Li: " << Li
            << ", wi: " << wi << ", pdf: " << lightPdf;
    if (lightPdf > 0 && !Li.IsBlack())
    {
        // Compute BSDF or phase function's value for light sample
        Spectrum f;
        if (it.IsSurfaceInteraction())
        {
            // Evaluate BSDF for light sampling strategy
            const SurfaceInteraction &isect = (const SurfaceInteraction &)it;
            f = isect.bsdf->f(isect.wo, wi, bsdfFlags) * AbsDot(wi, isect.shading.n);
            scatteringPdf = isect.bsdf->Pdf(isect.wo, wi, bsdfFlags);
            VLOG(2) << "  surf f*dot :" << f << ", scatteringPdf: " << scatteringPdf;
        }
        else
        {
            // Evaluate phase function for light sampling strategy
            const MediumInteraction &mi = (const MediumInteraction &)it;
            Float p = mi.phase->p(mi.wo, wi);
            f = Spectrum(p);
            scatteringPdf = p;
            VLOG(2) << "  medium p: " << p;
        }
        if (!f.IsBlack())
        {
            // Compute effect of visibility for light source sample
            if (handleMedia)
            {
                Li *= visibility.Tr(scene, sampler);
                VLOG(2) << "  after Tr, Li: " << Li;
            }
            else
            {
                if (!visibility.Unoccluded(scene))
                {
                    VLOG(2) << "  shadow ray blocked";
                    Li = Spectrum(0.f);
                }
                else
                    VLOG(2) << "  shadow ray unoccluded";
            }

            // Add light's contribution to reflected radiance
            if (!Li.IsBlack())
            {
                if (IsDeltaLight(light.flags))
                    Ld += f * Li / lightPdf;
                else
                {
                    Float weight = PowerHeuristic(1, lightPdf, 1, scatteringPdf);
                    Ld += f * Li * weight / lightPdf;
                }
            }
        }
    }

    // Sample BSDF with multiple importance sampling
    if (!IsDeltaLight(light.flags))
    {
        Spectrum f;
        bool sampledSpecular = false;
        if (it.IsSurfaceInteraction())
        {
            // Sample scattered direction for surface interactions
            BxDFType sampledType;
            const SurfaceInteraction &isect = (const SurfaceInteraction &)it;
            f = isect.bsdf->Sample_f(isect.wo, &wi, uScattering, &scatteringPdf,
                                     bsdfFlags, &sampledType);
            f *= AbsDot(wi, isect.shading.n);
            sampledSpecular = (sampledType & BSDF_SPECULAR) != 0;
        }
        else
        {
            // Sample scattered direction for medium interactions
            const MediumInteraction &mi = (const MediumInteraction &)it;
            Float p = mi.phase->Sample_p(mi.wo, &wi, uScattering);
            f = Spectrum(p);
            scatteringPdf = p;
        }
        VLOG(2) << "  BSDF / phase sampling f: " << f
                << ", scatteringPdf: " << scatteringPdf;
        if (!f.IsBlack() && scatteringPdf > 0)
        {
            // Account for light contributions along sampled direction _wi_
            Float weight = 1;
            if (!sampledSpecular)
            {
                lightPdf = light.Pdf_Li(it, wi);
                if (lightPdf == 0)
                    return Ld;
                weight = PowerHeuristic(1, scatteringPdf, 1, lightPdf);
            }

            // Find intersection and compute transmittance
            SurfaceInteraction lightIsect;
            Ray ray = it.SpawnRay(wi);
            Spectrum Tr(1.f);
            bool foundSurfaceInteraction =
                handleMedia ? scene.IntersectTr(ray, sampler, &lightIsect, &Tr)
                            : scene.Intersect(ray, &lightIsect);

            // Add light contribution from material sampling
            Spectrum Li(0.f);
            if (foundSurfaceInteraction)
            {
                if (lightIsect.primitive->GetAreaLight() == &light)
                    Li = lightIsect.Le(-wi);
            }
            else
                Li = light.Le(ray);
            if (!Li.IsBlack())
                Ld += f * Li * Tr * weight / scatteringPdf;
        }
    }
    return Ld;
}

std::unique_ptr<Distribution1D> ComputeLightPowerDistribution(
    const Scene &scene)
{
    if (scene.lights.empty())
        return nullptr;
    std::vector<Float> lightPower;
    for (const auto &light : scene.lights)
        lightPower.push_back(light->Power().y());
    return std::unique_ptr<Distribution1D>(
        new Distribution1D(&lightPower[0], lightPower.size()));
}

// SamplerIntegrator 方法定义
void SamplerIntegrator::Render(const Scene &scene)
{
    Preprocess(scene, *sampler); // 预处理
    // 并行渲染图像块

    // 计算图像块的数量 _nTiles_, 用于并行渲染
    Bounds2i sampleBounds =
        camera->film->GetSampleBounds();             // sampleBounds表示采样范围的边界
    Vector2i sampleExtent = sampleBounds.Diagonal(); // 采样范围的长宽
    const int tileSize = 16;                         // 每个图像块长宽16像素
    // 计算长宽各有多少块
    Point2i nTiles((sampleExtent.x + tileSize - 1) / tileSize,
                   (sampleExtent.y + tileSize - 1) / tileSize);
    ProgressReporter reporter(nTiles.x * nTiles.y, "Rendering"); // 进度汇报器

    {
        // 传递一个lambda表达式给线程池，类似传递一个函数指针，这个函数会被调用多次（一个tile一次）
        ParallelFor2D(
            [&](Point2i tile) {
                // 渲染对应的图像块

                // 为图像块配置内存池
                MemoryArena arena;

                // 获得采样器实例
                int seed = tile.y * nTiles.x + tile.x;                       // seed表示当前是第几个tile
                std::unique_ptr<Sampler> tileSampler = sampler->Clone(seed); // 因为Sampler会被并发调用，所以必须为每个线程单独拷贝一份

                // 计算当前tile的边界
                int x0 = sampleBounds.pMin.x + tile.x * tileSize;      // tile左边界
                int x1 = std::min(x0 + tileSize, sampleBounds.pMax.x); // tile右边界，需要和最大值比对，存在剩余像素不足一个tile的情况
                int y0 = sampleBounds.pMin.y + tile.y * tileSize;      // tile上边界
                int y1 = std::min(y0 + tileSize, sampleBounds.pMax.y); // tile下边界，需要和最大值比对，存在不足一个tile的情况
                Bounds2i tileBounds(Point2i(x0, y0), Point2i(x1, y1)); // 当前tile的边界
                LOG(INFO) << "Starting image tile " << tileBounds;

                // FilmTile是对应块的渲染结果，即最终图像的一部分
                std::unique_ptr<FilmTile> filmTile = camera->film->GetFilmTile(tileBounds);

                // 循环，渲染tile里每个像素
                for (Point2i pixel : tileBounds)
                {
                    {
                        ProfilePhase pp(Prof::StartPixel);
                        tileSampler->StartPixel(pixel);
                    }

                    // Do this check after the StartPixel() call; this keeps
                    // the usage of RNG values from (most) Samplers that use
                    // RNGs consistent, which improves reproducability /
                    // debugging.
                    if (!InsideExclusive(pixel, pixelBounds))
                        continue;

                    do
                    {
                        // 初始化CameraSample，它存储了当前在film平面上的采样点，以便于生成采样光线；
                        // 存储了采样时间，用于模拟运动物体；存储了镜片位置，用于模拟光圈虚化。
                        CameraSample cameraSample = tileSampler->GetCameraSample(pixel);

                        // 为当前采样点生成采样光线，RayDifferential包含一组光线，用于纹理抗锯齿
                        RayDifferential ray;
                        Float rayWeight = camera->GenerateRayDifferential(cameraSample, &ray); // rayWeight为权重
                        ray.ScaleDifferentials(1 / std::sqrt((Float)tileSampler->samplesPerPixel));
                        ++nCameraRays;

                        // Evaluate radiance along camera ray
                        Spectrum L(0.f);
                        if (rayWeight > 0)
                            L = Li(ray, scene, *tileSampler, arena);

                        // 如果采样结果错误，输出警告
                        if (L.HasNaNs()) // 采样结果不是数字
                        {
                            LOG(ERROR) << StringPrintf(
                                "Not-a-number radiance value returned "
                                "for pixel (%d, %d), sample %d. Setting to black.",
                                pixel.x, pixel.y,
                                (int)tileSampler->CurrentSampleNumber());
                            L = Spectrum(0.f);
                        }
                        else if (L.y() < -1e-5) // 采样结果为负
                        {
                            LOG(ERROR) << StringPrintf(
                                "Negative luminance value, %f, returned "
                                "for pixel (%d, %d), sample %d. Setting to black.",
                                L.y(), pixel.x, pixel.y,
                                (int)tileSampler->CurrentSampleNumber());
                            L = Spectrum(0.f);
                        }
                        else if (std::isinf(L.y())) // 采样结果无穷大
                        {
                            LOG(ERROR) << StringPrintf(
                                "Infinite luminance value returned "
                                "for pixel (%d, %d), sample %d. Setting to black.",
                                pixel.x, pixel.y,
                                (int)tileSampler->CurrentSampleNumber());
                            L = Spectrum(0.f);
                        }
                        VLOG(1) << "Camera sample: " << cameraSample << " -> ray: " << ray << " -> L = " << L;

                        // 添加采样光线对图像的贡献
                        filmTile->AddSample(cameraSample.pFilm, L, rayWeight);

                        // 释放内存池
                        arena.Reset();
                    } while (tileSampler->StartNextSample());
                }
                LOG(INFO) << "Finished image tile " << tileBounds;

                // 用右值引用，把filmtile里的动态资源转移过去
                camera->film->MergeFilmTile(std::move(filmTile));
                reporter.Update();
            },
            nTiles);
        reporter.Done();
    }
    LOG(INFO) << "Rendering finished";

    // Save final image after rendering
    camera->film->WriteImage();
}

Spectrum SamplerIntegrator::SpecularReflect(
    const RayDifferential &ray, const SurfaceInteraction &isect,
    const Scene &scene, Sampler &sampler, MemoryArena &arena, int depth) const
{
    // Compute specular reflection direction _wi_ and BSDF value
    Vector3f wo = isect.wo, wi;
    Float pdf;
    BxDFType type = BxDFType(BSDF_REFLECTION | BSDF_SPECULAR);
    Spectrum f = isect.bsdf->Sample_f(wo, &wi, sampler.Get2D(), &pdf, type);

    // Return contribution of specular reflection
    const Normal3f &ns = isect.shading.n;
    if (pdf > 0.f && !f.IsBlack() && AbsDot(wi, ns) != 0.f)
    {
        // Compute ray differential _rd_ for specular reflection
        RayDifferential rd = isect.SpawnRay(wi);
        if (ray.hasDifferentials)
        {
            rd.hasDifferentials = true;
            rd.rxOrigin = isect.p + isect.dpdx;
            rd.ryOrigin = isect.p + isect.dpdy;
            // Compute differential reflected directions
            Normal3f dndx =
                isect.shading.dndu * isect.dudx + isect.shading.dndv * isect.dvdx;
            Normal3f dndy =
                isect.shading.dndu * isect.dudy + isect.shading.dndv * isect.dvdy;
            Vector3f dwodx = -ray.rxDirection - wo, dwody = -ray.ryDirection - wo;
            Float dDNdx = Dot(dwodx, ns) + Dot(wo, dndx);
            Float dDNdy = Dot(dwody, ns) + Dot(wo, dndy);
            rd.rxDirection =
                wi - dwodx + 2.f * Vector3f(Dot(wo, ns) * dndx + dDNdx * ns);
            rd.ryDirection =
                wi - dwody + 2.f * Vector3f(Dot(wo, ns) * dndy + dDNdy * ns);
        }
        return f * Li(rd, scene, sampler, arena, depth + 1) * AbsDot(wi, ns) / pdf;
    }
    else
        return Spectrum(0.f);
}

Spectrum SamplerIntegrator::SpecularTransmit(
    const RayDifferential &ray, const SurfaceInteraction &isect,
    const Scene &scene, Sampler &sampler, MemoryArena &arena, int depth) const
{
    Vector3f wo = isect.wo, wi;
    Float pdf;
    const Point3f &p = isect.p;
    const BSDF &bsdf = *isect.bsdf;
    Spectrum f = bsdf.Sample_f(wo, &wi, sampler.Get2D(), &pdf,
                               BxDFType(BSDF_TRANSMISSION | BSDF_SPECULAR));
    Spectrum L = Spectrum(0.f);
    Normal3f ns = isect.shading.n;
    if (pdf > 0.f && !f.IsBlack() && AbsDot(wi, ns) != 0.f)
    {
        // Compute ray differential _rd_ for specular transmission
        RayDifferential rd = isect.SpawnRay(wi);
        if (ray.hasDifferentials)
        {
            rd.hasDifferentials = true;
            rd.rxOrigin = p + isect.dpdx;
            rd.ryOrigin = p + isect.dpdy;

            Normal3f dndx =
                isect.shading.dndu * isect.dudx + isect.shading.dndv * isect.dvdx;
            Normal3f dndy =
                isect.shading.dndu * isect.dudy + isect.shading.dndv * isect.dvdy;

            // The BSDF stores the IOR of the interior of the object being
            // intersected.  Compute the relative IOR by first out by
            // assuming that the ray is entering the object.
            Float eta = 1 / bsdf.eta;
            if (Dot(wo, ns) < 0)
            {
                // If the ray isn't entering, then we need to invert the
                // relative IOR and negate the normal and its derivatives.
                eta = 1 / eta;
                ns = -ns;
                dndx = -dndx;
                dndy = -dndy;
            }

            /*
        Notes on the derivation:
        - pbrt computes the refracted ray as: \wi = -\eta \omega_o + [
        \eta (\wo \cdot \N) - \cos \theta_t ] \N It flips the normal to
        lie in the same hemisphere as \wo, and then \eta is the relative
        IOR from \wo's medium to \wi's medium.
        - If we denote the term in brackets by \mu, then we have: \wi =
        -\eta \omega_o + \mu \N
        - Now let's take the partial derivative. (We'll use "d" for
        \partial in the following for brevity.) We get: -\eta d\omega_o /
        dx + \mu dN/dx + d\mu/dx N.
        - We have the values of all of these except for d\mu/dx (using
        bits from the derivation of specularly reflected ray
        deifferentials).
        - The first term of d\mu/dx is easy: \eta d(\wo \cdot N)/dx. We
        already have d(\wo \cdot N)/dx.
        - The second term takes a little more work. We have:
           \cos \theta_i = \sqrt{1 - \eta^2 (1 - (\wo \cdot N)^2)}.
           Starting from (\wo \cdot N)^2 and reading outward, we have
        \cos^2 \theta_o, then \sin^2 \theta_o, then \sin^2 \theta_i (via
        Snell's law), then \cos^2 \theta_i and then \cos \theta_i.
        - Let's take the partial derivative of the sqrt expression. We
        get: 1 / 2 * 1 / \cos \theta_i * d/dx (1 - \eta^2 (1 - (\wo \cdot
        N)^2)).
        - That partial derivatve is equal to:
          d/dx \eta^2 (\wo \cdot N)^2 = 2 \eta^2 (\wo \cdot N) d/dx (\wo
        \cdot N).
        - Plugging it in, we have d\mu/dx =
          \eta d(\wo \cdot N)/dx - (\eta^2 (\wo \cdot N) d/dx (\wo \cdot
        N))/(-\wi \cdot N).
       */
            Vector3f dwodx = -ray.rxDirection - wo, dwody = -ray.ryDirection - wo;
            Float dDNdx = Dot(dwodx, ns) + Dot(wo, dndx);
            Float dDNdy = Dot(dwody, ns) + Dot(wo, dndy);

            Float mu = eta * Dot(wo, ns) - AbsDot(wi, ns);
            Float dmudx = (eta - (eta * eta * Dot(wo, ns)) / AbsDot(wi, ns)) * dDNdx;
            Float dmudy = (eta - (eta * eta * Dot(wo, ns)) / AbsDot(wi, ns)) * dDNdy;

            rd.rxDirection = wi - eta * dwodx + Vector3f(mu * dndx + dmudx * ns);
            rd.ryDirection = wi - eta * dwody + Vector3f(mu * dndy + dmudy * ns);
        }
        L = f * Li(rd, scene, sampler, arena, depth + 1) * AbsDot(wi, ns) / pdf;
    }
    return L;
}

} // namespace pbrt
