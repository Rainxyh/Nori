#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

class DiIntegrator : public Integrator
{
public:
    DiIntegrator(const PropertyList &props) {}

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray, size_t depth) const
    {
        return Li(scene, sampler, ray);
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return BLACK;
        Point3f p = its.p;
        Vector3f wo = (-ray.d).normalized(), wi;
        std::vector<Emitter*> emitters = scene->getEmitterList();
        Color3f Lo(0.f);
        for (size_t i = 0; i < emitters.size(); ++i)
        {
            EmitterQueryRecord lRec;
            lRec.ref = p;
            emitters[i]->sample(lRec, sampler);
            wi = -lRec.wi;
            float V = 1.f;

            if (scene->rayIntersect(Ray3f(p, wi, Epsilon, (1.0f - Epsilon) * lRec.dist)))
                V = 0.f;
            BSDFQueryRecord bRec(its.toLocal(wi), its.toLocal(wo), ESolidAngle);
            Color3f fr = its.mesh->getBSDF()->eval(bRec);
            Color3f Li = emitters[i]->getRadiance() * INV_FOURPI * (1.0f / (lRec.dist * lRec.dist));
            float cosTheta = its.toLocal(wi).z();

            Lo += V * fr * Li * cosTheta;
        }
        return Lo; 
    }

    std::string toString() const
    {
        return tfm::format(
            "DiIntegrator[\n"
            "]");
    }

protected:
};

NORI_REGISTER_CLASS(DiIntegrator, "direct");
NORI_NAMESPACE_END