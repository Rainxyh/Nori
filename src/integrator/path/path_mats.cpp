#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class PathMatsIntegrator : public Integrator
{
public:
    PathMatsIntegrator(const PropertyList &props) {}

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray, size_t depth) const
    {
        Color3f Le(0.f);
        float RR = 0.8f;
        if (depth > 3 && sampler->next1D() > RR)
            return BLACK;

        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return BLACK;

        Point3f x;
        Normal3f Nx;
        Vector3f wo, wi;
        x = its.p;
        Nx = its.shFrame.n;
        wo = (-ray.d).normalized();
        if (its.mesh->isEmitter())
        {
            Le = its.mesh->getEmitter()->eval(Nx, wo);
        }

        BSDFQueryRecord bRec(its.toLocal(wo));
        Color3f fr = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
        wi = its.toWorld(bRec.wo);
        if (depth > 3)
        {
            fr /= RR;
            Le /= RR;
        }
        return Le + Li(scene, sampler, Ray3f(x, wi), depth + 1) * fr;
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        float probability = 0.8f;
        size_t depth = 0;
        Color3f Lo(0.f);
        Color3f fr(1.f);
        Ray3f _ray = ray;
        Point3f x, y;
        Normal3f Nx, Ny;
        Vector3f wo, wi;
        Intersection its;
        while (1)
        {
            if (!scene->rayIntersect(_ray, its))
            {
                break;
            }
            ++depth;
            if (depth > 3)
            {
                if (sampler->next1D() > probability)
                {
                    break;
                }
                fr /= probability;
            }
            x = its.p;
            Nx = its.shFrame.n;
            wo = (-_ray.d).normalized();
            if (its.mesh->isEmitter())
            {
                if (Nx.dot(wo) > 0)
                    Lo += fr * its.mesh->getEmitter()->getRadiance();
            }

            BSDFQueryRecord bRec(its.toLocal(wo));
            fr *= its.mesh->getBSDF()->sample(bRec, sampler->next2D()); // sample a dir
            wi = its.toWorld(bRec.wo);
            _ray = Ray3f(x, wi);
        }
        return Lo;
    }

    std::string toString() const
    {
        return tfm::format(
            "PathMatsIntegrator[\n"
            "]");
    }

protected:
};

NORI_REGISTER_CLASS(PathMatsIntegrator, "path_mats");
NORI_NAMESPACE_END