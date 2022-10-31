#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN
class WhittedIntegrator : public Integrator
{
public:
    WhittedIntegrator(const PropertyList &props)
    {
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray, size_t depth) const
    {
        return Li(scene, sampler, ray);
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        float RR = 0.9f;  // Russian Roulette
        if (sampler->next1D() > RR)
            return Color3f(0.f);

        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);

        Color3f Le(0.f), Lr(0.f), Lo(0.f), tex_color(1.f);
        Normal3f Nx = its.shFrame.n, Ny;
        if (its.mesh->isEmitter())
        {
            if (Nx.dot(-ray.d))
            {
                Le = its.mesh->getEmitter()->getRadiance(); // self-luminous
                if (its.mesh->getTexture())
                    tex_color = its.mesh->getTexture()->getValue(its.mtlName, its.uv);
                Le *= tex_color;
            }
        }

        if (its.mesh->getBSDF()->isDiffuse()) // diffuse
        {
            Point3f x = its.p, y;
            float G = 0.f, V = 1.f;
            Emitter *random_emitter = scene->getRandomEmitter();
            random_emitter->sample(sampler, y, Ny);
            Vector3f wo = -ray.d.normalized(), wi = (y - x).normalized();
            Lr = random_emitter->eval(Ny, -wi) / random_emitter->pdf();

            BSDFQueryRecord bRec(its.toLocal(wi), its.toLocal(wo), ESolidAngle);
            Color3f fr = its.mesh->getBSDF()->eval(bRec);

            if (scene->rayIntersect(Ray3f(x, wi, Epsilon, (y - x).norm() - Epsilon), its)) // if no Epsilon, ceiling will be full black. because x in emitter space, but whitted doesn't consider indirect lighting so it is no matter in this example
                if (!its.mesh->isEmitter())
                    V = 0.f;
            
            G = V * fabs(Nx.dot(wi)) * fabs(Ny.dot(-wi)) / (y - x).squaredNorm();
            Lr = fr * G * Lr / scene->getEmitterPdf() / RR;
            if (its.mesh->getTexture())
                tex_color = its.mesh->getTexture()->getValue(its.mtlName, its.uv);
            Lr *= tex_color;
            Lo = Le + Lr;
            return Lo ;
        }
        else // dielectric or mirror
        {
            Point3f x = its.p, y;
            Vector3f wo = -ray.d.normalized(), wi;
            BSDFQueryRecord bRec(its.toLocal(wo));
            Color3f c = its.mesh->getBSDF()->sample(bRec, sampler->next2D()); // wi = bRec.wo
            wi = its.toWorld(bRec.wo);
            Ray3f scatter_ray(x, wi);
            return Li(scene, sampler, scatter_ray) * c / RR;
        }
        return Color3f(0.f);
    }

    std::string toString() const
    {
        return tfm::format(
            "WhittedIntegrator[\n"
            "]");
    }

protected:
};

NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");
NORI_NAMESPACE_END