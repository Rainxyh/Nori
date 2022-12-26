#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN
class WhittedIntegrator : public Integrator
{
public:
    WhittedIntegrator(const PropertyList &props) {}

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray, size_t depth) const
    {
        float RR = 0.8f;
        if (sampler->next1D() > RR)
            return BLACK;

        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return BLACK;

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

        if (its.mesh->getBSDF()->isDiffuse())  // diffuse
        {
            Point3f x = its.p, y;
            float G = 0.f, V = 1.f;
            Emitter *rand_emitter = scene->getRandomEmitter();
            EmitterQueryRecord eRec;
            rand_emitter->sample(eRec, sampler); // just sample and lerp point and normal
            y = eRec.p; 
            Ny = eRec.n;
            Vector3f wo = -ray.d.normalized(), wi = (y - x).normalized();
            Lr = rand_emitter->eval(Ny, -wi) * eRec.mesh->surfaceArea();

            BSDFQueryRecord bRec(its.toLocal(wi), its.toLocal(wo), ESolidAngle);
            Color3f fr = its.mesh->getBSDF()->eval(bRec);

            if (scene->rayIntersect(Ray3f(x, wi, Epsilon, (y - x).norm() - Epsilon), its)) // if no Epsilon, ceiling will be full black. because x in emitter space, but whitted doesn't consider indirect lighting so it is no matter in this example
                if (!its.mesh->isEmitter())
                    V = 0.f;

            G = V * fabs(Nx.dot(wi)) * fabs(Ny.dot(-wi)) / (y - x).squaredNorm();
            Lr = fr * G * Lr / scene->getEmitterPdf();
            if (its.mesh->getTexture())
                tex_color = its.mesh->getTexture()->getValue(its.mtlName, its.uv);
            Lr *= tex_color;
            Lo = Le + Lr;
        } else  // dielectric or mirror
        {
            Point3f x = its.p, y;
            Vector3f wo = -ray.d.normalized(), wi;
            BSDFQueryRecord bRec(its.toLocal(wo));
            Color3f c = its.mesh->getBSDF()->sample(bRec, sampler->next2D()); // wi = bRec.wo
            wi = its.toWorld(bRec.wo);
            Ray3f scatter_ray(x, wi);
            Lo = Le + Li(scene, sampler, scatter_ray, depth + 1) * c;
        }
        return Lo / RR;
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        float probability = 0.8f;
        size_t depth = 0;
        Color3f Lo(0.f), fr(1.f), tex_color(1.f);
        Ray3f _ray = ray;
        Point3f x, y;
        Normal3f Nx, Ny;
        Vector3f wo, wi;
        Intersection its;
        while (1) {
            if (!scene->rayIntersect(_ray, its)) {
                break;
            }
            ++depth;
            if (depth > 3) {
                if (sampler->next1D() > probability) {
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

            if (its.mesh->getBSDF()->isDiffuse()) // diffuse
            {
                float G = 0.f, V = 1.f;
                Emitter *rand_emitter = scene->getRandomEmitter();
                EmitterQueryRecord eRec;
                rand_emitter->sample(eRec, sampler); // just sample and lerp point and normal
                y = eRec.p;
                Ny = eRec.n;
                wi = (y - x).normalized();
                Color3f Lr = rand_emitter->eval(Ny, -wi) * eRec.mesh->surfaceArea();

                BSDFQueryRecord bRec(its.toLocal(wi), its.toLocal(wo), ESolidAngle);
                fr *= its.mesh->getBSDF()->eval(bRec);

                if (scene->rayIntersect(Ray3f(x, wi, Epsilon, (y - x).norm() - Epsilon), its)) // if no Epsilon, ceiling will be full black. because x in emitter space, but whitted doesn't consider indirect lighting so it is no matter in this example
                    if (!its.mesh->isEmitter())
                        V = 0.f;

                G = V * fabs(Nx.dot(wi)) * fabs(Ny.dot(-wi)) / (y - x).squaredNorm();
                Lr = fr * G * Lr / scene->getEmitterPdf();
                if (its.mesh->getTexture())
                    tex_color = its.mesh->getTexture()->getValue(its.mtlName, its.uv);
                Lr *= tex_color;
                Lo += Lr;
                return Lo;
            }
            else // dielectric or mirror
            {
                BSDFQueryRecord bRec(its.toLocal(wo));
                fr *= its.mesh->getBSDF()->sample(bRec, sampler->next2D()); // wi = bRec.wo
                wi = its.toWorld(bRec.wo);
                _ray = Ray3f(x, wi);
            }
        }
        return Lo;
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