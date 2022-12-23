#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class PathEmsIntegrator : public Integrator
{
public:
    PathEmsIntegrator(const PropertyList &props)
    {
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray, size_t depth) const
    {
        float RR = 0.8f;
        if (depth > 3 && sampler->next1D() > RR)
            return BLACK;

        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return BLACK;

        Point3f x, y;
        Normal3f Nx, Ny;
        Vector3f wo, wi;
        x = its.p;
        Nx = its.shFrame.n;
        wo = (-ray.d).normalized();

        Color3f Le(0.f), Ld_ems(0.f), Li_mat(0.f);
        if (its.mesh->isEmitter())
        {
            Le = its.mesh->getEmitter()->eval(Nx, wo);
        }

        /*------------------------------------------- emitter sample -------------------------------------------*/
        bool last_sample_is_ems = false;
        if (its.mesh->getBSDF()->isDiffuse()) // diffuse, sampling a emitter
        {
            Emitter *rand_emitter = scene->getRandomEmitter(); // random sample an emitter
            EmitterQueryRecord eRec;
            rand_emitter->sampleWithoutCal(eRec, sampler); // just sample and lerp point and normal
            y = eRec.p;
            Ny = eRec.n;
            wi = (y - x).normalized();
            BSDFQueryRecord bRec(its.toLocal(wo), its.toLocal(wi), ESolidAngle);
            Intersection its_ems;
            if (scene->rayIntersect(Ray3f(x, wi), its_ems) && (rand_emitter == its_ems.mesh->getEmitter())) // shadow ray can reach the certain random emitter
            {
                float G = fabs(Nx.dot(wi)) * fabs(Ny.dot(-wi)) / (y - x).squaredNorm(); // dw (solid angle) change to dA (area)
                Color3f fr = its.mesh->getBSDF()->eval(bRec);                           // BRDF term
                Color3f coff = G * fr * its_ems.mesh->surfaceArea() / scene->getEmitterPdf();   // integrate the above coefficients
                Ld_ems = coff * rand_emitter->eval(Ny, -wi);
                last_sample_is_ems = true; // Has emitter sample been done in the current step ?
            }
        }

        /*------------------------------------------- material sample -------------------------------------------*/
        BSDFQueryRecord bRec(its.toLocal(wo));
        Color3f fr = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
        wi = its.toWorld(bRec.wo);
        fr *= fmaxf(0.f, Nx.dot(wi));
        if (depth > 3)
        {
            fr /= RR;
            Le /= RR;
            Ld_ems /= RR;
        }

        /*------------------------------------------- next event estimation -------------------------------------------*/
        Li_mat = Li(scene, sampler, Ray3f(x, wi), depth + 1) * fr; // indirect radiance in the corresponding direction wi
        if (Li_mat.x() < 0 || Li_mat.y() < 0 || Li_mat.z() < 0)
            Li_mat = Color3f(0.f);

        // If light source sampling has already been performed,
        // you need to limit the contribution of the next light source.
        // Also "Next Event Estimation" as I understand it.
        if (last_sample_is_ems)
        {
            Intersection next_its;
            if (scene->rayIntersect(Ray3f(x, wi), next_its) && next_its.mesh->isEmitter())
            {
                Color3f next_Ld_ems = next_its.mesh->getEmitter()->eval(next_its.shFrame.n, -wi);
                // Since Li contains the indirect light of all subsequent steps, and
                // we only need to limit the contribution of the light source in the next step, we just need to directly subtract it.
                Li_mat -= next_Ld_ems * fr;
            }
        }

        return Le + Ld_ems + Li_mat;
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        float probability = 0.8f;
        size_t depth = 0;
        Color3f Lo(0.f), L_dir(0.f), L_indir(0.f);
        Color3f cumulative_coff(1.f);
        bool last_sample_is_ems = false;
        Ray3f _ray = ray;
        Point3f x, y;
        Normal3f Nx, Ny;
        Vector3f wo, wi;
        Intersection its;
        while (1)
        {
            if (!scene->rayIntersect(_ray, its))
                break;

            x = its.p;
            Nx = its.shFrame.n;
            wo = (-_ray.d).normalized();
            if (its.mesh->isEmitter())
            {
                if (!last_sample_is_ems)
                    L_indir += cumulative_coff * its.mesh->getEmitter()->eval(Nx, wo);
            }

            ++depth;
            if (depth > 3) // Russian Roulette
            {
                if (sampler->next1D() > probability)
                    break;
                cumulative_coff /= probability;
            }

            last_sample_is_ems = false;
            if (its.mesh->getBSDF()->isDiffuse()) // diffuse, sampling a emitter
            {
                Emitter *rand_emitter = scene->getRandomEmitter();
                EmitterQueryRecord eRec;
                rand_emitter->sample(eRec, sampler); // just sample and lerp point and normal
                y = eRec.p;
                Ny = eRec.n;
                wi = (y - x).normalized();
                BSDFQueryRecord bRec(its.toLocal(wo), its.toLocal(wi), ESolidAngle);
                Intersection its_ems;
                if (scene->rayIntersect(Ray3f(x, wi), its_ems) && (rand_emitter == its_ems.mesh->getEmitter())) // shadow ray can reach emitter
                {
                    float G = fabs(Nx.dot(wi)) * fabs(Ny.dot(-wi)) / (y - x).squaredNorm();
                    Color3f fr = its.mesh->getBSDF()->eval(bRec);
                    Color3f coff = cumulative_coff * G * fr * its_ems.mesh->surfaceArea() / scene->getEmitterPdf();
                    Color3f Li = coff * rand_emitter->eval(Ny, -wi);
                    L_dir += Li;
                    last_sample_is_ems = true;
                }
            }

            // mirror and metal
            BSDFQueryRecord bRec(its.toLocal(wo));
            cumulative_coff *= its.mesh->getBSDF()->sample(bRec, sampler->next2D());
            wi = its.toWorld(bRec.wo);
            cumulative_coff *= fmaxf(0.f, Nx.dot(wi));
            _ray = Ray3f(x, wi);
        }
        Lo = L_indir + L_dir;
        return Lo;
    }

    std::string toString() const
    {
        return tfm::format(
            "PathEmsIntegrator[\n"
            "]");
    }

protected:
};

NORI_REGISTER_CLASS(PathEmsIntegrator, "path_ems");
NORI_NAMESPACE_END