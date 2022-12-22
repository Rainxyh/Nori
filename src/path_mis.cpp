#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class PathMisIntegrator : public Integrator
{
public:
    PathMisIntegrator(const PropertyList &props)
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

        float weight_mat = 0.f, weight_ems = 0.f;
        /*------------------------------------------- emitter sample -------------------------------------------*/
        if (its.mesh->getBSDF()->isDiffuse()) // diffuse, sampling a emitter
        {
            Emitter *rand_emitter = scene->getRandomEmitter(); // random sample an emitter
            EmitterQueryRecord eRec;
            rand_emitter->sample(eRec, sampler); // just sample and lerp point and normal
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
                if (fabs(Ny.dot(-wi) > Epsilon)) // except backforward ray
                    weight_ems = its_ems.mesh->surfaceArea() * scene->getEmitterPdf() * (y - x).squaredNorm() / fabs(Ny.dot(-wi));
                else
                    weight_ems = Epsilon;
                weight_mat = its.mesh->getBSDF()->pdf(bRec);

                Ld_ems *= weight_ems / (weight_ems + weight_mat);
            }
        }

        /*------------------------------------------- material sample -------------------------------------------*/
        BSDFQueryRecord bRec(its.toLocal(wo));
        Color3f fr = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
        wi = its.toWorld(bRec.wo);
        if (depth > 3)
        {
            fr /= RR;
            Le /= RR;
            Ld_ems /= RR;
        }

        /*------------------------------------------- next event estimation -------------------------------------------*/
        Intersection next_its;
        if (scene->rayIntersect(Ray3f(x, wi), next_its))
        {
            Li_mat = Li(scene, sampler, Ray3f(x, wi), depth + 1) * fr;
            if (Li_mat.x() < 0 || Li_mat.y() < 0 || Li_mat.z() < 0)
                Li_mat = Color3f(0.f);
            float next_event_estimation = 1.f;
            if (next_its.mesh->isEmitter())
            {
                weight_ems = next_its.mesh->surfaceArea() * scene->getEmitterPdf() * (next_its.p - x).squaredNorm() / fabs(next_its.shFrame.n.dot(-wi));
                weight_mat = its.mesh->getBSDF()->pdf(bRec); // may different with weight_mat in emitter sample
                next_event_estimation = weight_ems + weight_mat > 0 ? weight_mat / (weight_ems + weight_mat) : weight_ems;
                if (!its.mesh->getBSDF()->isDiffuse()) // mirror and dielectric pdf is 0, need to special treatment
                    next_event_estimation = 1.f;
                // Similar to the idea of path_ems,
                // if the next step is the indirect illumination brought by the light source,
                // the contribution needs to be evaluated in advance (NEE), and
                // the contribution of the light source in this step is taken out and
                // modified to use the contribution after importance sampling.
                Color3f next_Ld_ems = next_its.mesh->getEmitter()->eval(next_its.shFrame.n, -wi) * fr;
                Li_mat -= next_Ld_ems;
                Li_mat += next_Ld_ems * next_event_estimation;
            }
        }

        return Le + Ld_ems + Li_mat;
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        float probability = 0.8f;
        float weight_mat = 0.f, weight_ems = 0.f;
        float next_event_estimation = 1.f;
        size_t depth = 0;
        Color3f Lo(0.f), Le(0.f), L_dir(0.f), L_indir(0.f), Ld_ems(0.f);
        Color3f cumulative_coff(1.f);
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

            x = its.p;
            Nx = its.shFrame.n;
            wo = (-_ray.d).normalized();

            if (Nx.dot(wo) <= 0.f && its.mesh->getBSDF()->isDiffuse())
            {
                break;
            }

            if (its.mesh->isEmitter())
            {
                if (depth == 0)
                {
                    Le += its.mesh->getEmitter()->eval(Nx, wo);
                }
                else
                {
                    L_indir += cumulative_coff * its.mesh->getEmitter()->eval(Nx, wo) * next_event_estimation; // Except for the first time, the rest are indir radiance
                }
            }

            ++depth;
            if (depth > 3) // Russian Roulette
            {
                if (sampler->next1D() > probability)
                {
                    break;
                }
                cumulative_coff /= probability; // accumulate include BRDF and probability
            }

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
                if (scene->rayIntersect(Ray3f(x, wi), its_ems) && (rand_emitter == its_ems.mesh->getEmitter())) // shadow ray can reach the certain random emitter
                {
                    float G = fabs(Nx.dot(wi)) * fabs(Ny.dot(-wi)) / (y - x).squaredNorm();
                    Color3f fr = its.mesh->getBSDF()->eval(bRec);
                    // weight_ems include measure change from soild angle to area, which except fr and cos_theta_x
                    if (fabs(Ny.dot(-wi) > Epsilon)) // except backforward ray
                        weight_ems = its_ems.mesh->surfaceArea() * scene->getEmitterPdf() * (y - x).squaredNorm() / fabs(Ny.dot(-wi));
                    else
                        weight_ems = Epsilon;
                    Color3f coff = cumulative_coff * G * fr * its_ems.mesh->surfaceArea() / scene->getEmitterPdf(); // this its include the BRDF and prob before
                    Ld_ems = coff * rand_emitter->eval(Ny, -wi);

                    weight_mat = its.mesh->getBSDF()->pdf(bRec);
                    L_dir += Ld_ems * (weight_ems + weight_mat > 0 ? weight_ems / (weight_ems + weight_mat) : weight_ems);
                }
            }

            // Next Event Estimation
            BSDFQueryRecord bRec(its.toLocal(wo));
            cumulative_coff *= its.mesh->getBSDF()->sample(bRec, sampler->next2D());
            wi = its.toWorld(bRec.wo);
            _ray = Ray3f(x, wi);

            Intersection next_its;
            if (!scene->rayIntersect(_ray, next_its))
            {
                break;
            }
            if (next_its.mesh->isEmitter())
            {
                weight_ems = next_its.mesh->surfaceArea() * scene->getEmitterPdf() * (next_its.p - x).squaredNorm() / fabs(next_its.shFrame.n.dot(-wi));
                weight_mat = its.mesh->getBSDF()->pdf(bRec); // may different with weight_mat in emitter sample
                next_event_estimation = weight_ems + weight_mat > 0 ? weight_mat / (weight_ems + weight_mat) : weight_ems;
                if (!its.mesh->getBSDF()->isDiffuse()) // mirror and dielectric
                    next_event_estimation = 1.f;
            }
        }
        Lo = Le + L_dir + L_indir;
        return Lo;
    }

    std::string toString() const
    {
        return tfm::format(
            "PathMisIntegrator[\n"
            "]");
    }

protected:
};

NORI_REGISTER_CLASS(PathMisIntegrator, "path_mis");
NORI_NAMESPACE_END