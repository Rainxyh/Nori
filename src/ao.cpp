#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class AoIntegrator : public Integrator
{
public:
    AoIntegrator(const PropertyList &props) {}

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray, size_t depth) const
    {
        return Li(scene, sampler, ray);
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return BLACK;

        Color3f color(0.f);
        Point3f p = its.p;
        Vector3f wi = Warp::squareToCosineHemisphere(sampler->next2D());
        float cosTheta = wi.z();
        float pdf = Warp::squareToCosineHemispherePdf(wi);
        wi = its.shFrame.toWorld(wi);
        float V = 1.f;
        if (scene->rayIntersect(Ray3f(p + wi * (1e-5), wi)))
            V = 0.f;
        color += V * cosTheta * INV_PI / pdf;
        // if (fabs(1 - cosTheta * INV_PI / pdf) > 1e-5)
        //     std::cout << "dif" << std::endl;
        return color;
    }

    std::string toString() const
    {
        return tfm::format(
            "AoIntegrator[\n"
            "]");
    }

protected:
};

NORI_REGISTER_CLASS(AoIntegrator, "ao");
NORI_NAMESPACE_END