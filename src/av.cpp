#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

/*derives from Integrator to visualize the average visibility of surface points seen by a camera, 
while ignoring the actual material parameters (i.e. the surface's BSDF).*/
class AvIntegrator : public Integrator
{
public:
    AvIntegrator(const PropertyList &props) {}

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray, size_t depth) const
    {
        return Li(scene, sampler, ray);
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);

        Color3f color(1.f);
        Point3f p = its.p;
        Vector3f wi = Warp::squareToCosineHemisphere(sampler->next2D());
        wi = its.shFrame.toWorld(wi);
        float V = 1.f;
        if (scene->rayIntersect(Ray3f(p + wi * (1e-5), wi)))
            V = 0.f;
        return color * V;
    }

    std::string toString() const
    {
        return tfm::format(
            "AvIntegrator[\n"
            "]");
    }

protected:
};

NORI_REGISTER_CLASS(AvIntegrator, "av");
NORI_NAMESPACE_END