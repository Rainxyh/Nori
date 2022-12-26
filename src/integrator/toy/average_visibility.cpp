#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

/*derives from Integrator to visualize the average visibility of surface points seen by a camera, 
while ignoring the actual material parameters (i.e. the surface's BSDF).*/
class AvIntegrator : public Integrator
{
public:
    AvIntegrator(const PropertyList &props) {
        m_raylength = props.getFloat("length");
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return WHITE;

        Point3f p = its.p;
        Vector3f wi = Warp::squareToUniformHemisphere(sampler->next2D());
        
        wi = its.toWorld(wi);
        float V = 1.f;
        if (scene->rayIntersect(Ray3f(p, wi, Epsilon, m_raylength)))
            V = 0.f;
        return Color3f(V);
    }

    std::string toString() const
    {
        return tfm::format(
            "AvIntegrator[\n"
            "]");
    }

protected:
    float m_raylength;
};

NORI_REGISTER_CLASS(AvIntegrator, "av");
NORI_NAMESPACE_END