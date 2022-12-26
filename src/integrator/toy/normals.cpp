#include <nori/integrator.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class NormalIntegrator : public Integrator
{
public:
    NormalIntegrator(const PropertyList &props)
    {
        // m_myProperty = props.getString("myProperty");
        m_myProperty = "Hello!";
        std::cout << "Parameter value was : " << m_myProperty << std::endl;
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return BLACK;

        Normal3f n = its.shFrame.n.cwiseAbs();
        return Color3f(n.x(), n.y(), n.z());
    }

    std::string toString() const
    {
        return tfm::format(
            "NormalIntegrator[\n"
            "myProperty = \"%s\"\n"
            "]",
            m_myProperty);
    }

protected:
    std::string m_myProperty;
};

NORI_REGISTER_CLASS(NormalIntegrator, "normals");
NORI_NAMESPACE_END