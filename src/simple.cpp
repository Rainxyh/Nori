#include <nori/integrator.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class SimpleIntegrator : public Integrator
{
public:
    SimpleIntegrator(const PropertyList &props)
    {
        position = props.getPoint("position");
        energy = props.getColor("energy");
        // std::cout << "Parameter value was : \nposition:\n[" << position << "]\nenergy:\n[" << energy << "]" << std::endl;
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray, size_t depth) const
    {
        return Li(scene, sampler, ray);
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);

        Color3f radiance(0.f);
        Point3f p = its.p;
        Vector3f wi = (position - p).normalized();
        float V = 1.f;
        if (scene->rayIntersect(Ray3f(p + wi * (1e-5), wi), its))
            V = 0.f;
        float cosTheta = its.shFrame.n.dot(wi); // same with its.shFrame.cosTheta(its.shFrame.toLocal(wi)) 
        radiance += V * (energy / (4 * pow(M_PI, 2))) * std::max(0.f, cosTheta) / (p - position).dot(p - position);
        return radiance;
    }

    std::string toString() const
    {
        return tfm::format(
            "SimpleIntegrator[\n"
            "position = \"%f,%f,%f\"\n"
            "energy = \"%f,%f,%f\"\n"
            "]",
            position.x(), position.y(), position.z(),
            energy.x(), energy.y(), energy.z());
    }

protected:
    Vector3f position;
    Color3f energy;
};

NORI_REGISTER_CLASS(SimpleIntegrator, "simple");
NORI_NAMESPACE_END