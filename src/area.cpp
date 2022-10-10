#include <nori/emitter.h>
#include <nori/scene.h>
#include <nori/mesh.h>

NORI_NAMESPACE_BEGIN

class AreaLight : public Emitter
{
public:
    AreaLight(const PropertyList &props)
    {
        m_radiance = props.getColor("radiance");
    }

    void sample(Sampler *sampler, Point3f &point, Normal3f &normal) const
    {
        m_mesh->uniformSample(sampler, point, normal); // sample triangle from mesh
    }

    void sample(Sampler *sampler, Point3f &point, Normal3f &normal, float &pdf) const
    {
        m_mesh->uniformSample(sampler, point, normal, pdf); // sample triangle from mesh
    }

    Color3f eval(const Vector3f &normal, const Vector3f &wi) const
    {
        if (normal.dot(wi) <= 0)
            return Color3f(0.);
        return m_radiance;
    }

    float pdf() const
    {
        return 1.f / m_mesh->surfaceArea();
    }

    Color3f getRadiance() const
    {
        return m_radiance;
    }

    void setMesh(Mesh *mesh)
    {
        this->m_mesh = mesh;
    }

    std::string toString() const
    {
        return tfm::format(
            "AreaLight[\n"
            "m_radiance = \"%f,%f,%f\"\n"
            "]",
            m_radiance.x(), m_radiance.y(), m_radiance.z());
    }

protected:
    Color3f m_radiance;
    Mesh *m_mesh;
};

NORI_REGISTER_CLASS(AreaLight, "area");
NORI_NAMESPACE_END