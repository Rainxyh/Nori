#include <nori/emitter.h>
#include <nori/warp.h>
#include <nori/scene.h>
#include <nori/mesh.h>

NORI_NAMESPACE_BEGIN

class PointLight : public Emitter
{
public:
    PointLight(const PropertyList& props) {
        m_type = EmitterType::EMITTER_POINT;
        m_power = props.getColor("power", Color3f(1.0f));
        m_position = props.getPoint("position", Vector3f(0.f));
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
    // Sampling is the only way to get contribution from this light source
    virtual Color3f eval(const Vector3f &normal, const Vector3f &wi) const {
        return Color3f(0.0f);
    }
    virtual Color3f eval(const EmitterQueryRecord &lRec) const {
        return eval(lRec.n, lRec.wi);
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
    // Sampling is the only way to get light from this lightsource
    virtual Color3f sample(EmitterQueryRecord& lRec, Sampler* sampler) const {
        lRec.emitter = this;
        lRec.mesh = m_mesh;
        lRec.p = m_position;
        lRec.dist = (lRec.ref - m_position).norm();
        lRec.wi = (lRec.ref - m_position).normalized();   // wi is always chosen from reference point
        lRec.pdf = 1.0f;                                  // explicitly sampling - delta pdf
        lRec.n = lRec.wi.normalized();                    // the normal direction is the reverse direction of wi

        // Radiance is returned from sample
        // Radiance of the point light phi/(4*pi*r2)
        return m_power * INV_FOURPI * (1.0f / (lRec.dist * lRec.dist));
    }
    virtual void sampleWithoutCal(EmitterQueryRecord & lRec, Sampler *sampler) const {
        sample(lRec, sampler);
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
    // The pdf of choosing a point light is zero always.
    float pdf(const EmitterQueryRecord& lRec) const {
        // We can never sample the pointlight through a random process without explicit connection.
        return 0.0f;
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
    virtual Color3f getRadiance() const {
        return m_power;
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
    virtual Color3f samplePhoton(Ray3f &ray, const Point2f &sample1, const Point2f &sample2) const {
        throw NoriException("samplePhoton() method not implemented yet for PointLight");
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
    virtual std::string toString() const {
        return tfm::format(
            "PointLight[\n"
            "  power = %s,\n"
            "  position = %s,\n"
            "]",
            m_power.toString(),
            m_position.toString());
    }

private:
    Color3f m_power;
    Vector3f m_position;
};

NORI_REGISTER_CLASS(PointLight, "point");
NORI_NAMESPACE_END