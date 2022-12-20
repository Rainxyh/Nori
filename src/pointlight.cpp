#include <nori/emitter.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class PointLight : public Emitter
{
public:
    PointLight(const PropertyList& props) {
        m_power = props.getColor("power", Color3f(1.0f));
        m_position = props.getPoint3("position", Vector3f(0.f));
    }

    virtual std::string toString() const {
        return tfm::format(
            "PointLight[\n"
            "  power = %s,\n"
            "  position = %s,\n"
            "]",
            m_power.toString(),
            m_position.toString());
    }

    // Sampling is the only way to get contribution from this light source
    virtual Color3f eval(const EmitterQueryRecord & lRec) const {
        return Color3f(0.0f);
    }

    // Sampling is the only way to get light from this lightsource
    virtual Color3f sample(EmitterQueryRecord & lRec, const Point2f & sample) const {
        lRec.emitter = this;
        lRec.p = m_position;
        lRec.dist = (m_position - lRec.ref).norm();
        lRec.wi = (m_position - lRec.ref).normalized();   // wi is always chosen from reference point
        lRec.pdf = 1.0f;                                // explicitly sampling - delta pdf
        lRec.n = -lRec.wi.normalized();                 // the normal direction is the reverse direction of wi

        // Radiance is returned from sample
        // Radiance of the point light phi/(4*pi*r2)
        return m_power * INV_FOURPI * (1.0f / (lRec.dist * lRec.dist));
    }

    // The pdf of choosing a point light is zero always.
    virtual float pdf(const EmitterQueryRecord &lRec) const {
        return 0.0f;
    }

    virtual Color3f samplePhoton(Ray3f &ray, const Point2f &sample1, const Point2f &sample2) const {
        throw NoriException("samplePhoton() method not implemented yet for PointLight");
    }

private:
    Color3f m_power;
    Vector3f m_position;
};

NORI_REGISTER_CLASS(PointLight, "point");
NORI_NAMESPACE_END