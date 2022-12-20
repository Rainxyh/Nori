#include <nori/emitter.h>
#include <nori/warp.h>
#include <nori/mesh.h>

NORI_NAMESPACE_BEGIN

class AreaEmitter : public Emitter {
public:
    AreaEmitter(const PropertyList &props) {
        m_radiance = props.getColor("radiance");
    }

    virtual std::string toString() const {
        return tfm::format(
                "AreaLight[\n"
                "  radiance = %s,\n"
                "]",
                m_radiance.toString());
    }

    virtual Color3f eval(const EmitterQueryRecord & lRec) const {
        if(!m_mesh)
            throw NoriException("There is no shape attached to this Area light!");

        throw NoriException("To implement...");
    }

    virtual Color3f sample(EmitterQueryRecord & lRec, const Point2f & sample) const {
        if(!m_mesh)
            throw NoriException("There is no shape attached to this Area light!");

        throw NoriException("To implement...");
    }

    virtual float pdf(const EmitterQueryRecord &lRec) const {
        if(!m_mesh)
            throw NoriException("There is no shape attached to this Area light!");

        throw NoriException("To implement...");
    }


    virtual Color3f samplePhoton(Ray3f &ray, const Point2f &sample1, const Point2f &sample2) const {
        throw NoriException("To implement...");
    }


protected:
    Color3f m_radiance;
};

NORI_REGISTER_CLASS(AreaEmitter, "area")
NORI_NAMESPACE_END