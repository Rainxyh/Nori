#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class NprBsdf : public BSDF {
public:
    NprBsdf(const PropertyList &propList) {
        m_ambient = propList.getColor("ambient", Color3f(0.f));
        m_diffuse = propList.getColor("diffuse", Color3f(0.f));
        m_specular = propList.getColor("specular", Color3f(0.f));
        m_threshNL = propList.getFloat("nl", 0.f);
        m_threshNH = propList.getFloat("nh", 0.f);
    }

    /// Evaluate the BRDF model
    Color3f eval(const BSDFQueryRecord &bRec) const {
        /* This is a smooth BRDF -- return zero if the measure
        is wrong, or when queried for illumination on the backside */
        if (bRec.measure != ESolidAngle
            || Frame::cosTheta(bRec.wi) <= 0
            || Frame::cosTheta(bRec.wo) <= 0)
            return Color3f(0.0f);

        // compute half vector
        const Vector3f half = (bRec.wi + bRec.wo) * 0.5f;
        const float nl = bRec.wi.dot(Vector3f(0.f, 0.f, 1.f));
        const float nh = half.dot(Vector3f(0.f, 0.f, 1.f));
        if (nl < m_threshNL) return m_ambient;
        if (nh > m_threshNH) return m_specular;
        else return m_diffuse;
    }

    /// Compute the density of \ref sample() wrt. solid angles
    float pdf(const BSDFQueryRecord &bRec) const {
        return 0.f;
    }

    /// Draw a a sample from the BRDF model
    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const {
        return 0.f;
    }

    bool isDiffuse() const {
        return true;
    }

    /// Return a human-readable summary
    std::string toString() const {
        return tfm::format(
            "NprBsdf[\n"
            "  ambient = %s\n"
            "  diffuse = %s\n"
            "  specular = %s\n"
            "]", 
            m_ambient.toString(),
            m_diffuse.toString(),
            m_specular.toString());
    }

    EClassType getClassType() const { return EBSDF; }
private:
    Color3f m_ambient, m_diffuse, m_specular;   // Npr shading model.
    float m_threshNL, m_threshNH;               // thresholds for which color to send back
};

NORI_REGISTER_CLASS(NprBsdf, "nprbsdf");
NORI_NAMESPACE_END