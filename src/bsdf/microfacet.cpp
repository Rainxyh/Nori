#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class Microfacet : public BSDF {
public:
    Microfacet(const PropertyList &propList) {
        /* RMS surface roughness */
        m_alpha = propList.getFloat("alpha", 0.1f);

        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);

        /* Albedo of the diffuse base material (a.k.a "kd") */
        m_kd = propList.getColor("kd", Color3f(0.5f));

        /* To ensure energy conservation, we must scale the 
           specular component by 1-kd. 

           While that is not a particularly realistic model of what 
           happens in reality, this will greatly simplify the 
           implementation. Please see the course staff if you're 
           interested in implementing a more realistic version 
           of this BRDF. */
        m_ks = 1 - m_kd.maxCoeff();
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
    static float DistributeBeckmann(const Vector3f &wh, float alpha)
    { // Beckmann normal distribution term
        float tanTheta = Frame::tanTheta(wh);
        float cosTheta = Frame::cosTheta(wh);
        float a = std::exp(-(tanTheta * tanTheta) / (alpha * alpha));
        float b = M_PI * alpha * alpha * std::pow(cosTheta, 4.0f);
        return a / b;
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
    static float smithBeckmannG1(const Vector3f &wv, const Vector3f &wh, float alpha)
    { // Beckmann geometric masking term
        float c = wv.dot(wh) / Frame::cosTheta(wv);
        if (c <= 0) return 0;
        float b = 1.0f / (alpha * Frame::tanTheta(wv));
        return b < 1.6f ? (3.535f * b + 2.181f * b * b) / (1.f + 2.276f * b + 2.577f * b * b) : 1;
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
    /// Evaluate the BRDF for the given pair of directions
	virtual Color3f eval(const BSDFQueryRecord &bRec) const {

		// if (bRec.measure != ESolidAngle) return BLACK;
        if (Frame::cosTheta(bRec.wi) <= 0 || Frame::cosTheta(bRec.wo) <= 0)
			return BLACK;

		Color3f diffuse = m_kd * INV_PI;

		Normal3f wh = (bRec.wi + bRec.wo).normalized();
		float D = DistributeBeckmann(wh, m_alpha);
		float F = fresnel(wh.dot(bRec.wi), m_extIOR, m_intIOR);
		float G = smithBeckmannG1(bRec.wi, wh, m_alpha) * smithBeckmannG1(bRec.wo, wh, m_alpha);

		Color3f specular = m_ks * D * F * G / (4 * (Frame::cosTheta(bRec.wi) * Frame::cosTheta(bRec.wo)));
		if (!specular.isValid()) specular = BLACK;
		return specular + diffuse;
	}
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
    /// Evaluate the sampling density of \ref sample() wrt. solid angles
    virtual float pdf(const BSDFQueryRecord &bRec) const {
		
		if (Frame::cosTheta(bRec.wo) <= 0.0f || Frame::cosTheta(bRec.wi) <= 0.0f) return 0.0f;
		
		float dpdf = (1.0f - m_ks) * Warp::squareToCosineHemispherePdf(bRec.wo);
		Normal3f wh = (bRec.wi + bRec.wo).normalized();
        /* The microsurface model samples the direction of the normal h of the microsurface, and 
           the solid angle element of the probability density is the hemisphere determined relative to the normal h of the microsurface, 
           rather than the hemisphere determined by the normal n of the macroscopic surface, 
           so the probability reflection projection from the normal of the microsurface When the light probability is obtained, 
           it needs to be multiplied by the corresponding Jacobian matrix determinant. */
		float jacobian = 0.25f / (wh.dot(bRec.wo));
		float spdf = m_ks * Warp::squareToBeckmannPdf(wh, m_alpha) * jacobian;
		if (isnan(dpdf)) dpdf = 0.0f;
		if (isnan(spdf)) spdf = 0.0f;
		return dpdf + spdf;
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
    /// Sample the BRDF
    Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const
    {
        if (Frame::cosTheta(bRec.wi) <= 0)
        {
            return BLACK;
        }
        if (_sample.x() > m_ks)
        { // diffuse
            Point2f sample((_sample.x() - m_ks) / (1.f - m_ks), _sample.y());
            bRec.wo = Warp::squareToCosineHemisphere(sample);
        }
        else
        { // specular
            Point2f sample(_sample.x() / m_ks, _sample.y());
            Vector3f wh = Warp::squareToBeckmann(sample, m_alpha);
            bRec.wo = ((2.0f * wh.dot(bRec.wi) * wh) - bRec.wi).normalized();
        }
        if (bRec.wo.z() < 0.f)
        {
            return BLACK;
        }
        // Note: Once you have implemented the part that computes the scattered
        // direction, the last part of this function should simply return the
        // BRDF value divided by the solid angle density and multiplied by the
        // cosine factor from the reflection equation, i.e.
        return eval(bRec) / pdf(bRec) * Frame::cosTheta(bRec.wo);
    }
    Color3f sample(BSDFQueryRecord& bRec, Sampler* sampler) const {
        return sample(bRec, sampler->next2D());
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
    bool isDiffuse() const {
        /* While microfacet BRDFs are not perfectly diffuse, they can be
           handled by sampling techniques for diffuse/non-specular materials,
           hence we return true here */
        return true;
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
    std::string toString() const {
        return tfm::format(
            "Microfacet[\n"
            "  alpha = %f,\n"
            "  intIOR = %f,\n"
            "  extIOR = %f,\n"
            "  kd = %s,\n"
            "  ks = %f\n"
            "]",
            m_alpha,
            m_intIOR,
            m_extIOR,
            m_kd.toString(),
            m_ks
        );
    }
    
private:
    float m_alpha;
    float m_intIOR, m_extIOR;
    float m_ks;
    Color3f m_kd;
};

NORI_REGISTER_CLASS(Microfacet, "microfacet");
NORI_NAMESPACE_END
