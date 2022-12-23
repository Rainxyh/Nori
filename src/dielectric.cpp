#include <nori/bsdf.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

/// Ideal dielectric BSDF
class Dielectric : public BSDF {
public:
    Dielectric(const PropertyList &propList) {
        m_type = BsdfType::BSDF_DIELECTRIC;
        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);

        // m_tex_filename = propList.getString("filename", "none");
		// if (m_tex_filename != "none")
		// {
		// 	m_hasTexture = true;
		// 	m_texture = Texture(m_tex_filename);
		// }
    }

    Color3f eval(const BSDFQueryRecord &) const {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return BLACK;
    }

    float pdf(const BSDFQueryRecord &) const {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return 0.0f;
    }

    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const
    {
        float cosThetaI = bRec.wi.z();
        float reflectivity = fresnel(cosThetaI, m_extIOR, m_intIOR, bRec.wi, bRec.wo);

        // check if reflection or refraction
		// if TIR, Fr = 1, and hence this branch is always taken.
		Color3f f(1.0f);
		// if (m_hasTexture) f = m_texture.getval(bRec.uv.x(), bRec.uv.y());
        bRec.measure = EDiscrete;
        // ----------------------------------------------------------reflect----------------------------------------------------------
        if (sample.x() < reflectivity)
        { 
            bRec.wo = Vector3f(-bRec.wi.x(), -bRec.wi.y(), bRec.wi.z());
			bRec.pdf = 1.0f;

			// The wi can be under the surface and hence negative. however, we need to take only the absolute value.
			return f / fabsf(cosThetaI);
        }
        // ----------------------------------------------------------refract----------------------------------------------------------
        bool entering = Frame::cosTheta(bRec.wi) > 0.0f;
        float eta_i = m_extIOR;
        float eta_t = m_intIOR;

        if (!entering)
            std::swap(eta_i, eta_t);

        // Compute transmitted direction
        float sini2 = Frame::sinTheta2(bRec.wi);
        float eta = eta_i / eta_t;
        float sint2 = eta * eta * sini2;

        float cost = sqrtf(std::max(0.0f, 1.0f - sint2));
        if (entering) cost = -cost;
        float sintOverSini = eta;

        // Set all the values
        bRec.wo = Vector3f(sintOverSini * -bRec.wi.x(), sintOverSini * -bRec.wi.y(), cost);
        bRec.eta = eta_t;
        bRec.measure = EDiscrete;
        bRec.pdf = 1.0f;

        // The (1-fr) term disappears because the probability of sampling this refraction is also (1-fr)
        // Hence the numerator term and the denominator term cancel out.
        //return (square(eta_t) / square(eta_i)) / (fabsf(Frame::cosTheta(bRec.wo)));
        return (square(eta_t) / square(eta_i)) * f / (fabsf(Frame::cosTheta(bRec.wo)));
    }
    Color3f sample(BSDFQueryRecord &bRec, Sampler* sampler) const
    {
        return sample(bRec, sampler->next2D());
    }

    std::string toString() const {
        return tfm::format(
            "Dielectric[\n"
            "  intIOR = %f,\n"
            "  extIOR = %f\n"
            "]",
            m_intIOR, m_extIOR);
    }
private:
    float m_intIOR, m_extIOR;
};

NORI_REGISTER_CLASS(Dielectric, "dielectric");
NORI_NAMESPACE_END
