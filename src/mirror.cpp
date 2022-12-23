#include <nori/bsdf.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

/// Ideal mirror BRDF
class Mirror : public BSDF {
public:
    Mirror(const PropertyList &) { 
        m_type = BsdfType::BSDF_MIRROR;
		// m_tex_filename = props.getString("filename", "none");
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

    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const {
        if (Frame::cosTheta(bRec.wi) <= 0) 
            return BLACK;

        // Reflection in local coordinates
        bRec.wo = Vector3f(
            -bRec.wi.x(),
            -bRec.wi.y(),
             bRec.wi.z()
        );
        
        bRec.measure = EDiscrete;

        /* Relative index of refraction: no change */
        bRec.eta = 1.0f;
		bRec.pdf = 1.0f;

		Color3f f(1.0f);
		// if (m_hasTexture)
		// 	f = m_texture.getval(bRec.uv.x(), bRec.uv.y());

        return f / Frame::cosTheta(bRec.wi);
    }
    Color3f sample(BSDFQueryRecord& bRec, Sampler* sampler) const {
        return sample(bRec, sampler->next2D());
    }

    std::string toString() const {
        return "Mirror[]";
    }
};

NORI_REGISTER_CLASS(Mirror, "mirror");
NORI_NAMESPACE_END
