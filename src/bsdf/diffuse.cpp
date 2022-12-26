#include <nori/bsdf.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Diffuse / Lambertian BRDF model
 */
class Diffuse : public BSDF {
public:
    Diffuse(const PropertyList &propList) {
        m_type = BSDFType::BSDF_DIFFUSE;
        m_albedo = propList.getColor("albedo", Color3f(0.5f));

        // get texture if present
		// m_filename = propList.getString("filename", "none");
		// if (m_filename != "none")
		// {
		// 	m_hasTexture = true;
		// 	m_texture = Texture(m_filename);
		// }
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
    /// Evaluate the BRDF model
    Color3f eval(const BSDFQueryRecord &bRec) const {
        /* This is a smooth BRDF -- return zero if the measure
           is wrong, or when queried for illumination on the backside */
        if (bRec.measure != ESolidAngle
            || Frame::cosTheta(bRec.wi) <= 0
            || Frame::cosTheta(bRec.wo) <= 0)
            return BLACK;

        // if (m_hasTexture)
		// {
		// 	// get it from texture
		// 	return m_texture.getval(bRec.uv.x(), bRec.uv.y()) * INV_PI;
		// }
        /* The BRDF is simply the albedo / pi (albedo = radiosity / irradiance)
           albedo = Eo / Ei = (Lo * pi) / Ei -> BRDF = albedo / pi */
        return m_albedo * INV_PI;
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
    /// Compute the density of \ref sample() wrt. solid angles
    float pdf(const BSDFQueryRecord &bRec) const {
        /* This is a smooth BRDF -- return zero if the measure
           is wrong, or when queried for illumination on the backside */
        if (bRec.measure != ESolidAngle
            || Frame::cosTheta(bRec.wi) <= 0
            || Frame::cosTheta(bRec.wo) <= 0)
            return 0.0f;

        /* Importance sampling density wrt. solid angles:
           cos(theta) / pi.

           Note that the directions in 'bRec' are in local coordinates,
           so Frame::cosTheta() actually just returns the 'z' component. */
        return Warp::squareToCosineHemispherePdf(bRec.wo);
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
    /// Draw a a sample from the BRDF model
    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const {
        if (Frame::cosTheta(bRec.wi) <= 0)
            return BLACK;

        bRec.measure = ESolidAngle;

        /* Warp a uniformly distributed sample on [0,1]^2
           to a direction on a cosine-weighted hemisphere */
        bRec.wo = Warp::squareToCosineHemisphere(sample);

        /* Relative index of refraction: no change */
        bRec.eta = 1.0f;

        /* eval() / pdf() * cos(theta) = albedo. There
           is no need to call these functions. */
        return m_albedo; // eval(bRec) / pdf(bRec) * Frame::cosTheta(bRec.wo);
    }
    Color3f sample(BSDFQueryRecord &bRec, Sampler* sampler) const {
        return sample(bRec, sampler->next2D());
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
    bool isDiffuse() const {
        return true;
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
    /// Return a human-readable summary
    std::string toString() const {
        return tfm::format(
            "Diffuse[\n"
            "  albedo = %s\n"
            "]", m_albedo.toString());
    }

private:
    Color3f m_albedo;
};

NORI_REGISTER_CLASS(Diffuse, "diffuse");
NORI_NAMESPACE_END
