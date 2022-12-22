#include <nori/emitter.h>
#include <nori/mesh.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class AreaLight : public Emitter {
   public:
    AreaLight(const PropertyList& props) {
        m_type = EmitterType::EMITTER_AREA;
        m_radiance = props.getColor("radiance");
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
	// We don't assume anything about the visibility of points specified in 'ref' and 'p' in the EmitterQueryRecord.
    virtual Color3f eval(const Vector3f &normal, const Vector3f &wi) const {
        if (!m_mesh)
            throw NoriException(
                "There is no shape attached to this Area light!");
		// This function call can be done by bsdf sampling routines.
		// Hence the ray was already traced for us - i.e a visibility test was already performed.
		// Hence just check if the associated normal in emitter query record and incoming direction are not backfacing
        if (normal.dot(wi) > 0.0f)
            return m_radiance;
        else
            return BLACK;
    }
    virtual Color3f eval(const EmitterQueryRecord& lRec) const {
        return eval(lRec.n, lRec.wi);
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
    virtual Color3f sample(EmitterQueryRecord & lRec, Sampler* sampler) const {
        if(!m_mesh)
            throw NoriException("There is no shape attached to this Area light!");

		// Sample the underlying mesh for a position and normal.
        m_mesh->samplePosition(sampler, lRec.p, lRec.n);

		// Construct the EmitterQueryRecord structure.
		lRec.wi = (lRec.ref - lRec.p).normalized();
		lRec.emitter = this;
        lRec.mesh = m_mesh;
		lRec.dist = (lRec.p - lRec.ref).norm();
		lRec.pdf = pdf(lRec);
		// Return the appropriately weighted radiance term back
		// NOTE: We are not checking visibility here. It's the integrator's responsibility to check for the shadow ray test.
		if(lRec.pdf != 0.0f || fabsf(lRec.pdf) != INFINITY) return eval(lRec) / lRec.pdf;
		else return 0.0f;
    }
    virtual void sampleWithoutCal(EmitterQueryRecord & lRec, Sampler *sampler) const {
        m_mesh->uniformSample(sampler, lRec.p, lRec.n, lRec.pdf); // sample triangle from mesh
        lRec.mesh = m_mesh;
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
	// Returns probability with respect to solid angle given by all the information inside the emitterqueryrecord.
	// Assumes all information about the intersection point is already provided inside.
	// WARNING: Use with care. Malformed EmitterQueryRecords can result in undefined behavior. Plus no visibility is considered.
    virtual float pdf(const EmitterQueryRecord& lRec) const {
        if (!m_mesh)
            throw NoriException(
                "There is no shape attached to this Area light!");

        Vector3f inv_wi = lRec.wi;
        float costheta_here = fabsf(lRec.n.dot(inv_wi));
        // Contains the conversion from differential solid angle to differential area
        float pW = lRec.dist * lRec.dist / (costheta_here * m_mesh->surfaceArea()); 
        if (isnan(pW) || fabsf(pW) == INFINITY)
            return 0.0f;
        return pW;
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
    virtual Color3f getRadiance() const {
        return m_radiance;
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
    virtual Color3f samplePhoton(Ray3f &ray, Sampler* sampler) const {
		if (!m_mesh)
			throw NoriException("There is no shape attached to this area light");

		// Sample a point to emit from the mesh
		EmitterQueryRecord eRec;
		m_mesh->samplePosition(sampler, eRec.p, eRec.n);
		
		// Sample a direction
		Vector3f wi = Warp::squareToCosineHemisphere(sampler->next2D());

		Frame my_frame(eRec.n);
		Vector3f xfm_wi = my_frame.toWorld(wi);

		// global pdf
		float _pdf = (1.0f / m_mesh->surfaceArea()) * (Warp::squareToCosineHemispherePdf(wi));

		// Get the ray out.
		ray = Ray3f(eRec.p, xfm_wi, Epsilon, INFINITY);

		// Return power
		return M_PI * m_mesh->surfaceArea() * m_radiance;
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
    // Get the parent mesh
    void setParent(NoriObject* parent) {
        auto type = parent->getClassType();
        if (type == EMesh)
            m_mesh = static_cast<Mesh*>(parent);
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
    virtual std::string toString() const {
        return tfm::format(
            "AreaLight[\n"
            "  radiance = %s,\n"
            "]",
            m_radiance.toString());
    }

   protected:
    Color3f m_radiance;
};

NORI_REGISTER_CLASS(AreaLight, "area")
NORI_NAMESPACE_END