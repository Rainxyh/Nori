#include <nori/emitter.h>
#include <nori/frame.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

/**
	This class specifies a distant disk light which emits light in a cone of directions.
*/
class DistantDisk : public Emitter
{
public:
	DistantDisk(const PropertyList & props){
		m_type = EmitterType::EMITTER_DISTANT_DISK;
		m_radiance = props.getColor("radiance");
		m_thetaA = degToRad(props.getFloat("thetaA"));
		m_localToWorld = props.getTransform("toWorld", Transform());
		m_worldToLocal = m_localToWorld.getInverseMatrix();

		// Compute cosThetaMax that will be used for the hemispherical cap sampling
		m_cosThetaMax = cosf(m_thetaA);
	}
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
	virtual Color3f eval(const Vector3f &normal, const Vector3f &wi) const {
		// Return radiance only if it's within the accepted limits.
		Vector3f world_dir = wi;
		Vector3f local_dir = m_worldToLocal * world_dir;
		float cos_theta = Frame::cosTheta(local_dir);
		if (cos_theta < m_cosThetaMax) return m_radiance;
		else return 0.0f;
	}
	virtual Color3f eval(const EmitterQueryRecord & lRec) const {
		return eval(lRec.n, lRec.wi);
	}
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
	virtual Color3f sample(EmitterQueryRecord & lRec, Sampler* sampler) const
	{
		// Sample in local coordinate frame
		Vector3f sampled_dir = Warp::squareToUniformSphereCap(sampler->next2D(), m_cosThetaMax);
		float sampled_pdf = Warp::squareToUniformSphereCapPdf(sampled_dir, m_cosThetaMax);
		
		// Convert to world coordinate frame
		lRec.wi = (m_localToWorld * sampled_dir);
		lRec.pdf = sampled_pdf;
		lRec.dist = INFINITY;
		lRec.emitter = this;
		lRec.mesh = m_mesh;
		lRec.n = m_localToWorld * Vector3f(0.0f, 0.0f, 1.0f);
		
		// Appropriately scale the radiance and return back.
		if (sampled_pdf > 0.0f) return m_radiance / sampled_pdf;
		else return 0.0f;
	}
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
	virtual float pdf(const EmitterQueryRecord & lRec) const
	{
		// Compute the pdf of sampling the direction.
		Vector3f world_dir = lRec.wi;
		Vector3f local_dir = m_worldToLocal * world_dir;
		return Warp::squareToUniformSphereCapPdf(local_dir, m_cosThetaMax);
	}
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
    virtual Color3f getRadiance() const {
        return m_radiance;
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
    virtual Color3f samplePhoton(Ray3f &ray, const Point2f &sample1, const Point2f &sample2) const {
        throw NoriException("samplePhoton() method not implemented yet for PointLight");
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
	std::string toString() const{
		return tfm::format("DistantDisk[Radiance={%f,%f,%f}\n thetaA:%f]", m_radiance.x(), m_radiance.y(), m_radiance.z(), m_thetaA);
	}

private:
	Color3f   m_radiance;
	float     m_thetaA;
	float     m_cosThetaMax;
	Transform m_localToWorld;
	Transform m_worldToLocal;
};

NORI_REGISTER_CLASS(DistantDisk, "distantdisk")
NORI_NAMESPACE_END
