#pragma once

#include <nori/object.h>

NORI_NAMESPACE_BEGIN

enum class EmitterType
{
	EMITTER_POINT,
	EMITTER_DISTANT_DISK,
	EMITTER_AREA,
	EMITTER_ENVIRONMENT,
	EMITTER_UNKNOWN
};

/**
 * \brief Convenience data structure used to pass multiple
 * parameters to the evaluation and sampling routines in \ref Emitter
 */
struct EmitterQueryRecord {
    /// Pointer to the sampled emitter
    const Emitter* emitter;
    /// Pointer to the certain mesh of emitter
    const Mesh* mesh;
    /// Origin point from which we sample the emitter
    Point3f ref;
    /// Sampled position on the light source
    Point3f p;
    /// Associated surface normal
    Normal3f n;
    /// Solid angle density wrt. 'ref'
    float pdf = 1.f;
    /// Direction vector from 'ref' to 'p'
    Vector3f wi;
    /// Distance between 'ref' and 'p'
    float dist;

    /// Create an unitialized query record
    EmitterQueryRecord() : emitter(nullptr) {}

    /// Create a new query record that can be used to sample a emitter
    EmitterQueryRecord(const Point3f& ref) : ref(ref) {}

    /**
     * \brief Create a query record that can be used to query the
     * sampling density after having intersected an area emitter
     */
    EmitterQueryRecord(const Emitter* emitter,
                       const Point3f& ref,
                       const Point3f& p,
                       const Normal3f& n)
        : emitter(emitter), ref(ref), p(p), n(n) {
        wi = p - ref;
        dist = wi.norm();
        wi /= dist;
    }

    /// Return a human-readable string summary
    std::string toString()
        const;  // Due to the use of the emitter class, pre-declaration or
                // post-definition must be performed
};

/**
 * \brief Superclass of all emitters
 */
class Emitter : public NoriObject {
   public:
    /**
     * \brief Set the mesh if the emitter is attached to a mesh
     * */
    void setMesh(Mesh* mesh) { m_mesh = mesh; }

    EmitterType getEmitterType() const { return m_type; }

    /**
     *
     * \brief Sample the Emitter and return the importance weight (i.e. the
     * value of the Emitter * cos(theta_o) divided by the probability density
     * of the sample with respect to solid angles).
     *
     * \param eRec    A Emitter query record
     * \param sample  A uniformly distributed sample on \f$[0,1]^2\f$
     *
     * \return The Emitter value divided by the probability density of the
     * sample. The returned value also includes the cosine foreshortening factor
     * associated with the outgoing direction, when this is appropriate. A zero
     * value means that sampling failed.
     */
    virtual void sampleWithoutCal(EmitterQueryRecord& lRec, Sampler* sampler) const = 0;
    virtual Color3f sample(EmitterQueryRecord &lRec, Sampler* sampler) const = 0;

    /**
     * \brief Evaluate the Emitter for a pair of directions and measure
     * specified in \code eRec
     *
     * \param eRec
     *     A record with detailed information on the Emitter query
     * \return
     *     The Emitter value, evaluated for each color channel
     */
    virtual Color3f eval(const EmitterQueryRecord& lRec) const = 0;
    virtual Color3f eval(const Vector3f& normal, const Vector3f& wi) const = 0;

    /**
     * \brief Compute the probability of sampling \c eRec.wo
     * (conditioned on \c eRec.wi).
     *
     * This method provides access to the probability density that
     * is realized by the \ref sample() method.
     *
     * \param eRec
     *     A record with detailed information on the Emitter query
     *
     * \return
     *     A probability/density value expressed with respect
     *     to the specified measure
     */
    virtual float pdf(const EmitterQueryRecord &lRec) const = 0;

    /**
     * \brief Return emmited radiance
     *
     * \return
     *     Radiance
     */
    virtual Color3f getRadiance() const = 0;

    /// Sample a photon
    virtual Color3f samplePhoton(Ray3f& ray, Sampler* sampler) const {
        throw NoriException("Emitter::samplePhoton(): not implemented!");
    }

    /**
     * \brief Return the type of object (i.e. Mesh/Emitter/etc.)
     * provided by this instance
     * */
    EClassType getClassType() const { return EEmitter; }

   protected:
    /// Pointer to the mesh if the emitter is attached to a mesh
    Mesh* m_mesh = nullptr;
    EmitterType m_type;
};

inline std::string EmitterQueryRecord::toString() const {
    return tfm::format(
        "EmitterQueryRecord[\n"
        "  emitter = \"%s\",\n"
        "  ref = %s,\n"
        "  p = %s,\n"
        "  n = %s,\n"
        "  pdf = %f,\n"
        "  wi = %s,\n"
        "  dist = %f\n"
        "]",
        indent(emitter->toString()), ref.toString(), p.toString(), n.toString(),
        pdf, wi.toString(), dist);
}

NORI_NAMESPACE_END