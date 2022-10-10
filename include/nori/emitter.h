/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#pragma once

#include <nori/object.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Convenience data structure used to pass multiple
 * parameters to the evaluation and sampling routines in \ref Emitter
 */
struct EmitterQueryRecord {
    

    /// Create a new record for sampling the Emitter
    EmitterQueryRecord(){}
};

/**
 * \brief Superclass of all emitters
 */
class Emitter : public NoriObject {
public:
    virtual void setMesh(Mesh *mesh) = 0;

    /**
     * 
     * \brief Sample the Emitter and return the importance weight (i.e. the
     * value of the Emitter * cos(theta_o) divided by the probability density
     * of the sample with respect to solid angles).
     *
     * \param eRec    A Emitter query record
     * \param sample  A uniformly distributed sample on \f$[0,1]^2\f$
     *
     * \return The Emitter value divided by the probability density of the sample.
     *         The returned value also includes the cosine foreshortening
     *         factor associated with the outgoing direction, when this 
     *         is appropriate. A zero value means that sampling failed.
     */
    virtual void sample(Sampler *sampler, Point3f &point, Normal3f &normal) const = 0;
    virtual void sample(Sampler *sampler, Point3f &point, Normal3f &normal, float &pdf) const = 0;

    /**
     * \brief Evaluate the Emitter for a pair of directions and measure
     * specified in \code eRec
     *
     * \param eRec
     *     A record with detailed information on the Emitter query
     * \return
     *     The Emitter value, evaluated for each color channel
     */
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
    virtual float pdf() const = 0;

    /**
     * \brief Return emmited radiance
     *
     * \return
     *     Radiance
     */
    virtual Color3f getRadiance() const = 0;

    /**
     * \brief Return the type of object (i.e. Mesh/Emitter/etc.) 
     * provided by this instance
     * */
    EClassType getClassType() const { return EEmitter; }
};

NORI_NAMESPACE_END
