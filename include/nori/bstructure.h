#pragma once

#include <nori/ray.h>

NORI_NAMESPACE_BEGIN

template <typename _PointType>
struct TBoundingStructure
{
    typedef _PointType PointType;
    typedef typename PointType::Scalar Scalar;
    typedef typename PointType::VectorType VectorType;

protected:
    TBoundingStructure() {} // If an abstract class requires a constructor, it should be declared "protected".

    virtual ~TBoundingStructure() = 0; // A function declaration cannot provide both a pure-specifier and a definition

public:
    virtual void reset() = 0;

    /// Calculate the n-dimensional volume of the bounding structure
    virtual Scalar getVolume() const = 0;

    /// Calculate the n-1 dimensional volume of the boundary
    virtual Scalar getSurfaceArea() const = 0;

    /// Return the center point
    virtual PointType getCenter() const = 0;

    /**
     * \brief Check whether a point lies \a on or \a inside the bounding structure
     *
     * \param p The point to be tested
     *
     * \param strict Set this parameter to \c true if the bounding
     *               structure boundary should be excluded in the test
     */
    virtual bool contains(const PointType &p, bool strict = false) const = 0;

    /**
     * \brief Check whether a specified bounding structure lies \a on or \a within 
     * the current bounding structure
     *
     * Note that by definition, an 'invalid' bounding structure (where min=\f$\infty\f$
     * and max=\f$-\infty\f$) does not cover any space. Hence, this method will always 
     * return \a true when given such an argument.
     *
     * \param strict Set this parameter to \c true if the bounding
     *               structure boundary should be excluded in the test
     */
    virtual bool contains(TBoundingStructure &bstructure, bool strict = false) const = 0;

    /**
     * \brief Check two axis-aligned bounding structurees for possible overlap.
     *
     * \param strict Set this parameter to \c true if the bounding
     *               structure boundary should be excluded in the test
     *
     * \return \c true If overlap was detected.
     */
    virtual bool overlaps(TBoundingStructure &bstructure, bool strict = false) const = 0;

    /**
     * \brief Calculate the smallest squared distance between
     * the axis-aligned bounding structure and the point \c p.
     */
    virtual Scalar squaredDistanceTo(const PointType &p) const = 0;

    /**
     * \brief Calculate the smallest distance between
     * the axis-aligned bounding structure and the point \c p.
     */
    virtual Scalar distanceTo(const PointType &p) const = 0;

    /**
     * \brief Calculate the smallest square distance between
     * the axis-aligned bounding structure and \c bstructure.
     */
    virtual Scalar squaredDistanceTo(TBoundingStructure &bstructure) const = 0;

    /**
     * \brief Calculate the smallest distance between
     * the axis-aligned bounding structure and \c bstructure.
     */
    virtual Scalar distanceTo(TBoundingStructure &bstructure) const = 0;

    /**
     * \brief Check whether this is a valid bounding structure
     *
     * A bounding structure \c bstructure is valid when
     * \code
     * bstructure.min[dim] <= bstructure.max[dim]
     * \endcode
     * holds along each dimension \c dim.
     */
    virtual bool isValid() const = 0;

    /// Check whether this bounding structure has collapsed to a single point
    virtual bool isPoint() const = 0;

    /// Check whether this bounding structure has any associated volume
    virtual bool hasVolume() const = 0;

    /// Clip to another bounding structure
    virtual void clip(TBoundingStructure &bstructure) = 0;

    /// Expand the bounding structure to contain another point, fixed centor
    virtual void expandBy(const PointType &p) = 0;

    /// Expand the bounding structure to contain another bounding structure, fixed centor
    virtual void expandBy(TBoundingStructure &bstructure) = 0;

    /// Merge two bounding structurees, moved centor
    // virtual TBoundingStructure* merge(TBoundingStructure &bstructure1, TBoundingStructure &bstructure2) = 0;

    /// Return a string representation of the bounding structure
    virtual std::string toString() const {
        if (!isValid())
            return "BoundingStructure[invalid]";
        return "BoundingStructure[valid]";
    }

    /// Check if a ray intersects a bounding structure
    virtual bool rayIntersect(const Ray3f &ray) const = 0;

    /// Return the overlapping region of the bounding structure and an unbounded ray
    virtual bool rayIntersect(const Ray3f &ray, float &nearT, float &farT) const = 0;
};

template <typename _PointType>
TBoundingStructure<_PointType>::~TBoundingStructure()
{
    // Usually a pure virtual function does not need a function body,
    // because we generally do not call this function of the abstract class,
    // but only call the corresponding function of the derived class,
    // but the pure virtual function of the parent class needs a function body,
    // because we know that when the subclass When inheriting the parent class,
    // if the parent class has a pure virtual function,
    // the child class needs to implement the function, otherwise an error will be reported.
    // Note that even if it has a body, the function must still be overridden by any concrete classes derived from Abstract. 
}

NORI_NAMESPACE_END