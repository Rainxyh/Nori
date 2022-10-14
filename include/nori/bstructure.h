#pragma once

#include <nori/ray.h>

NORI_NAMESPACE_BEGIN

template <typename _PointType>
struct TBoundingStructure
{
    typedef _PointType PointType;
    typedef typename PointType::Scalar Scalar;
    typedef typename PointType::VectorType VectorType;

    // virtual void reset() =0;

    // /// Calculate the n-dimensional volume of the bounding structure
    // virtual float getVolume() const =0;

    // /// Calculate the n-1 dimensional volume of the boundary
    // virtual float getSurfaceArea() const =0;

    // /// Return the center point
    // virtual PointType getCenter() const =0;

    // /**
    //  * \brief Check whether a point lies \a on or \a inside the bounding structure
    //  *
    //  * \param p The point to be tested
    //  *
    //  * \param strict Set this parameter to \c true if the bounding
    //  *               structure boundary should be excluded in the test
    //  */
    // virtual bool contains(const PointType &p, bool strict = false) const =0;

    // /**
    //  * \brief Check whether a specified bounding structure lies \a on or \a within 
    //  * the current bounding structure
    //  *
    //  * Note that by definition, an 'invalid' bounding structure (where min=\f$\infty\f$
    //  * and max=\f$-\infty\f$) does not cover any space. Hence, this method will always 
    //  * return \a true when given such an argument.
    //  *
    //  * \param strict Set this parameter to \c true if the bounding
    //  *               structure boundary should be excluded in the test
    //  */
    // virtual bool contains(const TBoundingStructure &bstructure, bool strict = false) const =0;

    // /**
    //  * \brief Check two axis-aligned bounding structurees for possible overlap.
    //  *
    //  * \param strict Set this parameter to \c true if the bounding
    //  *               structure boundary should be excluded in the test
    //  *
    //  * \return \c true If overlap was detected.
    //  */
    // virtual bool overlaps(const TBoundingStructure &bstructure, bool strict = false) const =0;

    // /**
    //  * \brief Calculate the smallest squared distance between
    //  * the axis-aligned bounding structure and the point \c p.
    //  */
    // virtual float squaredDistanceTo(const PointType &p) const =0;

    // /**
    //  * \brief Calculate the smallest distance between
    //  * the axis-aligned bounding structure and the point \c p.
    //  */
    // virtual float distanceTo(const PointType &p) const =0;

    // /**
    //  * \brief Calculate the smallest square distance between
    //  * the axis-aligned bounding structure and \c bstructure.
    //  */
    // virtual float squaredDistanceTo(const TBoundingStructure &bstructure) const =0;

    // /**
    //  * \brief Calculate the smallest distance between
    //  * the axis-aligned bounding structure and \c bstructure.
    //  */
    // virtual float distanceTo(const TBoundingStructure &bstructure) const =0;

    // /**
    //  * \brief Check whether this is a valid bounding structure
    //  *
    //  * A bounding structure \c bstructure is valid when
    //  * \code
    //  * bstructure.min[dim] <= bstructure.max[dim]
    //  * \endcode
    //  * holds along each dimension \c dim.
    //  */
    // virtual bool isValid() const =0;

    // /// Check whether this bounding structure has collapsed to a single point
    // virtual bool isPoint() const =0;

    // /// Check whether this bounding structure has any associated volume
    // virtual bool hasVolume() const =0;

    // /// Clip to another bounding structure
    // virtual void clip(const TBoundingStructure &bstructure) =0;

    // /// Expand the bounding structure to contain another point, fixed centor
    // virtual void expandBy(const PointType &p) =0;

    // /// Expand the bounding structure to contain another bounding structure, fixed centor
    // virtual void expandBy(const TBoundingStructure &bstructure) =0;

    // /// Merge two bounding structurees, moved centor
    // virtual void merge(const TBoundingStructure &bstructure1, const TBoundingStructure &bstructure2) =0;

    // /// Return a string representation of the bounding structure
    // virtual std::string toString() const {
    //     if (!isValid())
    //         return "BoundingStructure[invalid]";
    // }

    // virtual bool rayStructureInsersect(const Ray3f &ray, float &t0, float &t1) const=0;

    // /// Check if a ray intersects a bounding structure
    // virtual bool rayIntersect(const Ray3f &ray) const=0;

    // /// Return the overlapping region of the bounding structure and an unbounded ray
    // virtual bool rayIntersect(const Ray3f &ray, float &nearT, float &farT) const =0;
};

NORI_NAMESPACE_END
