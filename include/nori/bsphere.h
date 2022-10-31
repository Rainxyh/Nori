#pragma once

#include <nori/ray.h>
#include <nori/bstructure.h>

NORI_NAMESPACE_BEGIN

template <typename _PointType>
struct TBoundingSphere : TBoundingStructure<_PointType>
{
    typedef _PointType                             PointType;
    typedef typename PointType::Scalar             Scalar;
    typedef typename PointType::VectorType         VectorType;
    typedef TBoundingStructure<PointType>          BS;

    TBoundingSphere()
    {
        reset();
    }
    ~TBoundingSphere() {}

    /// Create a collapsed bounding sphere from a single point
    TBoundingSphere(const PointType &ori)
        : ori(ori) {}
    TBoundingSphere(const PointType &ori, const Scalar radius)
        : ori(ori), radius(radius) {}

    template <typename PointType>
    TBoundingSphere &operator=(const TBoundingSphere<PointType> &bsphere){
        this->ori = bsphere.ori;
        this->radius = bsphere.radius;
        return *this;
    }

    /// Test for equality against another bounding sphere
    bool operator==(const TBoundingSphere &bsphere) const {
        return this->ori == bsphere.ori && this->radius == bsphere.radius;
    }

    /// Test for inequality against another bounding sphere
    bool operator!=(const TBoundingSphere &bsphere) const {
        return this->ori != bsphere.ori || this->radius != bsphere.radius;
    }

    void reset() {
        this->ori = PointType(0.f);
        this->radius = -1.f;
    }

    /// Calculate the n-dimensional volume of the bounding sphere
    Scalar getVolume() const {
        return 4.f / 3 * M_PI * pow(this->radius, 3);
    }

    /// Calculate the n-1 dimensional volume of the boundary
    Scalar getSurfaceArea() const {
        return 4.f * M_PI * pow(this->radius, 2);
    }

    /// Return the center point
    PointType getCenter() const {
        return this->ori;
    }

    /**
     * \brief Check whether a point lies \a on or \a inside the bounding sphere
     *
     * \param p The point to be tested
     *
     * \param strict Set this parameter to \c true if the bounding
     *               sphere boundary should be excluded in the test
     */
    bool contains(const PointType &p, bool strict = false) const {
        if (strict) {
            return (p - this->ori).squaredNorm() < this->radius;
        } else {
            return (p - this->ori).squaredNorm() <= this->radius;
        }
    }

    /**
     * \brief Check whether a specified bounding sphere lies \a on or \a within 
     * the current bounding sphere
     *
     * Note that by definition, an 'invalid' bounding sphere (where min=\f$\infty\f$
     * and max=\f$-\infty\f$) does not cover any space. Hence, this method will always 
     * return \a true when given such an argument.
     *
     * \param strict Set this parameter to \c true if the bounding
     *               sphere boundary should be excluded in the test
     */
    bool contains(const TBoundingSphere &bsphere, bool strict = false) const {
        if (strict) {
            return (this->ori - bsphere.ori).norm() + bsphere.radius < this->radius;
        } else {
            return (this->ori - bsphere.ori).norm() + bsphere.radius <= this->radius;
        }
    }
    bool contains(BS &_bsphere, bool strict = false) const {
        TBoundingSphere &bsphere = dynamic_cast<TBoundingSphere &>(_bsphere);
        return contains(bsphere, strict);
    }

    /**
     * \brief Check two axis-aligned bounding spherees for possible overlap.
     *
     * \param strict Set this parameter to \c true if the bounding
     *               sphere boundary should be excluded in the test
     *
     * \return \c true If overlap was detected.
     */
    bool overlaps(const TBoundingSphere &bsphere, bool strict = false) const {
        if (strict) {
            return (this->ori - bsphere.ori).norm() < this->radius;
        } else {
            return (this->ori - bsphere.ori).norm() <= this->radius;
        }
    }
    bool overlaps(BS &_bsphere, bool strict = false) const {
        TBoundingSphere &bsphere = dynamic_cast<TBoundingSphere &>(_bsphere);
        return overlaps(bsphere, strict);
    }

    /**
     * \brief Calculate the smallest squared distance between
     * the axis-aligned bounding sphere and the point \c p.
     */
    Scalar squaredDistanceTo(const PointType &p) const {
        return pow(distanceTo(p), 2);
    }

    /**
     * \brief Calculate the smallest distance between
     * the axis-aligned bounding sphere and the point \c p.
     */
    Scalar distanceTo(const PointType &p) const {
        Scalar result = (this->ori - p).norm() - this->radius;
        return result >= 0 ? result : 0;
    }

    /**
     * \brief Calculate the smallest square distance between
     * the axis-aligned bounding sphere and \c bsphere.
     */
    Scalar squaredDistanceTo(const TBoundingSphere &bsphere) const {
        return pow(distanceTo(bsphere), 2);
    }
    Scalar squaredDistanceTo(BS &_bsphere) const {
        TBoundingSphere &bsphere = dynamic_cast<TBoundingSphere &>(_bsphere);
        return squaredDistanceTo(bsphere);
    }

    /**
     * \brief Calculate the smallest distance between
     * the axis-aligned bounding sphere and \c bsphere.
     */
    Scalar distanceTo(const TBoundingSphere &bsphere) const {
        Scalar result = (this->ori - bsphere.ori).norm() - (this->radius + bsphere.radius);
        return result >= 0 ? result : 0;
    }
    Scalar distanceTo(BS &_bsphere) const {
        TBoundingSphere &bsphere = dynamic_cast<TBoundingSphere &>(_bsphere);
        return distanceTo(bsphere);
    }

    /**
     * \brief Check whether this is a valid bounding sphere
     *
     * A bounding sphere \c bsphere is valid when
     * \code
     * bsphere.min[dim] <= bsphere.max[dim]
     * \endcode
     * holds along each dimension \c dim.
     */
    bool isValid() const {
        return radius >= 0;
    }

    /// Check whether this bounding sphere has collapsed to a single point
    bool isPoint() const {
        return radius == 0;
    }

    /// Check whether this bounding sphere has any associated volume
    bool hasVolume() const {
        return !isPoint();
    }

    /// Clip to another bounding sphere
    void clip(const TBoundingSphere &bsphere) {
        Scalar clip_radius = (this->ori - bsphere.ori).norm() + bsphere.radius;
        if(clip_radius<this->radius){
            this->radius = clip_radius;
        }
        else {
            std::cerr<<"clip error, maybe you need expand ?"<<std::endl;
        }
    }
    void clip(BS &_bsphere) {
        TBoundingSphere &bsphere = dynamic_cast<TBoundingSphere &>(_bsphere);
        clip(bsphere);
    }

    /// Expand the bounding sphere to contain another point, fixed centor
    void expandBy(const PointType &p)
    {
        if (this->radius < -Epsilon)
        {
            this->ori = p;
            this->radius = 0.f;
        }
        else
        {
            this->radius = (this->ori - p).norm() > this->radius ? (this->ori - p).norm() : this->radius;
        }
    }

    /// Expand the bounding sphere to contain another bounding sphere, fixed centor
    void expandBy(const TBoundingSphere &bsphere)
    {
        if (this->radius < -Epsilon)
        {
            *this = bsphere;
        }
        else
        {
            this->radius = (this->ori - bsphere.ori).norm() + bsphere.radius > this->radius ? (this->ori - bsphere.ori).norm() + bsphere.radius : radius;
        }
    }
    void expandBy(BS &_bsphere)
    {
        TBoundingSphere &bsphere = dynamic_cast<TBoundingSphere &>(_bsphere);
        expandBy(bsphere);
    }

    /// Merge two bounding spherees, moved centor
    static TBoundingSphere merge(const TBoundingSphere &bsphere1, const TBoundingSphere &bsphere2) {
        PointType left, rigth, centor;
        Vector3f OC = bsphere2.ori - bsphere1.ori;
        rigth = bsphere2.ori + OC.normalized() * bsphere2.radius;
        left = bsphere1.ori - OC.normalized() * bsphere1.radius;
        centor = left + rigth / 2.f;
        Scalar merged_radius = (rigth - left).norm() / 2.f;
        return TBoundingSphere(centor, merged_radius);
    }
    // BS*  merge(BS &_bsphere1, BS &_bsphere2) {
    //     TBoundingSphere &bsphere1 = dynamic_cast<TBoundingSphere &>(_bsphere1);
    //     TBoundingSphere &bsphere2 = dynamic_cast<TBoundingSphere &>(_bsphere2);
    //     return merge(bsphere1, bsphere2);
    // }

    /// Merge bounding sphere list
    /// How to construct a sphere including any 6 points and make it the smallest ? Find the minimum and maximum points.
    static TBoundingSphere merge(const std::vector<TBoundingSphere> &bsphere_list) {
        PointType merged_min, merged_max; // 1st axis, 2nd max and min
        merged_min.setConstant(std::numeric_limits<Scalar>::infinity());
        merged_max.setConstant(-std::numeric_limits<Scalar>::infinity());
        for (size_t i = 0; i < bsphere_list.size(); ++i)
        {
            for (size_t j = 0; j < 3; ++j)
            {
                PointType min_axis, max_axis;
                max_axis = min_axis = bsphere_list[i].getCenter();
                min_axis[j] -= bsphere_list[i].radius;
                max_axis[j] += bsphere_list[i].radius;
                merged_min.cwiseMin(min_axis);
                merged_max.cwiseMin(max_axis);
            }
        }
        PointType centor = merged_min + merged_max / 2.f;
        Scalar radius = (merged_max - merged_min).norm() / 2.f;
        return TBoundingSphere(centor, radius);
    }

    /// Return a string representation of the bounding sphere
    std::string toString() const {
        if (!isValid())
            return "BoundingSphere[invalid]";
        else
            return tfm::format("BoundingSphere[ori=%s, rad=%f]", this->ori.toString(), this->radius);
    }

    bool solveQuadratic(const float a, const float b, const float c, float &x0, float &x1) const
    {
        float discr = b * b - 4 * a * c;
        if (discr < 0)
            return false;
        else if (discr == 0)
            x0 = x1 = -0.5 * b / a;
        else
        {
            float q = (b > 0) ? -0.5 * (b + sqrt(discr)) : -0.5 * (b - sqrt(discr));
            x0 = q / a;
            x1 = c / q;
        }
        if (x0 > x1)
            std::swap(x0, x1);

        return true;
    }

    bool raySphereInsersect(const Ray3f &ray, float &t0, float &t1) const
    {
        // Input(o, d, c, r)
        // Return ({REJECT, INTERSECT}, t, p)
        // l = c - o;
        // s = l.dot(d);
        // l2 = l.dot(l);
        // if (s < 0 && l2 > r2) return (REJECT, 0, 0);
        // m2 = l2 - s2;
        // if (m2 > r2) return (REJECT, 0, 0);
        // q = sqrt(r2 - m2);
        // if (l2 > r2) t = s - q;
        // else t = s + q;
        // return (INTERSECT, t, o + td);

        Scalar radius2 = this->radius * this->radius;
        Vector3f OC = this->ori - ray.o;
#if 0
        // Geometric solution
        float tc = OC.dot(ray.d);
        if (tc < 0) return false; // back to ori
        float d2 = OC.dot(OC) - tc * tc;
        if (d2 > radius2) return false;
        float th = sqrt(radius2 - d2);
        t0 = tc - th;
        t1 = tc + th;
#else
        // Algebraic solution
        // t^2 * d2 + 2t * d * CO + |CO|^2 - R^2 = 0
        Vector3f CO = -OC;
        float a = ray.d.dot(ray.d);
        float b = 2 * CO.dot(ray.d);
        float c = CO.dot(CO) - radius2;
        if (!solveQuadratic(a, b, c, t0, t1))
            return false;
#endif
        if (t0 > t1)
            std::swap(t0, t1);

        if (t0 < 0)
        {
            t0 = t1; // if t0 is less than 0 use t1 instead
            if (t0 < 0)
                return false; // If both t0 and t1 are negative, return false
        }
        return true;
    }

    /// Check if a ray intersects a bounding sphere
    bool rayIntersect(const Ray3f &ray) const
    {
        float t0, t1;
        return raySphereInsersect(ray, t0, t1);
    }

    /// Return the overlapping region of the bounding sphere and an unbounded ray
    bool rayIntersect(const Ray3f &ray, float &nearT, float &farT) const {
        float t0, t1;
        if (!raySphereInsersect(ray, t0, t1))
            return false;
        if ((nearT < t0 && t0 < farT) || (nearT < t1 && t1 < farT))
            return true;
        return false;
    }

    PointType ori;
    Scalar radius;
};

NORI_NAMESPACE_END
