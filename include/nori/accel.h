/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/
#pragma once

#include <nori/mesh.h>
#include <vector>

NORI_NAMESPACE_BEGIN

/**
 * \brief Acceleration data structure for ray intersection queries
 *
 * The current implementation falls back to a brute force loop
 * through the geometry.
 */

struct OcttreeNode
{
    bool is_leaf = false;
    OcttreeNode *child[8] = {0};
    std::vector<std::vector<uint32_t>> triangle_list;
    BoundingBox3f bbox;
};

class Accel {
public:
    /**
     * \brief Register a triangle mesh for inclusion in the acceleration
     * data structure
     *
     * This function can only be used before \ref build() is called
     */
    void addMesh(Mesh *mesh);
    void setOcttree(OcttreeNode *octtree) { this->octtree = octtree; }
    const OcttreeNode *getOcttree() { return this->octtree; }
    uint32_t getOcttreeDepth() { return this->m_maxDepth; }
    uint32_t getOcttreeNode() { return this->m_node; }
    uint32_t getOcttreeLeaf() { return this->m_leaf; }

    /// Build the acceleration data structure (currently a no-op)
    void build();
    OcttreeNode *build(BoundingBox3f box, std::vector<std::vector<uint32_t>> triangle_list, uint32_t depth);
    /// Return an axis-aligned box that bounds the scene
    const BoundingBox3f &getBoundingBox() const { return m_bbox; }

    /**
     * \brief Intersect a ray against all triangles stored in the scene and
     * return detailed intersection information
     *
     * \param ray
     *    A 3-dimensional ray data structure with minimum/maximum extent
     *    information
     *
     * \param its
     *    A detailed intersection record, which will be filled by the
     *    intersection query
     *
     * \param shadowRay
     *    \c true if this is a shadow ray query, i.e. a query that only aims to
     *    find out whether the ray is blocked or not without returning detailed
     *    intersection information.
     *
     * \return \c true if an intersection was found
     */
    bool rayIntersect(const Ray3f &ray, Intersection &its, bool shadowRay) const;
    bool travel_octtree(OcttreeNode* octtree, Ray3f &ray, Intersection &its, bool shadowRay, uint32_t& hitIdx) const;


private:
    std::vector<Mesh *> m_meshes; ///< Mesh (only a single one for now)
    BoundingBox3f      m_bbox;  ///< Bounding box of the entire scene

    OcttreeNode *octtree;
    uint32_t m_maxDepth = 0; // octtree max depth
    uint32_t m_leaf = 0; // octtree max leaf
    uint32_t m_node = 0; // octtree max node
};

NORI_NAMESPACE_END
