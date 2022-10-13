/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/
#pragma once

#include <nori/mesh.h>
#include <Eigen/Geometry>
#include <vector>

NORI_NAMESPACE_BEGIN

/**
 * \brief Acceleration data structure for ray intersection queries
 *
 * The current implementation falls back to a brute force loop
 * through the geometry.
 */
struct Node
{
    Node(uint16_t dim) { child = (Node **)malloc(sizeof(Node *) * dim); }
    bool is_leaf = false;
    Node **child = {0};
    std::vector<std::vector<uint32_t>> triangle_list;
    BoundingBox3f bbox;
};
class Accel
{
public:
    /**
     * \brief Register a triangle mesh for inclusion in the acceleration
     * data structure
     *
     * This function can only be used before \ref build() is called
     */
    virtual ~Accel()
    {
        m_meshes.clear();
        delete m_accelNode;
    }
    void addMesh(Mesh *mesh)
    {
        m_meshes.push_back(mesh);
        m_bbox.expandBy(mesh->getBoundingBox());
    }
    void setNode(Node *m_accelNode) { this->m_accelNode = m_accelNode; }
    const Node *getNode() { return this->m_accelNode; }
    uint32_t getTreeDepth() { return this->m_maxDepth; }
    uint32_t getTreeNodeNum() { return this->m_nodeNum; }
    uint32_t getTreeLeafNum() { return this->m_leafNum; }

    /// Build the acceleration data structure (currently a no-op)
    void build();
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
    virtual bool rayIntersect(const Ray3f &ray, Intersection &its, bool shadowRay) const;
    virtual bool travel(Node *treeNode, Ray3f &ray, Intersection &its, bool shadowRay, uint32_t &hitIdx) const;
    virtual Node *build(BoundingBox3f box, std::vector<std::vector<uint32_t>> triangle_list, uint32_t depth) = 0;

protected:
    std::vector<Mesh *> m_meshes; ///< Mesh (only a single one for now)
    BoundingBox3f m_bbox;         ///< Bounding box of the entire scene
    Node *m_accelNode;
    uint16_t m_Dim = 2;
    uint32_t m_maxDepth = 0; // treeNode max depth
    uint32_t m_leafNum = 0;  // treeNode max leaf
    uint32_t m_nodeNum = 0;  // treeNode max node
};

class Octtree : public Accel
{
public:
    Octtree() { m_Dim = 8; }
    // bool rayIntersect(const Ray3f &ray, Intersection &its, bool shadowRay) const override;
    Node *build(BoundingBox3f box, std::vector<std::vector<uint32_t>> triangle_list, uint32_t depth) override;
};

class KDtree : public Accel
{
public:
    KDtree() { m_Dim = 2; }
    // bool rayIntersect(const Ray3f &ray, Intersection &its, bool shadowRay) const override;
    Node *build(BoundingBox3f box, std::vector<std::vector<uint32_t>> triangle_list, uint32_t depth) override;
};

class BVH : public Accel
{
public:
    BVH() { m_Dim = 2; }
    // bool rayIntersect(const Ray3f &ray, Intersection &its, bool shadowRay) const override;
    Node *build(BoundingBox3f box, std::vector<std::vector<uint32_t>> triangle_list, uint32_t depth) override;
};

class SAH : public Accel
{
/*
Cost of making a partition
The expected cost of ray-node intersection, given that the node's primitives are partitioned into child sets A and B is:
C = C_trav + p_A * C_A + p_B * C_B
C_trav is the cost of traversing an interior node (e.g., load data, bbox check)
C_A and C_B are the costs of intersection with the resultant child subbtrees
p_A and p_B are the probability a ray intersects the bbox of the child nodes A and B
Surface area heuristic (SAH):
C = C_trav + (S_A * N_A + S_B * N_B)/S_N * C_isect
*/
public:
    SAH() { m_Dim = 2; }
    // bool rayIntersect(const Ray3f &ray, Intersection &its, bool shadowRay) const override;
    Node *build(BoundingBox3f box, std::vector<std::vector<uint32_t>> triangle_list, uint32_t depth) override;

private:
    struct Bucket
    {
        Bucket(BoundingBox3f bbox, std::vector<std::vector<uint32_t>> triangle_idx_list, uint32_t prim_count) : bbox(bbox), triangle_idx_list(triangle_idx_list), prim_count(prim_count) {}
        virtual ~Bucket() { triangle_idx_list.clear(); }
        BoundingBox3f bbox;
        std::vector<std::vector<uint32_t>> triangle_idx_list; // 1st dim mesh_idx, 2nd dim triangle_idx
        uint32_t prim_count;
    };
    const uint8_t nBuckets = 12;
    const float SAHTravCost = 0.125f;
    const int SAHInterCost = 1;
};

NORI_NAMESPACE_END

/*
空间数据结构(四叉树/八叉树/BVH树/BSP树/k-d树)
https://www.cnblogs.com/KillerAery/p/10878367.html
数据结构	   适用情形	                   n的数量级	构造所需时间  是否可以动态更新	占用空间
四叉树	场景管理（基于地形或不含高度）、渲染	物体数量	 O(n*logn)	    是	         大（取决于区域大小和物体数量）
八叉树	场景管理、渲染	                    物体数量     O(n*logn)	    是	         大（取决于区域大小和物体数量）
BVH树	任何情形（包括物理、渲染）	          物体数量	   O(n*logn)	  是	      中（取决于物体数量）
BSP树	编辑器、复杂室内场景	             平面数量	 O(n*log²n)	     否	         大（取决于平面数量）
k-d树	需要频繁查找最近静态目标	          物体数量	  O(k*n*logn)	  否	      中（取决于物体数量）
*/