/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/accel.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

void Accel::addMesh(Mesh *mesh)
{
    m_meshes.push_back(mesh);
    m_bbox.expandBy(mesh->getBoundingBox());
}

void Accel::build()
{
    /* Nothing to do here for now */
}

OcttreeNode *Accel::build(BoundingBox3f box, std::vector<std::vector<uint32_t>> triangle_list, uint32_t depth)
{
    if (!triangle_list.size())
        return nullptr;

    this->m_maxDepth = std::max(m_maxDepth, depth);

    OcttreeNode *node = new OcttreeNode();
    node->triangle_list = triangle_list; // 1st dim meshes pre scene, 2nd dim triangles pre mesh
    node->bbox = box;
    uint32_t triangle_num = 0;
    for (size_t i = 0; i < m_meshes.size(); ++i)
        triangle_num += triangle_list[i].size(); // total triangle number in this scene
        
    if (triangle_num < 70) // leaf node
    {
        node->is_leaf = true;
        ++this->m_leaf;
        for (size_t i = 0; i < 8; ++i)
            node->child[i] = nullptr;
        return node;
    }

    ++this->m_node; // internal node

    BoundingBox3f *child_bbox_list[8];
    for (size_t i = 0; i < 8; ++i) // 8 vertices and midpoints can be divided into 8 sub-bounding boxes
    {
        Vector3f minPoint;
        Vector3f maxPoint;
        Vector3f center = box.getCenter();
        Vector3f corner = box.getCorner(i);
        minPoint = center.cwiseMin(corner); // The diagonal line can represent a cube, and the maximum and minimum points of each dimension 
        maxPoint = center.cwiseMax(corner); // are taken as the maximum and minimum endpoints of the cube.
        child_bbox_list[i] = new BoundingBox3f(minPoint, maxPoint);
    }

    // Traverse all triangles and divide the triangles into corresponding sub-bounding boxes according to their positions
    std::vector<std::vector<uint32_t>> regional_division_triangle_list[8];
    for (size_t i = 0; i < 8; ++i)
    { // regional division
        for (size_t j = 0; j < m_meshes.size(); ++j)
        { // travel all meshes
            regional_division_triangle_list[i].push_back(std::vector<uint32_t>());
            for (size_t k = 0; k < node->triangle_list[j].size(); ++k)
            { // travel all triangles from certain mesh
                BoundingBox3f triangle_bbox = m_meshes[j]->getBoundingBox(node->triangle_list[j][k]); // get current triangle's bounding box
                if (child_bbox_list[i]->overlaps(triangle_bbox)) // if current triangle's bounding box in current regional division
                {                                                        // if triangle overlaps sub-node i
                    regional_division_triangle_list[i][j].push_back(node->triangle_list[j][k]); // add face_index to triangle_list
                }
            }
        }
    }

    // Recursion build sub-tree
    for (size_t i = 0; i < 8; ++i)
    {
        node->child[i] = build(*child_bbox_list[i], regional_division_triangle_list[i], depth + 1);
        delete child_bbox_list[i];
    }

    return node;
}

bool Accel::travel_octtree(OcttreeNode *octtree, Ray3f &ray, Intersection &its, bool shadowRay, uint32_t &hitIdx) const
{
    if (!octtree || !octtree->bbox.rayIntersect(ray)) // empty tree node or not hit any bounding box
        return false;

    bool is_hit = false;
    if (octtree->is_leaf) // process leaf node
    {
        for (size_t i = 0; i < octtree->triangle_list.size(); ++i)
        { // travel meshes
            for (size_t j = 0; j < octtree->triangle_list[i].size(); j++)
            { // travel triangles
                float u, v, t;
                if (m_meshes[i]->rayIntersect(octtree->triangle_list[i][j], ray, u, v, t))
                {
                    /* An intersection was found! Can terminate
                    immediately if this is a shadow ray query */
                    if (shadowRay)
                        return true;
                    its.t = ray.maxt = t;
                    its.uv = Point2f(u, v);
                    its.mesh = m_meshes[i];
                    hitIdx = octtree->triangle_list[i][j];
                    is_hit = true;
                }
            }
        }
    }
    else // recursively process internal nodes
    {
        std::pair<OcttreeNode *, float> rank_list[8];
        for (size_t i = 0; i < 8; ++i)
        {
            if (octtree->child[i])
            {
                rank_list[i].first = octtree->child[i];
                rank_list[i].second = octtree->child[i]->bbox.distanceTo(ray.o);
            }
        }
        std::sort(rank_list, rank_list + 8, [](const auto &l, const auto &r)
                  { return l.second < r.second; });

        for (size_t i = 0; i < 8; ++i)
        {
            if (rank_list[i].first)
            {
                is_hit |= travel_octtree(rank_list[i].first, ray, its, shadowRay, hitIdx);
                if (is_hit) // problems can arise when triangles are too dense
                    break;
            }
        }
    }
    return is_hit;
}

bool Accel::rayIntersect(const Ray3f &ray_, Intersection &its, bool shadowRay) const
{
    bool foundIntersection = false; // Was an intersection found so far?
    uint32_t hitIdx = (uint32_t)-1; // Triangle index of the closest intersection

    Ray3f ray(ray_); /// Make a copy of the ray (we will need to update its '.maxt' value)

    /* Brute force search through all triangles */
    // for (uint32_t idx = 0; idx < m_meshes->getTriangleCount(); ++idx) {
    //     float u, v, t;
    //     if (m_meshes->rayIntersect(idx, ray, u, v, t)) {
    //         /* An intersection was found! Can terminate
    //            immediately if this is a shadow ray query */
    //         if (shadowRay)
    //             return true;
    //         its.t = ray.maxt = t;
    //         its.uv = Point2f(u, v);
    //         its.mesh = m_meshes;
    //         hitIdx = idx;
    //         foundIntersection = true;
    //     }
    // }

    /*octtree acceleration structure*/
    foundIntersection = travel_octtree(octtree, ray, its, shadowRay, hitIdx);

    if (foundIntersection)
    {
        /* At this point, we now know that there is an intersection,
           and we know the triangle index of the closest such intersection.

           The following computes a number of additional properties which
           characterize the intersection (normals, texture coordinates, etc..)
        */

        /* Find the barycentric coordinates */
        Vector3f bary;
        bary << 1 - its.uv.sum(), its.uv;

        /* References to all relevant mesh buffers */
        const Mesh   *mesh = its.mesh;
        const MatrixXf &V  = mesh->getVertexPositions();
        const MatrixXf &N  = mesh->getVertexNormals();
        const MatrixXf &UV = mesh->getVertexTexCoords();
        const MatrixXu &F  = mesh->getIndices();

        /* Vertex indices of the triangle */
        uint32_t idx0 = F(0, hitIdx), idx1 = F(1, hitIdx), idx2 = F(2, hitIdx);

        Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);

        /* Compute the intersection positon accurately
           using barycentric coordinates */
        its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;

        /* Compute proper texture coordinates if provided by the mesh */
        if (UV.size() > 0)
            its.uv = bary.x() * UV.col(idx0) +
                     bary.y() * UV.col(idx1) +
                     bary.z() * UV.col(idx2);

        /* Compute the geometry frame */
        its.geoFrame = Frame((p1 - p0).cross(p2 - p0).normalized());

        if (N.size() > 0)
        {
            /* Compute the shading frame. Note that for simplicity,
               the current implementation doesn't attempt to provide
               tangents that are continuous across the surface. That
               means that this code will need to be modified to be able
               use anisotropic BRDFs, which need tangent continuity */

            its.shFrame = Frame(
                (bary.x() * N.col(idx0) +
                 bary.y() * N.col(idx1) +
                 bary.z() * N.col(idx2))
                    .normalized());
        }
        else
        {
            its.shFrame = its.geoFrame;
        }
    }

    return foundIntersection;
}

NORI_NAMESPACE_END
