/*
    This file is part of Nori, a simple educational ray tracer
    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/accel.h>

NORI_NAMESPACE_BEGIN

void Accel::build()
{
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
}

Node *BVH::build(BoundingBox3f box, std::vector<std::vector<uint32_t>> triangle_list, uint32_t depth)
{
    if (!triangle_list.size())
        return nullptr;
    uint32_t triangle_num = 0;
    for (size_t i = 0; i < triangle_list.size(); ++i)
        triangle_num += triangle_list[i].size(); // total triangle number in this scene
    if (triangle_num == 0)
        return nullptr;
    this->m_maxDepth = std::max(m_maxDepth, depth);
    Node *node = new Node(m_Dim);
    node->bbox = box;
    node->triangle_list = triangle_list; // 1st dim meshes pre scene, 2nd dim triangles pre mesh
    if (triangle_num < 30) // leaf node
    {
        node->is_leaf = true;
        ++this->m_leafNum;
        for (size_t i = 0; i < m_Dim; ++i)
            node->child[i] = nullptr;
        return node;
    }
    ++this->m_nodeNum; // internal node

    BoundingBox3f child_bbox_list[m_Dim];
    std::vector<std::vector<uint32_t>> regional_division_triangle_list[m_Dim];

    uint8_t chosen_axis = depth % 3;
    Point3f box_min = box.getCorner(0), box_max = box.getCorner(7);
    float axis_mid = ((box_max + box_min) / 2)[chosen_axis];
    for (size_t mesh_idx = 0; mesh_idx < m_meshes.size(); ++mesh_idx)
    {
        for (size_t i = 0; i < m_Dim; ++i)
        {
            regional_division_triangle_list[i].push_back(std::vector<uint32_t>());
        }
        for (size_t triangle_idx = 0; triangle_idx < triangle_list[mesh_idx].size(); ++triangle_idx)
        {
            uint32_t triangle_idx_in_mesh_buffer = triangle_list[mesh_idx][triangle_idx];
            BoundingBox3f triangle_bbox = m_meshes[mesh_idx]->getBoundingBox(triangle_idx_in_mesh_buffer);
            Point3f prim_center = triangle_bbox.getCenter();
            uint8_t switch_0_1=0;
            if (prim_center[chosen_axis] < axis_mid)
                switch_0_1 = 0;
            else
                switch_0_1 = 1;
            child_bbox_list[switch_0_1].expandBy(triangle_bbox);
            regional_division_triangle_list[switch_0_1][mesh_idx].push_back(triangle_idx_in_mesh_buffer);
        }
    }

    // Recursion build sub-tree
    for (size_t i = 0; i < m_Dim; ++i)
    {
        node->child[i] = build(child_bbox_list[i], regional_division_triangle_list[i], depth + 1);
    }
    return node;
}

Node *SAH::build(BoundingBox3f box, std::vector<std::vector<uint32_t>> triangle_list, uint32_t depth)
{
    /*
    For each axis: x, y, z:
        initialize B buckets
        For each primitive p in node:
            b = compute_bucket(p.centroid)
            b.bbox.union(p.bbox);
            ++b.prim_count;
        For each of the B-1 possible partitioning planes evaluate SAH
    Execute lowest cost partitioning found (or make node a leaf)
    */
    if (!triangle_list.size())
        return nullptr;
    uint32_t triangle_num = 0;
    for (size_t i = 0; i < triangle_list.size(); ++i)
        triangle_num += triangle_list[i].size(); // total triangle number in this scene
    if (triangle_num == 0)
        return nullptr;
    this->m_maxDepth = std::max(m_maxDepth, depth);
    Node *node = new Node(m_Dim);
    node->bbox = box;
    node->triangle_list = triangle_list; // 1st dim meshes pre scene, 2nd dim triangles pre mesh
    if (triangle_num < 30) // leaf node
    {
        node->is_leaf = true;
        ++this->m_leafNum;
        for (size_t i = 0; i < m_Dim; ++i)
            node->child[i] = nullptr;
        return node;
    }
    ++this->m_nodeNum; // internal node

    Point3f box_min = box.getCorner(0), box_max = box.getCorner(7);
    Point3f bucket_min, bucket_max;
    Bucket *buckets[3][nBuckets];
    float SAH_sorce = std::numeric_limits<float>::infinity();
    uint16_t chosen_dim = 0, chosen_plane = 0;
    for (size_t dim_idx = 0; dim_idx < 3; ++dim_idx) // x, y, z axis
    {
        float dim_min = box_min[dim_idx], dim_max = box_max[dim_idx];    // min/max in certain dim
        float delta = (dim_max - dim_min) / (float)nBuckets;             // regional division interval
        for (size_t bucket_idx = 0; bucket_idx < nBuckets; ++bucket_idx) // travel every bucket
        {
            float left_plane = delta * bucket_idx, right_plane;
            right_plane = left_plane + delta;
            bucket_min = box_min;              // first set bucket to min
            bucket_min[dim_idx] = left_plane;  // seconed set certain axis to bucket left plane value in same axis
            bucket_max = box_max;              // first set bucket to min
            bucket_max[dim_idx] = right_plane; // seconed set certain axis to bucket right plane value in same axis

            std::vector<std::vector<uint32_t>> triangle_idx_list;
            for (size_t mesh_idx = 0; mesh_idx < m_meshes.size(); ++mesh_idx) // bucket initial every mesh has a triangle_idx vector
                triangle_idx_list.push_back(std::vector<uint32_t>());
            buckets[dim_idx][bucket_idx] = new Bucket(BoundingBox3f(), triangle_idx_list, (uint32_t)0);
        }
        for (size_t mesh_idx = 0; mesh_idx < m_meshes.size(); ++mesh_idx) // every dim travel all triangle, dim * prim_num
        {
            for (size_t triangle_idx = 0; triangle_idx < triangle_list[mesh_idx].size(); ++triangle_idx)
            {
                uint32_t triangle_idx_in_mesh_buffer = triangle_list[mesh_idx][triangle_idx];
                BoundingBox3f triangle_bbox = m_meshes[mesh_idx]->getBoundingBox(triangle_idx_in_mesh_buffer);
                Point3f prim_center = triangle_bbox.getCenter();
                float dim_center = prim_center[dim_idx];
                uint16_t bucket_idx = floor((dim_center - dim_min) / (dim_max - dim_min) * nBuckets); // compute_bucket(centor) get the bucket which contain this triangle
                bucket_idx = clamp(bucket_idx, 0, nBuckets - 1);
                buckets[dim_idx][bucket_idx]->bbox.expandBy(triangle_bbox); // object based
                buckets[dim_idx][bucket_idx]->triangle_idx_list[mesh_idx].push_back(triangle_idx_in_mesh_buffer); // triangle index in certain mesh buffer
                ++buckets[dim_idx][bucket_idx]->prim_count;
            }
        }
        for (size_t planes_idx = 1; planes_idx < nBuckets; ++planes_idx)
        {
            float SA = 0, SB = 0;
            uint32_t NA = 0, NB = 0;
            for (size_t i = 0; i < planes_idx; ++i) // left -> partitioning plane
            {
                SA += buckets[dim_idx][i]->bbox.getSurfaceArea();
                NA += buckets[dim_idx][i]->prim_count;
            }
            for (size_t j = planes_idx; j < nBuckets; ++j) // partitioning plane -> right
            {
                SB += buckets[dim_idx][j]->bbox.getSurfaceArea();
                NB += buckets[dim_idx][j]->prim_count;
            }

            float cost = SAHTravCost + (SA * NA + SB * NB) / box.getSurfaceArea() * SAHInterCost;
            if (std::isfinite(cost) && !std::isnan(cost) && SAH_sorce > cost)
            {
                SAH_sorce = cost;
                chosen_plane = planes_idx;
                chosen_dim = dim_idx;
            }
        }
    }

    if (chosen_plane == 0 || chosen_plane == nBuckets - 1) // Prevents multiple left-most or right-most cyclic divisions
    {
        node->is_leaf = true;
        ++this->m_leafNum;
        for (size_t i = 0; i < m_Dim; ++i)
            node->child[i] = nullptr;
        return node;
    }

    // Divide the current root node to get two child nodes, and the division obtained by comparing the cost value
    BoundingBox3f child_bbox_list[m_Dim];
    std::vector<std::vector<uint32_t>> regional_division_triangle_list[m_Dim];
    for (size_t plane = 0; plane < chosen_plane; ++plane)
        child_bbox_list[0].expandBy(buckets[chosen_dim][plane]->bbox);
    for (size_t plane = chosen_plane; plane < nBuckets; ++plane)
        child_bbox_list[1].expandBy(buckets[chosen_dim][plane]->bbox);
    for (size_t mesh_idx = 0; mesh_idx < m_meshes.size(); ++mesh_idx)
    {
        regional_division_triangle_list[0].push_back(std::vector<uint32_t>()); // every mesh one triangle vector
        for (size_t plane = 0; plane < chosen_plane; ++plane)
        {
            regional_division_triangle_list[0][mesh_idx].insert(regional_division_triangle_list[0][mesh_idx].end(), // current mesh
                                                                buckets[chosen_dim][plane]->triangle_idx_list[mesh_idx].begin(),
                                                                buckets[chosen_dim][plane]->triangle_idx_list[mesh_idx].end()); // push_back certain list to certain mesh
        }
        regional_division_triangle_list[1].push_back(std::vector<uint32_t>()); // every mesh one triangle vector
        for (size_t plane = chosen_plane; plane < nBuckets; ++plane)
        {
            regional_division_triangle_list[1][mesh_idx].insert(regional_division_triangle_list[1][mesh_idx].end(), // current mesh
                                                                buckets[chosen_dim][plane]->triangle_idx_list[mesh_idx].begin(),
                                                                buckets[chosen_dim][plane]->triangle_idx_list[mesh_idx].end()); // push_back certain list to certain mesh
        }
    }

    // Recursion build sub-tree
    for (size_t i = 0; i < m_Dim; ++i)
    {
        node->child[i] = build(child_bbox_list[i], regional_division_triangle_list[i], depth + 1);
    }
    return node;
}

Node *Octtree::build(BoundingBox3f box, std::vector<std::vector<uint32_t>> triangle_list, uint32_t depth)
{
    if (!triangle_list.size())
        return nullptr;

    this->m_maxDepth = std::max(m_maxDepth, depth);

    Node *node = new Node(m_Dim);
    node->bbox = box;
    node->triangle_list = triangle_list; // 1st dim meshes pre scene, 2nd dim triangles pre mesh
    uint32_t triangle_num = 0;
    for (size_t i = 0; i < m_meshes.size(); ++i)
        triangle_num += triangle_list[i].size(); // total triangle number in this scene
        
    if (triangle_num < 30) // leaf node
    {
        node->is_leaf = true;
        ++this->m_leafNum;
        for (size_t i = 0; i < m_Dim; ++i)
            node->child[i] = nullptr;
        return node;
    }
    ++this->m_nodeNum; // internal node

    BoundingBox3f *child_bbox_list[m_Dim];
    for (size_t i = 0; i < m_Dim; ++i) // 8 vertices and midpoints can be divided into 8 sub-bounding boxes
    {
        Point3f minPoint;
        Point3f maxPoint;
        Point3f center = box.getCenter();
        Point3f corner = box.getCorner(i);
        minPoint = center.cwiseMin(corner); // The diagonal line can represent a cube, and the maximum and minimum points of each dimension 
        maxPoint = center.cwiseMax(corner); // are taken as the maximum and minimum endpoints of the cube.
        child_bbox_list[i] = new BoundingBox3f(minPoint, maxPoint);
    }

    // Traverse all triangles and divide the triangles into corresponding sub-bounding boxes according to their positions
    std::vector<std::vector<uint32_t>> regional_division_triangle_list[m_Dim];
    for (size_t i = 0; i < m_Dim; ++i)
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
    for (size_t i = 0; i < m_Dim; ++i)
    {
        node->child[i] = build(*child_bbox_list[i], regional_division_triangle_list[i], depth + 1);
        delete child_bbox_list[i];
    }

    return node;
}

Node *KDtree::build(BoundingBox3f box, std::vector<std::vector<uint32_t>> triangle_list, uint32_t depth)
{
    if (!triangle_list.size())
        return nullptr;

    this->m_maxDepth = std::max(m_maxDepth, depth);

    Node *node = new Node(m_Dim);
    node->bbox = box;
    node->triangle_list = triangle_list; // 1st dim meshes pre scene, 2nd dim triangles pre mesh
    uint32_t triangle_num = 0;
    for (size_t i = 0; i < m_meshes.size(); ++i)
        triangle_num += triangle_list[i].size(); // total triangle number in this scene
        
    if (triangle_num < 30) // leaf node
    {
        node->is_leaf = true;
        ++this->m_leafNum;
        for (size_t i = 0; i < m_Dim; ++i)
            node->child[i] = nullptr;
        return node;
    }
    ++this->m_nodeNum; // internal node

    BoundingBox3f *child_bbox_list[m_Dim];
    uint8_t chosen_axis = depth % 3; // Select the coordinate axis to be split in turn according to the depth
    Point3f center = box.getCenter(), box_min = box.getCorner(0), box_max = box.getCorner(7);
    Point3f child_box_min, child_box_max;
    child_box_min = box_min;
    child_box_min[chosen_axis] = center[chosen_axis];
    child_box_max = box_max;
    child_box_max[chosen_axis] = center[chosen_axis];
    child_bbox_list[0] = new BoundingBox3f(box_min, child_box_max); // box_min and box_min move along the positive direction of the corresponding axis to the corresponding distance from the midpoint
    child_bbox_list[1] = new BoundingBox3f(child_box_min, box_max); // box_min and box_min move along the negative direction of the corresponding axis to the corresponding distance from the midpoint

    // Traverse all triangles and divide the triangles into corresponding sub-bounding boxes according to their positions
    std::vector<std::vector<uint32_t>> regional_division_triangle_list[m_Dim];
    for (size_t i = 0; i < m_Dim; ++i)
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
    for (size_t i = 0; i < m_Dim; ++i)
    {
        node->child[i] = build(*child_bbox_list[i], regional_division_triangle_list[i], depth + 1);
        delete child_bbox_list[i];
    }

    return node;
}

bool Accel::travel(Node *treeNode, Ray3f &ray, Intersection &its, bool shadowRay, uint32_t &hitIdx) const
{
    if (!treeNode || !treeNode->bbox.rayIntersect(ray)) // empty tree node or not hit any bounding box
        return false;

    bool is_hit = false;
    if (treeNode->is_leaf) // process leaf node
    {
        for (size_t i = 0; i < treeNode->triangle_list.size(); ++i)
        { // travel meshes
            for (size_t j = 0; j < treeNode->triangle_list[i].size(); j++)
            { // travel triangles
                float u, v, t;
                if (m_meshes[i]->rayIntersect(treeNode->triangle_list[i][j], ray, u, v, t))
                {
                    /* An intersection was found! Can terminate
                    immediately if this is a shadow ray query */
                    if (shadowRay)
                        return true;
                    its.t = ray.maxt = t;
                    its.uv = Point2f(u, v);
                    its.mesh = m_meshes[i];
                    hitIdx = treeNode->triangle_list[i][j];
                    is_hit = true;
                }
            }
        }
    }
    else // recursively process internal nodes
    {
        std::pair<Node *, float> rank_list[m_Dim];
        for (size_t i = 0; i < m_Dim; ++i)
        {
            if (treeNode->child[i])
            {
                rank_list[i].first = treeNode->child[i];
                rank_list[i].second = treeNode->child[i]->bbox.distanceTo(ray.o);
            }
        }
        std::sort(rank_list, rank_list + m_Dim, [](const auto &l, const auto &r)
                  { return l.second < r.second; });
        for (size_t i = 0; i < m_Dim; ++i)
        {
            if (rank_list[i].first)
            {
                is_hit |= travel(rank_list[i].first, ray, its, shadowRay, hitIdx);
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

    /*treeNode acceleration structure*/
    foundIntersection = travel(m_accelNode, ray, its, shadowRay, hitIdx);

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