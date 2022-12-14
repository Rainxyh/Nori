#pragma once
#include <nori/object.h>
#include <nori/frame.h>
#include <nori/bbox.h>
#include <nori/bbugbox.h>
#include <nori/bsphere.h>
#include <nori/bstructure.h>
#include <nori/dpdf.h>
#include <nori/texture.h>

NORI_NAMESPACE_BEGIN
/**
 * \brief Intersection data structure
 *
 * This data structure records local information about a ray-triangle intersection.
 * This includes the position, traveled ray distance, uv coordinates, as well
 * as well as two local coordinate frames (one that corresponds to the true
 * geometry, and one that is used for shading computations).
 */
struct Intersection {
    /// Position of the surface intersection
    Point3f p;
    /// Unoccluded distance along the ray
    float t;
    /// UV coordinates, if any
    Point2f uv;
    /// Shading frame (based on the shading normal)
    Frame shFrame;
    /// Geometric frame (based on the true geometry)
    Frame geoFrame;
    /// Pointer to the associated mesh
    const Mesh *mesh;
    
    std::string mtlName;

    /// Create an uninitialized intersection record
    Intersection() : mesh(nullptr) { }

    /// Transform a direction vector into the local shading frame
    Vector3f toLocal(const Vector3f &d) const {
        return shFrame.toLocal(d);
    }

    /// Transform a direction vector from local to world coordinates
    Vector3f toWorld(const Vector3f &d) const {
        return shFrame.toWorld(d);
    }

    /// Return a human-readable summary of the intersection record
    std::string toString() const;
};

/**
 * \brief Triangle mesh
 *
 * This class stores a triangle mesh object and provides numerous functions
 * for querying the individual triangles. Subclasses of \c Mesh implement
 * the specifics of how to create its contents (e.g. by loading from an
 * external file)
 */
class Mesh : public NoriObject {
public:
    /// Release all memory
    virtual ~Mesh();

    /// Initialize internal data structures (called once by the XML parser)
    virtual void activate();

    void samplePosition(Sampler* sampler, Point3f& p, Normal3f& n) const ;

    void uniformSample(Sampler *sampler, Point3f &point, Normal3f &normal) const;

    void uniformSample(Sampler *sampler, Point3f &point, Normal3f &normal, Point2f& texCoord) const;

    void uniformSample(Sampler* sampler, Point3f &point, Normal3f &normal, float &pdf) const;

    /// Return the total number of triangles in this shape
    uint32_t getTriangleCount() const { return (uint32_t) m_F.cols(); }

    /// Return the total number of vertices in this shape
    uint32_t getVertexCount() const { return (uint32_t) m_V.cols(); }

    /// Return the surface area sum of this mesh
    float surfaceArea() const;

    /// Return the surface area of the given triangle
    float surfaceArea(uint32_t index) const;

    // Return an axis-aligned bounding box of the entire mesh
    BoundingBox3f     *getBoundingBox()       const { return m_bbox; }
    BoundingBugBox    *getBoundingBugBox()    const { return m_bbugbox; }
    BoundingSphere    *getBoundingSphere()    const { return m_bsphere; }
    BoundingStructure *getBoundingStructure() const { return m_BS; }

    // Return an axis-aligned bounding box containing the given triangle
    BoundingBox3f      getBoundingBox(uint32_t index)       const;
    BoundingBugBox     getBoundingBugBox(uint32_t index)    const;
    BoundingSphere     getBoundingSphere(uint32_t index)    const;
    BoundingStructure *getBoundingStructure(uint32_t index) const;

    // Return the centroid of the given triangle
    Point3f getCentroid(uint32_t index) const;

    /** \brief Ray-triangle intersection test
     *
     * Uses the algorithm by Moeller and Trumbore discussed at
     * <tt>http://www.acm.org/jgt/papers/MollerTrumbore97/code.html</tt>.
     *
     * Note that the test only applies to a single triangle in the mesh.
     * An acceleration data structure like \ref BVH is needed to search
     * for intersections against many triangles.
     *
     * \param index
     *    Index of the triangle that should be intersected
     * \param ray
     *    The ray segment to be used for the intersection query
     * \param t
     *    Upon success, \a t contains the distance from the ray origin to the
     *    intersection point,
     * \param u
     *   Upon success, \c u will contain the 'U' component of the intersection
     *   in barycentric coordinates
     * \param v
     *   Upon success, \c v will contain the 'V' component of the intersection
     *   in barycentric coordinates
     * \return
     *   \c true if an intersection has been detected
     */
    bool rayIntersect(uint32_t index, const Ray3f &ray, float &u, float &v, float &t) const;

    // Computes a ray-mesh intersection test, returning the point to the nearest intersection on the mesh.
	bool rayMeshIntersect(const Ray3f& ray, Intersection& isect) const;

	// Computes a shadow ray mesh intersection test -> returns true on the first intersection found
	bool rayMeshIntersectP(const Ray3f& ray) const;

    /// Return a pointer to the vertex positions
    const MatrixXf &getVertexPositions() const { return m_V; }

    /// Return a pointer to the vertex normals (or \c nullptr if there are none)
    const MatrixXf &getVertexNormals() const { return m_N; }

    /// Return a pointer to the texture coordinates (or \c nullptr if there are none)
    const MatrixXf &getVertexTexCoords() const { return m_UV; }

    /// Return a pointer to the triangle vertex index list
    const MatrixXu &getIndices() const { return m_F; }

    const std::vector<std::string> &getFaceMtlMap() const { return face_idx_2_mtl_map; }

    const NoriTexture *getTexture() const { return m_texture; }

    /// Is this mesh an area emitter?
    bool isEmitter() const { return m_emitter != nullptr; }

    /// Return a pointer to an attached area emitter instance
    Emitter *getEmitter() { return m_emitter; }

    /// Return a pointer to an attached area emitter instance (const version)
    const Emitter *getEmitter() const { return m_emitter; }

    /// Return a pointer to the BSDF associated with this mesh
    const BSDF *getBSDF() const { return m_bsdf; }

    /// Register a child object (e.g. a BSDF) with the mesh
    virtual void addChild(NoriObject *child);

    /// Return the name of this mesh
    const std::string &getName() const { return m_name; }

    const DiscretePDF *getDPDF() const { return m_dpdf; }

    /// Return a human-readable summary of this instance
    std::string toString() const;

    /**
     * \brief Return the type of object (i.e. Mesh/BSDF/etc.)
     * provided by this instance
     * */
    EClassType getClassType() const { return EMesh; }

protected:
    /// Create an empty mesh
    Mesh();

protected:
    std::string m_name;                  ///< Identifying name
    MatrixXf      m_V;                   ///< Vertex positions
    MatrixXf      m_N;                   ///< Vertex normals
    MatrixXf      m_UV;                  ///< Vertex texture coordinates
    MatrixXu      m_F;                   ///< Faces
    std::vector<std::string> face_idx_2_mtl_map; ///< Faces
    NoriTexture       *m_texture = nullptr;      ///< Texture of the mesh
    BSDF              *m_bsdf    = nullptr;      ///< BSDF of the surface
    Emitter           *m_emitter = nullptr;      ///< Associated emitter, if any
    DiscretePDF       *m_dpdf    = nullptr;      ///< Discrete probability density
    BoundingBox3f     *m_bbox    = nullptr;      ///< Bounding box of the mesh
    BoundingBugBox    *m_bbugbox = nullptr;      ///< Bounding box of the mesh
    BoundingSphere    *m_bsphere = nullptr;      ///< Bounding sphere of the mesh
    BoundingStructure *m_BS      = nullptr;      ///< Bounding structure of the mesh
}; 

NORI_NAMESPACE_END
