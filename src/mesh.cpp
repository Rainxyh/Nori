#include <nori/mesh.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/warp.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

Mesh::Mesh()
{
    m_bbugbox = new BoundingBugBox();
    m_bsphere = new BoundingSphere(); // associated with Accel, initial in WavefrontOBJ
#define Box 1
#define Sphere 0
#if Box
    m_BS = dynamic_cast<BoundingBugBox *>(m_bbugbox);
#else
    m_BS = dynamic_cast<BoundingSphere *>(m_bsphere);
#endif
}

Mesh::~Mesh() { 
    delete m_texture;
    delete m_bsdf;
    delete m_emitter;
    delete m_bbugbox;
    delete m_bsphere;
}

void Mesh::activate() {
    if (!m_bsdf) {
        /* If no material was assigned, instantiate a diffuse BRDF */
        m_bsdf = static_cast<BSDF *>(
            NoriObjectFactory::createInstance("diffuse", PropertyList()));
    }
    if (!m_dpdf) {
        /* If no pdf was assigned, instantiate a discrete pdf */
        m_dpdf = new DiscretePDF(getTriangleCount());
        for (size_t i = 0; i < getTriangleCount(); ++i)
        {
            m_dpdf->append(surfaceArea(i));
        }
        m_dpdf->normalize();
    }
}

void Mesh::addChild(NoriObject *obj) {
    switch (obj->getClassType()) {
        case ETexture:
            if (m_texture){
                    throw NoriException(
                        "Mesh: tried to register multiple BSDF instances!");
                m_texture = static_cast<NoriTexture *>(obj);
            }
            break;
        case EBSDF: {
                if (m_bsdf)
                    throw NoriException(
                        "Mesh: tried to register multiple BSDF instances!");
                m_bsdf = static_cast<BSDF *>(obj);
            }
            break;

        case EEmitter: {
                if (m_emitter)
                    throw NoriException(
                        "Mesh: tried to register multiple Emitter instances!");
                m_emitter = static_cast<Emitter *>(obj);
                m_emitter->setMesh(this);
            }
            break;

        default:
            throw NoriException("Mesh::addChild(<%s>) is not supported!",
                                classTypeName(obj->getClassType()));
    }
}

void Mesh::samplePosition(Sampler* sampler, Point3f& point, Normal3f& normal) const {
    size_t idx = m_dpdf->sample(sampler->next1D());
    Point2f sample(sampler->next2D());
    uint32_t i0 = m_F(0, idx), i1 = m_F(1, idx), i2 = m_F(2, idx);

    // barycentric sampling of triangle.
    float u1 = sqrt(sample.x());
    float u = 1.0f - u1;
    float v = sample.y() * u1;

    const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);
    if (m_N.size() != 0) {
        const Normal3f n0 = m_N.col(i0), n1 = m_N.col(i1), n2 = m_N.col(i2);
        normal = (1.0f - u - v) * n0 + u * n1 + v * n2;
    } else {
        normal = (p1 - p0).cross(p2 - p0).normalized();
    }
    point = (1.0f - u - v) * p0 + u * p1 + v * p2;
}

void Mesh::uniformSample(Sampler* sampler, Point3f &point, Normal3f &normal, Point2f& texCoord) const{
    size_t idx = m_dpdf->sample(sampler->next1D());
    Point3f a = m_V.col(m_F(0, idx)), b = m_V.col(m_F(1, idx)), c = m_V.col(m_F(2, idx));
    Normal3f an, bn, cn;
    /* m_N may not provided by the mesh */
    if (m_N.cols())
    {
        an = m_N.col(m_F(0, idx));
        bn = m_N.col(m_F(1, idx));
        cn = m_N.col(m_F(2, idx));
    }
    else
    {
        an = (b - a).cross(c - a).normalized();
        bn = (c - b).cross(a - b).normalized();
        cn = (a - c).cross(b - c).normalized();
    }
    Point2f auv, buv, cuv;
    if (m_UV.cols())
    {
        auv = m_UV.col(m_F(0, idx));
        buv = m_UV.col(m_F(1, idx));
        cuv = m_UV.col(m_F(2, idx));
    }
    Vector2f xi(sampler->next2D());
    Point3f bary_cood(1.f - sqrt(1.f - xi[0]), xi[1] * sqrt(1.f - xi[0]), sqrt(1.f - xi[0]) - xi[1] * sqrt(1.f - xi[0]));
    point    = Point3f(a   * bary_cood[0] + b   * bary_cood[1] + c   * bary_cood[2]);
    normal   = Normal3f(an * bary_cood[0] + bn  * bary_cood[1] + cn  * bary_cood[2]);
    texCoord = Point2f(auv * bary_cood[0] + buv * bary_cood[1] + cuv * bary_cood[2]);
}

void Mesh::uniformSample(Sampler* sampler, Point3f &point, Normal3f &normal) const{
    Point2f tex = {};
    uniformSample(sampler, point, normal, tex);
}

/**
 * \brief Uniformly sample a position on the mesh with
 * respect to surface area. Returns both position and normal
 */
void Mesh::uniformSample(Sampler* sampler, Point3f &point, Normal3f &normal, float &pdf) const{
    m_dpdf->sample(sampler->next1D(), pdf);
    uniformSample(sampler, point, normal);
}

float Mesh::surfaceArea() const {
    return m_dpdf->getSum();
}

float Mesh::surfaceArea(uint32_t index) const {
    uint32_t i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);
    const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);
    return 0.5f * Vector3f((p1 - p0).cross(p2 - p0)).norm();
}

bool Mesh::rayIntersect(const uint32_t index, const Ray3f &ray, float &u, float &v, float &t) const {
    uint32_t i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index); // m_F in 3*n matrix, matrix key is face_index, matrix value is vector_index
    // const Point3i idx = m_F.col(index);
    const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2); // m_V also in 3*n matrix, map face_index to vector_index, because some points can be shared

    /* Find vectors for two edges sharing v[0] */
    Vector3f edge1 = p1 - p0, edge2 = p2 - p0;

    /* Begin calculating determinant - also used to calculate U parameter */
    Vector3f pvec = ray.d.cross(edge2);

    /* If determinant is near zero, ray lies in plane of triangle */
    float det = edge1.dot(pvec);

    if (det > -1e-8f && det < 1e-8f)
        return false;
    float inv_det = 1.0f / det;

    /* Calculate distance from v[0] to ray origin */
    Vector3f tvec = ray.o - p0;

    /* Calculate U parameter and test bounds */
    u = tvec.dot(pvec) * inv_det;
    if (u < 0.0 || u > 1.0)
        return false;

    /* Prepare to test V parameter */
    Vector3f qvec = tvec.cross(edge1);

    /* Calculate V parameter and test bounds */
    v = ray.d.dot(qvec) * inv_det;
    if (v < 0.0 || u + v > 1.0)
        return false;

    /* Ray intersects triangle -> compute t */
    t = edge2.dot(qvec) * inv_det;

    return t >= ray.mint && t <= ray.maxt;
}

bool Mesh::rayMeshIntersect(const Ray3f& ray, Intersection& isect) const {
    float u, v, t;
    float min_t = INFINITY, min_u = 0.0f, min_v = 0.0f;
    int min_id = -1;
    bool intersected = false;
    for (size_t i = 0; i < getTriangleCount(); i++) {
        if (rayIntersect(i, ray, u, v, t)) {
            if (t < min_t) {
                min_id = i;
                min_t = t;
                min_u = u;
                min_v = v;
                intersected = true;
            }
        }
    }

    if (min_id != -1) {
        isect.p = ray(min_t);
        isect.t = min_t;
        isect.mesh = this;

        // compose the normal for a frame
        uint32_t i0 = m_F(0, min_id), i1 = m_F(1, min_id), i2 = m_F(2, min_id);
        const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);
        Normal3f surface_normal = (p1 - p0).cross(p2 - p0).normalized();
        isect.geoFrame = Frame(surface_normal);
        isect.shFrame = isect.geoFrame;
        isect.uv = Vector2f(min_u, min_v);
    }

    return intersected;
}

bool Mesh::rayMeshIntersectP(const Ray3f& ray) const {
        float u, v, t;
        for (size_t i = 0; i < getTriangleCount(); i++) {
                if (rayIntersect(i, ray, u, v, t))
                        return true;
        }
        return false;
}

BoundingBox3f Mesh::getBoundingBox(uint32_t index) const {
    BoundingBox3f result(m_V.col(m_F(0, index)));
    result.expandBy(m_V.col(m_F(1, index)));
    result.expandBy(m_V.col(m_F(2, index)));
    return result;
}

BoundingBugBox Mesh::getBoundingBugBox(uint32_t index) const {
    BoundingBugBox result(m_V.col(m_F(0, index)));
    result.expandBy(m_V.col(m_F(1, index)));
    result.expandBy(m_V.col(m_F(2, index)));
    return result;
}
BoundingSphere Mesh::getBoundingSphere(uint32_t index) const {
    BoundingSphere result(m_V.col(m_F(0, index)));
    result.expandBy(m_V.col(m_F(1, index)));
    result.expandBy(m_V.col(m_F(2, index)));
    return result;
}
BoundingStructure *Mesh::getBoundingStructure(uint32_t index) const{
    BoundingStructure *result = nullptr;
    if (typeid(BoundingBugBox) == typeid(*this->m_BS))
        result = dynamic_cast<BoundingBugBox *>(new BoundingBugBox(Mesh::getBoundingBugBox(index)));
    else if (typeid(BoundingSphere) == typeid(*this->m_BS))
        result = dynamic_cast<BoundingSphere *>(new BoundingSphere(Mesh::getBoundingSphere(index)));
    return result;
}

Point3f Mesh::getCentroid(uint32_t index) const {
    return (1.0f / 3.0f) *
        (m_V.col(m_F(0, index)) +
         m_V.col(m_F(1, index)) +
         m_V.col(m_F(2, index)));
}

std::string Mesh::toString() const {
    return tfm::format(
        "Mesh[\n"
        "  name = \"%s\",\n"
        "  vertexCount = %i,\n"
        "  triangleCount = %i,\n"
        "  bsdf = %s,\n"
        "  emitter = %s\n"
        "]",
        m_name,
        m_V.cols(),
        m_F.cols(),
        m_bsdf ? indent(m_bsdf->toString()) : std::string("null"),
        m_emitter ? indent(m_emitter->toString()) : std::string("null")
    );
}

std::string Intersection::toString() const {
    if (!mesh)
        return "Intersection[invalid]";

    return tfm::format(
        "Intersection[\n"
        "  p = %s,\n"
        "  t = %f,\n"
        "  uv = %s,\n"
        "  shFrame = %s,\n"
        "  geoFrame = %s,\n"
        "  mesh = %s\n"
        "]",
        p.toString(),
        t,
        uv.toString(),
        indent(shFrame.toString()),
        indent(geoFrame.toString()),
        mesh ? mesh->toString() : std::string("null")
    );
}

NORI_NAMESPACE_END
