#include <nori/mesh.h>
#include <nori/timer.h>
#include <filesystem/resolver.h>
#include <unordered_map>
#include <fstream>

NORI_NAMESPACE_BEGIN

/**
 * \brief Loader for Wavefront OBJ triangle meshes
 */
class WavefrontOBJ : public Mesh {
public:
    WavefrontOBJ(const PropertyList &propList) {
        typedef std::unordered_map<OBJVertex, uint32_t, OBJVertexHash> VertexMap;

        filesystem::path filename =
            getFileResolver()->resolve(propList.getString("filename"));

        std::ifstream is(filename.str());
        if (is.fail())
            throw NoriException("Unable to open OBJ file \"%s\"! (Check if file exists.)", filename);
        Transform trafo = propList.getTransform("toWorld", Transform());

        cout << "Loading \"" << filename << "\" .. ";
        cout.flush();
        Timer timer;

        std::vector<Vector3f>   positions;
        std::vector<Vector2f>   texcoords;
        std::vector<Vector3f>   normals;
        std::vector<uint32_t>   indices;
        std::vector<OBJVertex>  vertices;
        VertexMap vertexMap;
        std::string usemtl_pointer;

        std::string line_str;
        while (std::getline(is, line_str)) {
            std::istringstream line(line_str);

            std::string prefix;
            line >> prefix;

            if (prefix == "v") { // point
                Point3f p;
                line >> p.x() >> p.y() >> p.z();
                p = trafo * p;
                if (typeid(BoundingBugBox) == typeid(*m_BS))
                    m_bbugbox->expandBy(p);
                else if (typeid(BoundingSphere) == typeid(*m_BS))
                    m_bsphere->expandBy(p);
                positions.push_back(p);
            } else if (prefix == "vt") { // texture coord
                Point2f tc;
                line >> tc.x() >> tc.y();
                texcoords.push_back(tc);
            } else if (prefix == "vn") { // normal
                Normal3f n;
                line >> n.x() >> n.y() >> n.z();
                normals.push_back((trafo * n).normalized());
            } else if (prefix == "f") { // face
                std::string v1, v2, v3, v4;
                line >> v1 >> v2 >> v3 >> v4;
                OBJVertex verts[6];
                int nVertices = 3;

                verts[0] = OBJVertex(v1);
                verts[1] = OBJVertex(v2);
                verts[2] = OBJVertex(v3);

                if (!v4.empty()) {
                    /* This is a quad, split into two triangles */
                    verts[3] = OBJVertex(v4);
                    verts[4] = verts[0];
                    verts[5] = verts[2];
                    nVertices = 6;
                    face_idx_2_mtl_map.push_back(usemtl_pointer); // map face index to certain material library file
                }
                /* Convert to an indexed vertex list */
                for (int i=0; i<nVertices; ++i) {
                    const OBJVertex &v = verts[i];
                    VertexMap::const_iterator it = vertexMap.find(v);
                    if (it == vertexMap.end()) { // new vectex
                        vertexMap[v] = (uint32_t) vertices.size();
                        indices.push_back((uint32_t) vertices.size()); // currrent vertex's index in collection
                        vertices.push_back(v); // build vectex collection
                    } else { // vectex has been pushed in vertices
                        indices.push_back(it->second);
                    }
                }
                face_idx_2_mtl_map.push_back(usemtl_pointer); // map face index to certain material library file
            } else if (prefix == "mtllib"){ // material library file
                std::string mtl_file_name;
                line >> mtl_file_name;
            } else if (prefix == "usemtl"){ // map certain mtl
                line >> usemtl_pointer;
            }
        }
        m_F.resize(3, indices.size() / 3);
        memcpy(m_F.data(), indices.data(), sizeof(uint32_t) * indices.size());

        m_V.resize(3, vertices.size());
        for (uint32_t i=0; i<vertices.size(); ++i)
            m_V.col(i) = positions.at(vertices[i].p-1);

        if (!normals.empty()) {
            m_N.resize(3, vertices.size());
            for (uint32_t i=0; i<vertices.size(); ++i)
                m_N.col(i) = normals.at(vertices[i].n-1);
        }

        if (!texcoords.empty()) {
            m_UV.resize(2, vertices.size());
            for (uint32_t i=0; i<vertices.size(); ++i)
                m_UV.col(i) = texcoords.at(vertices[i].uv-1);
        }

        m_name = filename.str();
        cout << "done. (V=" << m_V.cols() << ", F=" << m_F.cols() << ", took "
             << timer.elapsedString() << " and "
             << memString(m_F.size() * sizeof(uint32_t) +
                          sizeof(float) * (m_V.size() + m_N.size() + m_UV.size()))
             << ")" << endl;
    }

protected:
    /// Vertex indices used by the OBJ format
    struct OBJVertex {
        uint32_t p = (uint32_t) -1;
        uint32_t n = (uint32_t) -1;
        uint32_t uv = (uint32_t) -1;

        inline OBJVertex() { }

        inline OBJVertex(const std::string &string) {
            std::vector<std::string> tokens = tokenize(string, "/", true);

            if (tokens.size() < 1 || tokens.size() > 3)
                throw NoriException("Invalid vertex data: \"%s\"", string);

            p = StringtoUInt(tokens[0]);

            if (tokens.size() >= 2 && !tokens[1].empty())
                uv = StringtoUInt(tokens[1]);

            if (tokens.size() >= 3 && !tokens[2].empty())
                n = StringtoUInt(tokens[2]);
        }

        inline bool operator==(const OBJVertex &v) const {
            return v.p == p && v.n == n && v.uv == uv;
        }
    };

    /// Hash function for OBJVertex
    struct OBJVertexHash {
        std::size_t operator()(const OBJVertex &v) const {
            size_t hash = std::hash<uint32_t>()(v.p);
            hash = hash * 37 + std::hash<uint32_t>()(v.uv);
            hash = hash * 37 + std::hash<uint32_t>()(v.n);
            return hash;
        }
    };
};

NORI_REGISTER_CLASS(WavefrontOBJ, "obj");
NORI_NAMESPACE_END
