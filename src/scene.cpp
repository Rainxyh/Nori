/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

Scene::Scene(const PropertyList &)
{
    m_accel = new Accel();
}

Scene::~Scene()
{
    delete m_accel;
    delete m_sampler;
    delete m_camera;
    delete m_integrator;
}

// build bounding volume hierarchy and check necessary scene components
void Scene::activate()
{
    std::cout << "Building OctTree" << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<uint32_t>> triangle_list;
    BoundingBox3f bbox;
    for (size_t i = 0; i < m_meshes.size(); ++i)
    {
        uint32_t *index_list = (uint32_t *)malloc(m_meshes[i]->getTriangleCount() * sizeof(uint32_t));
        for (size_t j = 0; j < m_meshes[i]->getTriangleCount(); ++j)
            index_list[j] = j;
        std::vector<uint32_t> triangle_vec(index_list, index_list + m_meshes[i]->getTriangleCount());
        triangle_list.push_back(triangle_vec);
        bbox.expandBy(m_meshes[i]->getBoundingBox());
    }
    m_accel->setOcttree(m_accel->build(bbox, triangle_list, 0));
    std::cout << tfm::format("Octree build successfully! [depth %d, nodes %d, leaves %d]", m_accel->getOcttreeDepth(), m_accel->getOcttreeNode(), m_accel->getOcttreeLeaf()) << std::endl;
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "OctTree build time:" << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms\n";

    if (!m_integrator)
        throw NoriException("No integrator was specified!");
    if (!m_camera)
        throw NoriException("No camera was specified!");
    if (!m_sampler)
    {
        /* Create a default (independent) sampler */
        m_sampler = static_cast<Sampler *>(
            NoriObjectFactory::createInstance("independent", PropertyList()));
    }

    cout << endl;
    cout << "Configuration: " << toString() << endl;
    cout << endl;
}

// add scene child from xml file by NoriObject's ClassType
void Scene::addChild(NoriObject *obj)
{
    switch (obj->getClassType())
    {
    case EMesh:
    {
        Mesh *mesh = static_cast<Mesh *>(obj);
        m_accel->addMesh(mesh);
        m_meshes.push_back(mesh);
        if (mesh->isEmitter())
            m_emitters.push_back(mesh->getEmitter());
        break;
    }

    case EEmitter:
    {
        Emitter *emitter = static_cast<Emitter *>(obj);
        m_emitters.push_back(emitter);
        break;
    }
    
    case ESampler:
        if (m_sampler)
            throw NoriException("There can only be one sampler per scene!");
        m_sampler = static_cast<Sampler *>(obj);
        break;

    case ECamera:
        if (m_camera)
            throw NoriException("There can only be one camera per scene!");
        m_camera = static_cast<Camera *>(obj);
        break;

    case EIntegrator:
        if (m_integrator)
            throw NoriException("There can only be one integrator per scene!");
        m_integrator = static_cast<Integrator *>(obj);
        break;

    default:
        throw NoriException("Scene::addChild(<%s>) is not supported!",
                            classTypeName(obj->getClassType()));
    }
}

std::string Scene::toString() const
{
    std::string meshes;
    for (size_t i = 0; i < m_meshes.size(); ++i)
    {
        meshes += std::string("  ") + indent(m_meshes[i]->toString(), 2);
        if (i + 1 < m_meshes.size())
            meshes += ",";
        meshes += "\n";
    }

    return tfm::format(
        "Scene[\n"
        "  integrator = %s,\n"
        "  sampler = %s\n"
        "  camera = %s,\n"
        "  meshes = {\n"
        "  %s  }\n"
        "]",
        indent(m_integrator->toString()),
        indent(m_sampler->toString()),
        indent(m_camera->toString()),
        indent(meshes, 2));
}

NORI_REGISTER_CLASS(Scene, "scene");
NORI_NAMESPACE_END
