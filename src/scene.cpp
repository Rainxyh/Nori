/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

Scene::Scene(const PropertyList &)
{
    // m_accel = new Accel();
    m_accel = new BVH(); // bunny [depth 9, innodes 103, leaves 104]  19ms, ajax [depth 23, innodes 29663, leaves 29663]  502ms
    // m_accel = new KDtree();  // bunny [depth 12, innodes 268, leaves 269] 22ms, ajax [depth 28, innodes 85162, leaves 85163]  943ms
    // m_accel = new Octtree(); // bunny [depth 4, innodes 89, leaves 624]   24ms, ajax [depth 10, innodes 28285, leaves 197996] 903ms
    // m_accel = new SAH();     // bunny [depth 7, innodes 71, leaves 44]    24ms, ajax [depth 17, innodes 20410, leaves 12216] 1.52s
}

Scene::~Scene()
{
    delete m_accel;
    delete m_sampler;
    delete m_camera;
    delete m_integrator;
}

void Scene::buildAccelStructure()
{
    std::cout << "Building accel structure" << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<uint32_t>> triangle_list;
    BoundingBox3f *bbox = new BoundingBox3f();
    BoundingSphere *bsphere = new BoundingSphere();
    BoundingStructure *BS = nullptr;
    for (size_t i = 0; i < m_meshes.size(); ++i)
    {
        uint32_t *index_list = (uint32_t *)malloc(m_meshes[i]->getTriangleCount() * sizeof(uint32_t));
        for (size_t j = 0; j < m_meshes[i]->getTriangleCount(); ++j)
            index_list[j] = j;
        std::vector<uint32_t> triangle_vec(index_list, index_list + m_meshes[i]->getTriangleCount());
        triangle_list.push_back(triangle_vec);
        if (typeid(BoundingBox3f) == typeid(*m_meshes[i]->getBoundingStructure()))
        {
            bbox->expandBy(*m_meshes[i]->getBoundingBox());
            BS = dynamic_cast<BoundingBox3f *>(bbox);
        }
        else if (typeid(BoundingSphere) == typeid(*m_meshes[i]->getBoundingStructure()))
        {
            bsphere->expandBy(*m_meshes[i]->getBoundingSphere());
            BS = dynamic_cast<BoundingSphere *>(bsphere);
        }
    }
    m_accel->setNode(m_accel->build(BS, triangle_list, 0));
    if (!m_accel)
    {
        std::cerr << "Accel structure build failed!" << std::endl;
    }
    else
    {
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << tfm::format("Accel structure build successfully! [depth %d, innodes %d, leaves %d]", m_accel->getTreeDepth(), m_accel->getTreeNodeNum(), m_accel->getTreeLeafNum()) << std::endl;
        std::cout << "Accel structure build time:" << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms\n";
    }
}

// build bounding volume hierarchy and check necessary scene components
void Scene::activate()
{
    Scene::buildAccelStructure();

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
