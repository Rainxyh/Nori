#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

Scene::Scene(const PropertyList &)
{
    // m_bvh = new BVH();
    m_accel = new BugVH(); // bunny [depth 9, innodes 103, leaves 104]  19ms, ajax [depth 23, innodes 29663, leaves 29663]  502ms
    // m_accel = new KDtree();  // bunny [depth 12, innodes 268, leaves 269] 22ms, ajax [depth 28, innodes 85162, leaves 85163]  943ms
    // m_accel = new Octtree(); // bunny [depth 4, innodes 89, leaves 624]   24ms, ajax [depth 10, innodes 28285, leaves 197996] 903ms
    // m_accel = new SAH();     // bunny [depth 7, innodes 71, leaves 44]    24ms, ajax [depth 17, innodes 20410, leaves 12216] 1.52s
}

Scene::~Scene()
{
    delete m_bvh;
    delete m_accel;
    delete m_sampler;
    delete m_camera;
    delete m_integrator;
}

// build bounding volume hierarchy and check necessary scene components
void Scene::activate()
{
    if (m_bvh)
        m_bvh->build();
    else
        m_accel->build();

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
        if (m_bvh)
            m_bvh->addMesh(mesh);
        else
            m_accel->addMesh(mesh);
        m_meshes.push_back(mesh);
        if (mesh->isEmitter())
            m_emitters.push_back(mesh->getEmitter());
        break;
    }

    case EEmitter:
    {
		// Add to the background emitter of the scene
		// We know for a fact that there can be only one distant disk in a scene.
		if (static_cast<Emitter*>(obj)->getEmitterType() == EmitterType::EMITTER_DISTANT_DISK || static_cast<Emitter*>(obj)->getEmitterType() == EmitterType::EMITTER_ENVIRONMENT)
			m_bgEmitter = static_cast<Emitter*>(obj);
        m_emitters.push_back(static_cast<Emitter *>(obj));
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

Color3f Scene::getBackground(const Ray3f& ray) const
{
	if (m_bgEmitter != nullptr)
	{
		EmitterQueryRecord eRec;
		eRec.ref = ray.o;
		eRec.wi = -ray.d;
		return m_bgEmitter->eval(eRec);
	}
	else
		return BLACK;
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
