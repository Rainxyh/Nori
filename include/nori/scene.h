#pragma once

#include <nori/accel.h>
#include <nori/bvh.h>
#include <nori/sampler.h>
#include <nori/bitmap.h>
#include <nori/integrator.h>
#include <nori/camera.h>
#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Main scene data structure
 *
 * This class holds information on scene objects and is responsible for
 * coordinating rendering jobs. It also provides useful query routines that
 * are mostly used by the \ref Integrator implementations.
 */
class Scene : public NoriObject {
public:
    /// Construct a new scene object
    Scene(const PropertyList &);

    /// Release all memory
    virtual ~Scene();

    /// Return a pointer to the scene's kd-tree
    const BVH *getBVH() const { return m_bvh; }

    /// Return a pointer to the scene's kd-tree
    const Accel *getAccel() const { return m_accel; }

    /// Return a pointer to the scene's integrator
    const Integrator *getIntegrator() const { return m_integrator; }

    /// Return a pointer to the scene's integrator
    Integrator *getIntegrator() { return m_integrator; }

    /// Return a pointer to the scene's camera
    const Camera *getCamera() const { return m_camera; }

    /// Return a pointer to the scene's sample generator (const version)
    const Sampler *getSampler() const { return m_sampler; }

    /// Return a pointer to the scene's sample generator
    Sampler *getSampler() { return m_sampler; }

    /// Return a reference to an array containing all meshes
    const std::vector<Mesh *> &getMeshes() const { return m_meshes; }

    /// Return a reference to an array containing all emitters
    std::vector<Emitter *> getEmitterList() const { return m_emitters; }

    /// Return a reference to a random emitter
    Emitter *getRandomEmitter() const {
        return m_emitters[std::floor(m_sampler->next1D() * m_emitters.size())]; }

    float getEmitterPdf() const
    {
        return 1.f / m_emitters.size();
    }

    //	Return the background color of the scene if the ray never intersected the scene or escaped the scene.
    Color3f getBackground(const Ray3f& ray) const;

    const Emitter* getBackgroundEmitter() const { return m_bgEmitter; }

    // const Medium* getSceneMedium() const { return m_scene_medium; }

    /**
     * \brief Intersect a ray against all triangles stored in the scene
     * and return detailed intersection information
     *
     * \param ray
     *    A 3-dimensional ray data structure with minimum/maximum
     *    extent information
     *
     * \param its
     *    A detailed intersection record, which will be filled by the
     *    intersection query
     *
     * \return \c true if an intersection was found
     */
    bool rayIntersect(const Ray3f &ray, Intersection &its) const {
        if (m_bvh)
            return m_bvh->rayIntersect(ray, its, false);
        return m_accel->rayIntersect(ray, its, false);
    }

    /**
     * \brief Intersect a ray against all triangles stored in the scene
     * and \a only determine whether or not there is an intersection.
     *
     * This method much faster than the other ray tracing function,
     * but the performance comes at the cost of not providing any
     * additional information about the detected intersection
     * (not even its position).
     *
     * \param ray
     *    A 3-dimensional ray data structure with minimum/maximum
     *    extent information
     *
     * \return \c true if an intersection was found
     */
    bool rayIntersect(const Ray3f &ray) const {
        Intersection its; /* Unused */
        if (m_bvh)
            return m_bvh->rayIntersect(ray, its, false);
        return m_accel->rayIntersect(ray, its, false);
    }

    /// \brief Return an axis-aligned box that bounds the scene
    const BoundingBox3f &getBoundingBox() const {
        return m_bvh->getBoundingBox();
    }
    const BoundingBugBox *getBoundingBugBox() const {
        return m_accel->getBoundingBugBox();
    }
    const BoundingSphere *getBoundingSphere() const {
        return m_accel->getBoundingSphere();
    } 
    const BoundingStructure *getBoundingStructure() const {
        return m_accel->getBoundingStructure();
    }

    void buildAccelStructure();

    /**
     * \brief Inherited from \ref NoriObject::activate()
     *
     * Initializes the internal data structures (kd-tree,
     * emitter sampling data structures, etc.)
     */
    void activate();

    /// Add a child object to the scene (meshes, integrators etc.)
    void addChild(NoriObject *obj);

    /// Return a string summary of the scene (for debugging purposes)
    std::string toString() const;

    EClassType getClassType() const { return EScene; }

private:
    std::vector<Mesh *> m_meshes; // meshes list
    Integrator *m_integrator   = nullptr;
    Sampler    *m_sampler      = nullptr;
    Camera     *m_camera       = nullptr;
    Accel      *m_accel        = nullptr;
    BVH        *m_bvh          = nullptr;
    Emitter    *m_bgEmitter    = nullptr;
    // Medium     *m_scene_medium = nullptr;
    std::vector<Emitter*> m_emitters;

};

NORI_NAMESPACE_END
