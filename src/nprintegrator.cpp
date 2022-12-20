#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/integrator.h>
#include <nori/sampler.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class NprIntegrator : public Integrator
{
public:
    // Constructor
    NprIntegrator(const PropertyList& props) {
        m_threshCrease = props.getFloat("crease", 0.f);
    }

    // Required method to hookup the class to nori
    virtual std::string toString() const {
        return tfm::format(
            "NprIntegrator[\n"
            "  \n"
            "]");
    }

    // Core integrator function
    virtual Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        return 0.f;
    }

    // Integrator function that uses ray differentials
    virtual Color3f Li(const Scene *scene, Sampler *sampler, const RayDifferential& ray) const {
        Intersection its;
        if (!scene->rayIntersect(ray, its)) {
            return Color3f(1.f);
        }
        else {
            const BSDF* currBSDF = its.mesh->getBSDF();
            if (currBSDF != nullptr) {
                // create the emitter query record and sample a light source
                // assuming just one light source for now.
                EmitterQueryRecord eRec;
                const Emitter* light = scene->getLights()[0];
                if (light != nullptr) {
                    // Sample the light source
                    eRec.ref = its.p;
                    Color3f Li = light->sample(eRec, sampler->next2D());

                    // compute the bsdf contribution
                    const Vector3f wo = its.shFrame.toLocal(-ray.d.normalized());
                    const Vector3f wi = its.shFrame.toLocal(eRec.wi);
                    BSDFQueryRecord bRec(wo, wi, ESolidAngle);
                    const Color3f f = currBSDF->eval(bRec);

                    // Compute visibility;
                    Ray3f shadowRay(its.p, eRec.wi, Epsilon, (1.0f - Epsilon) * eRec.dist);
                    const float vis = scene->rayIntersect(shadowRay) ? 0.f : 1.f;

                    // compute other terms and the final color
                    const float cosTheta = std::abs(Frame::cosTheta(wi));
                    Color3f L = Li * f * cosTheta * vis;

                    // Compute the edge color
                    if (ray.m_hasRayDifferentials) {
                        std::unique_ptr<Intersection> stencilHits(new Intersection[ray.m_totalStencilRays]);
                        for (int rd = 0; rd < ray.m_totalStencilRays; rd++) {
                            scene->rayIntersect(ray.getStencilRay(rd), stencilHits.get()[rd]);
                        }

                        // count m
                        int m = 0;
                        const Mesh* gS = its.mesh;
                        for (int i = 0; i < ray.m_totalStencilRays; i++) {
                            const Mesh* gR = stencilHits.get()[i].mesh;
                            if (gR != gS)
                                m++;
                        }

                        // check if m == 0
                        // we can shade crease edges
                        if (m == 0) {
                            // m == 0 only when all the intersections are actually valid
                            const Normal3f& n0 = stencilHits.get()[0].geoFrame.n;
                            const Normal3f& n1 = stencilHits.get()[1].geoFrame.n;
                            const Normal3f& n2 = stencilHits.get()[2].geoFrame.n;
                            const Normal3f& n3 = stencilHits.get()[3].geoFrame.n;
                            const Normal3f& n4 = stencilHits.get()[4].geoFrame.n;
                            const Normal3f& n5 = stencilHits.get()[5].geoFrame.n;
                            const Normal3f& n6 = stencilHits.get()[6].geoFrame.n;
                            const Normal3f& n7 = stencilHits.get()[7].geoFrame.n;

                            // front and sideways
                            float dot1 = n0.dot(n4);
                            float dot2 = n1.dot(n5);
                            float dot3 = n2.dot(n6);
                            float dot4 = n3.dot(n7);

                            // check for creases
                            if (abs(dot1) < m_threshCrease) m += 4;
                            else if (abs(dot2) < m_threshCrease) m += 4;
                            else if (abs(dot3) < m_threshCrease) m += 4;
                            else if (abs(dot4) < m_threshCrease) m += 4;
                        }

                        // compute the edge strength metric
                        const float factor = 0.5f * ray.m_totalStencilRays;
                        const float edgeStrength = clamp(1.0f - std::abs(m - factor) / factor, 0.f, 1.f);

                        // lerp between L and edge strength
                        return (1.0f - edgeStrength) * L;
                    }
                    return L;
                }
            }
            return Color3f(1.f);
        }
    }

private:
    float m_threshCrease;
};

NORI_REGISTER_CLASS(NprIntegrator, "npr")
NORI_NAMESPACE_END