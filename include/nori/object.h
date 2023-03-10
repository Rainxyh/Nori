#pragma once

#include <nori/proplist.h>
NORI_NAMESPACE_BEGIN

/**
 * \brief Base class of all objects
 *
 * A Nori object represents an instance that is part of
 * a scene description, e.g. a scattering model or emitter.
 */
class NoriObject {
public:
    enum EClassType {
        EScene = 0,
        EMesh,
        ETexture,
        EBSDF,
        EPhaseFunction,
        EEmitter,
        EMedium,
        ECamera,
        EIntegrator,
        ESampler,
        ETest,
        EReconstructionFilter,

        EClassTypeCount
    };

    /// Virtual destructor
    virtual ~NoriObject() { }

    /**
     * \brief Return the type of object (i.e. Mesh/BSDF/etc.) 
     * provided by this instance
     * */
    virtual EClassType getClassType() const = 0;

    /**
     * \brief Add a child object to the current instance
     *
     * The default implementation does not support children and
     * simply throws an exception
     */
    virtual void addChild(NoriObject *child);

    /**
     * \brief Set the parent object
     *
     * Subclasses may choose to override this method to be
     * notified when they are added to a parent object. The
     * default implementation does nothing.
     */
    virtual void setParent(NoriObject *parent);

    /**
     * \brief Perform some action associated with the object
     *
     * The default implementation throws an exception. Certain objects
     * may choose to override it, e.g. to implement initialization, 
     * testing, or rendering functionality.
     *
     * This function is called by the XML parser once it has
     * constructed an object and added all of its children
     * using \ref addChild().
     */
    virtual void activate();

    /// Return a brief string summary of the instance (for debugging purposes)
    virtual std::string toString() const = 0;
    
    /// Turn a class type into a human-readable string
    static std::string classTypeName(EClassType type) {
        switch (type) {
            case EScene:                return "scene";
            case EMesh:                 return "mesh";
            case ETexture:              return "texture";
            case EBSDF:                 return "bsdf";
            case EPhaseFunction:        return "phasefunction";
            case EEmitter:              return "emitter";
            case EMedium:               return "bsdf";
            case ECamera:               return "camera";
            case EIntegrator:           return "integrator";
            case ESampler:              return "sampler";
            case ETest:                 return "test";
            case EReconstructionFilter: return "filter";
            default:                    return "<unknown>";
        }
    }
};

/**
 * \brief Factory for Nori objects
 *
 * This utility class is part of a mini-RTTI framework and can 
 * instantiate arbitrary Nori objects by their name.
 */
class NoriObjectFactory {
public:
    using Constructor = std::function<NoriObject *(const PropertyList &)>;

    /**
     * \brief Register an object constructor with the object factory
     *
     * This function is called by the macro \ref NORI_REGISTER_CLASS
     *
     * \param name
     *     An internal name that is associated with this class. This is the
     *     'type' field found in the scene description XML files
     *
     * \param constr
     *     A function pointer to an anonymous function that is
     *     able to call the constructor of the class.
     */
    static void registerClass(const std::string &name, const Constructor &constr);

    /**
     * \brief Construct an instance from the class of the given name
     *
     * \param name
     *     An internal name that is associated with this class. This is the
     *     'type' field found in the scene description XML files
     *
     * \param propList
     *     A list of properties that will be passed to the constructor
     *     of the class.
     */
    static NoriObject *createInstance(const std::string &name, const PropertyList &propList);

private:
    static std::map<std::string, Constructor> *m_constructors; // nori object name and certain constructor
};


// NORI_REGISTER_CLASS(XXX, NAME)
// {
//     XXX *XXX_CRATE(const PropertyList &list)
//     {
//         return new XXX(list);
//     }

//     static struct XXX_ // underline is essential, prevent mutil define
//     {
//         // constructor calls registerClass to register
//         XXX_() 
//         {
//             // Get the constructor of the corresponding class and add it to the Map of the factory
//             NoriObjectFactory::registerClass(NAME, XXX_CRATE); 
//         }
//     } XXX__NORI_;
// }
/// Macro for registering an object constructor with the \ref NoriObjectFactory, macro use "##" to link string
#define NORI_REGISTER_CLASS(CLASS, NAME)                            \
    CLASS *CLASS##_CRATE(const PropertyList &list)                  \
    {                                                               \
        return new CLASS(list);                                     \
    }                                                               \
    static struct CLASS##_                                          \
    {                                                               \
        CLASS##_()                                                  \
        {                                                           \
            NoriObjectFactory::registerClass(NAME, CLASS##_CRATE);  \
        }                                                           \
    } CLASS##__NORI_;
    
NORI_NAMESPACE_END
