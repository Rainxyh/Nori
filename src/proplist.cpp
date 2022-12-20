#include <nori/proplist.h>

NORI_NAMESPACE_BEGIN

#define DEFINE_PROPERTY_ACCESSOR(Type, TypeName, XmlName)                                \
    void PropertyList::set##TypeName(const std::string &name, const Type &value)         \
    {                                                                                    \
        if (m_properties.find(name) != m_properties.end())                               \
            cerr << "Property \"" << name << "\" was specified multiple times!" << endl; \
        std::pair<std::string, PropertyList::Property> value_type;                       \
        value_type.first = name;                                                         \
        value_type.second.value.XmlName##_value = value;                                 \
        value_type.second.type = Property::XmlName##_type;                               \
        m_properties.insert(value_type);                                                 \
    }                                                                                    \
                                                                                         \
    Type PropertyList::get##TypeName(const std::string &name) const                      \
    {                                                                                    \
        auto it = m_properties.find(name);                                               \
        if (it == m_properties.end())                                                    \
            throw NoriException("Property '%s' is missing!", name);                      \
        if (it->second.type != Property::XmlName##_type)                                 \
            throw NoriException("Property '%s' has the wrong type! "                     \
                                "(expected <" #XmlName ">)!",                            \
                                name);                                                   \
        return it->second.value.XmlName##_value;                                         \
    }                                                                                    \
                                                                                         \
    Type PropertyList::get##TypeName(const std::string &name, const Type &defVal) const  \
    {                                                                                    \
        auto it = m_properties.find(name);                                               \
        if (it == m_properties.end())                                                    \
            return defVal;                                                               \
        if (it->second.type != Property::XmlName##_type)                                 \
            throw NoriException("Property '%s' has the wrong type! "                     \
                                "(expected <" #XmlName ">)!",                            \
                                name);                                                   \
        return it->second.value.XmlName##_value;                                         \
    }                                                                                    \
                                                                                         \
    std::vector<Type> PropertyList::getAll##TypeName(const std::string &name) const      \
    {                                                                                    \
        std::vector<Type> vec;                                                           \
        for (auto itr = m_properties.begin(); itr != m_properties.end(); ++itr)          \
        {                                                                                \
            if (itr->first != name)                                                      \
                continue;                                                                \
            if (itr->second.type != Property::XmlName##_type)                            \
                throw NoriException("Property '%s' has the wrong type! "                 \
                                    "(expected <" #XmlName ">)!",                        \
                                    name);                                               \
            vec.push_back(itr->second.value.XmlName##_value);                            \
        }                                                                                \
        if (vec.empty())                                                                 \
            throw NoriException("Property '%s' is missing!", name);                      \
        return vec;                                                                      \
    }

DEFINE_PROPERTY_ACCESSOR(bool, Boolean, boolean)
DEFINE_PROPERTY_ACCESSOR(int, Integer, integer)
DEFINE_PROPERTY_ACCESSOR(float, Float, float)
DEFINE_PROPERTY_ACCESSOR(Color3f, Color, color)
DEFINE_PROPERTY_ACCESSOR(Point3f, Point, point)
DEFINE_PROPERTY_ACCESSOR(Vector3f, Vector, vector)
DEFINE_PROPERTY_ACCESSOR(std::string, String, string)
DEFINE_PROPERTY_ACCESSOR(Transform, Transform, transform)

NORI_NAMESPACE_END
