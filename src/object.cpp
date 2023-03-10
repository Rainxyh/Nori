#include <nori/object.h>

NORI_NAMESPACE_BEGIN

void NoriObject::addChild(NoriObject *) {
    throw NoriException(
        "NoriObject::addChild() is not implemented for objects of type '%s'!",
        classTypeName(getClassType()));
}

void NoriObject::activate() { /* Do nothing */ }
void NoriObject::setParent(NoriObject *) { /* Do nothing */ }

std::map<std::string, NoriObjectFactory::Constructor> *NoriObjectFactory::m_constructors = nullptr; // Static member variables initialized outside the class

// Save corresponding constructor in map
void NoriObjectFactory::registerClass(const std::string &name, const Constructor &constr) {
    if (!m_constructors)
        m_constructors = new std::map<std::string, NoriObjectFactory::Constructor>();
    (*m_constructors)[name] = constr;
}

// Call the corresponding constructor to obtain an instance by parsing the xml parameter list obtained
 NoriObject *NoriObjectFactory::createInstance(const std::string &name, const PropertyList &propList){
    if (!m_constructors || m_constructors->end() == m_constructors->find(name))
        throw NoriException("A constructor for class \"%s\" could not be found!", name);
    return (*m_constructors)[name](propList); // call certain construct (std::function<NoriObject *(const PropertyList &)>)
 }

NORI_NAMESPACE_END
