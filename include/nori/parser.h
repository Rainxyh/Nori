#pragma once

#include <nori/object.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Load a scene from the specified filename and
 * return its root object
 */
extern NoriObject *loadFromXML(const std::string &filename);

NORI_NAMESPACE_END
