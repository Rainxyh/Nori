/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#pragma once

#include <nori/color.h>
#include <nori/transform.h>
#include <map>

NORI_NAMESPACE_BEGIN

/**
 * \brief This is an associative container used to supply the constructors
 * of \ref NoriObject subclasses with parameter information.
 */
class PropertyList {
public:
    PropertyList() { }

    /// Set a boolean property
    void setBoolean(const std::string &name, const bool &value);
    
    /// Get a boolean property, and throw an exception if it does not exist
    bool getBoolean(const std::string &name) const;

    /// Get a boolean property, and use a default value if it does not exist
    bool getBoolean(const std::string &name, const bool &defaultValue) const;

    std::vector<bool> getAllBoolean(const std::string &name) const;

    /// Set an integer property
    void setInteger(const std::string &name, const int &value);
    
    /// Get an integer property, and throw an exception if it does not exist
    int getInteger(const std::string &name) const;

    /// Get am integer property, and use a default value if it does not exist
    int getInteger(const std::string &name, const int &defaultValue) const;

    std::vector<int> getAllInteger(const std::string &name) const;

    /// Set a float property
    void setFloat(const std::string &name, const float &value);
    
    /// Get a float property, and throw an exception if it does not exist
    float getFloat(const std::string &name) const;

    /// Get a float property, and use a default value if it does not exist
    float getFloat(const std::string &name, const float &defaultValue) const;

    std::vector<float> getAllFloat(const std::string &name) const;

    /// Set a string property
    void setString(const std::string &name, const std::string &value);

    /// Get a string property, and throw an exception if it does not exist
    std::string getString(const std::string &name) const;

    /// Get a string property, and use a default value if it does not exist
    std::string getString(const std::string &name, const std::string &defaultValue) const;

    std::vector<std::string> getAllString(const std::string &name) const;

    /// Set a color property
    void setColor(const std::string &name, const Color3f &value);

    /// Get a color property, and throw an exception if it does not exist
    Color3f getColor(const std::string &name) const;

    /// Get a color property, and use a default value if it does not exist
    Color3f getColor(const std::string &name, const Color3f &defaultValue) const;

    std::vector<Color3f> getAllColor(const std::string &name) const;

    /// Set a point property
    void setPoint(const std::string &name, const Point3f &value);

    /// Get a point property, and throw an exception if it does not exist
    Point3f getPoint(const std::string &name) const;

    /// Get a point property, and use a default value if it does not exist
    Point3f getPoint(const std::string &name, const Point3f &defaultValue) const;

    std::vector<Point3f> getAllPoint(const std::string &name) const;

    /// Set a vector property
    void setVector(const std::string &name, const Vector3f &value);

    /// Get a vector property, and throw an exception if it does not exist
    Vector3f getVector(const std::string &name) const;

    /// Get a vector property, and use a default value if it does not exist
    Vector3f getVector(const std::string &name, const Vector3f &defaultValue) const;

    std::vector<Vector3f> getAllVector(const std::string &name) const;

    /// Set a transform property
    void setTransform(const std::string &name, const Transform &value);

    /// Get a transform property, and throw an exception if it does not exist
    Transform getTransform(const std::string &name) const;

    /// Get a transform property, and use a default value if it does not exist
    Transform getTransform(const std::string &name, const Transform &defaultValue) const;

    std::vector<Transform> getAllTransform(const std::string &name) const;

private:
    /* Custom variant data type (stores one of boolean/integer/float/...) */
    struct Property {
        enum Type{
            boolean_type, integer_type, float_type,
            string_type, color_type, point_type,
            vector_type, transform_type
        } type;

        /* Visual studio lacks support for unrestricted unions (as of ver. 2013) */
        struct Value
        {
            Value() : boolean_value(false) { }
            ~Value() { }

            bool        boolean_value;
            int         integer_value;
            float       float_value;
            std::string string_value;
            Color3f     color_value;
            Point3f     point_value;
            Vector3f    vector_value;
            Transform   transform_value;
        } value;

        Property() : type(boolean_type) { }
        Property(Type t, Value v) : type(t), value(v) {}
    };

    std::unordered_multimap<std::string, Property> m_properties;
};

NORI_NAMESPACE_END
