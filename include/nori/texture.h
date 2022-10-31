
#pragma once

#include <nori/object.h>

NORI_NAMESPACE_BEGIN

class NoriTexture : public NoriObject
{
public:
    struct Mtl
    {
        // Material library file name
        std::string name;
        // Ambient color
        Color3f Ka;
        // Diffuse color
        Color3f Kd;
        // Specular color
        Color3f Ks;
        // Reflection Index. Defines the specularity of the reflection. The higher the value, the denser the highlights. Generally, the value ranges from 0 to 1000.
        float Ns;
        // Refraction value. Can take a value between 0.001 and 10. With a value of 1.0, the light does not bend as it passes through the object. The refractive index of glass is 1.5.
        float Ni;
        // Filter transmittance
        Color3f Tf;
        // Fade Index Description. The parameter factor indicates the number of objects blended into the background, and the value ranges from 0.0 to 1.0. A value of 1.0 means completely opaque, and a value of 0.0 means complete transparency.
        float d;
        // 0	Color on and Ambient off
        // 1	Color on and Ambient on
        // 2	Highlight on
        // 3	Reflection on and Ray trace on
        // 4	Transparency: Glass on / Reflection: Ray trace on
        // 5	Reflection: Fresnel on and Ray trace on
        // 6	Transparency: Refraction on / Reflection: Fresnel off and Ray trace on
        // 7	Transparency: Refraction on / Reflection: Fresnel on and Ray trace on
        // 8	Reflection on and Ray trace off
        // 9	Transparency: Glass on / Reflection: Ray trace off
        // 10	Casts shadows onto invisible surfaces
        uint8_t illum;
        // Diffuse specified color texture file
        std::string map_Kd;
    };

    NoriTexture() {}
    virtual Color3f getValue(std::string mtl_name, float u, float v) const = 0;
    virtual Color3f getValue(std::string mtl_name, const Point2f &p) const = 0;
    EClassType getClassType() const { return ETexture; }

protected:
    std::unordered_map<std::string, Mtl> mtl_dict;
};

NORI_NAMESPACE_END 