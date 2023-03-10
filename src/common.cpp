#include <nori/object.h>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <filesystem/resolver.h>
#include <iomanip>

#if defined(PLATFORM_LINUX)
#include <malloc.h>
#endif

#if defined(PLATFORM_WINDOWS)
#include <windows.h>
#endif

#if defined(PLATFORM_MACOS)
#include <sys/sysctl.h>
#endif

NORI_NAMESPACE_BEGIN

std::string indent(const std::string &string, int amount) {
    /* This could probably be done faster (it's not
       really speed-critical though) */
    std::istringstream iss(string);
    std::ostringstream oss;
    std::string spacer(amount, ' ');
    bool firstLine = true;
    for (std::string line; std::getline(iss, line); ) {
        if (!firstLine)
            oss << spacer;
        oss << line;
        if (!iss.eof())
            oss << endl;
        firstLine = false;
    }
    return oss.str();
}

bool endsWith(const std::string &value, const std::string &ending) {
    if (ending.size() > value.size())
        return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

std::string StringtoLower(const std::string &value) {
    std::string result;
    result.resize(value.size());
    std::transform(value.begin(), value.end(), result.begin(), ::tolower);
    return result;
}

bool StringtoBool(const std::string &str) {
    std::string value = StringtoLower(str);
    if (value == "false")
        return false;
    else if (value == "true")
        return true;
    else
        throw NoriException("Could not parse boolean value \"%s\"", str);
}

int StringtoInt(const std::string &str) {
    char *end_ptr = nullptr;
    int result = (int) strtol(str.c_str(), &end_ptr, 10);
    if (*end_ptr != '\0')
        throw NoriException("Could not parse integer value \"%s\"", str);
    return result;
}

unsigned int StringtoUInt(const std::string &str) {
    char *end_ptr = nullptr;
    unsigned int result = (int) strtoul(str.c_str(), &end_ptr, 10);
    if (*end_ptr != '\0')
        throw NoriException("Could not parse integer value \"%s\"", str);
    return result;
}

float StringtoFloat(const std::string &str) {
    char *end_ptr = nullptr;
    float result = (float) strtof(str.c_str(), &end_ptr);
    if (*end_ptr != '\0')
        throw NoriException("Could not parse floating point value \"%s\"", str);
    return result;
}

Eigen::Vector3f StringtoVector3f(const std::string &str) {
    std::vector<std::string> tokens = tokenize(str);
    if (tokens.size() != 3)
        throw NoriException("Expected 3 values");
    Eigen::Vector3f result;
    for (int i=0; i<3; ++i)
        result[i] = StringtoFloat(tokens[i]);
    return result;
}

std::vector<std::string> tokenize(const std::string &string, const std::string &delim, bool includeEmpty) {
    std::string::size_type lastPos = 0, pos = string.find_first_of(delim, lastPos);
    std::vector<std::string> tokens;

    while (lastPos != std::string::npos) {
        if (pos != lastPos || includeEmpty)
            tokens.push_back(string.substr(lastPos, pos - lastPos));
        lastPos = pos;
        if (lastPos != std::string::npos) {
            lastPos += 1;
            pos = string.find_first_of(delim, lastPos);
        }
    }

    return tokens;
}

std::string timeString(double time, bool precise) {
    if (std::isnan(time) || std::isinf(time))
        return "inf";

    std::string suffix = "ms";
    if (time > 1000) {
        time /= 1000; suffix = "s";
        if (time > 60) {
            time /= 60; suffix = "m";
            if (time > 60) {
                time /= 60; suffix = "h";
                if (time > 12) {
                    time /= 12; suffix = "d";
                }
            }
        }
    }

    std::ostringstream os;
    os << std::setprecision(precise ? 4 : 1)
       << std::fixed << time << suffix;

    return os.str();
}

std::string memString(size_t size, bool precise) {
    double value = (double) size;
    const char *suffixes[] = {
        "B", "KiB", "MiB", "GiB", "TiB", "PiB"
    };
    int suffix = 0;
    while (suffix < 5 && value > 1024.0f) {
        value /= 1024.0f; ++suffix;
    }

    std::ostringstream os;
    os << std::setprecision(suffix == 0 ? 0 : (precise ? 4 : 1))
       << std::fixed << value << " " << suffixes[suffix];

    return os.str();
}

filesystem::resolver *getFileResolver() {
    static filesystem::resolver *resolver = new filesystem::resolver();
    return resolver;
}

Color3f Color3f::toSRGB() const { // gamma correct, from linear space to sRGB
    Color3f result;

    for (int i=0; i<3; ++i) {
        float value = coeff(i);

        if (value <= 0.0031308f)
            result[i] = 12.92f * value;
        else
            result[i] = (1.0f + 0.055f)
                * std::pow(value, 1.0f/2.4f) -  0.055f;
    }

    return result;
}

Color3f Color3f::toLinearRGB() const {
    Color3f result;

    for (int i=0; i<3; ++i) {
        float value = coeff(i);

        if (value <= 0.04045f)
            result[i] = value * (1.0f / 12.92f);
        else
            result[i] = std::pow((value + 0.055f)
                * (1.0f / 1.055f), 2.4f);
    }

    return result;
}

bool Color3f::isValid() const {
    for (int i=0; i<3; ++i) {
        float value = coeff(i);
        if (value < 0 || !std::isfinite(value))
            return false;
    }
    return true;
}

float Color3f::getLuminance() const {
    return coeff(0) * 0.212671f + coeff(1) * 0.715160f + coeff(2) * 0.072169f;
}

Transform::Transform(const Eigen::Matrix4f &trafo)
    : m_transform(trafo), m_inverse(trafo.inverse()) { }

std::string Transform::toString() const {
    std::ostringstream oss;
    oss << m_transform.format(Eigen::IOFormat(4, 0, ", ", ";\n", "", "", "[", "]"));
    return oss.str();
}

Transform Transform::operator*(const Transform &t) const {
    return Transform(m_transform * t.m_transform,
        t.m_inverse * m_inverse);
}

void stringIgnoreCase(std::string str, size_t depth, std::vector<std::string> &strList){ // depth first search
    if (depth == str.length()){
        strList.push_back(str);
        return;
    }
    stringIgnoreCase(str, depth + 1, strList);
    if (!isalpha(str[depth]))
        return;
    str[depth] ^= (1 << 5); // case conversion
    stringIgnoreCase(str, depth + 1, strList);
    str[depth] ^= (1 << 5); // conversion recovery, time travel is best not to affect the past
};

Vector3f sphericalDirection(float theta, float phi) {
    float sinTheta, cosTheta, sinPhi, cosPhi;

    sincosf(theta, &sinTheta, &cosTheta);
    sincosf(phi, &sinPhi, &cosPhi);

    return Vector3f(
        sinTheta * cosPhi,
        sinTheta * sinPhi,
        cosTheta
    );
}

Point2f sphericalCoordinates(const Vector3f &v) {
    Point2f result(
        std::acos(v.z()),
        std::atan2(v.y(), v.x())
    );
    if (result.y() < 0)
        result.y() += 2*M_PI;
    return result;
}

void coordinateSystem(const Vector3f &a, Vector3f &b, Vector3f &c) {
    if (std::abs(a.x()) > std::abs(a.y())) {
        float invLen = 1.0f / std::sqrt(a.x() * a.x() + a.z() * a.z());
        c = Vector3f(a.z() * invLen, 0.0f, -a.x() * invLen);
    } else {
        float invLen = 1.0f / std::sqrt(a.y() * a.y() + a.z() * a.z());
        c = Vector3f(0.0f, a.z() * invLen, -a.y() * invLen);
    }
    b = c.cross(a);
}

float fresnel(float cosThetaI, float extIOR, float intIOR) {
    Vector3f wo;
    return fresnel(cosThetaI, extIOR, intIOR, Vector3f(1.f), wo);
}

float fresnel(float cosThetaI, float extIOR, float intIOR, const Vector3f &wi, Vector3f &wo) {
    float etaI = extIOR, etaT = intIOR;

    if (extIOR == intIOR)
        return 0.0f;

    /* Swap the indices of refraction if the interaction starts
       at the inside of the object */
    if (cosThetaI < 0.0f)
    {
        std::swap(etaI, etaT);
        cosThetaI = -cosThetaI;
    }

    /* Using Snell's law, calculate the squared sine of the
       angle between the normal and the transmitted ray */
    float eta = etaI / etaT,
          sinThetaI = sqrt(1.f - pow(cosThetaI, 2)),
          sinThetaTSqr = eta * eta * (1 - cosThetaI * cosThetaI),
          cosThetaT = std::sqrt(1.0f - sinThetaTSqr);

    if (sinThetaTSqr > 1.0f)
        return 1.0f; /* Total internal reflection! */

    float Rs = (etaI * cosThetaI - etaT * cosThetaT) / (etaI * cosThetaI + etaT * cosThetaT);
    float Rp = (etaT * cosThetaI - etaI * cosThetaT) / (etaT * cosThetaI + etaI * cosThetaT);

    wo = Vector3f(sqrt(sinThetaTSqr) / sinThetaI * -wi.x(), sqrt(sinThetaTSqr) / sinThetaI * -wi.y(), cosThetaT / cosThetaI * -wi.z());

    return (Rs * Rs + Rp * Rp) / 2.0f;
}

Color3f sphericalGaussianApprox(const Color3f &F0, const Vector3f &v, const Vector3f &h){
    return F0 + (1.f - F0) * pow(2, (-5.55473 * v.dot(h) - 6.98316) * v.dot(h));
}

Color3f schlickApprox(const Color3f &F0, const Vector3f &v, const Vector3f &h){
    return F0 + (1.f - F0) * pow(1 - v.dot(h), 5);
}

NORI_NAMESPACE_END
