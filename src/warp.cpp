#include <nori/frame.h>
#include <nori/vector.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN
/* Tent */
Point2f Warp::squareToTent(const Point2f& sample) {
    float xi1 = sample.x(), xi2 = sample.y();
    float x, y;
    if (xi1 < 0.5f)
        x = sqrt(2 * xi1) - 1;
    else
        x = 1 - sqrt(2 * (1 - xi1));
    if (xi2 < 0.5f)
        y = sqrt(2 * xi2) - 1;
    else
        y = 1 - sqrt(2 * (1 - xi2));
    Point2f squareToTent_sample(x, y);
    return squareToTent_sample;
}
float Warp::squareToTentPdf(const Point2f& p) {
    float pdf = 1.f;
    if (fabs(p.x()) <= 1)
        pdf *= (1 - fabs(p.x()));
    else
        pdf *= 0;
    if (fabs(p.y()) <= 1)
        pdf *= (1 - fabs(p.y()));
    else
        pdf *= 0;
    return pdf;
}

/* Disk */
Point2f Warp::squareToUniformDisk(const Point2f& sample) {
    float r = sqrtf(sample.x());
    float theta = 2 * M_PI * sample.y();
    float x = r * cos(theta);
    float y = r * sin(theta);
    return Point2f(x, y);
}
float Warp::squareToUniformDiskPdf(const Point2f& p) {
    if (p.x() * p.x() + p.y() * p.y() <= 1.0f)
        return INV_PI;
    else
        return 0.0f;
}

/* Uniform Square */
Point2f Warp::squareToUniformSquare(const Point2f& sample) {
    return sample;
}
float Warp::squareToUniformSquarePdf(const Point2f& sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f
                                                                        : 0.0f;
}

/* Uniform Cylinder */
Vector3f Warp::squareToUniformCylinder(const Point2f& sample) {
    double phi = 2.0 * M_PI * sample.y();
    return Vector3f(std::cos(phi), std::sin(phi), 2 * sample.x() - 1);
}
Vector3f Warp::squareToUniformCylinderPdf(const Point2f& sample) {
    return INV_FOURPI;
}

/* Uniform Sphere */
Vector3f Warp::squareToUniformSphere(const Point2f& sample) {
    float theta = acosf(1.0f - 2 * sample.x());
    float phi = 2.0f * M_PI * sample.y();
    return Vector3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
}
float Warp::squareToUniformSpherePdf(const Vector3f& v) {
    return INV_FOURPI;
}

/* Uniform Sphere Cap */
Vector3f Warp::squareToUniformSphereCap(const Point2f& sample,
                                        float cosThetaMax) {
    float theta = acosf(1.0f - (1.0f - cosThetaMax) * sample.x());
    float phi = 2.0f * M_PI * sample.y();
    return Vector3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
}
float Warp::squareToUniformSphereCapPdf(const Vector3f& v, float cosThetaMax) {
    float cos_theta = Frame::cosTheta(v);
    if (cos_theta < cosThetaMax)
        return 0.0f;
    return INV_TWOPI * (1.0f / (1.0f - cosThetaMax));
}

/* Uniform Hemisphere */
Vector3f Warp::squareToUniformHemisphere(const Point2f& sample) {
    float theta = acosf(sample.x());
    float phi = 2 * M_PI * sample.y();
    return Vector3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
}
float Warp::squareToUniformHemispherePdf(const Vector3f& v) {
    float cos_theta = Frame::cosTheta(v);
    if (cos_theta < 0.0f)
        return 0.0f;
    return 2.0f * INV_FOURPI;
}

/* Cosine Hemisphere */
Vector3f Warp::squareToCosineHemisphere(const Point2f& sample) {
    float theta = 0.5f * acosf(1.0f - 2 * sample.x());
    float phi = 2.0f * M_PI * sample.y();
    return Vector3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
}
float Warp::squareToCosineHemispherePdf(const Vector3f& v) {
    float cos_theta = Frame::cosTheta(v);
    if (cos_theta < 0.0f)
        return 0.0f;
    return cos_theta / M_PI;
}

/* Beckmann */
Vector3f Warp::squareToBeckmann(const Point2f& sample, float alpha) {
    float theta = atanf(sqrtf(-(alpha * alpha * log(sample.x()))));
    float phi = 2.0f * M_PI * sample.y();
    return Vector3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
}
float Warp::squareToBeckmannPdf(const Vector3f& v, float alpha) {
    float cos_theta = Frame::cosTheta(v);
    // must exclude when z=0, z be used as the denominator
    if (cos_theta <= 0.0f)
        return 0.0f;

    float theta = acosf(cos_theta);
    float cos_theta2 = cos_theta * cos_theta;
    float cos_theta4 = cos_theta2 * cos_theta2;
    float tan_theta = tan(theta);
    float tan_theta2 = tan_theta * tan_theta;
    float alpha2 = alpha * alpha;

    float D_theta = (expf(-tan_theta2 / alpha2)) / (M_PI * alpha2 * cos_theta4);
    return D_theta * cos_theta;
}

/* Ggx */
Vector3f Warp::squareToGgx(const Point2f& sample, float alpha_g) {
    float theta = atanf(alpha_g * sqrtf(sample.x()) / sqrtf(1 - sample.x()));
    float phi = 2.0f * M_PI * sample.y();
    return Vector3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
}
float Warp::squareToGgxPdf(const Vector3f& m, float alpha_g) {
    float cos_theta = Frame::cosTheta(m);
    if (cos_theta <= 0.0f)
        return 0.0f;

    float theta = acosf(cos_theta);
    float cos_theta2 = cos_theta * cos_theta;
    float cos_theta4 = cos_theta2 * cos_theta2;
    float tan_theta = tan(theta);
    float tan_theta2 = tan_theta * tan_theta;

    float alpha2 = alpha_g * alpha_g;
    float term = alpha2 + tan_theta2;
    float term2 = term * term;

    float D = alpha2 / (M_PI * cos_theta4 * term2);
    return D * cos_theta;
}

/* Phong */
Vector3f Warp::squareToPhong(const Point2f& sample, float alpha_p) {
    float theta = acosf(std::pow(sample.x(), 1.0f / (alpha_p + 2.0f)));
    float phi = 2 * M_PI * sample.y();
    return Vector3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
}
float Warp::squareToPhongPdf(const Vector3f& m, float alpha_p) {
    float cos_theta = Frame::cosTheta(m);
    if (cos_theta <= 0.0f)
        return 0.0f;

    return (alpha_p + 2.0f) * INV_TWOPI * std::pow(cos_theta, alpha_p) *
           cos_theta;
}

NORI_NAMESPACE_END
