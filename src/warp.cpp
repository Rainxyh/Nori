/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample)
{
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample)
{
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToTent(const Point2f &sample)
{
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

float Warp::squareToTentPdf(const Point2f &p)
{
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

Point2f Warp::squareToUniformDisk(const Point2f &sample)
{
    float xi1 = sample.x(), xi2 = sample.y();
    float x, y;
    x = pow(xi2, 1.f / 2.f) * cos(2 * M_PI * xi1);
    y = pow(xi2, 1.f / 2.f) * sin(2 * M_PI * xi1);
    Point2f squareToTent_sample(x, y);
    return squareToTent_sample;
}

float Warp::squareToUniformDiskPdf(const Point2f &p)
{
    if (sqrt(pow(p.x(), 2) + pow(p.y(), 2)) > 1)
        return 0;
    return 1.f / M_PI;
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample)
{
    float xi1 = sample.x(), xi2 = sample.y();
    float x, y, z, phi, cos_theta, sin_theta;
    phi = 2 * M_PI * xi1;
    cos_theta = 1 - 2 * xi2;
    sin_theta = sqrt(1 - pow(cos_theta, 2));
    x = sin_theta * cos(phi);
    y = sin_theta * sin(phi);
    z = cos_theta;
    Point3f squareToTent_sample(x, y, z);
    return squareToTent_sample;
}

float Warp::squareToUniformSpherePdf(const Vector3f &v)
{
    if (fabs(1.f - sqrt(pow(v.x(), 2) + pow(v.y(), 2) + pow(v.z(), 2))) > 1e-5)
        return 0;
    return 1.f / (4.f * M_PI);
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample)
{
    float xi1 = sample.x(), xi2 = sample.y();
    float x, y, z, phi, cos_theta, sin_theta;
    phi = 2 * M_PI * xi1;
    cos_theta = 1 - xi2;
    sin_theta = sqrt(1 - pow(cos_theta, 2));
    x = sin_theta * cos(phi);
    y = sin_theta * sin(phi);
    z = cos_theta;
    Point3f squareToTent_sample(x, y, z);
    return squareToTent_sample;
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v)
{
    if (fabs(1.f - sqrt(pow(v.x(), 2) + pow(v.y(), 2) + pow(v.z(), 2))) > 1e-5 || (v.z() < 0))
        return 0;
    return 1.f / (2.f * M_PI);
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample)
{
    float xi1 = sample.x(), xi2 = sample.y();
    float x, y, z, phi, cos_theta, sin_theta;
    phi = 2 * M_PI * xi1;
    sin_theta = sqrt(xi2);
    cos_theta = sqrt(1 - xi2);
    x = sin_theta * cos(phi);
    y = sin_theta * sin(phi);
    z = cos_theta;
    Point3f squareToTent_sample(x, y, z);
    return squareToTent_sample;
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v)
{
    if (fabs(1.f - sqrt(pow(v.x(), 2) + pow(v.y(), 2) + pow(v.z(), 2))) > 1e-5 || (v.z() <= 0))
        return 0;
    return v.z() / M_PI;
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha)
{
    float xi1 = sample.x(), xi2 = sample.y();
    float x, y, z;
    float tan2theta = -pow(alpha, 2) * log(1 - xi2);
    x = 1.f / (sqrt(1.f + 1.f / tan2theta)) * cos(2 * M_PI * xi1);
    y = 1.f / (sqrt(1.f + 1.f / tan2theta)) * sin(2 * M_PI * xi1);
    z = 1 / (sqrt(1 + tan2theta));
    Point3f squareToTent_sample(x, y, z);
    return squareToTent_sample;
}

float Warp::squareToBeckmannPdf(const Vector3f &v, float alpha)
{
    if ((v.z() <= 0))
        return 0; // must exclude when z=0, z be used as the denominator
    return exp(-(1.f - pow(v.z(), 2)) / (pow(alpha, 2) * pow(v.z(), 2))) / (pow(alpha, 2) * pow(v.z(), 3)) * INV_PI;
}

NORI_NAMESPACE_END
