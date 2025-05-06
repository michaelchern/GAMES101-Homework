// Created by LEI XU on 4/11/19.

#ifndef RASTERIZER_TRIANGLE_H
#define RASTERIZER_TRIANGLE_H

#include <Eigen/Eigen>

using namespace Eigen;

class Triangle
{
public:
    /// <summary>
    /// the original coordinates of the triangle, v0, v1, v2 in counter clockwise order
    /// Per vertex values
    /// </summary>
    Vector3f v[3];

    /// <summary>
    /// color at each vertex
    /// </summary>
    Vector3f color[3];

    /// <summary>
    /// texture u,v
    /// </summary>
    Vector2f tex_coords[3];

    /// <summary>
    /// normal vector for each vertex
    /// </summary>
    Vector3f normal[3];

    // Texture *tex;
    Triangle();

    Eigen::Vector3f a() const { return v[0]; }
    Eigen::Vector3f b() const { return v[1]; }
    Eigen::Vector3f c() const { return v[2]; }

    /// <summary>
    /// set i-th vertex coordinates
    /// </summary>
    void setVertex(int ind, Vector3f ver);

    /// <summary>
    /// set i-th vertex normal vector
    /// </summary>
    void setNormal(int ind, Vector3f n);

    /// <summary>
    /// set i-th vertex color
    /// </summary>
    void setColor(int ind, float r, float g, float b);

    /// <summary>
    /// set i-th vertex texture coordinate
    /// </summary>
    void setTexCoord(int ind, float s, float t);

    std::array<Vector4f, 3> toVector4() const;
};

#endif
