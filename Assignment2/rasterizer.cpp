// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

constexpr auto SSAA = true;
constexpr auto MSAA = false;

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(float x, float y, const Vector3f* _v)
{   

    Vector3f P = { x,y,0 };
    Vector3f A = _v[0];
    Vector3f B = _v[1];
    Vector3f C = _v[2];

    Vector3f AB = B - A;
    Vector3f AP = P - A;

    Vector3f BC = C - B;
    Vector3f BP = P - B;

    Vector3f CA = A - C;
    Vector3f CP = P - C;

    //类定义里面已经定义是逆时针，所以只用考虑同正情况。
    return AB.cross(AP).z() > 0 && BC.cross(BP).z() > 0 && CA.cross(CP).z() > 0;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }

    if (SSAA)
    {
        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                Eigen::Vector3f color(0, 0, 0);
                //float infinity = std::numeric_limits<float>::infinity();
                for (float i = SSAA_pixel_point; i < 1.0; i += SSAA_pixel_size)
                {
                    for (float j = SSAA_pixel_point; j < 1.0; j += SSAA_pixel_size)
                    {
                        //int index = get_index_SSAA(x, y, i, j);
                        //color += frame_buf_SSAA[index];
                    }
                }

                set_pixel(Vector3f(x, y, 0), color / (SSAA_h * SSAA_w));
            }
        }
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t)
{
    auto v = t.toVector4();
    int bbminx = std::min(std::min(v[0].x(), v[1].x()), v[2].x());
    int bbmaxx = std::max(std::max(v[0].x(), v[1].x()), v[2].x());
    int bbminy = std::min(std::min(v[0].y(), v[1].y()), v[2].y());
    int bbmaxy = std::max(std::max(v[0].y(), v[1].y()), v[2].y());

    for (int x = bbminx; x <= bbmaxx; x++)
    {
        for (int y = bbminy; y <= bbmaxy; y++)
        {
            if (SSAA)
            {
                for (float i = SSAA_pixel_point; i < 1.0; i += SSAA_pixel_size)
                {
                    for (float j = SSAA_pixel_point; j < 1.0; j += SSAA_pixel_size)
                    {
                        if (insideTriangle(x + i, y + j, t.v))
                        {
                            auto [alpha, beta, gamma] = computeBarycentric2D(x + i, y + j, t.v);
                            float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                            z_interpolated *= w_reciprocal;

                            int index = get_index_SSAA(x, y, i, j);
                            if (depth_buf_SSAA[index] > z_interpolated)
                            {
                                depth_buf_SSAA[index] = z_interpolated;
                                frame_buf_SSAA[index] = t.getColor();
                            }
                        }
                    }
                }
            }
            else if (MSAA)
            {
                float count = 0.0;
                float max_count = SSAA_w * SSAA_h;
                for (float i = SSAA_pixel_point; i < 1.0; i += SSAA_pixel_size)
                {
                    for (float j = SSAA_pixel_point; j < 1.0; j += SSAA_pixel_size)
                    {
                        if (insideTriangle(x + i, y + j, t.v))
                        {
                            count += 1.0;
                        }
                    }
                }

                if (insideTriangle(x + 0.5, y + 0.5, t.v))
                {
                    auto [alpha, beta, gamma] = computeBarycentric2D((float)x + 0.5, (float)y + 0.5, t.v);
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    int index = get_index(x, y);
                    if (depth_buf[index] > z_interpolated)
                    {
                        depth_buf[index] = z_interpolated;
                        Eigen::Vector3f color = t.getColor() * (count / max_count);
                        set_pixel(Vector3f(x, y, z_interpolated), color);
                    }
                }

            }
            else
            {
                if (insideTriangle(x + 0.5, y + 0.5, t.v))
                {
                    auto [alpha, beta, gamma] = computeBarycentric2D((float)x + 0.5, (float)y + 0.5, t.v);
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    if (depth_buf[get_index(x, y)] > z_interpolated)
                    {
                        depth_buf[get_index(x, y)] = z_interpolated;
                        set_pixel(Vector3f(x, y, z_interpolated), t.getColor());
                    }
                }
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(frame_buf_SSAA.begin(), frame_buf_SSAA.end(), Eigen::Vector3f{ 0, 0, 0 });
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(depth_buf_SSAA.begin(), depth_buf_SSAA.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);

    // SSAA超采样，每个像素划分成四个独立的小像素
    frame_buf_SSAA.resize(w * SSAA_w * h * SSAA_h);
    depth_buf_SSAA.resize(w * SSAA_w * h * SSAA_h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height - 1 - y) * width + x;
}

int rst::rasterizer::get_index_SSAA(int x, int y, float i, float j)
{
    int SSAA_height = height * SSAA_h;
    int SSAA_width = width * SSAA_w;

    i = int((i - SSAA_pixel_point) / SSAA_pixel_size);
    j = int((j - SSAA_pixel_point) / SSAA_pixel_size);

    return (SSAA_height - 1 - y * SSAA_h + j) * SSAA_width + x * SSAA_w + i;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height - 1 - point.y()) * width + point.x();
    frame_buf[ind] = color;
}

// clang-format on