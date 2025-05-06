// Created by goksu on 4/6/19.

#include <algorithm>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <stdexcept>

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f>& positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return { id };
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i>& indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return { id };
}

// Bresenham's line drawing algorithm
// Code taken from a stack overflow answer: https://stackoverflow.com/a/16405254

void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    auto ax = begin.x();
    auto ay = begin.y();
    auto bx = end.x();
    auto by = end.y();

    Eigen::Vector3f line_color = { 255, 255, 255 };

    bool steep = std::abs(ax - bx) < std::abs(ay - by);

    if (steep) { // if the line is steep, we transpose the image
        std::swap(ax, ay);
        std::swap(bx, by);
    }
    if (ax > bx) { // make it left−to−right
        std::swap(ax, bx);
        std::swap(ay, by);
    }
    int y = ay;
    int ierror = 0;
    for (int x = ax; x <= bx; x++) {
        if (steep)
        {   // if transposed, de−transpose
            Eigen::Vector3f point = Eigen::Vector3f(y, x, 1.0f);
            set_pixel(point, line_color);
        }
        else
        {
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point, line_color);
        }
        ierror += 2 * std::abs(by - ay);
        if (ierror > bx - ax) {
            y += by > ay ? 1 : -1;
            ierror -= 2 * (bx - ax);
        }
    }
}

/// <summary>
/// 经典的 Bresenham 直线光栅化算法，用于在像素网格上绘制一条从点 begin 到 end 的直线
/// </summary>
//void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
//{
//    auto x1 = begin.x();
//    auto y1 = begin.y();
//    auto x2 = end.x();
//    auto y2 = end.y();
//
//    Eigen::Vector3f line_color = { 255, 255, 255 };
//
//    int x, y, dx, dy, dx1, dy1, px, py, xe, ye, i;
//
//    dx = x2 - x1;
//    dy = y2 - y1;
//    dx1 = fabs(dx);
//    dy1 = fabs(dy);
//    px = 2 * dy1 - dx1;
//    py = 2 * dx1 - dy1;
//
//    if (dy1 <= dx1)
//    {
//        if (dx >= 0)
//        {
//            x = x1;
//            y = y1;
//            xe = x2;
//        }
//        else
//        {
//            x = x2;
//            y = y2;
//            xe = x1;
//        }
//        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
//        set_pixel(point, line_color);
//        for (i = 0; x < xe; i++)
//        {
//            x = x + 1;
//            if (px < 0)
//            {
//                px = px + 2 * dy1;
//            }
//            else
//            {
//                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
//                {
//                    y = y + 1;
//                }
//                else
//                {
//                    y = y - 1;
//                }
//                px = px + 2 * (dy1 - dx1);
//            }
//
//            //delay(0);
//            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
//            set_pixel(point, line_color);
//        }
//    }
//    else
//    {
//        if (dy >= 0)
//        {
//            x = x1;
//            y = y1;
//            ye = y2;
//        }
//        else
//        {
//            x = x2;
//            y = y2;
//            ye = y1;
//        }
//        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
//        set_pixel(point, line_color);
//        for (i = 0; y < ye; i++)
//        {
//            y = y + 1;
//            if (py <= 0)
//            {
//                py = py + 2 * dx1;
//            }
//            else
//            {
//                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
//                {
//                    x = x + 1;
//                }
//                else
//                {
//                    x = x - 1;
//                }
//                py = py + 2 * (dx1 - dy1);
//            }
//
//            //delay(0);
//            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
//            set_pixel(point, line_color);
//        }
//    }
//}

//void rst::rasterizer::draw_line_antialiased(Eigen::Vector3f begin, Eigen::Vector3f end)
//{
//    auto x1 = begin.x();
//    auto y1 = begin.y();
//    auto x2 = end.x();
//    auto y2 = end.y();
//
//    Eigen::Vector3f line_color = { 255, 255, 255 };
//
//    // 斜率是否 > 1
//    bool steep = abs(y2 - y1) > abs(x2 - x1); 
//
//    // 交换x和y，确保斜率≤1
//    if (steep) 
//    { 
//        std::swap(x1, y1);
//        std::swap(x2, y2);
//    }
//
//    // 确保从左到右绘制
//    if (x1 > x2) 
//    { 
//        std::swap(x1, x2);
//        std::swap(y1, y2);
//    }
//
//    
//    float dx = x2 - x1;
//    float dy = y2 - y1;
//    float gradient = (dx == 0) ? 1 : dy / dx;
//
//    // 处理起点
//    float xend = round(x1);
//    float yend = y1 + gradient * (xend - x1);
//    float xgap = 1 - (x1 + 0.5) - floor(x1 + 0.5);
//    int xpxl1 = xend;
//    int ypxl1 = floor(yend);
//
//    if (steep) 
//    {
//        set_pixel_transparent(Eigen::Vector3f(ypxl1, xpxl1, 1.0f), line_color, (1 - (yend - floor(yend))) * xgap);
//        set_pixel_transparent(Eigen::Vector3f(ypxl1 + 1, xpxl1, 1.0f), line_color, (yend - floor(yend)) * xgap);
//    }
//    else {
//        set_pixel_transparent(Eigen::Vector3f(xpxl1, ypxl1, 1.0f), line_color, (1 - (yend - floor(yend))) * xgap);
//        set_pixel_transparent(Eigen::Vector3f(xpxl1, ypxl1 + 1, 1.0f), line_color, (yend - floor(yend)) * xgap);
//    }
//
//    // 中间点
//    float intery = yend + gradient;
//    xend = round(x2);
//    yend = y2 + gradient * (xend - x2);
//    xgap = (x2 + 0.5) - floor(x2 + 0.5);
//    int xpxl2 = xend;
//    int ypxl2 = floor(yend);
//
//    if (steep) 
//    {
//        set_pixel_transparent(Eigen::Vector3f(ypxl2, xpxl2, 1.0f), line_color, (1 - (yend - floor(yend))) * xgap);
//        set_pixel_transparent(Eigen::Vector3f(ypxl2 + 1, xpxl2, 1.0f), line_color, (yend - floor(yend)) * xgap);
//    }
//    else
//    {
//        set_pixel_transparent(Eigen::Vector3f(xpxl2, ypxl2, 1.0f), line_color, (1 - (yend - floor(yend))) * xgap);
//        set_pixel_transparent(Eigen::Vector3f(xpxl2, ypxl2 + 1, 1.0f), line_color, (yend - floor(yend)) * xgap);
//    }
//
//    // 主循环
//    if (steep) 
//    {
//        for (int x = xpxl1 + 1; x < xpxl2; x++)
//        {
//            set_pixel_transparent(Eigen::Vector3f(floor(intery), x, 1.0f), line_color, 1 - (intery - floor(intery)));
//            set_pixel_transparent(Eigen::Vector3f(floor(intery) + 1, x, 1.0f), line_color, intery - floor(intery));
//            intery += gradient;
//        }
//    }
//    else 
//    {
//        for (int x = xpxl1 + 1; x < xpxl2; x++) 
//        {
//            set_pixel_transparent(Eigen::Vector3f(x, floor(intery), 1.0f), line_color, 1 - (intery - floor(intery)));
//            set_pixel_transparent(Eigen::Vector3f(x, floor(intery) + 1, 1.0f), line_color, intery - floor(intery));
//            intery += gradient;
//        }
//    }
//}
//
//// 透明像素绘制函数
//void rst::rasterizer::set_pixel_transparent(Eigen::Vector3f point, Eigen::Vector3f color, float alpha)
//{
//    // alpha: 透明度（0-1），混合背景色
//    Eigen::Vector3f bg_color = { 0, 0, 0 }; // 假设获取背景色
//    Eigen::Vector3f final_color = bg_color * (1 - alpha) + color * alpha;
//    set_pixel(point, final_color);
//}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

void rst::rasterizer::draw(rst::pos_buf_id pos_buffer, rst::ind_buf_id ind_buffer, rst::Primitive type)
{
    if (type != rst::Primitive::Triangle)
    {
        throw std::runtime_error("Drawing primitives other than triangle is not implemented yet!");
    }

    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];

    float f1 = (100 - 0.1) / 2.0;
    float f2 = (100 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;

        Eigen::Vector4f v[] =
        {
                mvp * to_vec4(buf[i[0]], 1.0f),
                //{3.58518195, 0.00000000, 6.82765484, 7.00000000}

                mvp * to_vec4(buf[i[1]], 1.0f),
                // {0.00000000, 3.58518195, 6.82765484, 7.00000000}

                mvp * to_vec4(buf[i[2]], 1.0f)
                // {-3.58518195, 0.00000000, 6.82765484, 7.00000000}
        };

        for (auto& vec : v)
        {
            vec /= vec.w();
        }

        for (auto& vert : v)
        {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            // {529.259094, 350.000000, 98.7701950, 1.00000000}

            t.setVertex(i, v[i].head<3>());
            // {350.000000, 529.259094, 98.7701950, 1.00000000}

            t.setVertex(i, v[i].head<3>());
            // {170.740906, 350.000000, 98.7701950, 1.00000000}
        }

        t.setColor(0, 255.0, 0.0, 0.0);
        t.setColor(1, 0.0, 255.0, 0.0);
        t.setColor(2, 0.0, 0.0, 255.0);

        rasterize_wireframe(t);
    }
}

void rst::rasterizer::rasterize_wireframe(const Triangle& t)
{
    draw_line(t.c(), t.a());
    draw_line(t.c(), t.b());
    draw_line(t.b(), t.a());

    //draw_line_antialiased(t.c(), t.a());
    //draw_line_antialiased(t.c(), t.b());
    //draw_line_antialiased(t.b(), t.a());
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
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    if (point.x() < 0 || point.x() >= width || point.y() < 0 || point.y() >= height) 
        return;

    auto ind = (height - point.y()) * width + point.x();
    frame_buf[ind] = color;
}