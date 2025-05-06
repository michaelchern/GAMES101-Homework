#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate <<
        1, 0, 0, -eye_pos[0],
        0, 1, 0, -eye_pos[1],
        0, 0, 1, -eye_pos[2],
        0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    // 计算用的是弧度
    float angle = rotation_angle * MY_PI / 180.0f;

    // 模型绕Z轴的旋转矩阵
    model <<
        std::cos(angle), -std::sin(angle), 0, 0,
        std::sin(angle), std::cos(angle), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    // 第一步：将透视视锥体变成一个矩形
    Eigen::Matrix4f M_p2o;
    M_p2o <<
        zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zFar + zNear, zFar* zNear,
        0, 0, -1, 0;

    // 第二步：将视锥体中心位移到观察空间原点中心
    Eigen::Matrix4f M_trans;
    M_trans <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, (zFar + zNear) / 2,
        0, 0, 0, 1;

    // 第三步：将长方体视锥体的xyz坐标范围映射到(-1, 1)长宽高为2的正方体中
    Eigen::Matrix4f M_scale;
    M_scale <<
        1 / (aspect_ratio * zNear * std::tan(eye_fov / 2)), 0, 0, 0,
        0, 1 / (zNear * std::tan(eye_fov / 2)), 0, 0,
        0, 0, -(2 / (zFar - zNear)), 0,
        0, 0, 0, 1;

    projection = M_scale * M_trans * M_p2o;

    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    double fangle = angle / 180 * MY_PI;
    Eigen::Matrix4f rotation4 = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f rotation3 = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();

    Eigen::Matrix3f N;
    N << 0, -axis.z(), axis.y(),
        axis.z(), 0, -axis.x(),
        -axis.y(), axis.x(), 0;

    rotation3 = cos(fangle) * I + (1 - cos(fangle)) * axis * axis.transpose() + sin(fangle) * N;

    rotation4 << rotation3(0, 0), rotation3(0, 1), rotation3(0, 2), 0,
        rotation3(1, 0), rotation3(1, 1), rotation3(1, 2), 0,
        rotation3(2, 0), rotation3(2, 1), rotation3(2, 2), 0,
        0, 0, 0, 1;

    return rotation4;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3)
    {
        command_line = true;
        // -r by default
        angle = std::stof(argv[2]); 

        if (argc == 4)
        {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = { 0, 0, 5 };

    std::vector<Eigen::Vector3f> pos{ {2, 0, -2}, {0, 2, -2}, {-2, 0, -2} };

    std::vector<Eigen::Vector3i> ind{ {0, 1, 2} };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a')
        {
            angle += 10;
        }
        else if (key == 'd')
        {
            angle -= 10;
        }
    }

    return 0;
}
