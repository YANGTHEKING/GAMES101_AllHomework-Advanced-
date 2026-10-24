//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        u = std::fmin(1.0f, std::fmax(u, 0.0f));
        v = std::fmin(1.0f, std::fmax(v, 0.0f));

        const int u_img = static_cast<int>(u * (width - 1));
        const int v_img = static_cast<int>((1.0f - v) * (height - 1));
        const auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        u = std::fmin(1.0f, std::fmax(u, 0.0f));
        v = std::fmin(1.0f, std::fmax(v, 0.0f));

        const float x = u * (width - 1);
        const float y = (1.0f - v) * (height - 1);
        const int x0 = static_cast<int>(x);
        const int y0 = static_cast<int>(y);
        const int x1 = x0 + 1 < width ? x0 + 1 : width - 1;
        const int y1 = y0 + 1 < height ? y0 + 1 : height - 1;
        const float tx = x - x0;
        const float ty = y - y0;

        const auto sample = [this](int px, int py) {
            const auto color = image_data.at<cv::Vec3b>(py, px);
            return Eigen::Vector3f(color[0], color[1], color[2]);
        };

        const Eigen::Vector3f c00 = sample(x0, y0);
        const Eigen::Vector3f c10 = sample(x1, y0);
        const Eigen::Vector3f c01 = sample(x0, y1);
        const Eigen::Vector3f c11 = sample(x1, y1);
        const Eigen::Vector3f top = c00 + (c10 - c00) * tx;
        const Eigen::Vector3f bottom = c01 + (c11 - c01) * tx;
        return top + (bottom - top) * ty;
    }
};
#endif //RASTERIZER_TEXTURE_H
