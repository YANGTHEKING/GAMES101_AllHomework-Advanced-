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
        u = std::fmin(1, std::fmax(u, 0));
        v = std::fmin(1, std::fmax(v, 0));

        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
    Eigen::Vector3f getColorBilinear(float u, float v){
        float w1 = int(u * width), h1 = int(v * height);
        float w2 = w1 + 1, h2 = h1;
        float w3 = w1, h3 = h1 + 1;
        float w4 = w1 + 1, h4 = h1 + 1;

        Eigen::Vector3f color1, color2, color3, color4, color5, color6, color;
        color1 = getColor(w1 / width, h1 / height);
        color2 = getColor(w2 / width, h2 / height);
        color3 = getColor(w3 / width, h3 / height);
        color4 = getColor(w4 / width, h4 / height);
        color5 = color1 + (color2 - color1) * (u * width - w1);
        color6 = color3 + (color4 - color3) * (u * width - w1);
        color = color5 + (color6 - color5) * (v * height - h1);
        return color;
    }
};
#endif //RASTERIZER_TEXTURE_H
