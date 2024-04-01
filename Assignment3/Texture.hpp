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
        u = std::clamp(u, 0.0f, 1.0f);
        v = std::clamp(v, 0.0f, 1.0f);
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        u = std::clamp(u, 0.0f, 1.0f);
        v = std::clamp(v, 0.0f, 1.0f);
        float u_img = u * width;
        float v_img = (1.0f - v) * height;

        float cu = (u_img - (int) u_img) > 0.5f ? ceil(u_img) : floor(u_img);
        float cv = (v_img - (int) v_img) > 0.5f ? ceil(v_img) : floor(v_img);

        Eigen::Vector2f u00 = {cu - 0.5f, cv - 0.5f};
        Eigen::Vector2f u10 = {cu + 0.5f, cv - 0.5f};
        Eigen::Vector2f u01 = {cu - 0.5f, cv + 0.5f};
        Eigen::Vector2f u11 = {cu + 0.5f, cv + 0.5f};

        auto color_u00 = image_data.at<cv::Vec3b>(u00.y(), u00.x());
        auto color_u10 = image_data.at<cv::Vec3b>(u10.y(), u10.x());
        auto color_u01 = image_data.at<cv::Vec3b>(u01.y(), u01.x());
        auto color_u11 = image_data.at<cv::Vec3b>(u11.y(), u11.x());

        float s = (u_img - u00.x()) / (u10.x() - u00.x());
        float t = (v_img - u00.y()) / (u01.y() - u00.y());

        auto color_u0 = color_u00 + s * (color_u10 - color_u00);
        auto color_u1 = color_u01 + s * (color_u11 - color_u01);
        auto color = color_u0 + t * (color_u1 - color_u0);
        return {color[0], color[1], color[2]};
    }

};
#endif //RASTERIZER_TEXTURE_H
