#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // Implement de Casteljau's algorithm
    size_t size = control_points.size();
    if (size == 1)
    {
        return *control_points.begin();
    }
    if (size == 0)
    {
        return cv::Point2f();
    }

    std::vector<cv::Point2f> new_control_points(size - 1);
    for (int i = 1; i < size; ++i)
    {
        const cv::Point2f& p0 = control_points[i - 1];
        const cv::Point2f& p1 = control_points[i];
        new_control_points[i - 1] = (1.0f - t) * p0 + t * p1;
    }
    return recursive_bezier(new_control_points, t);
}

void AntiAliasing(const cv::Point2f& point, cv::Mat& window)
{
    float cx = point.x, cy = point.y;
    int x0 = static_cast<int>(cx - 0.5f);
    int x1 = static_cast<int>(cx + 0.5f);
    int y0 = static_cast<int>(cy - 0.5f);
    int y1 = static_cast<int>(cy + 0.5f);
    std::vector<cv::Point2f> points(4);
    points[0].x = x0; points[0].y = y0;
    points[1].x = x1; points[1].y = y0;
    points[2].x = x0; points[2].y = y1;
    points[3].x = x1; points[3].y = y1;

    const cv::Point2f& nearest_point = points[3];
    window.at<cv::Vec3b>(nearest_point.y, nearest_point.x)[1] = 255;

    cv::Point2f dist_vec = point - nearest_point;
    float dist_sqr = dist_vec.dot(dist_vec);
    if (dist_sqr < FLT_EPSILON)
    {
        return;
    }
    float dist = sqrt(dist_sqr);

    for (int i = 0; i < 3; ++i)
    {
        const cv::Point2f& p = points[i];
        cv::Vec3b& color = window.at<cv::Vec3b>(p.y, p.x);
        if (color[1] == 255)
        {
            return;
        }
        cv::Point2f v = p - point;
        float d = sqrt(v.dot(v));
        color[1] = dist / d * 255;
    }
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's
    // recursive Bezier algorithm.
    if (control_points.empty())
    {
        return;
    }

    for (float t = 0.0f; t < 1.0f; t += 0.001f)
    {
        cv::Point2f point = recursive_bezier(control_points, t);
        AntiAliasing(point, window);
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        control_points.push_back({411, 396});
        control_points.push_back({263, 490});
        control_points.push_back({651, 61});
        control_points.push_back({640, 654});

        for (auto &point : control_points)
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
