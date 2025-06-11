#include "common/math.hpp"

double dist(const double x1, const double y1, const double x2, const double y2)
{
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

pcl::PointXY calculate_direction_vector(
    const pcl::PointXY &p1,
    const pcl::PointXY &p2
) {
    pcl::PointXY direction;
    direction.x = p2.x - p1.x;
    direction.y = p2.y - p1.y;
    return direction;
}

pcl::PointXY calculate_direction_vector(
    const geometry_msgs::msg::Pose &pose
) {
    float yaw = quaternion_to_yaw(pose.orientation);

    pcl::PointXY direction;
    direction.x = std::cos(yaw);
    direction.y = std::sin(yaw);

    return direction;
}

float quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q)
{
    // Convert quaternion to yaw (rotation around the Z axis)
    float siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}


CubicSplineInterpolator::CubicSplineInterpolator(const std::vector<float>& t_coords,
                                                 const std::vector<float>& x_coords,
                                                 const std::vector<float>& y_coords)
    : t(t_coords), x(x_coords), y(y_coords) {
    if (t.size() < 2 || t.size() != x.size() || t.size() != y.size()) {
        throw std::invalid_argument("Invalid input sizes.");
    }
    computeSplineCoefficients(t, x, ax, bx, cx, dx);
    computeSplineCoefficients(t, y, ay, by, cy, dy);
}

float CubicSplineInterpolator::interpolate_x(float t_val) const {
    return interpolate(t_val, ax, bx, cx, dx);
}

float CubicSplineInterpolator::interpolate_y(float t_val) const {
    return interpolate(t_val, ay, by, cy, dy);
}

void CubicSplineInterpolator::computeSplineCoefficients(
    const std::vector<float>& t,
    const std::vector<float>& values,
    std::vector<float>& a,
    std::vector<float>& b,
    std::vector<float>& c,
    std::vector<float>& d) {

    size_t n = t.size();
    a = values;
    std::vector<float> h(n - 1), alpha(n - 1), l(n), mu(n), z(n);
    b.resize(n - 1);
    c.resize(n);
    d.resize(n - 1);

    for (size_t i = 0; i < n - 1; ++i) {
        h[i] = t[i + 1] - t[i];
    }

    for (size_t i = 1; i < n - 1; ++i) {
        alpha[i] = (3.0f / h[i]) * (a[i + 1] - a[i])
                 - (3.0f / h[i - 1]) * (a[i] - a[i - 1]);
    }

    l[0] = 1.0f;
    mu[0] = 0.0f;
    z[0] = 0.0f;

    for (size_t i = 1; i < n - 1; ++i) {
        l[i] = 2.0f * (t[i + 1] - t[i - 1]) - h[i - 1] * mu[i - 1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    l[n - 1] = 1.0f;
    z[n - 1] = 0.0f;
    c[n - 1] = 0.0f;

    for (int j = static_cast<int>(n) - 2; j >= 0; --j) {
        c[j] = z[j] - mu[j] * c[j + 1];
        b[j] = (a[j + 1] - a[j]) / h[j]
             - h[j] * (c[j + 1] + 2.0f * c[j]) / 3.0f;
        d[j] = (c[j + 1] - c[j]) / (3.0f * h[j]);
    }

    // Resize to n-1 because only n-1 intervals between n points
    a.resize(n - 1);
    c.resize(n - 1);
}

float CubicSplineInterpolator::interpolate(
    float t_val,
    const std::vector<float>& a,
    const std::vector<float>& b,
    const std::vector<float>& c,
    const std::vector<float>& d) const {

    size_t n = t.size();
    if (t_val < t.front() || t_val > t.back()) {
        throw std::out_of_range("t_val is outside interpolation range");
    }

    size_t i = n - 2;
    for (size_t j = 0; j < n - 1; ++j) {
        if (t_val >= t[j] && t_val <= t[j + 1]) {
            i = j;
            break;
        }
    }

    float dt = t_val - t[i];
    return a[i] + b[i] * dt + c[i] * dt * dt + d[i] * dt * dt * dt;
}
