#include "ray_casting.h"
#include <cmath>
#include <algorithm>

// OpenMP removed for thread-safety with OctoMap

namespace sonar_3d_reconstruction
{

RayCasting::RayCasting()
{
}

RayCasting::~RayCasting()
{
}

std::vector<Eigen::Vector3d> RayCasting::generate_ray_points(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& endpoint,
    double resolution,
    double max_range
) const
{
    std::vector<Eigen::Vector3d> points;
    
    Eigen::Vector3d direction = endpoint - origin;
    double ray_length = direction.norm();
    
    // Apply max range constraint
    if (max_range > 0.0 && ray_length > max_range) {
        ray_length = max_range;
    }
    
    if (ray_length < 1e-6) {
        return points;  // Ray too short
    }
    
    direction.normalize();
    
    // Calculate number of steps
    int num_steps = static_cast<int>(std::ceil(ray_length / resolution));
    double step_size = ray_length / num_steps;
    
    // Generate points along ray
    points.reserve(num_steps + 1);
    for (int i = 0; i <= num_steps; ++i) {
        double t = i * step_size;
        Eigen::Vector3d point = origin + t * direction;
        points.push_back(point);
    }
    
    return points;
}

std::vector<std::vector<Eigen::Vector3d>> RayCasting::cast_multiple_rays(
    const Eigen::Vector3d& origin,
    const std::vector<Eigen::Vector3d>& endpoints,
    double resolution,
    double max_range
) const
{
    std::vector<std::vector<Eigen::Vector3d>> all_ray_points(endpoints.size());
    
    // Sequential processing for thread-safety with OctoMap
    // Ray casting itself is thread-safe but used with OctoMap which is not
    for (size_t i = 0; i < endpoints.size(); ++i) {
        all_ray_points[i] = generate_ray_points(origin, endpoints[i], resolution, max_range);
    }
    
    return all_ray_points;
}

std::vector<Eigen::Vector3d> RayCasting::generate_sonar_beam_pattern(
    const Eigen::Vector3d& origin,
    double range,
    double bearing_angle,
    double vertical_aperture,
    int num_vertical_samples
) const
{
    std::vector<Eigen::Vector3d> beam_points;
    
    double half_aperture = vertical_aperture / 2.0;
    
    // Generate points along vertical aperture
    for (int v = 0; v < num_vertical_samples; ++v) {
        double vertical_angle;
        
        if (num_vertical_samples == 1) {
            vertical_angle = 0.0;
        } else {
            double t = static_cast<double>(v) / (num_vertical_samples - 1);
            vertical_angle = -half_aperture + t * vertical_aperture;
        }
        
        // Sonar coordinate system: +X forward, +Y right, +Z down
        double x_sonar = range * std::cos(vertical_angle) * std::cos(bearing_angle);
        double y_sonar = -range * std::cos(vertical_angle) * std::sin(bearing_angle);  // Note negative for right-hand rule
        double z_sonar = range * std::sin(vertical_angle);
        
        Eigen::Vector3d beam_point = origin + Eigen::Vector3d(x_sonar, y_sonar, z_sonar);
        beam_points.push_back(beam_point);
    }
    
    return beam_points;
}

double RayCasting::ray_sphere_intersection(
    const Eigen::Vector3d& ray_origin,
    const Eigen::Vector3d& ray_direction,
    const Eigen::Vector3d& sphere_center,
    double sphere_radius
) const
{
    Eigen::Vector3d oc = ray_origin - sphere_center;
    
    double a = ray_direction.dot(ray_direction);
    double b = 2.0 * oc.dot(ray_direction);
    double c = oc.dot(oc) - sphere_radius * sphere_radius;
    
    double discriminant = b * b - 4 * a * c;
    
    if (discriminant < 0) {
        return -1.0;  // No intersection
    }
    
    double sqrt_discriminant = std::sqrt(discriminant);
    double t1 = (-b - sqrt_discriminant) / (2.0 * a);
    double t2 = (-b + sqrt_discriminant) / (2.0 * a);
    
    // Return the closest positive intersection
    if (t1 > 0) {
        return t1;
    } else if (t2 > 0) {
        return t2;
    } else {
        return -1.0;  // No forward intersection
    }
}

bool RayCasting::is_point_in_sonar_fov(
    const Eigen::Vector3d& sonar_origin,
    const Eigen::Vector3d& sonar_direction,
    const Eigen::Vector3d& point,
    double horizontal_fov,
    double vertical_aperture,
    double max_range
) const
{
    Eigen::Vector3d to_point = point - sonar_origin;
    double distance = to_point.norm();
    
    // Check range
    if (distance > max_range || distance < 1e-6) {
        return false;
    }
    
    to_point.normalize();
    
    // Compute angles relative to sonar direction
    auto [bearing, elevation, range] = compute_sonar_angles(sonar_origin, sonar_direction, point);
    
    // Check horizontal FOV
    if (std::abs(bearing) > horizontal_fov / 2.0) {
        return false;
    }
    
    // Check vertical aperture
    if (std::abs(elevation) > vertical_aperture / 2.0) {
        return false;
    }
    
    return true;
}

std::tuple<double, double, double> RayCasting::compute_sonar_angles(
    const Eigen::Vector3d& sonar_origin,
    const Eigen::Vector3d& sonar_direction,
    const Eigen::Vector3d& point
) const
{
    Eigen::Vector3d to_point = point - sonar_origin;
    double range = to_point.norm();
    
    if (range < 1e-6) {
        return {0.0, 0.0, 0.0};
    }
    
    to_point.normalize();
    
    // Create local coordinate system
    Eigen::Vector3d forward = sonar_direction.normalized();
    Eigen::Vector3d up = Eigen::Vector3d(0, 0, 1);  // Assume Z-up world
    Eigen::Vector3d right = forward.cross(up).normalized();
    up = right.cross(forward).normalized();
    
    // Project to_point onto local axes
    double forward_component = to_point.dot(forward);
    double right_component = to_point.dot(right);
    double up_component = to_point.dot(up);
    
    // Compute bearing (horizontal angle)
    double bearing = std::atan2(right_component, forward_component);
    
    // Compute elevation (vertical angle)  
    double elevation = std::atan2(up_component, std::sqrt(forward_component * forward_component + right_component * right_component));
    
    return {bearing, elevation, range};
}

std::vector<Eigen::Vector3d> RayCasting::generate_beam_voxels(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& direction,
    double range,
    double beam_width,
    double resolution
) const
{
    std::vector<Eigen::Vector3d> voxels;
    
    Eigen::Vector3d forward = direction.normalized();
    
    // Create perpendicular vectors for beam cone
    Eigen::Vector3d up = Eigen::Vector3d(0, 0, 1);
    if (std::abs(forward.dot(up)) > 0.9) {
        up = Eigen::Vector3d(0, 1, 0);  // Use Y if forward is too close to Z
    }
    
    Eigen::Vector3d right = forward.cross(up).normalized();
    up = right.cross(forward).normalized();
    
    double half_beam_width = beam_width / 2.0;
    
    // Sample within beam cone
    int num_range_steps = static_cast<int>(std::ceil(range / resolution));
    int num_radial_steps = static_cast<int>(std::ceil(range * std::tan(half_beam_width) / resolution));
    
    for (int r = 1; r <= num_range_steps; ++r) {
        double current_range = r * resolution;
        double max_radius = current_range * std::tan(half_beam_width);
        
        int steps_at_range = std::max(1, static_cast<int>(std::ceil(max_radius / resolution)));
        
        for (int x = -steps_at_range; x <= steps_at_range; ++x) {
            for (int y = -steps_at_range; y <= steps_at_range; ++y) {
                double offset_x = x * resolution;
                double offset_y = y * resolution;
                double offset_radius = std::sqrt(offset_x * offset_x + offset_y * offset_y);
                
                if (offset_radius <= max_radius) {
                    Eigen::Vector3d voxel_pos = origin + current_range * forward + 
                                              offset_x * right + offset_y * up;
                    voxels.push_back(voxel_pos);
                }
            }
        }
    }
    
    return voxels;
}

double RayCasting::normalize_angle(double angle) const
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

Eigen::Vector3d RayCasting::lerp(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double t) const
{
    return a + t * (b - a);
}

std::tuple<double, double, double> RayCasting::cartesian_to_spherical(const Eigen::Vector3d& point) const
{
    double range = point.norm();
    double bearing = std::atan2(point.y(), point.x());
    double elevation = std::atan2(point.z(), std::sqrt(point.x() * point.x() + point.y() * point.y()));
    
    return {range, bearing, elevation};
}

}  // namespace sonar_3d_reconstruction