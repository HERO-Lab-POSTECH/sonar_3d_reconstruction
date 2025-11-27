#ifndef SONAR_3D_RECONSTRUCTION__RAY_CASTING_H_
#define SONAR_3D_RECONSTRUCTION__RAY_CASTING_H_

#include <Eigen/Dense>
#include <vector>
#include <tuple>

namespace sonar_3d_reconstruction
{

/**
 * Ray casting utilities for sonar data processing
 * Provides efficient algorithms for sonar ray processing and voxel traversal
 */
class RayCasting
{
public:
    RayCasting();
    ~RayCasting();
    
    /**
     * Generate points along a ray from origin to endpoint
     * Uses DDA-like algorithm for efficient voxel traversal
     * 
     * @param origin Ray origin point
     * @param endpoint Ray endpoint
     * @param resolution Voxel resolution for sampling
     * @param max_range Maximum ray range
     * @return Vector of points along the ray
     */
    std::vector<Eigen::Vector3d> generate_ray_points(
        const Eigen::Vector3d& origin,
        const Eigen::Vector3d& endpoint,
        double resolution,
        double max_range = -1.0
    ) const;
    
    /**
     * Cast multiple rays in parallel for sonar beam processing
     * 
     * @param origin Common origin for all rays
     * @param endpoints Vector of ray endpoints
     * @param resolution Voxel resolution
     * @param max_range Maximum ray range
     * @return Vector of ray point vectors
     */
    std::vector<std::vector<Eigen::Vector3d>> cast_multiple_rays(
        const Eigen::Vector3d& origin,
        const std::vector<Eigen::Vector3d>& endpoints,
        double resolution,
        double max_range = -1.0
    ) const;
    
    /**
     * Generate sonar beam pattern for given parameters
     * Creates points for sonar beam with horizontal and vertical apertures
     * 
     * @param origin Sonar origin
     * @param range Distance to endpoint
     * @param bearing_angle Horizontal angle (radians)
     * @param vertical_aperture Total vertical aperture (radians)
     * @param num_vertical_samples Number of vertical samples
     * @return Vector of beam endpoint coordinates
     */
    std::vector<Eigen::Vector3d> generate_sonar_beam_pattern(
        const Eigen::Vector3d& origin,
        double range,
        double bearing_angle,
        double vertical_aperture,
        int num_vertical_samples = 5
    ) const;
    
    /**
     * Check ray-sphere intersection
     * Used for sonar range validation
     * 
     * @param ray_origin Ray origin
     * @param ray_direction Ray direction (normalized)
     * @param sphere_center Sphere center
     * @param sphere_radius Sphere radius
     * @return Intersection distance, negative if no intersection
     */
    double ray_sphere_intersection(
        const Eigen::Vector3d& ray_origin,
        const Eigen::Vector3d& ray_direction,
        const Eigen::Vector3d& sphere_center,
        double sphere_radius
    ) const;
    
    /**
     * Check if a point is within sonar field of view
     * 
     * @param sonar_origin Sonar position
     * @param sonar_direction Sonar forward direction
     * @param point Point to check
     * @param horizontal_fov Horizontal field of view (radians)
     * @param vertical_aperture Vertical aperture (radians)
     * @param max_range Maximum range
     * @return True if point is within FOV
     */
    bool is_point_in_sonar_fov(
        const Eigen::Vector3d& sonar_origin,
        const Eigen::Vector3d& sonar_direction,
        const Eigen::Vector3d& point,
        double horizontal_fov,
        double vertical_aperture,
        double max_range
    ) const;
    
    /**
     * Compute bearing and elevation angles for a point relative to sonar
     * 
     * @param sonar_origin Sonar position
     * @param sonar_direction Sonar forward direction  
     * @param point Target point
     * @return Tuple of (bearing, elevation, range) in radians and meters
     */
    std::tuple<double, double, double> compute_sonar_angles(
        const Eigen::Vector3d& sonar_origin,
        const Eigen::Vector3d& sonar_direction,
        const Eigen::Vector3d& point
    ) const;
    
    /**
     * Generate dense voxel grid for sonar beam volume
     * Creates all voxels within a sonar beam for occupancy updates
     * 
     * @param origin Beam origin
     * @param direction Beam direction (normalized)
     * @param range Beam range
     * @param beam_width Beam width (radians)
     * @param resolution Voxel resolution
     * @return Vector of voxel coordinates within beam
     */
    std::vector<Eigen::Vector3d> generate_beam_voxels(
        const Eigen::Vector3d& origin,
        const Eigen::Vector3d& direction,
        double range,
        double beam_width,
        double resolution
    ) const;

private:
    /**
     * Normalize angle to [-pi, pi] range
     */
    double normalize_angle(double angle) const;
    
    /**
     * Linear interpolation between two points
     */
    Eigen::Vector3d lerp(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double t) const;
    
    /**
     * Convert Cartesian coordinates to spherical (range, bearing, elevation)
     */
    std::tuple<double, double, double> cartesian_to_spherical(const Eigen::Vector3d& point) const;
};

}  // namespace sonar_3d_reconstruction

#endif  // SONAR_3D_RECONSTRUCTION__RAY_CASTING_H_