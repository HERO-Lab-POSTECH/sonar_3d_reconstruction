/**
 * @file mapper_backend.h
 * @brief Abstract interface for 3D mapper backends
 *
 * Phase 3 refactoring: Defines common interface for In-Memory (RAM)
 * and Out-of-Core (Disk) mapper implementations.
 */

#ifndef SONAR_3D_RECONSTRUCTION__MAPPER_BACKEND_H_
#define SONAR_3D_RECONSTRUCTION__MAPPER_BACKEND_H_

#include "octree_mapper.h"  // For MemoryStats
#include <Eigen/Dense>
#include <vector>
#include <string>

namespace sonar_3d_reconstruction
{

/**
 * @class IMapperBackend
 * @brief Abstract interface for 3D occupancy mapping backends
 *
 * Defines the common API shared between:
 * - ProbabilityUpdater (In-Memory/RAM backend)
 * - OutofcoreTileMapper (Out-of-Core/Disk backend)
 *
 * All backends must implement IWLO (Intensity-Weighted Log-Odds) updates.
 */
class IMapperBackend
{
public:
    virtual ~IMapperBackend() = default;

    // =========================================================================
    // Core Update API (Required)
    // =========================================================================

    /**
     * @brief Batch update using IWLO algorithm
     * @param points Nx3 matrix of point coordinates
     * @param intensities N-vector of intensity values
     * @param is_occupied N-vector of boolean flags
     */
    virtual void batch_update_iwlo(
        const Eigen::MatrixXd& points,
        const Eigen::VectorXd& intensities,
        const std::vector<bool>& is_occupied) = 0;

    // =========================================================================
    // Parameter Configuration API (Required)
    // =========================================================================

    /**
     * @brief Set IWLO parameters
     * @param sharpness Sigmoid steepness for intensity-to-weight
     * @param decay_rate Learning rate decay rate
     * @param min_alpha Minimum learning rate
     * @param L_min Saturation lower bound
     * @param L_max Saturation upper bound
     */
    virtual void set_iwlo_params(double sharpness, double decay_rate, double min_alpha,
                                  double L_min, double L_max) = 0;

    /**
     * @brief Set log-odds parameters
     * @param log_odds_occupied Log-odds increment for occupied voxels
     * @param log_odds_free Log-odds decrement for free voxels
     */
    virtual void set_log_odds_params(double log_odds_occupied, double log_odds_free) = 0;

    /**
     * @brief Set intensity parameters
     * @param intensity_threshold Minimum intensity to consider
     * @param intensity_max Maximum intensity value
     */
    virtual void set_intensity_params(double intensity_threshold, double intensity_max) = 0;

    /**
     * @brief Set adaptive update parameters
     * @param enabled Enable adaptive updating
     * @param threshold Threshold for adaptive behavior
     * @param max_ratio Maximum update ratio
     */
    virtual void set_adaptive_params(bool enabled, double threshold, double max_ratio) = 0;

    /**
     * @brief Set occupied threshold
     * @param threshold Probability threshold for occupied classification
     */
    virtual void set_occupied_threshold(double threshold) = 0;

    // =========================================================================
    // Query API (Required)
    // =========================================================================

    /**
     * @brief Get all occupied voxels above probability threshold
     * @param min_probability Minimum probability threshold
     * @return Nx4 matrix [x, y, z, probability]
     */
    virtual Eigen::MatrixXd get_occupied_voxels(double min_probability = 0.5) = 0;

    /**
     * @brief Get memory usage statistics
     */
    virtual MemoryStats get_memory_usage() const = 0;

    /**
     * @brief Get resolution
     */
    virtual double get_resolution() const = 0;

    /**
     * @brief Get total number of nodes
     */
    virtual size_t get_num_nodes() const = 0;

    /**
     * @brief Clear all data
     */
    virtual void clear() = 0;

    // =========================================================================
    // Optional API (Default implementations provided)
    // =========================================================================

    /**
     * @brief Flush data to persistent storage (Disk backend only)
     * @note RAM backend: no-op
     */
    virtual void flush() {}

    /**
     * @brief Preload region into cache (Disk backend only)
     * @param min_bound Minimum corner
     * @param max_bound Maximum corner
     * @note RAM backend: no-op
     */
    virtual void preload_region(const Eigen::Vector3d& /* min_bound */,
                                 const Eigen::Vector3d& /* max_bound */) {}

    /**
     * @brief Get disk usage in bytes (Disk backend only)
     * @return 0 for RAM backend
     */
    virtual size_t get_disk_usage() const { return 0; }

    /**
     * @brief Prune tree to reduce memory
     * @return Number of nodes removed
     */
    virtual size_t prune() { return 0; }

    /**
     * @brief Check if backend supports persistent storage
     * @return true for Disk backend, false for RAM backend
     */
    virtual bool supports_persistence() const { return false; }

    /**
     * @brief Get backend type name
     * @return "RAM" or "Disk"
     */
    virtual std::string get_backend_type() const = 0;
};

}  // namespace sonar_3d_reconstruction

#endif  // SONAR_3D_RECONSTRUCTION__MAPPER_BACKEND_H_
