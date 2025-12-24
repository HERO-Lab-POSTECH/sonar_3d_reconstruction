/**
 * @file iwlo_updater.h
 * @brief Intensity-Weighted Log-Odds (IWLO) probability update algorithm
 *
 * This class provides stateless static methods for IWLO algorithm,
 * eliminating code duplication between ProbabilityUpdater and Tile classes.
 */

#ifndef SONAR_3D_RECONSTRUCTION__IWLO_UPDATER_H_
#define SONAR_3D_RECONSTRUCTION__IWLO_UPDATER_H_

#include <cmath>
#include <algorithm>

namespace sonar_3d_reconstruction
{

// Forward declaration - IWLOParams is defined in tile.h
struct IWLOParams;

/**
 * @class IWLOUpdater
 * @brief Stateless utility class for IWLO probability calculations
 *
 * Provides core algorithms for:
 * - Intensity to weight conversion (sigmoid transformation)
 * - Learning rate decay calculation
 * - Log-odds to probability conversion
 * - Delta log-odds computation for voxel updates
 */
class IWLOUpdater
{
public:
    // =========================================================================
    // Core Conversion Functions
    // =========================================================================

    /**
     * @brief Convert log-odds to probability
     * @param log_odds Log-odds value
     * @return Probability in range [0, 1]
     */
    static double log_odds_to_probability(double log_odds);

    /**
     * @brief Convert probability to log-odds
     * @param probability Probability in range (0, 1)
     * @return Log-odds value
     */
    static double probability_to_log_odds(double probability);

    // =========================================================================
    // IWLO Algorithm Functions (with explicit parameters)
    // =========================================================================

    /**
     * @brief Convert intensity to weight using sigmoid transformation
     *
     * Maps intensity values to weights in [0, 1] using:
     * 1. Threshold filtering: intensity <= threshold -> 0
     * 2. Normalization: [threshold, max] -> [0, 1]
     * 3. Sigmoid: centered at 0.5, controlled by sharpness
     *
     * @param intensity Raw intensity value
     * @param intensity_threshold Minimum intensity threshold
     * @param intensity_max Maximum intensity value
     * @param sharpness Sigmoid curve sharpness (higher = steeper)
     * @return Weight in range [0, 1]
     */
    static double intensity_to_weight(double intensity,
                                      double intensity_threshold,
                                      double intensity_max,
                                      double sharpness);

    /**
     * @brief Compute learning rate with decay
     *
     * Learning rate decreases with observation count:
     * alpha = max(min_alpha, 1 / (1 + decay_rate * count))
     *
     * @param observation_count Number of observations for this voxel
     * @param decay_rate Rate of learning rate decay
     * @param min_alpha Minimum learning rate floor
     * @return Learning rate in range [min_alpha, 1]
     */
    static double compute_alpha(int observation_count,
                                double decay_rate,
                                double min_alpha);

    // =========================================================================
    // IWLO Algorithm Functions (with IWLOParams struct)
    // =========================================================================

    /**
     * @brief Convert intensity to weight using IWLOParams
     * @param intensity Raw intensity value
     * @param params IWLO parameters struct
     * @return Weight in range [0, 1]
     */
    static double intensity_to_weight(double intensity, const IWLOParams& params);

    /**
     * @brief Compute learning rate using IWLOParams
     * @param observation_count Number of observations
     * @param params IWLO parameters struct
     * @return Learning rate
     */
    static double compute_alpha(int observation_count, const IWLOParams& params);

    /**
     * @brief Compute adaptive scaling factor
     *
     * Reduces update magnitude for high-confidence voxels:
     * - If |prob - 0.5| > adaptive_threshold: apply max_ratio scaling
     * - Otherwise: full update
     *
     * @param current_probability Current voxel probability
     * @param params IWLO parameters struct
     * @return Scaling factor in range [adaptive_max_ratio, 1]
     */
    static double compute_adaptive_scale(double current_probability,
                                         const IWLOParams& params);

    /**
     * @brief Compute delta log-odds for a voxel update
     *
     * Full IWLO update formula:
     * delta_L = alpha * scale * w * L_occ + alpha * scale * (1 - w) * L_free
     *
     * @param intensity Raw intensity value
     * @param current_log_odds Current log-odds of the voxel
     * @param observation_count Number of previous observations
     * @param params IWLO parameters struct
     * @return Delta log-odds to add to current value
     */
    static double compute_delta_log_odds(double intensity,
                                         double current_log_odds,
                                         int observation_count,
                                         const IWLOParams& params);

    /**
     * @brief Clamp log-odds to valid range
     * @param log_odds Log-odds value to clamp
     * @param L_min Minimum log-odds
     * @param L_max Maximum log-odds
     * @return Clamped log-odds
     */
    static double clamp_log_odds(double log_odds, double L_min, double L_max);
};

}  // namespace sonar_3d_reconstruction

#endif  // SONAR_3D_RECONSTRUCTION__IWLO_UPDATER_H_
