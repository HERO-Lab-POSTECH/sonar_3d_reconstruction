/**
 * @file iwlo_updater.cpp
 * @brief Implementation of IWLO probability update algorithm
 */

#include "iwlo_updater.h"
#include "tile.h"  // For IWLOParams definition

namespace sonar_3d_reconstruction
{

// =============================================================================
// Core Conversion Functions
// =============================================================================

double IWLOUpdater::log_odds_to_probability(double log_odds)
{
    return 1.0 / (1.0 + std::exp(-log_odds));
}

double IWLOUpdater::probability_to_log_odds(double probability)
{
    // Clamp to avoid log(0) or log(inf)
    probability = std::max(1e-6, std::min(1.0 - 1e-6, probability));
    return std::log(probability / (1.0 - probability));
}

// =============================================================================
// IWLO Algorithm Functions (explicit parameters)
// =============================================================================

double IWLOUpdater::intensity_to_weight(double intensity,
                                        double intensity_threshold,
                                        double intensity_max,
                                        double sharpness)
{
    if (intensity <= intensity_threshold) {
        return 0.0;
    }

    // Normalize: [threshold, max] -> [0, 1]
    double normalized = (intensity - intensity_threshold) /
                        (intensity_max - intensity_threshold);
    normalized = std::max(0.0, std::min(1.0, normalized));

    // Sigmoid transformation centered at 0.5
    double x = sharpness * (normalized - 0.5);
    return 1.0 / (1.0 + std::exp(-x));
}

double IWLOUpdater::compute_alpha(int observation_count,
                                  double decay_rate,
                                  double min_alpha)
{
    return std::max(min_alpha, 1.0 / (1.0 + decay_rate * observation_count));
}

// =============================================================================
// IWLO Algorithm Functions (with IWLOParams)
// =============================================================================

double IWLOUpdater::intensity_to_weight(double intensity, const IWLOParams& params)
{
    return intensity_to_weight(intensity,
                               params.intensity_threshold,
                               params.intensity_max,
                               params.sharpness);
}

double IWLOUpdater::compute_alpha(int observation_count, const IWLOParams& params)
{
    return compute_alpha(observation_count, params.decay_rate, params.min_alpha);
}

double IWLOUpdater::compute_adaptive_scale(double current_probability,
                                           const IWLOParams& params)
{
    if (!params.adaptive_enabled) {
        return 1.0;
    }

    double distance_from_uncertain = std::abs(current_probability - 0.5);
    if (distance_from_uncertain > params.adaptive_threshold) {
        return params.adaptive_max_ratio;
    }
    return 1.0;
}

double IWLOUpdater::compute_delta_log_odds(double intensity,
                                           double current_log_odds,
                                           int observation_count,
                                           const IWLOParams& params)
{
    // 1. Compute intensity-based weight
    double w = intensity_to_weight(intensity, params);

    // 2. Compute learning rate based on observation count
    double alpha = compute_alpha(observation_count, params);

    // 3. Compute adaptive scaling
    double current_prob = log_odds_to_probability(current_log_odds);
    double scale = compute_adaptive_scale(current_prob, params);

    // 4. Compute delta log-odds
    // delta_L = alpha * scale * [w * L_occ + (1-w) * L_free]
    double delta = alpha * scale * (
        w * params.log_odds_occupied +
        (1.0 - w) * params.log_odds_free
    );

    return delta;
}

double IWLOUpdater::clamp_log_odds(double log_odds, double L_min, double L_max)
{
    return std::max(L_min, std::min(L_max, log_odds));
}

}  // namespace sonar_3d_reconstruction
