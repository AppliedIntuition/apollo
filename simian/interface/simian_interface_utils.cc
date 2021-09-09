/**
 * @file simian_interface_utils.cc
 * @brief Simian Customer Interface Utilities
 * @version 0.1
 * @date 2019-10-02
 *
 */
#include "interface/simian_interface_utils.h"
#include <GeographicLib/UTMUPS.hpp>

namespace simian_interface_utils {
constexpr bool use_mgrs_limits = false;

/**
 * @brief Convert UTM point to WSG Lat Long
 *
 * @param zone
 * @param north
 * @param x
 * @param y
 * @param lat
 * @param lng
 * @param meridian_convergence
 */
void convert_utm_latlng(int zone, bool north, double x, double y, double* lat, double* lng,
                        double* meridian_convergence) {
  double unused_scale;
  GeographicLib::UTMUPS::Reverse(zone, north, x, y, *lat, *lng, *meridian_convergence, unused_scale,
                                 use_mgrs_limits);
}

/**
 * @brief Convert WGS Lat Long to UTM
 *
 * @param lat
 * @param lng
 * @param zone
 * @param north
 * @param x
 * @param y
 * @param meridian_convergence
 */
void convert_latlng_utm(double lat, double lng, int* zone, bool* north, double* x, double* y,
                        double* meridian_convergence) {
  double unused_scale;
  GeographicLib::UTMUPS::Forward(lat, lng, *zone, *north, *x, *y, *meridian_convergence,
                                 unused_scale);
}
}  // namespace simian_interface_utils
