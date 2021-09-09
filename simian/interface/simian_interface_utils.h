namespace simian_interface_utils {

// Convert UTM to lat lng
void convert_utm_latlng(int zone, bool north, double x, double y, double* lat, double* lng,
                        double* meridian_convergence);

// Convert lat lng to UTM
void convert_latlng_utm(double lat, double lng, int* zone, bool* north, double* x, double* y,
                        double* meridian_convergence);

}  // namespace simian_interface_utils
