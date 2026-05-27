#include <stdlib.h>
#include <math.h>

double generate_gaussian_noise(double sigma, double f_sampling) {
    double pi = 3.141592653589793;
    double sigma_discret = sigma * sqrt(f_sampling);

    double u1 = (double)rand() / RAND_MAX;
    double u2 = (double)rand() / RAND_MAX;
    
    if (u1 < 1e-10) u1 = 1e-10; 
    
    double z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * pi * u2);
    
    return z0 * sigma_discret;
}