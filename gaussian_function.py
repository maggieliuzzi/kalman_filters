from math import *


def gaussian_fn(mu, sigma_sq, x):
    """
    mu, sigma square, x
    """
    return 1 / sqrt(2. * pi * sigma_sq) * exp(-.5 * (x - mu)**2 / sigma_sq)


print gaussian_fn(10., 4., 8.)  
print gaussian_fn(10., 4., 10.)  # 10., 4., 10. maximises gaussian_fn, since the exp gets cancelled out
print gaussian_fn(8., 4., 8.)
print gaussian_fn(10., 4., 6.) 
print gaussian_fn(6., 4., 10.) 
