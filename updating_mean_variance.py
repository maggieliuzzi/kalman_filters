'''
Measurement update step of Kalman Filter

Note that the resulting variance will always be smaller than both original and measurement ones
'''


def update_mean_var(mean1, var1, mean2, var2):
    new_mean = (var2 * mean1 + var1 * mean2) / (var1 + var2)
    new_var = 1 / (1 / var1 + 1 / var2)
    return [new_mean, new_var]


print(update_mean_var(10., 8., 13., 2.))
print(update_mean_var(10., 4., 12., 4.))