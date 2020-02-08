'''
Iteratively update and predict based on location measurements and inferred motions
'''

# Measurement update step of Kalman Filter
def update(mean1, var1, mean2, var2):
    # The resulting variance will always be smaller than both original and measurement ones because more information has been received
    new_mean = float(var2 * mean1 + var1 * mean2) / (var1 + var2)
    new_var = 1. / (1. / var1 + 1. / var2)
    return [new_mean, new_var]


# Prediction step of Kalman Filter
def predict(mean1, var1, mean2, var2):
    """
    The first set refer to our current estimate and its variance
    The second set refer to the move/motion and its residual uncertainty
    """
    new_mean = mean1 + mean2
    new_var = var1 + var2
    return [new_mean, new_var]


# print(update(10., 8., 13., 2.))
# print(update(10., 4., 12., 4.))
# print(predict(10., 4., 12., 4.))

measurements = [5., 6., 7., 9., 10.]
motion = [1., 1., 2., 1., 1.]
measurement_sig = 4.
motion_sig = 2.
mu = 0.
sig = 0.0000000001  # Very strong initial belief  # 10000. high uncertainty related to initial belief

for n in range(len(measurements)):
    [mu, sig] = update(mu, sig, measurements[n], measurement_sig)
    print("Update: {}".format([mu, sig]))
    [mu, sig] = predict(mu, sig, motion[n], motion_sig)
    print("Predict: {}".format([mu, sig]))

print([mu, sig])  # mean and variance