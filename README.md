# Kalman Filters

Small functions for implementing Kalman Filters in Python.


# Functions

## Gaussian Function Definition

    def gaussian_fn(mu, sigma_sq, x)

## Measurement Update/ Correction Step
Bayes rule, multiplication. Updates the state estimate and its uncertainty

## Prediction/ State Transition/ Motion Update Step
Total probability, addition/ convolution

# Theoretical Equations

Kalman Filter states (variables) can be divided in observable and hidden. Multiple instances of an observable variable allows us to make inferences about a hidden variable, which cannot be observed directly.

New location is equivalent to old location plus velocity: 

    x' = x + Δt * ẋ'
