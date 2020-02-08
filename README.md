# Kalman Filters

Small functions for implementing Kalman Filters in Python


# Functions

## Gaussian Function Definition

    def gaussian_fn(mu, sigma_sq, x)

## Measurement Update Step
Bayes rule, multiplication

    def update_mean_var(mean1, var1, mean2, var2)

## Prediction/ Motion Update Step
Total probability, addition


# Theoretical Equations

Kalman Filter states (variables) can be divided in observable and hidden. Multiple instances of an observable variable allows us to make inferences about a hidden variable, which cannot be observed directly.

    x' = x + Δtẋ'
