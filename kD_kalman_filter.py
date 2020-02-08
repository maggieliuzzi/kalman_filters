'''
Multi-dimensional Kalman filter for vehicle tracking

From measurement data/ a sequence of position observations, 
we make a position prediction for the next time step 
and infer the (non-observable) velocity, which cannot be observed directly
'''

from math import *


class Matrix:
    
    def __init__(self, value):
        self.value = value
        self.dimx = len(value)
        self.dimy = len(value[0])
        if value == [[]]:
            self.dimx = 0
    
    def zero(self, dimx, dimy):
        # check if valid dimensions
        if dimx < 1 or dimy < 1:
            raise(ValueError, "Invalid size of matrix")
        else:
            self.dimx = dimx
            self.dimy = dimy
            self.value = [[0 for row in range(dimy)] for col in range(dimx)]
    
    def identity(self, dim):
        # check if valid dimension
        if dim < 1:
            print("Invalid size of matrix")  # raise(ValueError, "Invalid size of matrix")  # TODO: fix calls to raise, TypeError: exceptions must derive from BaseException
        else:
            self.dimx = dim
            self.dimy = dim
            self.value = [[0 for row in range(dim)] for col in range(dim)]
            for i in range(dim):
                self.value[i][i] = 1
    
    def show(self):
        for i in range(self.dimx):
            print(self.value[i])
        print(' ')
    
    def __add__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            print("Matrices must be of equal dimensions to add")  # raise(ValueError, "Matrices must be of equal dimensions to add")
        else:
            # add if correct dimensions
            res = Matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] + other.value[i][j]
            return res
    
    def __sub__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            print("Matrices must be of equal dimensions to subtract")  # raise(ValueError, "Matrices must be of equal dimensions to subtract")
        else:
            # subtract if correct dimensions
            res = Matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] - other.value[i][j]
            return res
    
    def __mul__(self, other):
        # check if correct dimensions
        if self.dimy != other.dimx:
            print("Matrices must be m*n and n*p to multiply")  # raise(ValueError, "Matrices must be m*n and n*p to multiply")
        else:
            # multiply if correct dimensions
            res = Matrix([[]])
            res.zero(self.dimx, other.dimy)
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] += self.value[i][k] * other.value[k][j]
            return res
    
    def transpose(self):
        # compute transpose
        res = Matrix([[]])
        res.zero(self.dimy, self.dimx)
        for i in range(self.dimx):
            for j in range(self.dimy):
                res.value[j][i] = self.value[i][j]
        return res
    
    def Cholesky(self, ztol=1.0e-5):
        # Computes the upper triangular Cholesky factorization of a positive definite matrix.
        res = Matrix([[]])
        res.zero(self.dimx, self.dimx)
        
        for i in range(self.dimx):
            S = sum([(res.value[k][i])**2 for k in range(i)])
            d = self.value[i][i] - S
            if abs(d) < ztol:
                res.value[i][i] = 0.0
            else:
                if d < 0.0:
                    print("Matrix not positive-definite")  # raise(ValueError, "Matrix not positive-definite")
                res.value[i][i] = sqrt(d)
            for j in range(i+1, self.dimx):
                S = sum([res.value[k][i] * res.value[k][j] for k in range(self.dimx)])
                if abs(S) < ztol:
                    S = 0.0
                try:
                   res.value[i][j] = (self.value[i][j] - S)/res.value[i][i]
                except:
                   print("Zero diagonal")  # raise(ValueError, "Zero diagonal")
        return res
    
    def CholeskyInverse(self):
        # Computes inverse of matrix given its Cholesky upper Triangular decomposition of matrix
        res = Matrix([[]])
        res.zero(self.dimx, self.dimx)
        
        # Backward step for inverse
        for j in reversed(range(self.dimx)):
            tjj = self.value[j][j]
            S = sum([self.value[j][k]*res.value[j][k] for k in range(j+1, self.dimx)])
            res.value[j][j] = 1.0/tjj**2 - S/tjj
            for i in reversed(range(j)):
                res.value[j][i] = res.value[i][j] = -sum([self.value[i][k]*res.value[k][j] for k in range(i+1, self.dimx)])/self.value[i][i]
        return res
    
    def inverse(self):
        aux = self.Cholesky()
        res = aux.CholeskyInverse()
        return res
    
    def __repr__(self):
        return repr(self.value)


########################################

# Implement the filter function below

def kalman_filter(x, P):
    
    for n in range(len(measurements)):

        # measurement update
        Z = Matrix([[measurements[n]]])  # create measurement matrix of nth measurement
        y = Z - (H * x)  # calculate error
        S = H * P * H.transpose() + R  # matrix S with a transpose
        K = P * H.transpose() * S.inverse()  # Kalman gain K with the inverse
        x = x + (K * y)  # next prediction
        P = (I - (K * H)) * P  # measurement update

        # prediction
        x = (F * x) + u
        P = F * P * F.transpose()

    return x, P


measurements = [1, 2, 3]  # sequence of position estimates

x = Matrix([[0.], [0.]]) # initial state (location and velocity)
P = Matrix([[1000., 0.], [0., 1000.]]) # initial uncertainty
u = Matrix([[0.], [0.]]) # external motion
F = Matrix([[1., 1.], [0, 1.]]) # next state function
H = Matrix([[1., 0.]]) # measurement function
R = Matrix([[1.]]) # measurement uncertainty
I = Matrix([[1., 0.], [0., 1.]]) # identity matrix

print(kalman_filter(x, P))
# x: [[3.9996664447958645], [0.9999998335552873]]
# P: [[2.3318904241194827, 0.9991676099921091], [0.9991676099921067, 0.49950058263974184]]
