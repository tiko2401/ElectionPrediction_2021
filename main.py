
import numpy as np
from numpy.linalg import inv

class KalmanFilter():
    def __init__(self):
        self.state_matrix = None            #X
        self.state_covariance = None        #P
        self.process_noise = None           #Q
        self.transition_matrix = None       #A
        self.control_matrix = None          #U
        self.input_effect = None            #B
        self.transition_noise = None        #w

        self.kalman_gain = None             #K
        self.measurement_vector = None      #Y
        self.measurement_matrix= None       #H
        self.measurement_covariance = None  #R

        self.measurement_error = None       #Z
        self.measurement_transform_matrix = None #C

    def predict(self):
        X_predict = np.dot(self.state_matrix, self.transition_matrix) + np.dot(self.input_effect, self.control_matrix) + self.transition_noise
        P_predict = np.dot(self.transition_matrix, np.dot(self.state_covariance, self.transition_matrix.T)) + self.process_noise

        return X_predict, P_predict

    def update(self, P_predict, X_predict):
        self.kalman_gain = (np.dot(P_predict, self.measurement_matrix.T))/(np.dot(self.measurement_matrix, np.dot(P_predict, self.measurement_matrix.T)) + self.measurement_covariance )
        self.measurement_vector = np.dot(self.measurement_transform_matrix, self.measurement_vector) + self.measurement_error

        self.state_covariance = (np.identity(len(self.kalman_gain)) - self.kalman_gain.dot(self.measurement_matrix)).dot(P_predict)
        self.state_matrix = X_predict + self.kalman_gain.dot(self.measurement_vector-self.measurement_matrix.dot(X_Predict))

    def run(self, n_iterations):
        pass




x = 50
x_dot = 5

x_std = 0.5
x_dot_std = 0.2

x_var = x_std**2
x_dot_var = x_dot_std**2



x_observations = np.array([4000, 4260, 4550, 4860, 5110])
v_observations = np.array([280, 282, 285, 286, 290])

z = np.c_[x_observations, v_observations]

a = 2  # Acceleration
v = 280
t = 1  # Difference in time


error_est_x = 20
error_est_v = 5