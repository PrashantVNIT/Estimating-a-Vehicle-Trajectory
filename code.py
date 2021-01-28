import pickle
import numpy as np
import matplotlib.pyplot as plt

# Loading all the data from "data.pickle" file to data
file = "data.pickle"
fileoj = open(file, 'rb')
data = pickle.load(fileoj)

# declaring and initializing all the variables
t = data['t']  # timestamp in sec.[s]
x_init = data['x_init']  # initial x position/coordinate in meter[m]
y_init = data['y_init']  # initial y position/coordinate in  meter[m]
th_init = data['th_init']  # initial theta angle in radians[rad]

# Control commands(inputs)
v = data['v']   # translational velocity input in [m/s]
om = data['om']  # rotational velocity input in [rad/s]

# bearing and range measurements, LIDAR constants
b = data['b']  # bearing measurement of LiDAR (basically angle measurements) in [rad]
r = data['r']  # range measurements of LiDAR in  [m]
l = data['l']  # x,y positions of landmarks in [m]
d = data['d']  # distance between Centre of mass of the robot and Centre of mass of LiDAR in [m]

# Variances in control commands and LiDAR measurements
v_var = 0.004  # translation velocity variance
om_var = 0.008  # rotational velocity variance
r_var = 0.001   # range measurements variance
b_var = 0.0005  # bearing measurement variance

Q = np.diag([v_var, om_var])    # input noise covariance
R = np.diag([r_var,b_var])  # measurement noise covariance

x_est = np.zeros([len(v), 3])   # estimated states: x, y, and theta
P_est = np.zeros([len(v), 3, 3])    # state covariance matrices

x_est[0] = np.array([x_init, y_init, th_init])  # initial estimated states x,y and theta
P_est[0] = np.diag([1, 1, 0.1])  # initial state covariance matrix

# Setting values of the updated/corrected pose and covariance matrix
P_check = P_est[0]
x_check = x_est[0].reshape(3,1)

# Limiting theta from (-pi,+pi]
def theta_limit(th):
    th %= (2*np.pi)
    if th > np.pi:
        th -= 2 * np.pi
    elif th < -np.pi:
      th += 2 * np.pi
    return th

# Measurement update function to update the estimated pose and covariance matrix
def measurement_update(lk, rk, bk, P_check, x_check):
    x_k = x_check[0]
    y_k = x_check[1]
    theta_k = theta_limit(x_check[2])
    x_l = lk[0]
    y_l = lk[1]

    d_x = x_l - x_k - d * np.cos(theta_k)
    d_y = y_l - y_k - d * np.sin(theta_k)

    r = np.sqrt(d_x ** 2 + d_y ** 2)
    phi = np.arctan2(d_y, d_x) - theta_k

    #  measurement Jacobian
    H_k = np.zeros((2, 3))
    H_k[0, 0] = -d_x / r
    H_k[0, 1] = -d_y / r
    H_k[0, 2] = d * (d_x * np.sin(theta_k) - d_y * np.cos(theta_k)) / r
    H_k[1, 0] = d_y / r ** 2
    H_k[1, 1] = -d_x / r ** 2
    H_k[1, 2] = -1 - d * (d_y * np.sin(theta_k) + d_x * np.cos(theta_k)) / r ** 2

    M_k = np.identity(2)
    y_out = np.vstack([r, theta_limit(phi)])
    y_mes = np.vstack([rk, theta_limit(bk)])
    #  Kalman Gain
    K_k = P_check.dot(H_k.T).dot(np.linalg.inv(H_k.dot(P_check).dot(H_k.T) + M_k.dot(R).dot(M_k.T)))

    # Corrected predicted state
    x_check = x_check + K_k.dot(y_mes - y_out)
    x_check[2] = theta_limit(x_check[2])

    #  Corrected covariance
    P_check = (np.identity(3) - K_k.dot(H_k)).dot(P_check)

    return x_check, P_check


# Main Filter loop code
for k in range(1, len(t)):    # start at 1 because we've set the initial prediction

    delta_t = t[k] - t[k - 1]  # time step (difference between timestamps)
    theta = theta_limit(x_check[2])

    # 1. Update state with odometry readings
    F = np.array([[np.cos(theta), 0],
                  [np.sin(theta), 0],
                  [0, 1]], dtype='float')
    inp = np.array([[v[k-1]], [om[k-1]]])

    x_check = x_check + F.dot(inp).dot(delta_t)
    x_check[2] = theta_limit(x_check[2])

    # 2. Motion model jacobian with respect to last state
    F_km = np.zeros([3, 3])    # F_km is F at k-1
    F_km = np.array([[1, 0, -np.sin(theta)*delta_t*v[k-1]],
                     [0, 1, np.cos(theta)*delta_t*v[k-1]],
                     [0, 0, 1]], dtype='float')

    # 3. Motion model jacobian with respect to noise
    L_km = np.zeros([3, 2]) # L_km is L at k-1s
    L_km = np.array([[np.cos(theta)*delta_t, 0],
                    [np.sin(theta)*delta_t, 0],
                    [0,1]], dtype='float')

    # 4. Predicted Covariance
    P_check = F_km.dot(P_check.dot(F_km.T)) + L_km.dot(Q.dot(L_km.T))

    # 5. Update state estimate using available landmark measurements
    for i in range(len(r[k])):
        x_check, P_check = measurement_update(l[i], r[k, i], b[k, i], P_check, x_check)

    # Set final state predictions for timestamp
    x_est[k, 0] = x_check[0]
    x_est[k, 1] = x_check[1]
    x_est[k, 2] = x_check[2]
    P_est[k, :, :] = P_check

# Plotting the resulting state estimates:
e_fig = plt.figure()
ax = e_fig.add_subplot(111)
ax.plot(x_est[:, 0], x_est[:, 1])
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_title('Ground Truth trajectory')
plt.show()

e_fig = plt.figure()
ax = e_fig.add_subplot(111)
ax.plot(t[:], x_est[:, 2])
ax.set_xlabel('Time [s]')
ax.set_ylabel('theta [rad]')
ax.set_title('Ground Truth trajectory')
plt.show()
