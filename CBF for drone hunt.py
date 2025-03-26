import numpy as np
import sympy as sym
from scipy.optimize import minimize, LinearConstraint, NonlinearConstraint

def barrier_func(drone_state, object_state, x_min, x_max, drone_radius, object_radius, delta_T):
    # Define drone and object position and velocity
    x_d = drone_state[0:2]
    x_d = x_d.reshape(3,1)
    x_o = object_state[0:2]
    x_o = x_o.reshape(3,1)
    v_d = drone_state[3:5]
    v_d = v_d.reshape(3,1)
    v_o = object_state[3:5]
    v_o = v_o.reshape(3,1)
    h = np.zeros(7,1)
    # compute the three barriers
    h[0:2] = x_d - x_min
    h[3:5] = x_max - x_d
    h[6] = np.linalg.norm(x_d - x_o) - delta_T * (v_o - v_d).T @ (x_d - x_o) * (np.linalg.norm(x_d - x_o))**(-1) - object_radius - drone_radius
    return h

def minimized_func(u, u_nom):
    return np.linalg.norm(u - u_nom)**2

def solve_CBF(barrier_func, u0, u_nom, f, g, drone_state, object_state, x_min, x_max, drone_radius, object_radius, delta_T, alpha):
    #Compute the gradients of the three barriers
    grad_h_1 = np.zeros(12, 1)
    grad_h_1[0] = 1
    grad_h_2 = np.zeros(12, 1)
    grad_h_2[0] = -1
    grad_h_3 = np.zeros(12,1)
    x_d = drone_state[0:2]
    x_d = x_d.reshape(3,1)
    x_o = object_state[0:2]
    x_o = x_o.reshape(3,1)
    v_d = drone_state[3:5]
    v_d = v_d.reshape(3,1)
    v_o = object_state[3:5]
    v_o = v_o.reshape(3,1)
    p = x_d - x_o
    v_rel = v_o - v_d
    grad_h_3[0:2] = p/np.linalg.norm(p) - delta_T * (v_rel/np.linalg.norm(p) - ((v_rel.T @ p) @ p)/(np.linalg.norm(p)**3))
    grad_h_3[3:5] = delta_T * p/np.linalg.norm(p)
    grad_h_3[6:8] = - p/np.linalg.norm(p) + delta_T * (v_rel/np.linalg.norm(p) - ((v_rel.T @ p) @ p)/(np.linalg.norm(p)**3))
    grad_h_3[9:11] = - delta_T * p/np.linalg.norm(p)
    h = barrier_func(drone_state, object_state, x_min, x_max, drone_radius, object_radius, delta_T)
    h_1 = np.linalg.norm(h[0:2])
    h_2 = np.linalg.norm(h[3:5])
    h_3 = h[6]
    # Determine which gradient to use
    if h_1 == np.min(h_1, h_2, h_3):
        grad_h = grad_h_1
    elif h_2 == np.min(h_1, h_2, h_3):
        grad_h = grad_h_2
    elif h_3 == np.min(h_1, h_2, h_3):
        grad_h = grad_h_3
    L_f = np.dot(grad_h, f(drone_state, object_state))
    L_g = np.dot(grad_h, g(drone_state, object_state))
    # Define function to minimize
    min_func = lambda u:minimized_func(u, u_nom)
    con = lambda u: L_f + L_g @ u + alpha(h)
    nlc = NonlinearConstraint(con, 0, np.inf)
    # Solve the minimization problem
    u_des = minimize(min_func, u0, constraints = nlc)
    return u_des

