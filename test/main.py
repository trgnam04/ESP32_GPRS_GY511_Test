import numpy as np
from typing import Tuple, List, Dict

def f(state: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
    """
    Process model: predict the next state given the current state and control input.
    
    Parameters:
        state: np.ndarray - current state [x, y, v, theta]
        u: np.ndarray - control input [a, omega]
        dt: float - time step

    Returns:
        np.ndarray - predicted state [x_new, y_new, v_new, theta_new]
    """
    x, y, v, theta = state
    a, omega = u
    x_new = x + v * np.cos(theta) * dt
    y_new = y + v * np.sin(theta) * dt
    v_new = v + a * dt
    theta_new = theta + omega * dt
    return np.array([x_new, y_new, v_new, theta_new])


def F_jacobian(state: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
    """
    Compute the Jacobian of the process model with respect to the state.
    
    Parameters:
        state: np.ndarray - current state [x, y, v, theta]
        u: np.ndarray - control input [a, omega] (not directly used here)
        dt: float - time step

    Returns:
        np.ndarray - Jacobian matrix (4x4) of the process model.
    """
    _, _, v, theta = state
    F = np.eye(4)
    F[0, 2] = np.cos(theta) * dt
    F[0, 3] = -v * np.sin(theta) * dt
    F[1, 2] = np.sin(theta) * dt
    F[1, 3] = v * np.cos(theta) * dt
    return F


def h(state: np.ndarray) -> np.ndarray:
    """
    Measurement model: maps the state to the measurement space [v, theta].
    
    Parameters:
        state: np.ndarray - state vector [x, y, v, theta]

    Returns:
        np.ndarray - measurement vector [v, theta]
    """
    return np.array([state[2], state[3]])


def H_jacobian(state: np.ndarray) -> np.ndarray:
    """
    Compute the Jacobian of the measurement function with respect to the state.
    
    Parameters:
        state: np.ndarray - state vector [x, y, v, theta]

    Returns:
        np.ndarray - Jacobian matrix (2x4) of the measurement model.
    """
    H = np.zeros((2, 4))
    H[0, 2] = 1.0  # derivative of v with respect to v
    H[1, 3] = 1.0  # derivative of theta with respect to theta
    return H


def ekf_predict(state: np.ndarray, P: np.ndarray, u: np.ndarray,
                dt: float, Q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    EKF prediction step.
    
    Parameters:
        state: np.ndarray - current state estimate
        P: np.ndarray - current covariance matrix
        u: np.ndarray - control input [a, omega]
        dt: float - time step
        Q: np.ndarray - process noise covariance

    Returns:
        Tuple[np.ndarray, np.ndarray] - predicted state and covariance matrix.
    """
    state_pred = f(state, u, dt)
    F = F_jacobian(state, u, dt)
    P_pred = F @ P @ F.T + Q
    return state_pred, P_pred


def ekf_update(state_pred: np.ndarray, P_pred: np.ndarray,
               z: np.ndarray, R: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    EKF update step.
    
    Parameters:
        state_pred: np.ndarray - predicted state from the prediction step
        P_pred: np.ndarray - predicted covariance matrix
        z: np.ndarray - measurement vector [v_meas, theta_meas]
        R: np.ndarray - measurement noise covariance matrix

    Returns:
        Tuple[np.ndarray, np.ndarray] - updated state and covariance matrix.
    """
    z_pred = h(state_pred)
    y_res = z - z_pred
    H = H_jacobian(state_pred)
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)
    state_upd = state_pred + K @ y_res
    I = np.eye(len(state_pred))
    P_upd = (I - K @ H) @ P_pred
    return state_upd, P_upd


def run_ekf(sensor_data: List[Dict[str, float]], dt: float,
            Q: np.ndarray, R: np.ndarray) -> List[np.ndarray]:
    """
    Run the EKF over a series of sensor measurements.
    
    Parameters:
        sensor_data: List[Dict[str, float]] - list of sensor data dictionaries. Each dict should have
                     keys: 'a', 'omega', 'v_meas', 'theta_meas'
        dt: float - time step
        Q: np.ndarray - process noise covariance matrix
        R: np.ndarray - measurement noise covariance matrix

    Returns:
        List[np.ndarray] - list of state estimates at each time step.
    """
    # Initialize state at the origin with zero velocity and heading.
    state = np.array([0.0, 0.0, 0.0, 0.0])
    P = np.eye(4)
    estimates = [state.copy()]

    for data in sensor_data:
        # Safely extract data with defaults.
        a = data.get('a', 0.0)
        omega = data.get('omega', 0.0)
        v_meas = data.get('v_meas', 0.0)
        theta_meas = data.get('theta_meas', 0.0)

        u = np.array([a, omega])
        z = np.array([v_meas, theta_meas])

        state_pred, P_pred = ekf_predict(state, P, u, dt, Q)
        state, P = ekf_update(state_pred, P_pred, z, R)
        estimates.append(state.copy())

    return estimates


if __name__ == "__main__":
    # Example sensor data.
    sensor_data = [
        {'a': 1.0, 'omega': 0.1, 'v_meas': 0.5, 'theta_meas': 0.05},
        {'a': 0.8, 'omega': 0.05, 'v_meas': 0.7, 'theta_meas': 0.07},
        # Add more sensor measurements as needed.
    ]
    dt = 0.1  # time step in seconds
    Q = 0.01 * np.eye(4)  # process noise covariance matrix
    R = 0.05 * np.eye(2)  # measurement noise covariance matrix

    estimates = run_ekf(sensor_data, dt, Q, R)
    for idx, est in enumerate(estimates):
        print(f"Step {idx}: State = {est}")