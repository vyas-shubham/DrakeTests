# Utilities for the simple pendulum demonstration for high-school children

import math
import numpy as np
from pydrake.examples.pendulum import PendulumPlant, PendulumState
from pydrake.all import DirectCollocation, PiecewisePolynomial, Solve


def testPrint(textToPrint):
    '''
    test print function...
    '''
    print(textToPrint)


def runPendulumSimulation(mass, damping, max_torque):
    '''
    run pendulum simulation function
    '''
    plant = PendulumPlant()
    context = plant.CreateDefaultContext()
    params = plant.get_mutable_parameters(context)
    print(params)
    params[0] = 0.57288
    params[1] = mass
    params[2] = damping
    print(plant.get_mutable_parameters(context))

    N = 50
    max_dt = 0.5
    max_tf = N * max_dt
    dircol = DirectCollocation(plant,
                               context,
                               num_time_samples=N,
                               minimum_timestep=0.05,
                               maximum_timestep=max_tf)

    dircol.AddEqualTimeIntervalsConstraints()

    torque_limit = max_torque  # N*m.
    print(torque_limit)
    u = dircol.input()
    dircol.AddConstraintToAllKnotPoints(-torque_limit <= u[0])
    dircol.AddConstraintToAllKnotPoints(u[0] <= torque_limit)

    initial_state = PendulumState()
    initial_state.set_theta(0.0)
    initial_state.set_thetadot(0.0)
    dircol.AddBoundingBoxConstraint(initial_state.get_value(),
                                    initial_state.get_value(),
                                    dircol.initial_state())

    final_state = PendulumState()
    final_state.set_theta(math.pi)
    final_state.set_thetadot(0.0)
    dircol.AddBoundingBoxConstraint(final_state.get_value(),
                                    final_state.get_value(), dircol.final_state())

    R = 1  # Cost on input "effort".
    dircol.AddRunningCost(R * u[0]**2)

    Q = 1
    dircol.AddRunningCost(Q * dircol.time()**2)

    initial_x_trajectory = PiecewisePolynomial.FirstOrderHold(
        [0., 4.], [initial_state.get_value(),
                   final_state.get_value()])
    dircol.SetInitialTrajectory(PiecewisePolynomial(), initial_x_trajectory)

    result = Solve(dircol)

    if result.is_success():
        x_trajectory = dircol.ReconstructStateTrajectory(result)
        u_trajectory = dircol.ReconstructInputTrajectory(result)
        return result, x_trajectory, u_trajectory
    else:
        return result, None, None


def extractTrajectory(x_trajectory, u_trajectory):
    '''
    '''
    # Extract Time
    control_freq = 600  # In Hz
    n_points = int(x_trajectory.end_time() / (1 / control_freq))
    time_traj = np.linspace(x_trajectory.start_time(), x_trajectory.end_time(), n_points)
    time_traj = time_traj.reshape(n_points, 1).T
    print("Time Array Shape: {}".format(time_traj.shape))

    # Extract State
    theta_theta_dot = np.hstack([
        x_trajectory.value(t) for t in np.linspace(x_trajectory.start_time(),
                                                   x_trajectory.end_time(), n_points)
    ])
    print("Theta&ThetaDot Array Shape: {}".format(theta_theta_dot.shape))
    theta = theta_theta_dot[0, :].reshape(n_points, 1).T
    print("Theta Array Shape: {}".format(theta.shape))
    theta_dot = theta_theta_dot[1, :].reshape(n_points, 1).T

    # Extract Control Inputs
    torque_traj = np.hstack([
        u_trajectory.value(t) for t in np.linspace(x_trajectory.start_time(),
                                                   x_trajectory.end_time(), n_points)
    ])
    print("Control Torque Array Shape: {}".format(torque_traj.shape))

    return time_traj, theta, theta_dot
