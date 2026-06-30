#!/usr/bin/env python3
"""
Nonlinear MPC controller for a differential-drive (unicycle) robot.
State:   x = [px, py, theta]
Control: u = [v, omega]   (this maps directly to geometry_msgs/TwistStamped)
"""

import numpy as np
import casadi as ca
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from tf_transformations import euler_from_quaternion


class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')

        # MPC parameters 
        self.N = 15          # prediction horizon (steps)
        self.dt = 0.1        # discretization step [s], matches control loop rate
        self.v_max, self.v_min = 0.22, -0.22
        self.omega_max, self.omega_min = 2.84, -2.84

        # Weights: tune these first (see Part 8)
        self.Q = np.diag([5.0, 5.0, 1.0])    # state tracking: x, y, theta
        self.R = np.diag([0.5, 0.1])         # control effort: v, omega
        self.Qf = np.diag([10.0, 10.0, 2.0]) # terminal weight

        self._build_solver()

        self.state = np.zeros(3)  # [x, y, theta], updated by odom callback
        self.u_prev = np.zeros((2, self.N))  # warm-start cache

        self.odom_sub = self.create_subscription(
            Odometry, '/robot_controller/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/robot_controller/cmd_vel', 10)

        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('MPC controller started.')

    def _build_solver(self):
        nx, nu, N = 3, 2, self.N

        # Symbolic dynamics: unicycle model, RK4 discretization
        x = ca.SX.sym('x', nx)
        u = ca.SX.sym('u', nu)

        def f(x, u):
            return ca.vertcat(
                u[0] * ca.cos(x[2]),
                u[0] * ca.sin(x[2]),
                u[1]
            )

        dt = self.dt
        k1 = f(x, u)
        k2 = f(x + dt / 2 * k1, u)
        k3 = f(x + dt / 2 * k2, u)
        k4 = f(x + dt * k3, u)
        x_next = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        self.F = ca.Function('F', [x, u], [x_next])  # discrete dynamics

        # Decision variables over the horizon
        X = ca.SX.sym('X', nx, N + 1)
        U = ca.SX.sym('U', nu, N)
        X_ref = ca.SX.sym('X_ref', nx, N + 1)  # reference, set fresh each solve
        X0 = ca.SX.sym('X0', nx)               # current measured state

        Q, R, Qf = ca.DM(self.Q), ca.DM(self.R), ca.DM(self.Qf)

        cost = 0
        g = [X[:, 0] - X0]  # initial condition constraint

        for k in range(N):
            e = X[:, k] - X_ref[:, k]
            cost += ca.mtimes([e.T, Q, e]) + ca.mtimes([U[:, k].T, R, U[:, k]])
            g.append(X[:, k + 1] - self.F(X[:, k], U[:, k]))  # dynamics constraint

        e_N = X[:, N] - X_ref[:, N]
        cost += ca.mtimes([e_N.T, Qf, e_N])

        g = ca.vertcat(*g)
        opt_vars = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
        params = ca.vertcat(X0, ca.reshape(X_ref, -1, 1))

        nlp = {'x': opt_vars, 'f': cost, 'g': g, 'p': params}
        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.max_iter': 100,
            'ipopt.tol': 1e-4,
        }
        self.solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

        # Bounds: equality on dynamics/init constraints, box bounds on U
        self.lbg = np.zeros(g.shape[0])
        self.ubg = np.zeros(g.shape[0])

        self.nx, self.nu = nx, nu
        n_X, n_U = nx * (N + 1), nu * N
        self.lbx = np.concatenate([
            -np.inf * np.ones(n_X),
            np.tile([self.v_min, self.omega_min], N)
        ])
        self.ubx = np.concatenate([
            np.inf * np.ones(n_X),
            np.tile([self.v_max, self.omega_max], N)
        ])
        self.n_X, self.n_U = n_X, n_U

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.state = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw
        ])

    def reference_trajectory(self, t0):
        """Circular reference trajectory, generated fresh each solve."""
        radius, speed = 0.5, 0.15
        omega_ref = speed / radius
        ts = t0 + np.arange(self.N + 1) * self.dt
        xs = radius * np.cos(omega_ref * ts) - radius
        ys = radius * np.sin(omega_ref * ts)
        thetas = omega_ref * ts + np.pi / 2
        return np.vstack([xs, ys, thetas])

    def control_loop(self):
        now_time = self.get_clock().now()
        t = (now_time.nanoseconds / 1e9) - self.t0
        x_ref = self.reference_trajectory(t)

        params = np.concatenate([self.state, x_ref.flatten(order='F')])

        # Warm start: shift previous solution, reuse as initial guess
        X_init = np.tile(self.state.reshape(-1, 1), (1, self.N + 1))
        U_init = self.u_prev
        x0_guess = np.concatenate([X_init.flatten(order='F'), U_init.flatten(order='F')])

        sol = self.solver(
            x0=x0_guess, lbx=self.lbx, ubx=self.ubx,
            lbg=self.lbg, ubg=self.ubg, p=params
        )

        if not self.solver.stats()['success']:
            self.get_logger().warn('MPC solve did not converge — holding last command.')
            return

        opt = sol['x'].full().flatten()
        U_opt = opt[self.n_X:].reshape(self.N, self.nu).T
        self.u_prev = U_opt  # cache for next warm start

        v_cmd, omega_cmd = U_opt[0, 0], U_opt[1, 0]

        cmd = TwistStamped()
        cmd.header.stamp = now_time.to_msg()
        cmd.header.frame_id = 'base_footprint'
        cmd.twist.linear.x = float(v_cmd)
        cmd.twist.angular.z = float(omega_cmd)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MPCController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()