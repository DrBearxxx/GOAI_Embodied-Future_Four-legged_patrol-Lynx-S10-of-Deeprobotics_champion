"""Coupled planar pose PID; physical velocity outputs, no policy speed clamps.

Errors and PID memory live in the map frame. Only the resulting velocity is
rotated into the robot frame, so yaw motion cannot rotate accumulated error.
The integral uses a finite time window: a moving lookahead reference otherwise
accumulates intentional along-track error indefinitely. Derivative is taken on
measured pose and low-pass filtered, avoiding derivative kick at route corners.
"""
from collections import deque
import math


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class PosePID:
    def __init__(self, kp=(.9, .9, 1.1), ki=(.08, .08, .03),
                 kd=(.22, .22, .35), integral_window_s=2., derivative_tau_s=.12):
        self.kp, self.ki, self.kd = tuple(kp), tuple(ki), tuple(kd)
        self.integral_window_s = integral_window_s
        self.derivative_tau_s = derivative_tau_s
        self.reset()

    def reset(self):
        self.history = deque()
        self.previous = None
        self.measured_previous = None
        self.odometry_previous = None
        self.rate = [0., 0., 0.]

    def step(self, pose, reference_xy, now, reference_yaw=None, reference_velocity=None,
             measurement_mono=None,measurement_source=None,follow_path=False,measurement_pose=None,
             measurement_frame=None):
        x, y, _, yaw = pose
        mx,my,_,myaw=pose if measurement_pose is None else measurement_pose
        dx, dy = reference_xy[0] - x, reference_xy[1] - y
        heading = reference_yaw if reference_yaw is not None else (math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-9 else yaw)
        feedforward = list(reference_velocity) if reference_velocity is not None else [0.,0.,0.]
        error = [dx, dy, wrap(heading - yaw)]
        stamp=now if measurement_mono is None else measurement_mono
        if self.measured_previous is not None:
            previous_t,previous_pose,previous_source=self.measured_previous
            dt=stamp-previous_t
            if measurement_frame is not None and self.odometry_previous is not None:
                # Express both real odometry samples in the SAME current map
                # alignment. A smooth map correction is not body velocity.
                A=measurement_frame['map_to_odom'];B=self.odometry_previous
                x0=sum(A[0][k]*B[k][3] for k in range(4))
                y0=sum(A[1][k]*B[k][3] for k in range(4))
                yaw0=math.atan2(sum(A[1][k]*B[k][0] for k in range(3)),sum(A[0][k]*B[k][0] for k in range(3)))
                previous_pose=(x0,y0,yaw0)
            if measurement_source!=previous_source:
                # A source offset is not physical velocity. Re-prime the
                # derivative without clearing integral or interrupting motion.
                self.measured_previous=None
            elif dt>0:
                delta=[mx-previous_pose[0],my-previous_pose[1],wrap(myaw-previous_pose[2])]
                alpha=-math.expm1(-dt/self.derivative_tau_s)
                self.rate=[v+alpha*(d/dt-v) for v,d in zip(self.rate,delta)]
        if self.measured_previous is None or stamp>self.measured_previous[0]:
            self.measured_previous=(stamp,(mx,my,myaw),measurement_source)
            self.odometry_previous=None if measurement_frame is None else measurement_frame['odom_T']
        if self.previous is not None:
            previous_t, previous_pose, previous_error = self.previous
            dt = now - previous_t
            if dt > 0:
                mean_error = [(a + b) / 2 for a, b in zip(previous_error, error)]
                # Integrate circular yaw error without crossing the long way at +/-pi.
                mean_error[2] = wrap(previous_error[2] + wrap(error[2] - previous_error[2]) / 2)
                self.history.append((previous_t, now, mean_error))
        cutoff = now - self.integral_window_s
        while self.history and self.history[0][1] <= cutoff:
            self.history.popleft()
        integral = [sum((end - max(begin, cutoff)) * value[i]
                        for begin, end, value in self.history) for i in range(3)]
        self.previous = (now, (x, y, yaw), error)
        proportional = [k * e for k, e in zip(self.kp, error)]
        accumulated = [k * e for k, e in zip(self.ki, integral)]
        reference_rate=list(feedforward)
        if follow_path:
            # The spatial reference advances with measured route progress,
            # not at the nominal cruise velocity. Differentiate that reference
            # consistently: damp cross-track motion without adding a second
            # forward-speed boost when delayed pose samples are held still.
            speed=math.hypot(*feedforward[:2])
            tangent=[v/max(speed,1e-12) for v in feedforward[:2]]
            along=sum(a*b for a,b in zip(tangent,self.rate[:2]))
            reference_rate[:2]=[v*along for v in tangent]
        derivative = [k * (target-v) for k, target, v in zip(self.kd, reference_rate, self.rate)]
        command = [f + p + i + d for f, p, i, d in zip(feedforward, proportional, accumulated, derivative)]
        c, s = math.cos(yaw), math.sin(yaw)
        vx = c * command[0] + s * command[1]
        vy = -s * command[0] + c * command[1]
        return dict(vx=vx, vy=vy, wz=command[2], heading_error_rad=error[2],
                    tracking=dict(controller='map_pose_pid', reference_xy=list(reference_xy[:2]),
                        reference_yaw=heading, error_map=error, measured_rate_map=list(self.rate),
                        rate_measurement_mono=stamp,measurement_source=measurement_source,
                        reference_rate_map=reference_rate,
                        p=proportional, i=accumulated, d=derivative, feedforward_map=feedforward,command_map=command,
                        kp=list(self.kp), ki=list(self.ki), kd=list(self.kd),
                        integral_window_s=self.integral_window_s, derivative_tau_s=self.derivative_tau_s))


class HeadingReference:
    """Critically damped heading trajectory; angle and rate evolve together.

No angular-rate or acceleration clipping, and no turn-only motion phase.
The trajectory rate replaces kappa*nominal_speed, which was inconsistent
with a reference point whose progress follows measured robot movement.
"""
    def __init__(self,bandwidth=2.5):
        self.bandwidth=bandwidth;self.reset()

    def reset(self):self.yaw=None;self.rate=0.;self.stamp=None

    def step(self,target_yaw,measured_yaw,now):
        if self.yaw is None:self.yaw=measured_yaw;self.stamp=now-.05
        dt=max(0.,now-self.stamp);self.stamp=now
        error=wrap(self.yaw-target_yaw);w=self.bandwidth;decay=math.exp(-w*dt)
        combined=self.rate+w*error
        self.yaw=wrap(target_yaw+(error+combined*dt)*decay)
        self.rate=(self.rate-w*combined*dt)*decay
        return self.yaw,self.rate
