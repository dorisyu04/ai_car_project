# -*- coding: utf-8 -*-

import time


class PIDController:
    def __init__(self, kp=0.18, ki=0.0, kd=0.03, output_limit=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

    def update(self, error):
        now = time.time()

        if self.prev_time is None:
            self.prev_time = now
            self.prev_error = error
            return self._clamp(self.kp * error)

        dt = now - self.prev_time
        if dt <= 0:
            dt = 1e-6

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        self.prev_error = error
        self.prev_time = now

        return self._clamp(output)

    def _clamp(self, value):
        if value > self.output_limit:
            return self.output_limit
        if value < -self.output_limit:
            return -self.output_limit
        return value