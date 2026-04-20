import time
import math
import smbus2 as smbus
from gpiozero import LED


class PCA9685:
    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06
    LED0_ON_H = 0x07
    LED0_OFF_L = 0x08
    LED0_OFF_H = 0x09

    def __init__(self, address=0x40, bus_id=1):
        self.bus = smbus.SMBus(bus_id)
        self.address = address
        self.write(self.MODE1, 0x00)

    def write(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value)

    def read(self, reg):
        return self.bus.read_byte_data(self.address, reg)

    def set_pwm_freq(self, freq):
        prescale_val = 25000000.0 / 4096.0 / float(freq) - 1.0
        prescale = math.floor(prescale_val + 0.5)

        old_mode = self.read(self.MODE1)
        sleep_mode = (old_mode & 0x7F) | 0x10

        self.write(self.MODE1, sleep_mode)
        self.write(self.PRESCALE, int(prescale))
        self.write(self.MODE1, old_mode)

        time.sleep(0.005)
        self.write(self.MODE1, old_mode | 0x80)

    def set_pwm(self, channel, on, off):
        self.write(self.LED0_ON_L + 4 * channel, on & 0xFF)
        self.write(self.LED0_ON_H + 4 * channel, on >> 8)
        self.write(self.LED0_OFF_L + 4 * channel, off & 0xFF)
        self.write(self.LED0_OFF_H + 4 * channel, off >> 8)

    def set_duty_cycle(self, channel, percent):
        percent = max(0, min(100, percent))
        self.set_pwm(channel, 0, int(percent * (4095 / 100)))

    def set_level(self, channel, value):
        self.set_pwm(channel, 0, 4095 if value == 1 else 0)

    def set_servo_pulse(self, channel, pulse_us):
        pulse = int(pulse_us * 4096 / 20000)
        self.set_pwm(channel, 0, pulse)

    # Backward compatibility
    def setPWMFreq(self, freq):
        self.set_pwm_freq(freq)

    def setPWM(self, channel, on, off):
        self.set_pwm(channel, on, off)

    def setDutycycle(self, channel, percent):
        self.set_duty_cycle(channel, percent)

    def setLevel(self, channel, value):
        self.set_level(channel, value)

    def setServoPulse(self, channel, pulse_us):
        self.set_servo_pulse(channel, pulse_us)


class RobotControl:
    def __init__(self):
        self.pwm = PCA9685(0x40)
        self.pwm.set_pwm_freq(50)

        self.channels = {
            "A_PWM": 0,
            "A_IN1": 2,
            "A_IN2": 1,
            "B_PWM": 5,
            "B_IN1": 3,
            "B_IN2": 4,
            "C_PWM": 6,
            "C_IN1": 8,
            "C_IN2": 7,
            "D_PWM": 11,
            "SERVO_PAN": 9,
            "SERVO_TILT": 10,
        }

        self.motor_d1 = LED(25)
        self.motor_d2 = LED(24)

        self.pan_angle = 90
        self.tilt_angle = 90

        self.set_servo_angle(self.channels["SERVO_PAN"], self.pan_angle)
        self.set_servo_angle(self.channels["SERVO_TILT"], self.tilt_angle)

    def set_servo_angle(self, channel, angle):
        angle = max(0, min(180, angle))
        pulse_us = 500 + (angle / 180.0) * 2000
        self.pwm.set_servo_pulse(channel, pulse_us)

    def set_camera_angle(self, pan_angle, tilt_angle):
        self.pan_angle = max(0, min(180, pan_angle))
        self.tilt_angle = max(0, min(180, tilt_angle))

        self.set_servo_angle(self.channels["SERVO_PAN"], self.pan_angle)
        self.set_servo_angle(self.channels["SERVO_TILT"], self.tilt_angle)

        print(f"set_camera_angle pan={self.pan_angle} tilt={self.tilt_angle}")

    def camera_left(self, step=10):
        self.pan_angle = min(180, self.pan_angle + step)
        self.set_servo_angle(self.channels["SERVO_PAN"], self.pan_angle)
        print(f"camera_left pan={self.pan_angle}")

    def camera_right(self, step=10):
        self.pan_angle = max(0, self.pan_angle - step)
        self.set_servo_angle(self.channels["SERVO_PAN"], self.pan_angle)
        print(f"camera_right pan={self.pan_angle}")

    def camera_up(self, step=10):
        self.tilt_angle = min(180, self.tilt_angle + step)
        self.set_servo_angle(self.channels["SERVO_TILT"], self.tilt_angle)
        print(f"camera_up tilt={self.tilt_angle}")

    def camera_down(self, step=10):
        self.tilt_angle = max(0, self.tilt_angle - step)
        self.set_servo_angle(self.channels["SERVO_TILT"], self.tilt_angle)
        print(f"camera_down tilt={self.tilt_angle}")

    def camera_center(self):
        self.pan_angle = 90
        self.tilt_angle = 90
        self.set_servo_angle(self.channels["SERVO_PAN"], self.pan_angle)
        self.set_servo_angle(self.channels["SERVO_TILT"], self.tilt_angle)
        print("camera_center")

    def stop(self):
        for ch in [
            self.channels["A_PWM"],
            self.channels["B_PWM"],
            self.channels["C_PWM"],
            self.channels["D_PWM"],
        ]:
            self.pwm.set_duty_cycle(ch, 0)

        self.motor_d1.off()
        self.motor_d2.off()
        print("stop")

    def forward(self, speed=25):
        print(f"forward {speed}")

        self.pwm.set_duty_cycle(self.channels["A_PWM"], speed)
        self.pwm.set_level(self.channels["A_IN1"], 0)
        self.pwm.set_level(self.channels["A_IN2"], 1)

        self.pwm.set_duty_cycle(self.channels["B_PWM"], speed)
        self.pwm.set_level(self.channels["B_IN1"], 1)
        self.pwm.set_level(self.channels["B_IN2"], 0)

        self.pwm.set_duty_cycle(self.channels["C_PWM"], speed)
        self.pwm.set_level(self.channels["C_IN1"], 1)
        self.pwm.set_level(self.channels["C_IN2"], 0)

        self.pwm.set_duty_cycle(self.channels["D_PWM"], speed)
        self.motor_d1.off()
        self.motor_d2.on()

    def backward(self, speed=25):
        print(f"backward {speed}")

        self.pwm.set_duty_cycle(self.channels["A_PWM"], speed)
        self.pwm.set_level(self.channels["A_IN1"], 1)
        self.pwm.set_level(self.channels["A_IN2"], 0)

        self.pwm.set_duty_cycle(self.channels["B_PWM"], speed)
        self.pwm.set_level(self.channels["B_IN1"], 0)
        self.pwm.set_level(self.channels["B_IN2"], 1)

        self.pwm.set_duty_cycle(self.channels["C_PWM"], speed)
        self.pwm.set_level(self.channels["C_IN1"], 0)
        self.pwm.set_level(self.channels["C_IN2"], 1)

        self.pwm.set_duty_cycle(self.channels["D_PWM"], speed)
        self.motor_d1.on()
        self.motor_d2.off()

    def left(self, speed=20):
        print(f"left {speed}")

        self.pwm.set_duty_cycle(self.channels["A_PWM"], speed)
        self.pwm.set_level(self.channels["A_IN1"], 1)
        self.pwm.set_level(self.channels["A_IN2"], 0)

        self.pwm.set_duty_cycle(self.channels["B_PWM"], speed)
        self.pwm.set_level(self.channels["B_IN1"], 1)
        self.pwm.set_level(self.channels["B_IN2"], 0)

        self.pwm.set_duty_cycle(self.channels["C_PWM"], speed)
        self.pwm.set_level(self.channels["C_IN1"], 0)
        self.pwm.set_level(self.channels["C_IN2"], 1)

        self.pwm.set_duty_cycle(self.channels["D_PWM"], speed)
        self.motor_d1.off()
        self.motor_d2.on()

    def right(self, speed=20):
        print(f"right {speed}")

        self.pwm.set_duty_cycle(self.channels["A_PWM"], speed)
        self.pwm.set_level(self.channels["A_IN1"], 0)
        self.pwm.set_level(self.channels["A_IN2"], 1)

        self.pwm.set_duty_cycle(self.channels["B_PWM"], speed)
        self.pwm.set_level(self.channels["B_IN1"], 0)
        self.pwm.set_level(self.channels["B_IN2"], 1)

        self.pwm.set_duty_cycle(self.channels["C_PWM"], speed)
        self.pwm.set_level(self.channels["C_IN1"], 1)
        self.pwm.set_level(self.channels["C_IN2"], 0)

        self.pwm.set_duty_cycle(self.channels["D_PWM"], speed)
        self.motor_d1.on()
        self.motor_d2.off()

    def move(self, direction, speed=25):
        if direction == "forward":
            self.forward(speed)
        elif direction == "backward":
            self.backward(speed)
        elif direction == "left":
            self.left(speed)
        elif direction == "right":
            self.right(speed)
        else:
            self.stop()

    # Backward compatibility aliases
    @property
    def CHANNELS(self):
        return self.channels

    @property
    def motorD1(self):
        return self.motor_d1

    @property
    def motorD2(self):
        return self.motor_d2