import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.steering_yaw_controller = YawController(wheel_base, steer_ratio, 0.1, 0.3, max_steer_angle)
        self.steering_pid_controller = PID(kp = 50.0, ki = 0.1, kd = 0.0, mn = -max_steer_angle, mx = max_steer_angle)
        
        self.throttle_controller = PID(kp = 0.52, ki = 0.45, kd = 0.23, mn = 0.0, mx = 0.3)
        
        self.vel_lpf = LowPassFilter(tau = 0.5 ,ts = 0.02)
        #self.vel_lpf = LowPassFilter(tau = 0.2 ,ts = 0.1)
        self.ang_vel_lpf = LowPassFilter(tau = 0.5 ,ts = 0.02)
        
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.max_steer_angle = max_steer_angle
        
        self.last_time = rospy.get_time()

    def control(self, current_vel, curr_ang_vel, dbw_enabled, target_linear_vel, target_angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.,0.,0.
        
        current_vel_in = current_vel
        current_vel = self.vel_lpf.filt(current_vel)
        
        curr_ang_vel_in = curr_ang_vel
        curr_ang_vel = self.ang_vel_lpf.filt(curr_ang_vel)
        
        vel_error = target_linear_vel - current_vel
        self.last_vel = current_vel
        
        ang_vel_error = target_angular_vel 
        
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        
        steering_yaw = self.steering_yaw_controller.get_steering(target_linear_vel, target_angular_vel, current_vel)
        steering_pid = self.steering_pid_controller.step(ang_vel_error, sample_time)
        steering = (1.0*steering_yaw + 0*steering_pid)
        
        if target_linear_vel <= 0.1 and current_vel <= 0.1:
            throttle = 0
            brake = 400 #N*m - to hold the car in place if we are stopped at a light. Acceleration - 1m/s^2
        elif vel_error >= 0:
            throttle = self.throttle_controller.step(vel_error, sample_time)
            brake = 0
        else:
            print vel_error,current_vel,target_linear_vel
            decel = max(vel_error/sample_time,self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius # Torque N*m
            throttle = 0
        
        print current_vel, target_linear_vel, curr_ang_vel, target_angular_vel, throttle, brake, steering
            
        return throttle, brake, steering
