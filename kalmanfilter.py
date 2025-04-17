from XRPLib.board import Board
from XRPLib.differential_drive import DifferentialDrive
from machine import Timer
from XRPLib.imu import IMU
import time, math, gc, os

drivetrain = DifferentialDrive.get_default_differential_drive()
imu = IMU.get_default_imu()

#Data Collection
gc.collect()
data_interval = 100
data = []
filename = "data.csv"
if filename in os.listdir():
    os.remove(filename)
    print(f"{filename} deleted. ")
else:
    print(f"{filename} does not exist. ")

# Hardware timer
hardware_timer_period = 0.1 #s

class PositionEstimations:
    def __init__(self):
        self.track_width = 15.5
        self.wheel_diameter = 6
        self.RPMtoCMPS = (math.pi* self.wheel_diameter) / 60
        self.CMPStoRPM = 60 / (math.pi*self.wheel_diameter)
        
        self.x = 0
        self.y = 0
        self.theta = 0

        self.x_kf = 0
        self.y_kf = 0
        self.theta_kf = 0
        self.prev_theta_kf = 0

        self.P = 0.1
        self.Q = 0.01
        self.R = 20

        self.last_data_time = time.ticks_ms()

    def get_filtered_imu(self):
        readings = []
        for i in range(5):
            d = imu.get_gyro_z_rate()*(math.pi / 180000)
            readings.append(d)
            time.sleep(0.01)

        readings.sort()
        return readings[2]  # Return the median

    def kalman_filter_theta(self, w_enc, w_imu):
        self.prev_theta_kf = self.theta_kf
        theta_pred = self.theta_kf + w_enc * hardware_timer_period
        P_pred = self.P + self.Q

        K = P_pred/(P_pred + self.R)
        self.theta_kf = theta_pred + K*((w_imu*hardware_timer_period) - theta_pred)
        self.P = (1-K)* P_pred


    def pose_update(self):
        #read motor velocities and calculate motor w
        right_motor_speed = drivetrain.right_motor.get_speed()*self.RPMtoCMPS
        left_motor_speed = drivetrain.left_motor.get_speed()*self.RPMtoCMPS
        w_enc  = (right_motor_speed-left_motor_speed)/self.track_width
        
        w_imu = self.get_filtered_imu()

        #Update Kalman theta estimate
        self.kalman_filter_theta(w_enc, w_imu)

        if w_enc == 0: 
            V = (left_motor_speed+right_motor_speed)*0.5
            self.x += V*math.cos(self.theta)*hardware_timer_period
            self.y += V*math.sin(self.theta)*hardware_timer_period
        else:
            R = self.track_width*0.5*(right_motor_speed+left_motor_speed)/(right_motor_speed-left_motor_speed)
            self.x += -R*math.sin(self.theta) + R*math.sin(self.theta+w_enc*hardware_timer_period)
            self.y += R*math.cos(self.theta) - R*math.cos(self.theta+w_enc*hardware_timer_period)
            self.theta += w_enc*hardware_timer_period
        
        if w_enc == 0:
            V = (left_motor_speed + right_motor_speed) * 0.5
            self.x_kf += V * math.cos(self.prev_theta_kf) * hardware_timer_period
            self.y_kf += V * math.sin(self.prev_theta_kf) * hardware_timer_period
        else:
            R = self.track_width * 0.5 * (right_motor_speed + left_motor_speed) / (right_motor_speed - left_motor_speed)
            self.x_kf += -R * math.sin(self.prev_theta_kf) + R * math.sin(self.theta_kf)
            self.y_kf += R * math.cos(self.prev_theta_kf) - R * math.cos(self.theta_kf)


        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, self.last_data_time) >= data_interval:
            data.append([
                self.x, self.y, self.theta,
                self.x_kf, self.y_kf, self.theta_kf
            ])
            self.last_data_time = current_time
            # print(current_time)


forward_kinematics = PositionEstimations()

# Start timer for periodic updates
timer = Timer()  # Use hardware timer 0
timer.init(period=int(hardware_timer_period * 1000),
           mode=Timer.PERIODIC, 
           callback=lambda t: forward_kinematics.pose_update())

drivetrain.set_speed(10,10)
time.sleep(5)
drivetrain.set_speed(10,30)
time.sleep(2)
drivetrain.set_speed(20,0)
time.sleep(10)
drivetrain.set_speed(0,10)
time.sleep(2)
drivetrain.set_speed(0,0)

#After experiment, write to file
with open(filename, "w") as f:
    f.write('x_raw,y_raw,theta_raw,x_kf,y_kf,theta_kf\n')
    for row in data:
        f.write(f"{row[0]},{row[1]},{row[2]},{row[3]},{row[4]},{row[5]}\n")
print("Experiment completed")
data.clear()
