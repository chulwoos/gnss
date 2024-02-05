import rclpy
import math
import numpy as np
import csv
import pyproj
import random

from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped

class GNSS(Node):  
    def __init__(self):
        super().__init__('GNSS_Node')

        # print("==================GNSS_RUN==================")

        # sub
        qos_profile = QoSProfile(depth=10)
        # self.utm = self.create_subscription(PointStamped, '/UTM', self.gps_callback, qos_profile) 
        self.fix = self.create_subscription(NavSatFix, '/fix', self.gps_callback, qos_profile)

        # pub
        self.utm_kf = self.create_publisher(PointStamped, '/UTM_kf', 10)

        # QoS
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.ROS_publisher)

        # fix init
        self.fix_x = 0
        self.fix_y = 0
        self.fix_head = 0

        # gps init
        self.gps_in = False
        self.utm_x = 0
        self.utm_y = 0
        self.utm_head = 0

        # Kalman Filter init
        self.x = np.zeros([3, 1], dtype=np.float64)
        self.init_x = np.zeros([3, 1], dtype=np.float64)
        self.P = np.identity(n=3, dtype=np.float64)
        self.A = np.identity(n=3, dtype=np.float64)  # 상태 전이 행렬
        self.Pp = np.identity(n=3, dtype=np.float64)  # 예측된 공분산 행렬
        self.Q = np.identity(n=3, dtype=np.float64) * 0.05
        self.R = np.identity(n=3, dtype=np.float64) * 1 # 0.001
        self.S = np.zeros([3, 3], dtype=np.float64)
        self.K = np.zeros([3, 3], dtype=np.float64)

        self.proj_wgs84 = pyproj.Proj(init='epsg:4326')  # WGS84 coordinate system
        self.proj_utm = pyproj.Proj(init='epsg:32652')  # UTM coordinate system
        



    def gps_callback(self, msg):
        # print("GNSS_in !!!!")
        self.utm_x = msg.latitude
        self.utm_y = msg.longitude
        self.utm_head = msg.altitude
        
        self.gps_in = True

        self.utm_x, self.utm_y = pyproj.transform(self.proj_wgs84, self.proj_utm, self.utm_y, self.utm_x)

        self.x[0, 0] = self.utm_x
        self.x[1, 0] = self.utm_y

        trigger_num1= random.randrange(0,1)
        trigger_num2= random.randrange(0,1)
        if trigger_num1 <= 0.2:
            print("dasmjfnasnhfasnjfosan")
            rand_num = random.randrange(-1,1)
            buho_num = random.randrange(0,1)
            if buho_num <= 0.5:
                self.utm_x += rand_num
            else:
                self.utm_x -= rand_num
        if trigger_num2 <= 0.1:
            print("111111111111122222222222222222222222222222")
            rand_num_2 = random.randrange(-1,1)
            buho_num_2 = random.randrange(0,1)
            if buho_num_2 <= 0.5:
                self.utm_y += rand_num_2
            else:
                self.utm_y -= rand_num_2


        

    def kf_cal(self):
        if not self.gps_in:
            return
        elif self.utm_x == 0 and self.utm_y == 0:
            return
        else:
            # print("kf_cal_start!!!!")
            print(self.utm_x)
            print(self.utm_y)

            # 예측 단계
            # 간단한 선형 모델을 가정하고 있습니다 (A 및 Q를 실제 시스템 모델에 맞게 조절해야 합니다).
            self.x = np.dot(self.A, self.x)  # 예측에 따라 상태를 업데이트
            self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q  # 예측에 따라 공분산 행렬 업데이트


            # 업데이트 단계
            # 간단한 선형 관측 모델을 가정하고 있습니다 (H 및 R을 실제 측정 모델에 맞게 조절해야 합니다).
            H = np.identity(n=3, dtype=np.float64)[:2, :]  # H를 헤딩 정보를 제외하도록 수정
            observed_data = np.array([[self.utm_x, self.utm_y]])
            innovation = observed_data.T - np.dot(H, self.x)  # 혁신 계산
            S = np.dot(np.dot(H, self.P), H.T) + self.R  # 혁신 공분산 계산
            K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))  # 칼만 이득 계산
            self.x = self.x + np.dot(K, innovation)  # 측정에 따라 상태를 업데이트
            self.P = np.dot((np.identity(3) - np.dot(K, H)), self.P)  # 측정에 따라 공분산 행렬 업데이트

            self.save_csv()

    def ROS_publisher(self):
        self.kf_cal()

        kf_msg = PointStamped()
        kf_msg.header.frame_id = "GPS"
        kf_msg.header.stamp = self.get_clock().now().to_msg()
        kf_msg.point.x = float(self.x[0, 0])
        kf_msg.point.y = float(self.x[1, 0])
        kf_msg.point.z = float(self.x[2, 0])
        self.utm_kf.publish(kf_msg)

    
    def save_csv(self):
        file_path_1 = 'GNSS_before.csv'
        fieldnames = ['UTM_X', 'UTM_Y']

        with open(file_path_1, mode='a', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            if csvfile.tell() == 0:
                writer.writeheader()

            writer.writerow({
                'UTM_X' : self.utm_x,
                'UTM_Y' : self.utm_y
            })
        
        file_path_2 = 'GNSS_after.csv' 
        fieldnames = ['UTM_X', 'UTM_Y']

        with open(file_path_2, mode='a', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            if csvfile.tell() == 0:
                writer.writeheader()

            writer.writerow({
                'UTM_X' : float(self.x[0, 0]),
                'UTM_Y' : float(self.x[1, 0])
            })

def main(args=None):
    rclpy.init(args=args)
    gnss_node = GNSS()

    try:
        rclpy.spin(gnss_node)
    except KeyboardInterrupt:
        pass
    
    gnss_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
