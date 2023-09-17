import numpy as np
import matplotlib.pyplot as plt
import copy as copy_module

import rclpy
from rclpy.node import Node
from autoware_auto_planning_msgs.msg import Path, Trajectory
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_adapi_v1_msgs.msg import MrmState
from autoware_auto_vehicle_msgs.msg import VelocityReport
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class ForcedTurnaroundOperator(Node):
    def __init__(self) -> None:
        super().__init__("forced_turnaround_operator")
        # Variables
        self.mrm_state = MrmState()
        self.mrm_state.stamp.sec = 0
        self.mrm_state.stamp.nanosec = 0
        self.mrm_state.state = 0
        self.mrm_state.behavior = 0

        self.drivable_area_x = []
        self.drivable_area_y = []

        self.longitudinal_velocity = 0.0
        self.lateral_velocity = 0.0
        self.position_ego_x = 3783.627518547305
        self.position_ego_y = 73752.9142289495
        self.orientation_ego_roll = 0
        self.orientation_ego_pitch = 0
        self.orientation_ego_yaw = 0
        self.velocity_ego_x = 0
        self.velocity_ego_y = 0

        self.forced_operation_continue = False
        self.forced_operation_counter = 0
        self.n_trial_turnaround = 0
        self.current_state = 0
        self.turnaround_area_x = [3725, 3725, 3717.5, 3717.5, 3725]
        self.turnaround_area_y = [73739.5, 73735, 73735, 73739.5, 73739.5]# [73740, 73735, 73735, 73740, 73740]

        # Subscriber - control_cmd(33Hz)
        self.sub_control_cmd = self.create_subscription(
            AckermannControlCommand,
            "/control/trajectory_follower/control_cmd_raw",
            self.callback_control_cmd,
            1,
        )
        self.sub_control_cmd # prevent unused variable warning

        # Subscriber - mrm_state(10Hz)
        # # For state
        # uint16 NORMAL = 1
        # uint16 MRM_OPERATING = 2
        # uint16 MRM_SUCCEEDED = 3
        # uint16 MRM_FAILED = 4
        # # For behavior
        # uint16 NONE = 1
        # uint16 EMERGENCY_STOP = 2
        # uint16 COMFORTABLE_STOP = 3
        self.sub_mrm_state = self.create_subscription(
            MrmState,
            "/system/fail_safe/mrm_state",
            self.callback_mrm_state,
            1,
        )
        self.sub_mrm_state # prevent unused variable warning

        # Subscriber - velocity_status(25Hz)
        self.sub_velocity_status = self.create_subscription(
            VelocityReport,
            "/vehicle/status/velocity_status",
            self.callback_velocity_status,
            1,
        )
        self.sub_mrm_state # prevent unused variable warning

        # Subscriver - drivable_area
        self.sub_drivable_area = self.create_subscription(
            MarkerArray,
            "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/maximum_drivable_area",
            self.callback_drivable_area,
            1,
        )
        self.sub_drivable_area

        # Subscriver - kinematic_state
        self.sub_kinematic_state = self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self.callback_kinematic_state,
            1,
        )

        # Publisher - control_cmd(33Hz)
        self.pub_control_cmd = self.create_publisher(
            AckermannControlCommand,
            "/control/trajectory_follower/control_cmd",
            1,
        )
        self.pub_control_cmd # prevent unused variable warning

    def callback_control_cmd(self, msg: AckermannControlCommand):
        # self.get_logger().info('I heard: "%s"' % msg.stamp)
        # self.get_logger().info('I heard: "%s"' % msg.lateral)
        # self.get_logger().info('I heard: "%s"' % msg.longitudinal)

        new_control_cmd = copy_module.deepcopy(msg)

        # パラメータ(G30As)
        # vehicle_length = 3.670 
        # vehicle_width = 1.265

        # 2023.9.6
        # vehicle_length = 3.45 
        # vehicle_width = 1.45
        # offset_lidar = 1.115

        vehicle_length = 3.110
        vehicle_width = 1.265
        # offset_lidar = 1.105
        offset_lidar = 1.115

        # デフォルト
        # time_predict = 0.5
        # steering_angle_1st_forward = -0.75
        # speed_1st_forward = 1.5/3.6
        # steering_angle_K_turn_backward = 0.0
        # speed_K_turn_backward = -1.5/3.6
        # steering_angle_K_turn_forward = -0.75
        # speed_K_turn_forward = 1.5/3.6
        # turnaround_area_y_ub = 73739.5
        # threshold_n_trial_turnaround = 30

        # 27 iter best
        # time_predict = 0.5492589999872608
        # steering_angle_1st_forward = 0.2600284239574323
        # speed_1st_forward = 0.2574850690962136
        # steering_angle_K_turn_backward = 0.4996101996086604
        # speed_K_turn_backward = -0.29673571758037365
        # steering_angle_K_turn_forward = -0.32710405550076926
        # speed_K_turn_forward = 0.5185754688092461
        # turnaround_area_y_ub = 73739.02861497483
        # threshold_n_trial_turnaround = 1

        time_predict = 1.5051187131331505
        steering_angle_1st_forward = -0.46349660776769863
        speed_1st_forward = 0.416682825947887
        steering_angle_K_turn_backward = 0.5959044694786704
        speed_K_turn_backward = -0.4574607813389212
        steering_angle_K_turn_forward = -0.4695637340875366
        speed_K_turn_forward = 0.6230359987022561
        turnaround_area_y_ub = 73739.62962531458
        threshold_n_trial_turnaround = 4
        
        self.turnaround_area_y[0], self.turnaround_area_y[-2], self.turnaround_area_y[-1] = turnaround_area_y_ub, turnaround_area_y_ub, turnaround_area_y_ub

        # X秒後における、車両後方の位置を予測し、ドライバブルエリア内に収まるかチェック
        # リア中央
        position_rear_center_x = self.position_ego_x - ((vehicle_length * 0.5 - offset_lidar) * np.cos(self.orientation_ego_yaw))
        position_rear_center_y = self.position_ego_y - ((vehicle_length * 0.5 - offset_lidar) * np.sin(self.orientation_ego_yaw))

        # リア左隅
        position_rear_left_x = position_rear_center_x - (0.5 * vehicle_width * np.sin(self.orientation_ego_yaw))
        position_rear_left_y = position_rear_center_y + (0.5 * vehicle_width * np.cos(self.orientation_ego_yaw))

        # リア右隅
        position_rear_right_x = position_rear_center_x + (0.5 * vehicle_width * np.sin(self.orientation_ego_yaw))
        position_rear_right_y = position_rear_center_y - (0.5 * vehicle_width * np.cos(self.orientation_ego_yaw))

        # フロント中央
        position_front_center_x = self.position_ego_x + ((vehicle_length * 0.5 + offset_lidar) * np.cos(self.orientation_ego_yaw))
        position_front_center_y = self.position_ego_y + ((vehicle_length * 0.5 + offset_lidar) * np.sin(self.orientation_ego_yaw))

        # フロント左隅
        position_front_left_x = position_front_center_x - (0.5 * vehicle_width * np.sin(self.orientation_ego_yaw))
        position_front_left_y = position_front_center_y + (0.5 * vehicle_width * np.cos(self.orientation_ego_yaw))

        # フロント右隅
        position_front_right_x = position_front_center_x + (0.5 * vehicle_width * np.sin(self.orientation_ego_yaw))
        position_front_right_y = position_front_center_y - (0.5 * vehicle_width * np.cos(self.orientation_ego_yaw))

        # # フロント右横 1
        # position_front_side_right_1_x = (self.position_ego_x + ((vehicle_length * 0.5 + 1.0 - 0.33) * np.cos(self.orientation_ego_yaw))) + (0.5 * vehicle_width * np.sin(self.orientation_ego_yaw))
        # position_front_side_right_1_y = (self.position_ego_y + ((vehicle_length * 0.5 + 1.0 - 0.33) * np.sin(self.orientation_ego_yaw))) - (0.5 * vehicle_width * np.cos(self.orientation_ego_yaw))

        # # フロント右横 2
        # position_front_side_right_2_x = (self.position_ego_x + ((vehicle_length * 0.5 + 1.0 - 0.66) * np.cos(self.orientation_ego_yaw))) + (0.5 * vehicle_width * np.sin(self.orientation_ego_yaw))
        # position_front_side_right_2_y = (self.position_ego_y + ((vehicle_length * 0.5 + 1.0 - 0.66) * np.sin(self.orientation_ego_yaw))) - (0.5 * vehicle_width * np.cos(self.orientation_ego_yaw))

        # # フロント右横 3
        # position_front_side_right_3_x = (self.position_ego_x + ((vehicle_length * 0.5 + 1.0 - 0.99) * np.cos(self.orientation_ego_yaw))) + (0.5 * vehicle_width * np.sin(self.orientation_ego_yaw))
        # position_front_side_right_3_y = (self.position_ego_y + ((vehicle_length * 0.5 + 1.0 - 0.99) * np.sin(self.orientation_ego_yaw))) - (0.5 * vehicle_width * np.cos(self.orientation_ego_yaw))

        # 判定
        velocity_x = self.longitudinal_velocity * np.cos(self.orientation_ego_yaw) - self.lateral_velocity * np.sin(self.orientation_ego_yaw)
        velocity_y = self.longitudinal_velocity * np.sin(self.orientation_ego_yaw) + self.lateral_velocity * np.cos(self.orientation_ego_yaw)

        # predicted_x = position_front_side_right_1_x + velocity_x * time_predict
        # predicted_y = position_front_side_right_1_y + velocity_y * time_predict

        is_rear_inside_drivable_area = (self.is_point_inside_area(position_rear_center_x + velocity_x * time_predict, position_rear_center_y + velocity_y * time_predict, self.drivable_area_x, self.drivable_area_y) 
                                    and self.is_point_inside_area(position_rear_left_x   + velocity_x * time_predict, position_rear_left_y   + velocity_y * time_predict, self.drivable_area_x, self.drivable_area_y) 
                                    and self.is_point_inside_area(position_rear_right_x  + velocity_x * time_predict, position_rear_right_y  + velocity_y * time_predict, self.drivable_area_x, self.drivable_area_y) )

        is_front_inside_drivable_area = (self.is_point_inside_area(position_front_center_x + velocity_x * time_predict, position_front_center_y + velocity_y * time_predict, self.drivable_area_x, self.drivable_area_y) 
                                     and self.is_point_inside_area(position_front_left_x   + velocity_x * time_predict, position_front_left_y   + velocity_y * time_predict, self.drivable_area_x, self.drivable_area_y) 
                                     and self.is_point_inside_area(position_front_right_x  + velocity_x * time_predict, position_front_right_y  + velocity_y * time_predict, self.drivable_area_x, self.drivable_area_y) )

        # is_front_side_inside_drivable_area = (self.is_point_inside_area(position_front_side_right_1_x + velocity_x * time_predict, position_front_side_right_1_y + velocity_y * time_predict, self.drivable_area_x, self.drivable_area_y) 
        #                                   and self.is_point_inside_area(position_front_side_right_2_x + velocity_x * time_predict, position_front_side_right_2_y + velocity_y * time_predict, self.drivable_area_x, self.drivable_area_y) 
        #                                   and self.is_point_inside_area(position_front_side_right_3_x + velocity_x * time_predict, position_front_side_right_3_y + velocity_y * time_predict, self.drivable_area_x, self.drivable_area_y) )
        
        # まとめて
        positions_x = [position_front_left_x, position_front_right_x, position_rear_right_x, position_rear_left_x, position_front_left_x]
        positions_y = [position_front_left_y, position_front_right_y, position_rear_right_y, position_rear_left_y, position_front_left_y]
        predicted_positions_x = [position_front_left_x  + velocity_x * time_predict, 
                                 position_front_right_x + velocity_x * time_predict, 
                                 position_rear_right_x  + velocity_x * time_predict, 
                                 position_rear_left_x   + velocity_x * time_predict, 
                                 position_front_left_x  + velocity_x * time_predict]
        predicted_positions_y = [position_front_left_y  + velocity_y * time_predict,
                                 position_front_right_y + velocity_y * time_predict, 
                                 position_rear_right_y  + velocity_y * time_predict, 
                                 position_rear_left_y   + velocity_y * time_predict, 
                                 position_front_left_y  + velocity_y * time_predict]
        is_vehicle_inside_area = self.is_vehicle_inside_area(predicted_positions_x, predicted_positions_y, self.drivable_area_x, self.drivable_area_y)

        # plt.clf()
        # plt.plot(self.position_ego_x, self.position_ego_y,'o')
        # plt.plot(positions_x, positions_y, '-ko')
        # if(not is_vehicle_inside_area):
        #     plt.plot(predicted_positions_x, predicted_positions_y, '-ro')
        # # plt.plot(position_front_side_right_1_x, position_front_side_right_1_y,'o')
        # # plt.plot(position_front_side_right_2_x, position_front_side_right_2_y,'o')
        # # plt.plot(position_front_side_right_3_x, position_front_side_right_3_y,'o')
        # plt.plot(self.drivable_area_x, self.drivable_area_y,'-ko')
        # plt.plot(self.turnaround_area_x, self.turnaround_area_y,'-ro')
        # plt.xlim([self.position_ego_x -5, self.position_ego_x +5])
        # plt.ylim([self.position_ego_y -5, self.position_ego_y +5])
        # plt.title("State: " + str(self.current_state) + ", n_turnaround: " + str(self.n_trial_turnaround) + ", is_vehicle_inside_area: " + str(is_vehicle_inside_area))
        # plt.gca().set_aspect('equal')
        # plt.grid()
        # plt.draw()
        # plt.pause(0.00000000001)

        # 狭路エリアに入ったら作動
        # State 0 : Default control
        # State 1 : 1st forward
        # State 2 : Repetitive K-turn(backward)
        # State 3 : Repetitive K-turn(forward)
        # State 4 : Last forward
        if(self.position_ego_x < 3730):
            if self.current_state == 0:
                # State 0: Default control
                if self.is_point_inside_area(position_front_center_x, position_front_center_y, self.turnaround_area_x, self.turnaround_area_y):  # Check condition to transition to State 1
                    self.current_state = 1  # Transition to State 1

            elif self.current_state == 1:
                # State 1: 1st forward
                new_control_cmd.lateral.steering_tire_angle = steering_angle_1st_forward
                new_control_cmd.lateral.steering_tire_rotation_rate = 0.0
                # new_control_cmd.longitudinal.speed = speed_1st_forward
                # new_control_cmd.longitudinal.acceleration = 9.8*0.01

                # 後退中の場合は、減速度を強めて、まずは停止
                if(self.longitudinal_velocity < 0.0):
                    new_control_cmd.longitudinal.acceleration = 9.8*0.2
                
                # Calculate predicted position for front left corner                
                # if not self.is_point_inside_area(position_front_left_x + velocity_x * time_predict, position_front_left_y + velocity_y * time_predict, self.drivable_area_x, self.drivable_area_y):
                # if not is_front_inside_drivable_area:
                if ((not is_vehicle_inside_area) and (is_rear_inside_drivable_area)):
                    # Transition to State 2 if the front left corner is about to exit the drivable area
                    self.current_state = 2
            
            elif self.current_state == 2:
                # State 2: Repetitive K-turn(backward)
                new_control_cmd.lateral.steering_tire_angle = steering_angle_K_turn_backward
                new_control_cmd.lateral.steering_tire_rotation_rate = 0.0
                new_control_cmd.longitudinal.speed = speed_K_turn_backward
                new_control_cmd.longitudinal.acceleration = 9.8*0.02

                # 前進中の場合は、減速度を強めて、まずは停止
                if(self.longitudinal_velocity > 0.0):
                    new_control_cmd.lateral.steering_tire_angle = steering_angle_K_turn_forward
                    new_control_cmd.longitudinal.acceleration = 9.8*0.2

                # Calculate predicted position for rear left corner                
                # if not self.is_point_inside_area(position_rear_left_x + velocity_x * time_predict, position_rear_left_y + velocity_y * time_predict, self.drivable_area_x, self.drivable_area_y):
                # if not (is_rear_inside_drivable_area):
                # if not (is_rear_inside_drivable_area and is_front_side_inside_drivable_area):
                if ((not is_vehicle_inside_area) and (is_front_inside_drivable_area)):
                    # Transition to State 1 if the rear left corner is about to exit the drivable area
                    self.current_state = 3
                    self.n_trial_turnaround += 1

            elif self.current_state == 3:
                # State 3: Repetitive K-turn(forward)
                new_control_cmd.lateral.steering_tire_angle = steering_angle_K_turn_forward
                new_control_cmd.lateral.steering_tire_rotation_rate = 0.0
                new_control_cmd.longitudinal.speed = speed_K_turn_forward
                new_control_cmd.longitudinal.acceleration = 9.8*0.02

                # 後退中の場合は、減速度を強めて、まずは停止
                if(self.longitudinal_velocity < 0.0):
                    new_control_cmd.lateral.steering_tire_angle = steering_angle_K_turn_backward
                    new_control_cmd.longitudinal.acceleration = 9.8*0.2

                # Calculate predicted position for front left corner                
                # if not self.is_point_inside_area(position_front_left_x + velocity_x * time_predict, position_front_left_y + velocity_y * time_predict, self.drivable_area_x, self.drivable_area_y):
                # if not (is_front_inside_drivable_area):
                # if not (is_front_inside_drivable_area and is_front_side_inside_drivable_area):
                if ((not is_vehicle_inside_area) and (is_rear_inside_drivable_area)):
                    # Transition to State 1 if the rear left corner is about to exit the drivable area
                    if(self.n_trial_turnaround >= threshold_n_trial_turnaround):
                        self.current_state = 4
                    else:
                        self.current_state = 2
            
            elif self.current_state == 4:
                # State 4: Last forward (autonomous turn)
                # Generate control commands for the final turn
                # Adjust the commands based on the desired turn angle and other factors
                
                # You can add more states if needed for additional actions
                None

            self.pub_control_cmd.publish(new_control_cmd)
        
        # 狭路エリア外はそのまま素通り
        else:
            self.pub_control_cmd.publish(msg)

    # 各種コールバック
    def callback_mrm_state(self, msg: MrmState):
        # self.get_logger().info('I heard: "%s"' % msg.stamp)
        # self.get_logger().info('I heard: "%s"' % msg.state)
        # self.get_logger().info('I heard: "%s"' % msg.behavior)
        self.mrm_state.stamp = msg.stamp
        self.mrm_state.state = msg.state
        self.mrm_state.behavior = msg.behavior

    def callback_velocity_status(self, msg: VelocityReport):
        # self.get_logger().info('I heard: "%s"' % msg.longitudinal_velocity)
        # self.get_logger().info('I heard: "%s"' % msg.lateral_velocity)
        # self.get_logger().info('I heard: "%s"' % msg.heading_rate)
        self.longitudinal_velocity = msg.longitudinal_velocity
        self.lateral_velocity = msg.lateral_velocity

    def callback_drivable_area(self, msg: MarkerArray):
        # self.get_logger().info('I heard: "%s"' % len(msg.markers[0].points))

        self.drivable_area_x = []
        self.drivable_area_y = []
        for i in range(len(msg.markers[0].points)):
            self.drivable_area_x.append(msg.markers[0].points[i].x)
            self.drivable_area_y.append(msg.markers[0].points[i].y)
    
    def callback_kinematic_state(self, msg: Odometry):
        # self.get_logger().info('I heard: "%s"' % msg.pose.pose.position)
        # self.get_logger().info('I heard: "%s"' % msg.pose.pose.orientation)
        # self.get_logger().info('I heard: "%s"' % msg.twist.twist.linear)
        # self.get_logger().info('I heard: "%s"' % msg.twist.twist.angular)

        self.position_ego_x = msg.pose.pose.position.x
        self.position_ego_y = msg.pose.pose.position.y
        self.orientation_ego_roll, self.orientation_ego_pitch, self.orientation_ego_yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.velocity_ego_x = msg.twist.twist.linear.x
        self.velocity_ego_y = msg.twist.twist.linear.y
        self.velocity_ego_z = msg.twist.twist.linear.z

    # エリア内判定
    def is_point_inside_area(self, position_x, position_y, area_x, area_y):
        num_vertices = len(area_x)
        is_inside = False
        j = num_vertices - 1
        for i in range(num_vertices):
            if ((area_y[i] > position_y) != (area_y[j] > position_y)) and \
            (position_x < (area_x[j] - area_x[i]) * (position_y - area_y[i]) / (area_y[j] - area_y[i]) + area_x[i]):
                is_inside = not is_inside
            j = i

        return is_inside
    
    # エリア内判定
    def is_vehicle_inside_area(self, positions_x, positions_y, area_x, area_y):
        edges = [
            ((positions_x[0], positions_y[0]), (positions_x[1], positions_y[1])),
            ((positions_x[1], positions_y[1]), (positions_x[2], positions_y[2])),
            ((positions_x[2], positions_y[2]), (positions_x[3], positions_y[3])),
            ((positions_x[3], positions_y[3]), (positions_x[0], positions_y[0]))
        ]

        def orientation(p, q, r):
            val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
            if val == 0:
                return 0
            return 1 if val > 0 else 2

        def on_segment(p, q, r):
            return (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and
                    q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1]))

        def do_intersect(p1, q1, p2, q2):
            o1 = orientation(p1, q1, p2)
            o2 = orientation(p1, q1, q2)
            o3 = orientation(p2, q2, p1)
            o4 = orientation(p2, q2, q1)

            if o1 != o2 and o3 != o4:
                return True

            if o1 == 0 and on_segment(p1, p2, q1):
                return True

            if o2 == 0 and on_segment(p1, q2, q1):
                return True

            if o3 == 0 and on_segment(p2, p1, q2):
                return True

            if o4 == 0 and on_segment(p2, q1, q2):
                return True

            return False

        for edge in edges:
            for i in range(len(area_x) - 1):
                p1, p2 = edge
                q1, q2 = (area_x[i], area_y[i]), (area_x[i+1], area_y[i+1])

                if do_intersect(p1, p2, q1, q2):
                    return False

        return True
    
    # クォータニオンからオイラー角
    def euler_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    print('Hi from forced_turnaround_operator.')
    rclpy.init(args=args)
    forced_turnaround_operator = ForcedTurnaroundOperator()

    rclpy.spin(forced_turnaround_operator)

    forced_turnaround_operator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
