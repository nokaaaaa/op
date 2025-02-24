import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import time

class MotorSpeedPublisher(Node):
    def __init__(self):
        super().__init__('motor_speed_publisher')
        self.publisher_ = self.create_publisher(Float32, '/motor5_speed', 10)
        self.subscription = self.create_subscription(Bool, '/launcher1', self.launch_callback, 10)
        self.timer_period = 0.01  # 10msごとに更新
        self.timer_ = self.create_timer(self.timer_period, self.timer_callback)
        self.speed = 0.0
        self.state = 'WAIT_FOR_TRIGGER'  # 初期状態
        self.start_time = None
        self.launch_triggered = False

        self.max_speed = 34.2
        self.accel = 0.1
        self.time = 0.11

    def launch_callback(self, msg):
        if msg.data and not self.launch_triggered:
            self.get_logger().info("Launcher triggered. Starting speed change.")
            self.launch_triggered = True
            self.state = 'INCREASING'
            self.start_time = time.time()

    def timer_callback(self):
        msg = Float32()
        
        if self.state == 'WAIT_FOR_TRIGGER':
            msg.data = 0.0
            self.publisher_.publish(msg)
        
        elif self.state == 'INCREASING':
            elapsed = time.time() - self.start_time
            if elapsed >= self.accel:
                self.speed = self.max_speed
                self.state = 'HOLDING'
                self.start_time = time.time()
            else:
                self.speed = elapsed / self.accel * self.max_speed  # (0→max_speed)に線形変化
            
            msg.data = self.speed
            self.publisher_.publish(msg)

        elif self.state == 'HOLDING':
            if time.time() - self.start_time >= self.time:
                self.state = 'DECREASING'
                self.start_time = time.time()
            msg.data = self.max_speed
            self.publisher_.publish(msg)

        elif self.state == 'DECREASING':
            elapsed = time.time() - self.start_time
            if elapsed >= self.accel:
                self.speed = 0.0
                msg.data = self.speed
                self.publisher_.publish(msg)
                self.get_logger().info("Sequence complete. Resetting state.")
                self.state = 'WAIT_FOR_TRIGGER'
                self.launch_triggered = False  # トリガーをリセット
            else:
                self.speed = self.max_speed * (1 - elapsed / self.accel)  # (max_speed→0)に線形変化
            
            msg.data = self.speed
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorSpeedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
