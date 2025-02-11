import rclpy
import sys
import select
import time
from rclpy.node import Node
from std_msgs.msg import Float32

class MotorSpeedPublisher(Node):
    def __init__(self):
        super().__init__('motor_speed_publisher')
        self.publisher_ = self.create_publisher(Float32, '/motor0_speed', 10)
        self.timer_period = 0.001  # 100msごとに更新
        self.timer_ = self.create_timer(self.timer_period, self.timer_callback)
        self.speed = 0.0
        self.state = 'WAIT_FOR_ENTER'  # 初期状態
        self.start_time = None

        self.max_speed =34.2
        self.accel =0.1

        self.time =0.11

    def timer_callback(self):
        msg = Float32()

        if self.state == 'WAIT_FOR_ENTER':
            # エンターキーが押されるまで 0.0 をパブリッシュ
            msg.data = 0.0
            self.publisher_.publish(msg)
            if self.is_enter_pressed():
                self.get_logger().info("Enter key pressed. Starting speed change.")
                self.state = 'INCREASING'
                self.start_time = time.time()

        elif self.state == 'INCREASING':
            # 1秒間で 0 → 10 に変化
            elapsed = time.time() - self.start_time
            if elapsed >= self.accel:
                self.speed = self.max_speed
                self.state = 'HOLDING'
                self.start_time = time.time()
            else:
                self.speed = elapsed * self.max_speed  # (0→10)に線形変化
            
            msg.data = self.speed
            self.publisher_.publish(msg)

        elif self.state == 'HOLDING':
            # 1秒間 10 を維持
            if time.time() - self.start_time >= self.time:
                self.state = 'DECREASING'
                self.start_time = time.time()
            
            msg.data = self.max_speed
            self.publisher_.publish(msg)

        elif self.state == 'DECREASING':
            # 1秒間で 10 → 0 に変化
            elapsed = time.time() - self.start_time
            if elapsed >= self.accel:
                self.speed = 0.0
                msg.data = self.speed
                self.publisher_.publish(msg)
                self.get_logger().info("Sequence complete. Shutting down node.")
                rclpy.shutdown()  # ノードを終了
            else:
                self.speed = self.max_speed * (self.accel - elapsed)  # (10→0)に線形変化
            
            msg.data = self.speed
            self.publisher_.publish(msg)

    def is_enter_pressed(self):
        # 非ブロッキングでEnterキーの入力をチェック
        i, _, _ = select.select([sys.stdin], [], [], 0)
        return bool(i)

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
