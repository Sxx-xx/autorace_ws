import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

class CustomTeleop(Node):
    def __init__(self):
        super().__init__('custom_teleop')
        # MUX를 거치지 않고 바퀴로 직접(cmd_vel) 명령을 보냅니다.
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.speed = 0.15 # 직진/후진 속도 (m/s)
        self.turn = 0.5   # 회전 속도 (rad/s)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        # 0.1초 동안 키보드 입력을 기다림
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def timer_callback(self):
        key = self.get_key()
        twist = Twist()
        
        if key == 'w':
            twist.linear.x = self.speed
        elif key == 's':
            twist.linear.x = -self.speed
        elif key == 'a':
            twist.angular.z = self.turn
        elif key == 'd':
            twist.angular.z = -self.turn
        elif key == '\x03': # Ctrl+C
            raise KeyboardInterrupt
        else:
            # 손을 떼면(0.1초간 입력이 없으면) 즉시 0을 퍼블리시하여 정지
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CustomTeleop()
    try:
        print("=== 데이터 기록용 커스텀 수동 조작 노드 ===")
        print("W / A / S / D 키를 꾹 누르고 있을 때만 움직입니다.")
        print("손을 떼면 자동으로 로봇이 멈춥니다.")
        print("종료하려면 Ctrl+C 를 누르세요.")
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 안전을 위해 종료 시 정지 명령 한 번 더 전송
        node.publisher_.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
