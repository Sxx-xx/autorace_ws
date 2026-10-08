#!/usr/bin/env python3
"""Start/stop switch for the lane run: Enter toggles /autorace/run_active.

Started once, so a toggle takes effect at once; a fresh `ros2 topic pub` per
press takes seconds to come up on a busy Pi. The state is repeated at 5 Hz so a
late subscriber still gets it. Ctrl+C, end of input or a hang-up (window
closed) sends 'stopped' before leaving.
"""
import signal
import threading
import time

import rclpy
import rclpy.executors
from std_msgs.msg import Bool

REPEAT_PERIOD = 0.2


def main():
    rclpy.init()
    node = rclpy.create_node('run_switch')
    pub = node.create_publisher(Bool, '/autorace/run_active', 1)
    state = {'run': False}
    node.create_timer(REPEAT_PERIOD, lambda: pub.publish(Bool(data=state['run'])))
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    def hang_up(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGHUP, hang_up)
    signal.signal(signal.SIGTERM, hang_up)

    try:
        while True:
            print('[주행 중]  Enter: 정지   Ctrl+C: 전부 종료' if state['run']
                  else '[정지]     Enter: 주행 시작   Ctrl+C: 전부 종료', flush=True)
            input()
            state['run'] = not state['run']
            pub.publish(Bool(data=state['run']))
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        state['run'] = False
        for _ in range(10):
            pub.publish(Bool(data=False))
            time.sleep(0.05)
        print('정지 신호 보냄', flush=True)
        executor.shutdown()
        spinner.join(timeout=1.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
