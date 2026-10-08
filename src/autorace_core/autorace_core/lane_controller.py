#!/usr/bin/env python3
#
# Copyright 2026 AutoRace Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Lane following controller.

Steering is pure pursuit: the lane detector hands over a point on the lane
centre a short way ahead, and the robot drives the arc that passes through it.
On an arc that is exact, so the robot holds the middle of the lane round a
bend instead of drifting to one side of it, and the turn rate follows from the
speed rather than from a gain tuned for one speed.  The lane leaves the robot
about 2 cm either side, which a fixed gain on the sideways offset could not
keep to.  Speed comes down where the arc is tight.

When the lane goes missing the controller does not simply give up: it keeps
creeping forward while turning the way it was last steering, which is usually
enough to bring the line back into view.  A robot that stands still for 30 s
ends its run, so stopping is the last resort, not the first.  Not before the
run has started, though (/autorace/run_active from the mission manager):
on the start line a lane that is not yet in view means the camera is still
coming up, and the robot stands still rather than go looking for it.
"""

import math

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import UInt8


# Of /detect/lane_state: both lines in view.
LANE_BOTH = 2
LANE_LEFT_ONLY = 1     # only the yellow line in view
LANE_RIGHT_ONLY = 3    # only the white line in view


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def line_turn_angle(points, colour, half_width):
    """How far to turn so that one line runs along the robot on its own side.

    points: (N, 2) metres from the axle, x ahead, y left. The line's
    direction u comes from a straight-line fit (PCA). Of u and -u the robot
    takes the one that leaves the yellow line on its left (white: right):
    the sign of cross(d, c), c the middle of the points, is the side the
    line lies on seen along d. The turn is the angle of d: a yellow line
    straight across the way gives -90 deg (turn right), a white one +90 deg
    (left), a line running ahead 0.

    Also returns how far to drive straight on first, so that after the turn
    the line is half a lane away: the way ahead meets the line at t, and
    a = t - half_width / |sin turn|.

    Returns (turn rad, advance m, length m, straightness) or None.
    """
    if len(points) < 3:
        return None
    c = points.mean(axis=0)
    cov = np.cov((points - c).T)
    values, vectors = np.linalg.eigh(cov)          # ascending
    u = vectors[:, 1]
    length = 4.0 * math.sqrt(max(values[1], 0.0))  # about the extent of the points
    straightness = math.sqrt(max(values[0], 0.0) / max(values[1], 1e-12))
    want_left = colour == 'yellow'
    for d in (u, -u):
        cross = d[0] * c[1] - d[1] * c[0]
        if (cross > 0.0) == want_left:
            break
    turn = math.atan2(d[1], d[0])
    advance = 0.0
    if abs(u[1]) > 1e-3:
        t = c[0] - c[1] * u[0] / u[1]              # where y = 0 on the line
        if t > 0.0 and abs(math.sin(turn)) > 0.2:
            advance = max(0.0, t - half_width / abs(math.sin(turn)))
    return turn, advance, length, straightness


def crossing_line(points, side, half_width, min_angle, max_angle, rng,
                  tolerance=0.015, iterations=150):
    """A straight line across the way, and the turn onto it towards `side`.

    For a turn the mission orders (the intersection sign), whatever the
    line's colour. RANSAC, so that the lane's own lines running ahead do not
    pull the fit: of the lines through two points that lie between
    min_angle and max_angle of the heading, the one with most points within
    `tolerance`. The turn is along that line towards `side`; the robot drives
    on first until it is half a lane short of where the line crosses its way.

    Returns (turn rad, advance m, inliers) or None.
    """
    n = len(points)
    if n < 2:
        return None
    best = None
    for _ in range(iterations):
        i, j = rng.integers(0, n, 2)
        d = points[j] - points[i]
        norm = math.hypot(d[0], d[1])
        if norm < 0.05:
            continue
        u = d / norm
        across = abs(math.atan2(u[1], u[0]))
        across = min(across, math.pi - across)          # 0 ahead, pi/2 straight across
        if not (min_angle <= across <= max_angle):
            continue
        normal = np.array([-u[1], u[0]])
        inliers = np.abs((points - points[i]) @ normal) < tolerance
        count = int(inliers.sum())
        if best is None or count > best[0]:
            best = (count, inliers)
    if best is None:
        return None
    count, inliers = best
    sel = points[inliers]
    c = sel.mean(axis=0)
    _, vectors = np.linalg.eigh(np.cov((sel - c).T))
    u = vectors[:, 1]
    if abs(u[1]) < 1e-3:
        return None
    d = u if (u[1] > 0) == (side == 'left') else -u
    turn = math.atan2(d[1], d[0])
    t = c[0] - c[1] * u[0] / u[1]                     # where y = 0 on the line
    if t <= 0.0:
        return None
    advance = max(0.0, t - half_width / max(abs(math.sin(turn)), 0.2))
    return turn, advance, count


class LaneController(Node):

    def __init__(self):
        super().__init__('lane_controller')

        self.declare_parameter('max_speed', 0.16)
        self.declare_parameter('min_speed', 0.05)
        # Up to the stop line before the start: no hurry, and the stop line
        # detector must not be outrun.
        self.declare_parameter('standby_speed', 0.08)
        # 1.0 is plain pure pursuit; more turns in harder towards the target.
        self.declare_parameter('pursuit_gain', 1.0)
        self.declare_parameter('max_angular', 2.0)
        # With one line in view the target is half a lane from it, and where
        # that line is cut short at the exit of a bend the point jumps. The
        # curvature may then change by no more than this per cycle (1/m).
        self.declare_parameter('single_line_slew', 0.5)
        # Turn rate above which the robot slows down rather than turn faster.
        self.declare_parameter('cornering_rate', 0.6)
        self.declare_parameter('lane_timeout', 0.5)
        self.declare_parameter('recovery_speed', 0.05)
        self.declare_parameter('recovery_angular', 0.45)
        self.declare_parameter('recovery_timeout', 8.0)
        self.declare_parameter('follow_timeout', 0.5)
        # A speed limit on /control/max_vel holds only while it keeps coming.
        self.declare_parameter('limit_timeout', 0.5)
        self.declare_parameter('publish_rate', 50.0)
        # Stop and turn on the spot at bends instead of steering through them
        # (the real robot's lane camera sees too little road to steer round a
        # bend at speed). Off by default: the sim keeps pure pursuit.
        self.declare_parameter('turn_in_place', False)
        # Target this far off the heading (rad) is a bend: stop and turn ...
        self.declare_parameter('turn_enter_angle', 0.35)
        # ... at this rate (rad/s) until it is within this angle again.
        self.declare_parameter('turn_rate', 0.25)
        self.declare_parameter('turn_exit_angle', 0.09)
        # With only one line in view, lying well across the way (a corner:
        # yellow across = the road goes right, white across = left), drive up
        # to it and turn on the spot by the line's angle, then follow the
        # lane again. Off by default.
        self.declare_parameter('line_turn', False)
        self.declare_parameter('line_turn_enter_angle', 1.05)   # rad (60 deg) off the heading
        # A corner is never more than this. A line that would need more lies on
        # the wrong side of the robot (it is across the yellow line, in the
        # other lane): turning would mean turning back. Ignore it.
        self.declare_parameter('line_turn_max_angle', 1.92)     # rad (110 deg)
        self.declare_parameter('line_turn_rate', 1.0)           # rad/s, slower near the end
        self.declare_parameter('line_turn_speed', 0.10)         # m/s while driving up to it
        self.declare_parameter('line_turn_half_width', 0.165)   # m, half the lane
        self.declare_parameter('line_turn_max_advance', 0.40)   # m
        self.declare_parameter('line_turn_min_points', 12)
        self.declare_parameter('line_turn_min_length', 0.10)    # m of line seen
        self.declare_parameter('line_turn_straightness', 0.25)  # max minor/major spread
        self.declare_parameter('line_turn_other_max', 5)        # points of the other line
        self.declare_parameter('line_turn_cooldown', 1.5)       # s after a turn
        self.declare_parameter('line_turn_timeout', 8.0)        # s for drive up + turn
        # No turn at a line goes further than this, whatever the line's angle:
        # a slanted fit (109 deg at the island, 2026-10-08) swung the robot off
        # the course; the lane follower straightens out the rest.
        self.declare_parameter('line_turn_limit', 1.571)        # rad (90 deg)
        # A turn the intersection mission orders on /control/turn_at_line
        # ('left'/'right'; 'hold' only keeps the automatic corner turns off).
        self.declare_parameter('ordered_turn_min_angle', 1.22)  # rad (70 deg) across the way
        self.declare_parameter('ordered_turn_max_angle', 1.75)  # rad (100 deg)
        self.declare_parameter('ordered_turn_min_points', 15)

        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.standby_speed = self.get_parameter('standby_speed').value
        self.pursuit_gain = self.get_parameter('pursuit_gain').value
        self.max_angular = self.get_parameter('max_angular').value
        self.single_line_slew = self.get_parameter('single_line_slew').value
        self.cornering_rate = self.get_parameter('cornering_rate').value
        self.lane_timeout = self.get_parameter('lane_timeout').value
        self.recovery_speed = self.get_parameter('recovery_speed').value
        self.recovery_angular = self.get_parameter('recovery_angular').value
        self.recovery_timeout = self.get_parameter('recovery_timeout').value
        self.follow_timeout = self.get_parameter('follow_timeout').value
        self.limit_timeout = self.get_parameter('limit_timeout').value
        rate = self.get_parameter('publish_rate').value
        self.turn_in_place = self.get_parameter('turn_in_place').value
        self.turn_enter_angle = self.get_parameter('turn_enter_angle').value
        self.turn_rate = self.get_parameter('turn_rate').value
        self.turn_exit_angle = self.get_parameter('turn_exit_angle').value
        lt = lambda name: self.get_parameter('line_turn' + name).value
        self.line_turn = lt('')
        self.lt_enter = lt('_enter_angle')
        self.lt_max = lt('_max_angle')
        self.lt_rate = lt('_rate')
        self.lt_speed = lt('_speed')
        self.lt_half_width = lt('_half_width')
        self.lt_max_advance = lt('_max_advance')
        self.lt_min_points = lt('_min_points')
        self.lt_min_length = lt('_min_length')
        self.lt_straightness = lt('_straightness')
        self.lt_other_max = lt('_other_max')
        self.lt_cooldown = lt('_cooldown')
        self.lt_timeout = lt('_timeout')
        self.lt_limit = lt('_limit')
        self.ot_min = self.get_parameter('ordered_turn_min_angle').value
        self.ot_max = self.get_parameter('ordered_turn_max_angle').value
        self.ot_min_points = self.get_parameter('ordered_turn_min_points').value

        self.create_subscription(PointStamped, '/detect/lane_target', self.callback_target, 1)
        self.create_subscription(Float64, '/control/max_vel', self.callback_max_vel, 1)
        self.create_subscription(UInt8, '/detect/lane_state', self.callback_state, 1)
        self.create_subscription(String, '/detect/lane_follow', self.callback_follow, 1)
        self.create_subscription(Bool, '/autorace/run_active', self.callback_run_active, 1)
        self.create_subscription(String, '/control/turn_at_line', self.callback_order, 1)
        self.pub_turn_done = self.create_publisher(Bool, '/control/turn_at_line/done', 1)
        if self.line_turn:
            self.create_subscription(Odometry, '/odom', self.callback_odom, 5)
            for colour in ('yellow', 'white'):
                self.create_subscription(
                    PointCloud2, f'/detect/lane_lines/{colour}',
                    lambda msg, c=colour: self.callback_lines(msg, c), 1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel/lane', 1)

        self.target = None          # (ahead, left) of the axle, metres
        self.target_time = 0.0
        self.last_angular = 0.0
        self.last_curvature = 0.0
        self.lane_state = 0
        self.follow = None
        self.follow_time = 0.0
        self.limit = None           # a mission's speed limit, and when it last came
        self.limit_time = 0.0
        self.recovering_since = None
        self.run_active = False
        self.turning = 0.0          # turn on the spot: +1 left, -1 right, 0 not turning
        self.turn_lost_since = None
        self.pose = None            # (x, y, yaw) from /odom
        self.lines = {'yellow': (None, 0.0), 'white': (None, 0.0)}
        self.lt_state = 'idle'      # idle / advance / spin
        self.lt_since = 0.0
        self.lt_start = None        # pose where the drive up began
        self.lt_advance = 0.0
        self.lt_target_yaw = 0.0
        self.lt_done = -1e9
        self.order = None           # 'left' / 'right' / 'hold' from the mission, and when
        self.order_time = -1e9
        self.lt_ordered = False     # the turn under way was ordered
        self.order_finished = None  # the side of the last ordered turn completed
        self.rng = np.random.default_rng(0)

        self.create_timer(1.0 / rate, self.update)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def callback_target(self, msg):
        self.target = (msg.point.x, msg.point.y)
        self.target_time = self.now()

    def callback_run_active(self, msg):
        self.run_active = msg.data

    def callback_max_vel(self, msg):
        """Take a mission's speed limit; it holds for as long as it keeps coming."""
        self.limit = msg.data
        self.limit_time = self.now()

    def callback_state(self, msg):
        self.lane_state = msg.data

    def callback_follow(self, msg):
        self.follow = msg.data if msg.data in ('left', 'right') else None
        self.follow_time = self.now()

    def update(self):
        now = self.now()
        fresh = self.target is not None and now - self.target_time < self.lane_timeout

        if self.line_turn and self.run_active:
            turn = self.line_turn_step(now)
            if turn is not None:
                self.pub_cmd_vel.publish(turn)
                return

        if self.turn_in_place and self.run_active:
            turn = self.turn_step(now, fresh)
            if turn is not None:
                self.pub_cmd_vel.publish(turn)
                return

        twist = Twist()
        if fresh:
            self.recovering_since = None
            ahead, left = self.target
            # Curvature of the arc from the axle, along the heading, through
            # the target.
            curvature = self.pursuit_gain * 2.0 * left / (ahead * ahead + left * left)
            if self.lane_state != LANE_BOTH:
                step = max(-self.single_line_slew,
                           min(self.single_line_slew, curvature - self.last_curvature))
                curvature = self.last_curvature + step
            self.last_curvature = curvature

            speed = self.max_speed if self.run_active else self.standby_speed
            if self.limit is not None and now - self.limit_time < self.limit_timeout:
                speed = min(speed, self.limit)
            if abs(curvature) * speed > self.cornering_rate:
                speed = max(self.min_speed, self.cornering_rate / abs(curvature))
            angular = speed * curvature
            angular = max(-self.max_angular, min(self.max_angular, angular))
            self.last_angular = angular

            twist.linear.x = speed
            twist.angular.z = angular
        elif not self.run_active:
            # Before the start, with no lane in sight, the robot stays where
            # it was put: a camera that is slow to come up must not send it
            # wandering off the start line.
            self.recovering_since = None
        else:
            if self.recovering_since is None:
                self.recovering_since = now
                self.get_logger().warn('Lane lost, searching.')
            if now - self.recovering_since > self.recovery_timeout:
                # Give up steering blind; the mux watchdog takes it from here.
                self.get_logger().error(
                    'Lane not recovered within %.0f s.' % self.recovery_timeout,
                    throttle_duration_sec=5.0
                )
                return
            twist.linear.x = self.recovery_speed
            if self.follow is not None and now - self.follow_time < self.follow_timeout:
                # Asked to follow one line and it is not in view: it is on
                # its own side of the robot, whichever way the robot was
                # steering before.
                direction = 1.0 if self.follow == 'left' else -1.0
                twist.angular.z = direction * self.recovery_angular
            elif self.last_angular == 0.0:
                # Nothing has been tracked yet, so there is no direction to
                # search in; creeping straight brings the lane into view.
                twist.angular.z = 0.0
            else:
                direction = 1.0 if self.last_angular > 0.0 else -1.0
                twist.angular.z = direction * self.recovery_angular

        self.pub_cmd_vel.publish(twist)

    def callback_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def callback_lines(self, msg, colour):
        pts = np.array([(p[0], p[1]) for p in point_cloud2.read_points(msg, ('x', 'y'), skip_nans=True)],
                       dtype=np.float64).reshape(-1, 2)
        self.lines[colour] = (pts, self.now())

    def callback_order(self, msg):
        self.order = msg.data if msg.data in ('left', 'right', 'hold') else None
        self.order_time = self.now()

    def line_turn_step(self, now):
        """The corner turn: the twist while it is going on, else None."""
        ordered = self.order if now - self.order_time < self.lane_timeout else None
        if self.order_finished is not None:
            if ordered == self.order_finished:
                self.pub_turn_done.publish(Bool(data=True))
            else:
                self.order_finished = None
        if self.pose is None:
            return None
        if self.lt_state == 'idle' and ordered is not None:
            if ordered == 'hold' or ordered == self.order_finished:
                return None          # the mission is steering: no corner turns of our own
            return self.start_ordered_turn(now, ordered)
        if self.lt_state != 'idle' and now - self.lt_since > self.lt_timeout:
            self.get_logger().warn('Corner turn took too long, back to the lane.')
            self.lt_state = 'idle'
            self.lt_done = now
            return None

        if self.lt_state == 'idle':
            if self.follow is not None and now - self.follow_time < self.follow_timeout:
                return None          # a mission picks the branch: leave the corners to it
            if now - self.lt_done < self.lt_cooldown:
                return None
            if self.lane_state not in (LANE_LEFT_ONLY, LANE_RIGHT_ONLY):
                return None
            colour, other = (('yellow', 'white') if self.lane_state == LANE_LEFT_ONLY
                             else ('white', 'yellow'))
            pts, when = self.lines[colour]
            other_pts, other_when = self.lines[other]
            if pts is None or now - when > self.lane_timeout or len(pts) < self.lt_min_points:
                return None
            if other_pts is not None and now - other_when < self.lane_timeout \
                    and len(other_pts) > self.lt_other_max:
                return None
            fit = line_turn_angle(pts, colour, self.lt_half_width)
            if fit is None:
                return None
            turn, advance, length, straightness = fit
            if (abs(turn) < self.lt_enter or abs(turn) > self.lt_max or length < self.lt_min_length
                    or straightness > self.lt_straightness):
                return None
            turn = math.copysign(min(abs(turn), self.lt_limit), turn)
            self.lt_advance = min(advance, self.lt_max_advance)
            self.lt_target_yaw = wrap(self.pose[2] + turn)
            self.lt_start = self.pose
            self.lt_state = 'advance'
            self.lt_since = now
            self.lt_ordered = False
            self.get_logger().info(
                'Corner: only the %s line, %.0f deg across; %.2f m on, then turn %s %.0f deg.'
                % (colour, math.degrees(turn), self.lt_advance,
                   'left' if turn > 0 else 'right', abs(math.degrees(turn))))

        twist = Twist()
        if self.lt_state == 'advance':
            done = math.hypot(self.pose[0] - self.lt_start[0], self.pose[1] - self.lt_start[1])
            if done < self.lt_advance:
                twist.linear.x = self.lt_speed
                return twist
            self.lt_state = 'spin'

        error = wrap(self.lt_target_yaw - self.pose[2])
        if abs(error) < 0.03:
            self.get_logger().info('Corner turned, following the lane again.')
            self.lt_state = 'idle'
            self.lt_done = now
            self.last_angular = 0.0
            if self.lt_ordered:
                self.order_finished = 'left' if self.lt_target_side > 0 else 'right'
                self.pub_turn_done.publish(Bool(data=True))
            return Twist()
        twist.angular.z = math.copysign(max(0.15, min(self.lt_rate, 2.0 * abs(error))), error)
        self.last_angular = twist.angular.z
        return twist

    def start_ordered_turn(self, now, side):
        """Look for the line across the way; when it is there, start the turn."""
        clouds = [pts for pts, when in self.lines.values()
                  if pts is not None and len(pts) and now - when < self.lane_timeout]
        if not clouds:
            return None
        found = crossing_line(np.vstack(clouds), side, self.lt_half_width,
                              self.ot_min, self.ot_max, self.rng)
        if found is None or found[2] < self.ot_min_points:
            return None              # not in sight yet: lane following brings it closer
        turn, advance, count = found
        turn = math.copysign(min(abs(turn), self.lt_limit), turn)
        self.lt_advance = min(advance, self.lt_max_advance)
        self.lt_target_yaw = wrap(self.pose[2] + turn)
        self.lt_target_side = 1.0 if turn > 0 else -1.0
        self.lt_start = self.pose
        self.lt_state = 'advance'
        self.lt_since = now
        self.lt_ordered = True
        self.get_logger().info(
            'Ordered %s turn: line across at %.0f deg (%d points); %.2f m on, then turn %.0f deg.'
            % (side, math.degrees(turn), count, self.lt_advance, math.degrees(turn)))
        return self.line_turn_step_continue(now)

    def line_turn_step_continue(self, now):
        twist = Twist()
        twist.linear.x = self.lt_speed if self.lt_advance > 0 else 0.0
        return twist

    def turn_step(self, now, fresh):
        """Turning on the spot at a bend: the twist, or None to drive as usual.

        The turn ends on what the camera sees, not on odometry: when the lane
        centre is straight ahead again. If the lane goes out of view while
        turning, the turn goes on the same way for up to recovery_timeout.
        """
        if fresh:
            self.turn_lost_since = None
            ahead, left = self.target
            angle = math.atan2(left, ahead)
            if self.turning:
                if abs(angle) < self.turn_exit_angle:
                    self.turning = 0.0
                    self.get_logger().info('Facing the lane again, straight on.')
                    return None
            elif abs(angle) > self.turn_enter_angle:
                self.turning = math.copysign(1.0, angle)
                self.get_logger().info('Bend: turning on the spot %s (%.0f deg off).' % (
                    'left' if angle > 0 else 'right', math.degrees(angle)))
            else:
                return None
        elif not self.turning:
            return None
        else:
            if self.turn_lost_since is None:
                self.turn_lost_since = now
            if now - self.turn_lost_since > self.recovery_timeout:
                self.turning = 0.0
                self.turn_lost_since = None
                return None
        twist = Twist()
        twist.angular.z = self.turning * self.turn_rate
        self.last_angular = twist.angular.z
        return twist

    def shut_down(self):
        self.pub_cmd_vel.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = LaneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shut_down()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
