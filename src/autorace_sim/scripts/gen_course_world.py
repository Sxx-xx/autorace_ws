#!/usr/bin/env python3
"""Generate the Gazebo Harmonic world for the competition map.

Every placement is measured on map.png (the "TB3 Auto Race Map" drawing at
the workspace root) and converted to world coordinates here, so the numbers
below are map pixels. Run gen_course_texture.py first for the floor.

Writes:
  models/autorace_course_map/model.sdf   the floor
  worlds/autorace_course.sdf             the world
  params/course.yaml                     start pose, sign and bay positions
  params/course_sim_nodes.yaml           sim node parameters
"""
import math
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]

# The board inside map.png (inclusive). The drawing is not square, so each
# axis has its own scale.
BOARD_X = (319, 643)
BOARD_Y = (53, 361)
BOARD_M = 4.0


def world(px, py):
    """Map pixel -> world (X, Y). Image up = +X, image left = +Y."""
    sx = BOARD_M / (BOARD_X[1] - BOARD_X[0] + 1)
    sy = BOARD_M / (BOARD_Y[1] - BOARD_Y[0] + 1)
    return (round(BOARD_M / 2 - (py - BOARD_Y[0]) * sy, 3),
            round(BOARD_M / 2 - (px - BOARD_X[0]) * sx, 3))


def pose(x, y, z=0.0, yaw=0.0):
    return f'{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}'


# Start: in the lane of the top road, on the stop line (see below),
# (white y 61, yellow y 81), heading image-left (+Y) towards the light.
# On the stop line, as the rules have it: the line spans y 0.868-0.918 in the
# lane of the top road (centre x 1.752), and the body reaches 0.038 ahead of
# the axle, so the axle 2 cm short of the line's centre puts the front on it.
START_X, START_Y = 1.752, 0.870
START_YAW = math.pi / 2

# Traffic light: past the start line, outside the road on the robot's right.
# Its lamps face local -y, so yaw 0 faces them towards -Y, at the robot.
LIGHT_X, LIGHT_Y = world(370, 56)
LIGHT_Z = 0.13
LIGHT_YAW = 0.0

# Tunnel: the dark square, x 482-643 / y 53-207 in the map, i.e. the
# quadrant X > 0, Y < 0. Walls are inset so the inside is about 1.8 m, as in
# the rules. Openings where the road meets the square: the top road enters on
# the Y = 0 side (white y 61 to yellow y 83), the right road on the X = 0 side
# (yellow x 613 to white x 634).
TUNNEL_INSET = 0.07
TUNNEL_HEIGHT = 0.24
WALL = 0.03
TUNNEL_LEFT_OPENING = (world(0, 84)[0] - 0.02, world(0, 60)[0] + 0.02)     # X range
TUNNEL_BOTTOM_OPENING = (world(636, 0)[1] - 0.02, world(611, 0)[1] + 0.02)  # Y range
# Random obstacles inside, per the rules; the shapes are ours.
TUNNEL_OBSTACLES = [
    ('box', (1.00, -0.70), (0.20, 0.15, 0.20)),
    ('cylinder', (0.60, -1.30), (0.08, 0.20)),
    ('box', (1.40, -1.20), (0.12, 0.30, 0.20)),
]

# Construction: the three cyan blocks, 19-20 x 7 px = 0.24 x 0.09 m,
# alternating across the double-width road on the left of the board. On the
# course (wkrsus.png) they are tall posts in black and white bands, well
# above the Burger's lidar at about 0.18 m.
CONSTRUCTION_HEIGHT = 0.40
CONSTRUCTION_BANDS = 4
CONSTRUCTION = [
    (world(359.9, 249), (0.09, 0.234)),
    (world(339.6, 285), (0.09, 0.246)),
    (world(359.9, 321), (0.09, 0.234)),
]

# Parking: two bays either side of the entrance, x 412-429 and 453-469,
# y 248-283. The house robot (a TurtleBot3 Burger sized box) takes one.
PARKING_BAYS = {'left': world(420.5, 265.5), 'right': world(461, 265.5)}
HOUSE_BAY = 'left'
HOUSE_SIZE = (0.14, 0.18, 0.19)

# Level crossing: hinge above the road at x 533, y 289; the bar hangs down
# across the road (image down = world -X). The robot arrives heading +Y.
HINGE_X, HINGE_Y = world(533, 289)
BAR_YAW = math.pi                       # model +x (the bar) -> world -X
BAR_ORIGIN = (HINGE_X - 0.16, HINGE_Y)  # hinge is 0.16 m behind the origin
LANE_AT_BAR_X = world(0, 304)[0]
SENSOR2 = (LANE_AT_BAR_X, round(HINGE_Y - 0.06, 3))  # 6 cm before the bar
SENSOR1 = (LANE_AT_BAR_X, round(HINGE_Y - 0.45, 3))
# Sensor 1 may stand anywhere on the approach: between this near and this far
# from the bar, along the straight the robot comes up.
SENSOR1_DISTANCE = (0.30, 0.85)
# The sensors are shown as discs on the floor. Only the Gazebo window draws
# them: the robot's cameras leave out visuals with this flag (see the
# visibility_mask of the cameras in models/autorace_burger).
MARKER_FLAG = 2

# Signs face local +-y. yaw 0 faces a robot travelling along Y, pi/2 one
# travelling along X.
SIGNS = [
    ('autorace_sign_intersection', world(324.8, 129.7), math.pi / 2),
    # Direction for the loop, on its island facing the robot coming in.
    ('autorace_sign_left', world(400, 146), 0.0),
    ('autorace_sign_construction', world(371.6, 208.3), 0.0),
    ('autorace_sign_parking', world(414, 359), 0.0),
    # Drawn over the tunnel edge in the map; moved just outside the wall.
    ('autorace_sign_tunnel', (-0.05, -1.50), math.pi / 2),
]
# The other direction sign waits below the floor; see scripts/intersection.sh.
PARKED_SIGNS = ['autorace_sign_right']


def box_model(name, xyz, size, colour, yaw=0.0):
    sx, sy, sz = size
    x, y = xyz
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{pose(x, y, sz / 2, yaw)}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material>
            <ambient>{colour} {colour} {colour} 1</ambient>
            <diffuse>{colour} {colour} {colour} 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>'''


def post_model(name, xy, size, height, bands):
    """A static post in black and white bands, one collision box."""
    sx, sy = size
    x, y = xy
    band = height / bands
    visuals = ''
    for i in range(bands):
        colour = 0.9 if i % 2 == 0 else 0.05
        visuals += f'''
        <visual name="band_{i}">
          <pose>0 0 {band * (i + 0.5) - height / 2:.3f} 0 0 0</pose>
          <geometry><box><size>{sx} {sy} {band:.3f}</size></box></geometry>
          <material>
            <ambient>{colour} {colour} {colour} 1</ambient>
            <diffuse>{colour} {colour} {colour} 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>'''
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{pose(x, y, height / 2)}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {height}</size></box></geometry>
        </collision>{visuals}
      </link>
    </model>'''


def cylinder_model(name, xy, radius, length, colour):
    x, y = xy
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{pose(x, y, length / 2)}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
          <material>
            <ambient>{colour} {colour} {colour} 1</ambient>
            <diffuse>{colour} {colour} {colour} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>'''


def marker_model(name, xy, radius, rgb):
    """A disc on the floor that marks a spot for whoever is watching."""
    x, y = xy
    r, g, b = rgb
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{pose(x, y, 0.011)}</pose>
      <link name="link">
        <visual name="visual">
          <visibility_flags>{MARKER_FLAG}</visibility_flags>
          <cast_shadows>false</cast_shadows>
          <geometry><cylinder><radius>{radius}</radius><length>0.002</length></cylinder></geometry>
          <material>
            <ambient>{r} {g} {b} 1</ambient>
            <diffuse>{r} {g} {b} 1</diffuse>
            <emissive>{r} {g} {b} 1</emissive>
          </material>
        </visual>
      </link>
    </model>'''


def include(model, name, pose_text):
    return f'''    <include>
      <uri>model://{model}</uri>
      <name>{name}</name>
      <pose>{pose_text}</pose>
    </include>'''


def tunnel():
    """Walls with the two openings, a roof, and the obstacles inside."""
    lo, hi = TUNNEL_INSET, BOARD_M / 2 - TUNNEL_INSET
    black = 0.03
    walls = []

    def wall(name, x0, y0, x1, y1):
        # Axis-aligned segment from (x0, y0) to (x1, y1).
        length = math.hypot(x1 - x0, y1 - y0)
        if length < 1e-3:
            return
        along_x = abs(x1 - x0) > abs(y1 - y0)
        size = (length, WALL, TUNNEL_HEIGHT) if along_x else (WALL, length, TUNNEL_HEIGHT)
        walls.append(box_model(name, ((x0 + x1) / 2, (y0 + y1) / 2), size, black))

    # Side facing the rest of the board at Y = -lo, open for the top road.
    open_lo, open_hi = TUNNEL_LEFT_OPENING
    wall('tunnel_wall_y0_a', lo, -lo, open_lo, -lo)
    wall('tunnel_wall_y0_b', open_hi, -lo, hi, -lo)
    # Side at X = lo, open for the right road.
    open_lo, open_hi = TUNNEL_BOTTOM_OPENING
    wall('tunnel_wall_x0_a', lo, -lo, lo, open_hi)
    wall('tunnel_wall_x0_b', lo, open_lo, lo, -hi)
    wall('tunnel_wall_x1', hi, -lo, hi, -hi)
    wall('tunnel_wall_y1', lo, -hi, hi, -hi)

    centre = (BOARD_M / 4, -BOARD_M / 4)
    span = hi - lo + WALL
    roof = f'''    <model name="tunnel_roof">
      <static>true</static>
      <pose>{pose(centre[0], centre[1], TUNNEL_HEIGHT + 0.005)}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{span:.3f} {span:.3f} 0.01</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{span:.3f} {span:.3f} 0.01</size></box></geometry>
          <material>
            <ambient>0.03 0.03 0.03 1</ambient>
            <diffuse>0.03 0.03 0.03 1</diffuse>
          </material>
        </visual>
      </link>
    </model>'''

    obstacles = []
    for i, (shape, xy, size) in enumerate(TUNNEL_OBSTACLES):
        name = f'tunnel_obstacle_{i + 1}'
        if shape == 'box':
            obstacles.append(box_model(name, xy, size, 0.5))
        else:
            obstacles.append(cylinder_model(name, xy, size[0], size[1], 0.5))
    return '\n'.join(walls + [roof] + obstacles)


def course_model():
    path = PKG / 'models/autorace_course_map/model.sdf'
    path.write_text('''<?xml version="1.0" ?>
<!-- Generated by autorace_sim/scripts/gen_course_world.py. -->
<sdf version="1.10">
  <model name="autorace_course_map">
    <static>true</static>
    <link name="course_link">
      <collision name="course_collision">
        <pose>0 0 0.005 0 0 0</pose>
        <geometry><box><size>4 4 0.01</size></box></geometry>
        <surface>
          <friction><ode><mu>100</mu><mu2>50</mu2></ode></friction>
        </surface>
      </collision>
      <visual name="course_visual">
        <pose>0 0 0.005 0 0 0</pose>
        <cast_shadows>false</cast_shadows>
        <geometry><box><size>4 4 0.01</size></box></geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
          <pbr>
            <metal>
              <albedo_map>model://autorace_course_map/materials/textures/course_map.png</albedo_map>
              <metalness>0.0</metalness>
              <roughness>0.9</roughness>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
''')
    (PKG / 'models/autorace_course_map/model.config').write_text('''<?xml version="1.0" ?>
<model>
  <name>autorace_course_map</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <description>AutoRace board from the competition map drawing.</description>
</model>
''')


def world_file():
    parts = [include('autorace_course_map', 'autorace_course_map', pose(0, 0))]
    parts.append(include('autorace_traffic_light', 'autorace_traffic_light',
                         pose(LIGHT_X, LIGHT_Y, LIGHT_Z, LIGHT_YAW)))
    for colour in ('red', 'yellow', 'green'):
        # Parked below the floor; sim_traffic_light moves the lit one in.
        parts.append(include(f'autorace_lamp_{colour}', f'autorace_lamp_{colour}',
                             pose(0, 0, -5)))
    for model, (x, y), yaw in SIGNS:
        parts.append(include(model, model, pose(x, y, 0.125, yaw)))
    for i, model in enumerate(PARKED_SIGNS):
        parts.append(include(model, model, pose(i, 0, -5)))
    parts.append(include('autorace_level_bar', 'autorace_level_bar',
                         pose(*BAR_ORIGIN, 0.0, BAR_YAW)))
    for i, (xy, size) in enumerate(CONSTRUCTION):
        parts.append(post_model(f'construction_{i + 1}', xy, size,
                                CONSTRUCTION_HEIGHT, CONSTRUCTION_BANDS))
    parts.append(box_model('house_robot', PARKING_BAYS[HOUSE_BAY], HOUSE_SIZE, 0.2))
    # Sensor 1 green, sensor 2 red; sim_level_crossing moves the first one.
    parts.append(marker_model('crossing_sensor_1', SENSOR1, 0.04, (0.0, 0.9, 0.2)))
    parts.append(marker_model('crossing_sensor_2', SENSOR2, 0.03, (1.0, 0.1, 0.1)))
    parts.append(tunnel())
    body = '\n\n'.join(parts)

    path = PKG / 'worlds/autorace_course.sdf'
    path.write_text(f'''<?xml version="1.0" ?>
<!-- AutoRace competition map for Gazebo Harmonic (gz-sim 8).
     Generated by autorace_sim/scripts/gen_course_world.py from map.png;
     do not edit by hand. -->
<sdf version="1.10">
  <world name="autorace">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-contact-system"
            name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system"
            name="gz::sim::systems::Imu"/>

    <scene>
      <ambient>0.85 0.85 0.85 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.35 0.35 0.35 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

{body}

  </world>
</sdf>
''')
    print('wrote', path)


def params_files():
    path = PKG / 'params/course.yaml'
    path.write_text(f'''# Generated by autorace_sim/scripts/gen_course_world.py; do not edit by hand.
# Where things are in worlds/autorace_course.sdf, for launch files and scripts.
course:
  start: {{x: {START_X}, y: {START_Y}, yaw: {START_YAW:.4f}}}
  intersection_sign: {{x: {SIGNS[1][1][0]}, y: {SIGNS[1][1][1]}, yaw: {SIGNS[1][2]}}}
  parking_bays:
    left: {{x: {PARKING_BAYS['left'][0]}, y: {PARKING_BAYS['left'][1]}}}
    right: {{x: {PARKING_BAYS['right'][0]}, y: {PARKING_BAYS['right'][1]}}}
''')
    print('wrote', path)

    path = PKG / 'params/course_sim_nodes.yaml'
    path.write_text(f'''# Generated by autorace_sim/scripts/gen_course_world.py; do not edit by hand.
# The sim nodes' view of worlds/autorace_course.sdf.
sim_traffic_light:
  ros__parameters:
    housing_x: {LIGHT_X}
    housing_y: {LIGHT_Y}
    housing_z: {LIGHT_Z}
    housing_yaw: {LIGHT_YAW}

sim_level_crossing:
  ros__parameters:
    start_x: {START_X}
    start_y: {START_Y}
    start_yaw: {START_YAW:.4f}
    sensor1_x: {SENSOR1[0]}
    sensor1_y: {SENSOR1[1]}
    sensor2_x: {SENSOR2[0]}
    sensor2_y: {SENSOR2[1]}
    bar_x: {LANE_AT_BAR_X}
    bar_y: {HINGE_Y}
    sensor1_distance_min: {SENSOR1_DISTANCE[0]}
    sensor1_distance_max: {SENSOR1_DISTANCE[1]}
''')
    print('wrote', path)


if __name__ == '__main__':
    course_model()
    world_file()
    params_files()
