#!/usr/bin/env python3
"""Generate the Gazebo Harmonic AutoRace world.

The Classic world is only used as a source of coordinates: model placements are
re-emitted as includes of the ported models, and the inline walls/obstacles are
rebuilt as clean static box models.
"""
import xml.etree.ElementTree as ET
from pathlib import Path

CLASSIC = (Path.home() / 'Desktop/autorace_ws/src/turtlebot3_simulations'
           / 'turtlebot3_gazebo/worlds/turtlebot3_autorace_2020.world')
OUT = Path.home() / 'Desktop/autorace_ws/src/autorace_sim/worlds/autorace_2023.sdf'

# Classic include uri -> ported model name.
PORTED = {
    'course': 'autorace_course',
    'checker': 'autorace_checker',
    'traffic_construction': 'autorace_sign_construction',
    'traffic_intersection': 'autorace_sign_intersection',
    'traffic_left': 'autorace_sign_left',
    'traffic_right': 'autorace_sign_right',
    'traffic_noentry': 'autorace_sign_noentry',
    'traffic_parking': 'autorace_sign_parking',
    'traffic_pl_left': 'autorace_sign_pl_left',
    'traffic_stop': 'autorace_sign_stop',
    'traffic_tunnel': 'autorace_sign_tunnel',
    'traffic_light': 'autorace_traffic_light',
    'traffic_bar': 'autorace_level_bar',
}
# Rebuilt as clean models from the inline definitions.
INLINE = ('tunnel_wall', 'tunnel_obstacle', 'barrier_1', 'barrier_2', 'barrier_3')


def text_pose(element):
    pose = element.find('pose')
    return (pose.text or '').strip() if pose is not None else '0 0 0 0 0 0'


root = ET.parse(CLASSIC).getroot()
world = root.find('world')

includes = []
for include in world.findall('include'):
    uri = include.find('uri').text.strip()
    name = uri.rsplit('/', 1)[-1]
    if name not in PORTED:
        print('skipping unported include:', uri)
        continue
    pose = text_pose(include)
    if PORTED[name] == 'autorace_level_bar':
        # The rebuilt bar stands on the ground instead of being centred at the
        # Classic model's 0.125 m.
        parts = pose.split()
        parts[2] = '0'
        pose = ' '.join(parts)
    includes.append((PORTED[name], name, pose))

blocks = []
for model in world.findall('model'):
    name = model.get('name')
    if name not in INLINE:
        continue
    lines = [f'    <model name="{name}">', '      <static>true</static>',
             f'      <pose>{text_pose(model)}</pose>']
    for link in model.findall('link'):
        link_name = link.get('name')
        collision = link.find('collision')
        visual = link.find('visual')
        box = link.find('.//box/size')
        cylinder = link.find('.//cylinder')
        if box is not None:
            geometry = f'<box><size>{box.text.strip()}</size></box>'
        elif cylinder is not None:
            radius = cylinder.find('radius').text.strip()
            length = cylinder.find('length').text.strip()
            geometry = f'<cylinder><radius>{radius}</radius><length>{length}</length></cylinder>'
        else:
            print(f'  {name}/{link_name}: unsupported geometry, skipped')
            continue
        inner_pose = text_pose(visual if visual is not None else collision)
        lines += [
            f'      <link name="{link_name}">',
            f'        <pose>{text_pose(link)}</pose>',
            f'        <collision name="{link_name}_collision">',
            f'          <pose>{inner_pose}</pose>',
            f'          <geometry>{geometry}</geometry>',
            '        </collision>',
            f'        <visual name="{link_name}_visual">',
            f'          <pose>{inner_pose}</pose>',
            f'          <geometry>{geometry}</geometry>',
            '          <material>',
            '            <ambient>0.25 0.25 0.25 1</ambient>',
            '            <diffuse>0.55 0.55 0.55 1</diffuse>',
            '            <specular>0.05 0.05 0.05 1</specular>',
            '          </material>',
            '        </visual>',
            '      </link>',
        ]
    lines.append('    </model>')
    blocks.append('\n'.join(lines))
    print(f'rebuilt inline model: {name} ({len(model.findall("link"))} links)')

include_xml = '\n'.join(
    f'    <!-- {original} -->\n'
    f'    <include>\n'
    f'      <uri>model://{ported}</uri>\n'
    f'      <name>{ported}</name>\n'
    f'      <pose>{pose}</pose>\n'
    f'    </include>'
    for ported, original, pose in includes
)

# Lit lamps start parked below the course; sim_traffic_light moves the active
# one into its socket.
lamps = '\n'.join(
    f'    <include>\n'
    f'      <uri>model://autorace_lamp_{color}</uri>\n'
    f'      <name>autorace_lamp_{color}</name>\n'
    f'      <pose>0 0 -5 0 0 0</pose>\n'
    f'    </include>'
    for color in ('red', 'yellow', 'green')
)

OUT.parent.mkdir(parents=True, exist_ok=True)
OUT.write_text(f'''<?xml version="1.0" ?>
<!-- AutoRace 2023 course for Gazebo Harmonic (gz-sim 8).
     Generated from turtlebot3_autorace_2020.world; do not edit by hand,
     see autorace_sim/scripts/gen_world.py. -->
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

{include_xml}

{lamps}

{chr(10).join(blocks)}

  </world>
</sdf>
''')
print('\nwrote', OUT)
