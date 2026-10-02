#!/usr/bin/env python3
"""Convert the Gazebo Classic autorace models to Gazebo Harmonic (gz-sim 8).

Two things change: Ogre material scripts become SDF PBR materials with an
albedo map, and the model names are flattened to `autorace_*` so they cannot
clash with the stock turtlebot3 models.
"""
import re
import shutil
from pathlib import Path

SRC = Path('/tmp/claude-1000/-home-sxx-Desktop-colcon-ws--colcon-ws-colcon-ws'
           '/db65840d-6692-4797-a6b6-6ba086580943/scratchpad/humble_models'
           '/turtlebot3_gazebo/models/turtlebot3_autorace_2020')
DST = Path.home() / 'Desktop/autorace_ws/src/autorace_sim/models'

# old model dir -> new flattened model name. traffic_bar/traffic_light are
# rebuilt by hand because they relied on Classic C++ plugins.
RENAME = {
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
}

MATERIAL_RE = re.compile(r'<material>\s*<script>.*?</script>\s*</material>', re.S)
TEXTURE_RE = re.compile(r'texture\s+(\S+\.png)')


def texture_for(model_dir: Path) -> str:
    scripts = sorted(model_dir.glob('materials/scripts/*.material'))
    textures = sorted(model_dir.glob('materials/textures/*.png'))
    if scripts:
        found = TEXTURE_RE.search(scripts[0].read_text())
        if found:
            return found.group(1)
    return textures[0].name if textures else ''


def pbr_material(new_name: str, texture: str, indent: str = '        ') -> str:
    return (
        f'{indent}<material>\n'
        f'{indent}  <diffuse>1 1 1 1</diffuse>\n'
        f'{indent}  <specular>0.1 0.1 0.1 1</specular>\n'
        f'{indent}  <pbr>\n'
        f'{indent}    <metal>\n'
        f'{indent}      <albedo_map>model://{new_name}/materials/textures/{texture}</albedo_map>\n'
        f'{indent}      <metalness>0.0</metalness>\n'
        f'{indent}      <roughness>0.9</roughness>\n'
        f'{indent}    </metal>\n'
        f'{indent}  </pbr>\n'
        f'{indent}</material>'
    )


def model_config(name: str) -> str:
    return (
        '<?xml version="1.0"?>\n'
        '<model>\n'
        f'  <name>{name}</name>\n'
        '  <version>1.0</version>\n'
        '  <sdf version="1.10">model.sdf</sdf>\n'
        '  <description>AutoRace 2023 course asset, ported to Gazebo Harmonic.</description>\n'
        '</model>\n'
    )


def convert(old: str, new: str) -> None:
    src_dir = SRC / old
    dst_dir = DST / new
    if dst_dir.exists():
        shutil.rmtree(dst_dir)
    (dst_dir / 'materials/textures').mkdir(parents=True)
    for png in (src_dir / 'materials/textures').glob('*.png'):
        shutil.copy(png, dst_dir / 'materials/textures' / png.name)

    texture = texture_for(src_dir)
    sdf = (src_dir / 'model.sdf').read_text()
    sdf = sdf.replace('<sdf version="1.6">', '<sdf version="1.10">')
    sdf = sdf.replace("<sdf version='1.6'>", '<sdf version="1.10">')
    sdf = re.sub(r'<model name=[\'"][^\'"]+[\'"]>', f'<model name="{new}">', sdf, count=1)
    sdf = MATERIAL_RE.sub(pbr_material(new, texture), sdf)
    # Classic plugins do not exist in gz-sim.
    sdf = re.sub(r'<plugin[^>]*>\s*</plugin>', '', sdf)
    # Some signs carry their position in the 2020 course as a link offset;
    # the world file is what places them.
    sdf = re.sub(r'(<link name="box">\s*<pose>)[^<]*(</pose>)', r'\g<1>0 0 0 0 0 0\2', sdf)
    if '<static>' not in sdf:
        sdf = sdf.replace('</model>', '    <static>true</static>\n  </model>')

    if old == 'course':
        # A textured <plane> has no usable UVs in gz-sim; a thin box does.
        sdf = re.sub(
            r'<plane>\s*<normal>[^<]*</normal>\s*<size>4 4</size>\s*</plane>',
            '<box><size>4 4 0.01</size></box>',
            sdf,
        )
        sdf = sdf.replace('<visual name="course_visual">',
                          '<visual name="course_visual">\n        <pose>0 0 0.005 0 0 0</pose>')
        sdf = sdf.replace('<collision name="course_collision">',
                          '<collision name="course_collision">\n        '
                          '<pose>0 0 0.005 0 0 0</pose>')

    (dst_dir / 'model.sdf').write_text(sdf)
    (dst_dir / 'model.config').write_text(model_config(new))
    print(f'{old:24s} -> {new:28s} texture={texture}')


DST.mkdir(parents=True, exist_ok=True)
for old_name, new_name in RENAME.items():
    convert(old_name, new_name)
