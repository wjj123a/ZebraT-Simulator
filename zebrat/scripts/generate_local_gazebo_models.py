#!/usr/bin/env python3

import colorsys
import hashlib
import math
import re
from pathlib import Path


WORLD_DIR = Path(__file__).resolve().parents[1] / "worlds"
MODEL_DIR = Path(__file__).resolve().parents[1] / "gazebo_models"

PNG_BYTES = bytes.fromhex(
    "89504e470d0a1a0a0000000d4948445200000001000000010802000000907753de"
    "0000000c4944415408d763606060000000040001f61738550000000049454e44ae426082"
)


def rgba(r, g, b, a=1.0):
    return (float(r), float(g), float(b), float(a))


def clamp(value, low=0.0, high=1.0):
    return max(low, min(high, value))


def tint(color, amount):
    r, g, b, a = color
    return (
        clamp(r + amount),
        clamp(g + amount),
        clamp(b + amount),
        a,
    )


PALETTE = {
    "apartment": rgba(0.80, 0.76, 0.68),
    "ambulance": rgba(0.96, 0.96, 0.96),
    "bus": rgba(0.98, 0.74, 0.18),
    "cessna": rgba(0.97, 0.97, 0.99),
    "construction_barrel": rgba(0.95, 0.47, 0.12),
    "construction_cone": rgba(0.98, 0.49, 0.10),
    "demo_joint_damping": rgba(0.22, 0.48, 0.82),
    "drc_practice_blue_cylinder": rgba(0.16, 0.46, 0.92),
    "drc_practice_orange_jersey_barrier": rgba(0.96, 0.48, 0.14),
    "drc_practice_valve_wall": rgba(0.86, 0.86, 0.84),
    "drc_practice_white_jersey_barrier": rgba(0.93, 0.93, 0.93),
    "drc_practice_wheel_valve": rgba(0.78, 0.14, 0.12),
    "dumpster": rgba(0.14, 0.46, 0.22),
    "fast_food": rgba(0.80, 0.16, 0.14),
    "fire_station": rgba(0.74, 0.18, 0.14),
    "gas_station": rgba(0.88, 0.22, 0.16),
    "ground_plane": rgba(0.48, 0.48, 0.48),
    "house_3": rgba(0.86, 0.78, 0.64),
    "law_office": rgba(0.62, 0.34, 0.22),
    "mud_box": rgba(0.35, 0.22, 0.12),
    "person_walking": rgba(0.20, 0.26, 0.66),
    "playground": rgba(0.18, 0.66, 0.82),
    "polaris_ranger_ev": rgba(0.22, 0.42, 0.22),
    "racecar_description": rgba(0.86, 0.86, 0.86),
    "robocup09_spl_field": rgba(0.16, 0.54, 0.18),
    "salon": rgba(0.24, 0.58, 0.64),
    "smartcar_plane": rgba(0.16, 0.16, 0.18),
    "suv": rgba(0.14, 0.28, 0.52),
    "sun": rgba(0.96, 0.92, 0.60),
    "table_marble": rgba(0.93, 0.92, 0.89),
    "thrift_shop": rgba(0.72, 0.48, 0.22),
    "tower_crane": rgba(0.95, 0.82, 0.14),
    "water_tower": rgba(0.62, 0.76, 0.92),
}

MATERIAL_OVERRIDES = {
    "brick_box_3x1x3": {
        "BrickBox/Diffuse": rgba(0.72, 0.34, 0.24),
    },
    "demo_joint_types": {
        "demo_joint_types/child": rgba(0.18, 0.46, 0.90),
        "demo_joint_types/legend": rgba(0.94, 0.94, 0.94),
        "demo_joint_types/parent": rgba(0.96, 0.64, 0.14),
        "demo_joint_types/reference": rgba(0.90, 0.24, 0.16),
    },
    "dumpster": {
        "Dumpster/Diffuse": rgba(0.16, 0.46, 0.22),
    },
    "fast_food": {
        "FastFood/Diffuse": rgba(0.82, 0.18, 0.14),
    },
    "gas_station": {
        "GasStation/Diffuse": rgba(0.90, 0.20, 0.16),
    },
    "house_3": {
        "House_3/Diffuse": rgba(0.85, 0.78, 0.66),
    },
    "mud_box": {
        "vrc/mud": rgba(0.34, 0.22, 0.12),
    },
    "polaris_ranger_ev": {
        "FNR_switch_F": rgba(0.14, 0.56, 0.14),
        "FNR_switch_R": rgba(0.74, 0.16, 0.14),
        "Polaris/Diffuse": rgba(0.22, 0.42, 0.22),
    },
    "robocup09_spl_field": {
        "RoboCup/Net": rgba(0.95, 0.95, 0.95),
    },
    "smartcar_plane": {
        "SmartcarPlane/Image": rgba(0.16, 0.16, 0.18),
    },
    "table_marble": {
        "Table/Marble_Lightmap": rgba(0.94, 0.93, 0.90),
    },
}


def hashed_color(key):
    digest = hashlib.sha1(key.encode("utf-8")).digest()
    hue = digest[0] / 255.0
    saturation = 0.45 + (digest[1] / 255.0) * 0.20
    value = 0.68 + (digest[2] / 255.0) * 0.18
    r, g, b = colorsys.hsv_to_rgb(hue, saturation, value)
    return rgba(r, g, b)


def base_color_for_key(key):
    parts = key.split("/")
    for part in reversed(parts):
        if part in PALETTE:
            return PALETTE[part]
    return hashed_color(key)


def write_text(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def write_binary(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content)


def material_config_block(name):
    return f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author><name>Codex</name></author>
  <description>Local offline placeholder for {name}</description>
</model>
"""


def static_box_model(name, size, color):
    sx, sy, sz = size
    r, g, b, a = color
    z = sz / 2.0
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <pose>0 0 {z:.4f} 0 0 0</pose>
      <collision name="collision">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} {a}</ambient>
          <diffuse>{r} {g} {b} {a}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def parking_model():
    return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="parking_1">
    <static>true</static>
    <link name="surface">
      <collision name="collision">
        <geometry><box><size>6 4 0.05</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>6 4 0.05</size></box></geometry>
        <material>
          <ambient>0.15 0.15 0.15 1</ambient>
          <diffuse>0.15 0.15 0.15 1</diffuse>
        </material>
      </visual>
    </link>
    <link name="divider_1">
      <pose>-1.5 -1.0 0.051 0 0 0</pose>
      <visual name="visual">
        <geometry><box><size>0.08 2.8 0.01</size></box></geometry>
        <material><ambient>1 1 0.8 1</ambient><diffuse>1 1 0.8 1</diffuse></material>
      </visual>
    </link>
    <link name="divider_2">
      <pose>0 -1.0 0.051 0 0 0</pose>
      <visual name="visual">
        <geometry><box><size>0.08 2.8 0.01</size></box></geometry>
        <material><ambient>1 1 0.8 1</ambient><diffuse>1 1 0.8 1</diffuse></material>
      </visual>
    </link>
    <link name="divider_3">
      <pose>1.5 -1.0 0.051 0 0 0</pose>
      <visual name="visual">
        <geometry><box><size>0.08 2.8 0.01</size></box></geometry>
        <material><ambient>1 1 0.8 1</ambient><diffuse>1 1 0.8 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""


def tunnel_model():
    return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="MIT_Tunnel">
    <static>true</static>
    <link name="left_wall">
      <pose>0 -1.4 1 0 0 0</pose>
      <collision name="collision"><geometry><box><size>14 0.25 2</size></box></geometry></collision>
      <visual name="visual"><geometry><box><size>14 0.25 2</size></box></geometry></visual>
    </link>
    <link name="right_wall">
      <pose>0 1.4 1 0 0 0</pose>
      <collision name="collision"><geometry><box><size>14 0.25 2</size></box></geometry></collision>
      <visual name="visual"><geometry><box><size>14 0.25 2</size></box></geometry></visual>
    </link>
    <link name="roof">
      <pose>0 0 2.05 0 0 0</pose>
      <collision name="collision"><geometry><box><size>14 3.0 0.1</size></box></geometry></collision>
      <visual name="visual"><geometry><box><size>14 3.0 0.1</size></box></geometry></visual>
    </link>
  </model>
</sdf>
"""


def walker_course_model():
    return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="walker_racecourse">
    <static>true</static>
    <link name="wall_a">
      <pose>0 -1.8 0.5 0 0 0</pose>
      <collision name="collision"><geometry><box><size>10 0.2 1</size></box></geometry></collision>
      <visual name="visual"><geometry><box><size>10 0.2 1</size></box></geometry></visual>
    </link>
    <link name="wall_b">
      <pose>0 1.8 0.5 0 0 0</pose>
      <collision name="collision"><geometry><box><size>10 0.2 1</size></box></geometry></collision>
      <visual name="visual"><geometry><box><size>10 0.2 1</size></box></geometry></visual>
    </link>
    <link name="barrier">
      <pose>1.5 0 0.5 0 0 0.6</pose>
      <collision name="collision"><geometry><box><size>0.4 2.0 1</size></box></geometry></collision>
      <visual name="visual"><geometry><box><size>0.4 2.0 1</size></box></geometry></visual>
    </link>
  </model>
</sdf>
"""


def sun_model():
    return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="sun">
    <static>true</static>
    <link name="link"/>
    <light name="sun_light" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
  </model>
</sdf>
"""


def ground_plane_model():
    return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="ground_plane">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
      </collision>
      <visual name="visual">
        <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
        <material>
          <ambient>0.4 0.4 0.4 1</ambient>
          <diffuse>0.5 0.5 0.5 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


SPECIAL_MODELS = {
    "sun": sun_model(),
    "ground_plane": ground_plane_model(),
    "racecar_description/models/cone": static_box_model("cone", (0.25, 0.25, 0.35), rgba(1.0, 0.45, 0.1)),
    "racecar_description/models/ar_tags/marker1": static_box_model("marker1", (0.12, 0.02, 0.12), rgba(0.95, 0.95, 0.95)),
    "racecar_description/models/parking_1": parking_model(),
    "racecar_description/models/MIT_Tunnel": tunnel_model(),
    "racecar_description/models/walker_racecourse": walker_course_model(),
}


def part(name, color, triangles):
    return {
        "name": name,
        "color": color,
        "triangles": triangles,
    }


def box_triangles(center, size):
    cx, cy, cz = center
    sx, sy, sz = (size[0] / 2.0, size[1] / 2.0, size[2] / 2.0)
    vertices = [
        (cx - sx, cy - sy, cz - sz),
        (cx + sx, cy - sy, cz - sz),
        (cx + sx, cy + sy, cz - sz),
        (cx - sx, cy + sy, cz - sz),
        (cx - sx, cy - sy, cz + sz),
        (cx + sx, cy - sy, cz + sz),
        (cx + sx, cy + sy, cz + sz),
        (cx - sx, cy + sy, cz + sz),
    ]
    faces = [
        (0, 1, 2), (0, 2, 3),
        (4, 6, 5), (4, 7, 6),
        (0, 4, 5), (0, 5, 1),
        (1, 5, 6), (1, 6, 2),
        (2, 6, 7), (2, 7, 3),
        (3, 7, 4), (3, 4, 0),
    ]
    return [[vertices[a], vertices[b], vertices[c]] for a, b, c in faces]


def frustum_triangles(center, bottom_radius, top_radius, height, sides=16):
    cx, cy, cz = center
    z0 = cz - height / 2.0
    z1 = cz + height / 2.0
    bottom = []
    top = []
    for index in range(sides):
        angle = (2.0 * math.pi * index) / sides
        bottom.append((cx + bottom_radius * math.cos(angle), cy + bottom_radius * math.sin(angle), z0))
        top.append((cx + top_radius * math.cos(angle), cy + top_radius * math.sin(angle), z1))
    triangles = []
    bottom_center = (cx, cy, z0)
    top_center = (cx, cy, z1)
    for index in range(sides):
        nxt = (index + 1) % sides
        triangles.append([bottom[index], bottom[nxt], top[index]])
        triangles.append([top[index], bottom[nxt], top[nxt]])
        if bottom_radius > 0.0:
            triangles.append([bottom_center, bottom[index], bottom[nxt]])
        if top_radius > 0.0:
            triangles.append([top_center, top[nxt], top[index]])
    return triangles


def cylinder_triangles(center, radius, height, sides=16):
    return frustum_triangles(center, radius, radius, height, sides=sides)


def combine(*groups):
    parts = []
    for group in groups:
        parts.extend(group)
    return parts


def building_parts(width, depth, height, wall, roof, accent):
    return [
        part("main", wall, box_triangles((0.0, 0.0, height / 2.0), (width, depth, height))),
        part("roof", roof, box_triangles((0.0, 0.0, height + 0.35), (width + 0.7, depth + 0.7, 0.7))),
        part("entrance", accent, box_triangles((0.0, -(depth / 2.0) - 0.18, height * 0.24), (width * 0.22, 0.24, height * 0.30))),
    ]


def storefront_parts(width, depth, height, wall, canopy, roof):
    return [
        part("main", wall, box_triangles((0.0, 0.0, height / 2.0), (width, depth, height))),
        part("canopy", canopy, box_triangles((0.0, -(depth / 2.0) - 0.16, height * 0.62), (width * 0.92, 0.34, 0.24))),
        part("roof", roof, box_triangles((0.0, 0.0, height + 0.20), (width + 0.4, depth + 0.4, 0.40))),
    ]


def vehicle_parts(length, width, body_height, cabin_height, body_color, cabin_color, wheel_color):
    wheel_size = (length * 0.18, width * 0.14, body_height * 0.46)
    wheel_x = length * 0.28
    wheel_y = width * 0.42
    wheel_z = wheel_size[2] / 2.0
    return [
        part("body", body_color, box_triangles((0.0, 0.0, body_height * 0.56), (length, width, body_height))),
        part("cabin", cabin_color, box_triangles((length * 0.08, 0.0, body_height + cabin_height * 0.40), (length * 0.46, width * 0.88, cabin_height))),
        part("wheel_fl", wheel_color, box_triangles((wheel_x, wheel_y, wheel_z), wheel_size)),
        part("wheel_fr", wheel_color, box_triangles((wheel_x, -wheel_y, wheel_z), wheel_size)),
        part("wheel_rl", wheel_color, box_triangles((-wheel_x, wheel_y, wheel_z), wheel_size)),
        part("wheel_rr", wheel_color, box_triangles((-wheel_x, -wheel_y, wheel_z), wheel_size)),
    ]


def jersey_barrier_parts(main_color):
    stripe = rgba(0.96, 0.96, 0.96)
    return [
        part("base", main_color, box_triangles((0.0, 0.0, 0.28), (1.80, 0.42, 0.56))),
        part("top", tint(main_color, 0.08), box_triangles((0.0, 0.0, 0.74), (1.20, 0.24, 0.34))),
        part("stripe", stripe, box_triangles((0.0, 0.0, 0.48), (1.82, 0.05, 0.12))),
    ]


def cone_parts(final_radius, final_height, color, scale=1.0):
    base = final_radius / scale
    height = final_height / scale
    return [
        part("cone", color, frustum_triangles((0.0, 0.0, height / 2.0), base, base * 0.25, height, sides=20)),
        part("base_ring", rgba(0.16, 0.16, 0.16), box_triangles((0.0, 0.0, height * 0.08), (base * 2.4, base * 2.4, height * 0.10))),
    ]


def barrel_parts(radius, height, color):
    return [
        part("barrel", color, frustum_triangles((0.0, 0.0, height / 2.0), radius * 0.86, radius, height, sides=18)),
        part("stripe_top", rgba(0.95, 0.95, 0.95), box_triangles((0.0, 0.0, height * 0.72), (radius * 1.9, radius * 1.9, height * 0.10))),
        part("stripe_mid", rgba(0.95, 0.95, 0.95), box_triangles((0.0, 0.0, height * 0.45), (radius * 1.9, radius * 1.9, height * 0.10))),
    ]


def water_tower_parts():
    tank = rgba(0.64, 0.78, 0.92)
    steel = rgba(0.56, 0.58, 0.62)
    return [
        part("tank", tank, cylinder_triangles((0.0, 0.0, 12.5), 2.8, 3.4, sides=18)),
        part("roof", tint(tank, 0.08), frustum_triangles((0.0, 0.0, 14.6), 2.9, 0.2, 1.2, sides=18)),
        part("leg_fl", steel, box_triangles((1.8, 1.8, 6.0), (0.34, 0.34, 12.0))),
        part("leg_fr", steel, box_triangles((1.8, -1.8, 6.0), (0.34, 0.34, 12.0))),
        part("leg_rl", steel, box_triangles((-1.8, 1.8, 6.0), (0.34, 0.34, 12.0))),
        part("leg_rr", steel, box_triangles((-1.8, -1.8, 6.0), (0.34, 0.34, 12.0))),
    ]


def crane_parts():
    yellow = rgba(0.95, 0.82, 0.14)
    dark = rgba(0.20, 0.20, 0.20)
    return [
        part("mast", yellow, box_triangles((0.0, 0.0, 12.0), (1.0, 1.0, 24.0))),
        part("jib", yellow, box_triangles((9.0, 0.0, 23.2), (18.0, 0.55, 0.55))),
        part("counter", yellow, box_triangles((-4.0, 0.0, 22.8), (4.5, 0.70, 0.70))),
        part("cab", dark, box_triangles((0.8, 0.0, 22.2), (1.4, 1.1, 1.1))),
        part("hook", dark, box_triangles((15.2, 0.0, 18.0), (0.12, 0.12, 10.0))),
    ]


def playground_parts():
    return [
        part("frame_left", rgba(0.18, 0.66, 0.82), box_triangles((-1.6, 0.0, 1.4), (0.24, 0.24, 2.8))),
        part("frame_right", rgba(0.18, 0.66, 0.82), box_triangles((1.6, 0.0, 1.4), (0.24, 0.24, 2.8))),
        part("crossbar", rgba(0.96, 0.64, 0.16), box_triangles((0.0, 0.0, 2.7), (3.6, 0.20, 0.20))),
        part("slide", rgba(0.84, 0.18, 0.18), box_triangles((0.0, -1.3, 1.0), (2.2, 0.45, 0.16))),
        part("platform", rgba(0.16, 0.52, 0.20), box_triangles((0.0, 0.8, 1.4), (1.8, 1.1, 0.18))),
    ]


def human_parts():
    shirt = rgba(0.20, 0.30, 0.70)
    pants = rgba(0.12, 0.12, 0.16)
    skin = rgba(0.90, 0.78, 0.68)
    return [
        part("torso", shirt, box_triangles((0.0, 0.0, 0.95), (0.34, 0.18, 0.58))),
        part("head", skin, cylinder_triangles((0.0, 0.0, 1.42), 0.12, 0.20, sides=14)),
        part("leg_left", pants, box_triangles((0.09, 0.0, 0.36), (0.10, 0.10, 0.72))),
        part("leg_right", pants, box_triangles((-0.09, 0.0, 0.36), (0.10, 0.10, 0.72))),
        part("arm_left", skin, box_triangles((0.22, 0.0, 0.98), (0.08, 0.08, 0.48))),
        part("arm_right", skin, box_triangles((-0.22, 0.0, 0.98), (0.08, 0.08, 0.48))),
    ]


def table_parts():
    top = rgba(0.92, 0.91, 0.88)
    legs = rgba(0.28, 0.24, 0.20)
    return [
        part("top", top, box_triangles((0.0, 0.0, 2.70), (4.8, 2.8, 0.24))),
        part("leg_fl", legs, box_triangles((1.9, 1.0, 1.35), (0.18, 0.18, 2.70))),
        part("leg_fr", legs, box_triangles((1.9, -1.0, 1.35), (0.18, 0.18, 2.70))),
        part("leg_rl", legs, box_triangles((-1.9, 1.0, 1.35), (0.18, 0.18, 2.70))),
        part("leg_rr", legs, box_triangles((-1.9, -1.0, 1.35), (0.18, 0.18, 2.70))),
    ]


def valve_wheel_parts():
    red = rgba(0.78, 0.16, 0.14)
    dark = rgba(0.18, 0.18, 0.18)
    return [
        part("rim", red, cylinder_triangles((0.0, 0.0, 0.0), 0.45, 0.08, sides=18)),
        part("hub", dark, cylinder_triangles((0.0, 0.0, 0.0), 0.12, 0.18, sides=14)),
        part("spoke_a", red, box_triangles((0.0, 0.0, 0.0), (0.82, 0.08, 0.08))),
        part("spoke_b", red, box_triangles((0.0, 0.0, 0.0), (0.08, 0.82, 0.08))),
    ]


def plane_body_parts():
    white = rgba(0.97, 0.97, 0.99)
    red = rgba(0.82, 0.18, 0.16)
    return [
        part("fuselage", white, box_triangles((0.0, 0.0, 0.0), (8.0, 1.2, 1.4))),
        part("nose", red, box_triangles((3.8, 0.0, 0.0), (1.0, 1.0, 1.0))),
        part("cockpit", rgba(0.28, 0.42, 0.62), box_triangles((1.2, 0.0, 0.36), (1.6, 0.9, 0.55))),
    ]


def plane_surface_parts(span, chord, color):
    return [part("surface", color, box_triangles((0.0, 0.0, 0.0), (span, chord, 0.14)))]


def plane_rudder_parts():
    return [part("rudder", rgba(0.82, 0.18, 0.16), box_triangles((0.0, 0.0, 0.4), (0.18, 0.90, 0.80)))]


def plane_prop_parts():
    blade = rgba(0.18, 0.18, 0.18)
    return [
        part("blade_a", blade, box_triangles((0.0, 0.0, 0.0), (1.60, 0.08, 0.06))),
        part("blade_b", blade, box_triangles((0.0, 0.0, 0.0), (0.08, 1.60, 0.06))),
        part("hub", rgba(0.72, 0.72, 0.72), cylinder_triangles((0.0, 0.0, 0.0), 0.12, 0.16, sides=12)),
    ]


def plane_wheel_parts(radius, width):
    return [
        part("wheel", rgba(0.12, 0.12, 0.12), cylinder_triangles((0.0, 0.0, 0.0), radius, width, sides=14)),
        part("hub", rgba(0.60, 0.60, 0.60), cylinder_triangles((0.0, 0.0, 0.0), radius * 0.38, width * 1.4, sides=12)),
    ]


def demo_arm_parts():
    blue = rgba(0.22, 0.48, 0.82)
    return [
        part("arm", blue, box_triangles((350.0, 0.0, 0.0), (700.0, 120.0, 120.0))),
        part("joint", tint(blue, 0.12), cylinder_triangles((0.0, 0.0, 0.0), 80.0, 180.0, sides=16)),
    ]


def demo_base_parts():
    steel = rgba(0.34, 0.40, 0.52)
    return [
        part("base", steel, box_triangles((0.0, 0.0, 120.0), (1000.0, 600.0, 240.0))),
        part("pedestal", tint(steel, 0.10), box_triangles((0.0, 0.0, 320.0), (340.0, 260.0, 160.0))),
    ]


def wall_panel_parts():
    wall = rgba(0.84, 0.84, 0.82)
    frame = rgba(0.36, 0.36, 0.36)
    return [
        part("panel", wall, box_triangles((0.0, 0.0, 0.90), (2.5, 0.20, 1.8))),
        part("frame", frame, box_triangles((0.0, 0.0, 1.55), (2.7, 0.08, 0.10))),
        part("plate", rgba(0.92, 0.78, 0.18), box_triangles((0.0, 0.0, 0.75), (1.60, 0.10, 0.65))),
    ]


def dumpster_parts():
    green = rgba(0.14, 0.46, 0.22)
    lid = rgba(0.06, 0.22, 0.12)
    return [
        part("bin", green, box_triangles((0.0, 0.0, 0.95), (2.6, 1.4, 1.9))),
        part("lid", lid, box_triangles((0.0, 0.0, 1.96), (2.5, 1.36, 0.14))),
        part("caster_fl", rgba(0.10, 0.10, 0.10), box_triangles((0.95, 0.50, 0.12), (0.20, 0.20, 0.24))),
        part("caster_fr", rgba(0.10, 0.10, 0.10), box_triangles((0.95, -0.50, 0.12), (0.20, 0.20, 0.24))),
        part("caster_rl", rgba(0.10, 0.10, 0.10), box_triangles((-0.95, 0.50, 0.12), (0.20, 0.20, 0.24))),
        part("caster_rr", rgba(0.10, 0.10, 0.10), box_triangles((-0.95, -0.50, 0.12), (0.20, 0.20, 0.24))),
    ]


def gas_station_parts():
    white = rgba(0.94, 0.94, 0.94)
    red = rgba(0.88, 0.22, 0.16)
    gray = rgba(0.40, 0.40, 0.42)
    return [
        part("shop", white, box_triangles((-3.8, 0.0, 2.0), (5.2, 4.4, 4.0))),
        part("canopy", white, box_triangles((2.4, 0.0, 3.6), (10.0, 6.0, 0.50))),
        part("stripe", red, box_triangles((2.4, 0.0, 3.75), (10.2, 6.1, 0.14))),
        part("col_left", gray, box_triangles((-0.2, 1.4, 1.8), (0.38, 0.38, 3.6))),
        part("col_right", gray, box_triangles((-0.2, -1.4, 1.8), (0.38, 0.38, 3.6))),
        part("pump_a", red, box_triangles((1.2, 1.2, 0.9), (0.70, 0.70, 1.8))),
        part("pump_b", red, box_triangles((1.2, -1.2, 0.9), (0.70, 0.70, 1.8))),
    ]


def wheel_box_parts(length, width, height, body_color):
    roof = tint(body_color, 0.12)
    dark = rgba(0.12, 0.12, 0.12)
    return [
        part("body", body_color, box_triangles((0.0, 0.0, height * 0.50), (length, width, height))),
        part("roof", roof, box_triangles((0.0, 0.0, height * 0.98), (length * 0.60, width * 0.88, height * 0.46))),
        part("wheel_fl", dark, box_triangles((length * 0.28, width * 0.42, height * 0.16), (length * 0.18, width * 0.14, height * 0.30))),
        part("wheel_fr", dark, box_triangles((length * 0.28, -width * 0.42, height * 0.16), (length * 0.18, width * 0.14, height * 0.30))),
        part("wheel_rl", dark, box_triangles((-length * 0.28, width * 0.42, height * 0.16), (length * 0.18, width * 0.14, height * 0.30))),
        part("wheel_rr", dark, box_triangles((-length * 0.28, -width * 0.42, height * 0.16), (length * 0.18, width * 0.14, height * 0.30))),
    ]


def mesh_parts_for_ref(ref):
    if ref == "apartment/meshes/apartment.dae":
        return building_parts(16.0, 12.0, 28.0, PALETTE["apartment"], rgba(0.38, 0.34, 0.30), rgba(0.32, 0.46, 0.72))
    if ref == "ambulance/meshes/ambulance.obj":
        return wheel_box_parts(216.0, 92.0, 102.0, PALETTE["ambulance"])
    if ref == "bus/meshes/bus.obj":
        return wheel_box_parts(1200.0, 280.0, 340.0, PALETTE["bus"])
    if ref == "suv/meshes/suv.obj":
        return wheel_box_parts(76.0, 34.0, 32.0, PALETTE["suv"])
    if ref == "construction_cone/meshes/construction_cone.dae":
        return cone_parts(0.45, 0.90, PALETTE["construction_cone"], scale=10.0)
    if ref == "construction_barrel/meshes/construction_barrel.dae":
        return barrel_parts(0.34, 0.92, PALETTE["construction_barrel"])
    if ref == "drc_practice_orange_jersey_barrier/meshes/jersey_barrier.dae":
        return jersey_barrier_parts(PALETTE["drc_practice_orange_jersey_barrier"])
    if ref == "drc_practice_white_jersey_barrier/meshes/jersey_barrier.dae":
        return jersey_barrier_parts(PALETTE["drc_practice_white_jersey_barrier"])
    if ref == "drc_practice_blue_cylinder/meshes/cylinder.dae":
        return [part("cylinder", PALETTE["drc_practice_blue_cylinder"], cylinder_triangles((0.0, 0.0, 0.7), 0.34, 1.4, sides=18))]
    if ref == "drc_practice_valve_wall/meshes/five_dice_wall.dae":
        return wall_panel_parts()
    if ref == "drc_practice_wheel_valve/meshes/valve_wheel.dae":
        return valve_wheel_parts()
    if ref == "demo_joint_damping/meshes/base_model.dae":
        return demo_base_parts()
    if ref == "demo_joint_damping/meshes/arm.stl":
        return demo_arm_parts()
    if ref == "dumpster/meshes/dumpster.dae":
        return dumpster_parts()
    if ref == "fast_food/meshes/fast_food.dae":
        return storefront_parts(4.6, 3.2, 3.6, rgba(0.96, 0.82, 0.58), rgba(0.84, 0.16, 0.14), rgba(0.98, 0.76, 0.18))
    if ref == "fire_station/meshes/fire_station.dae":
        return building_parts(20.0, 9.7, 10.0, rgba(0.80, 0.20, 0.16), rgba(0.36, 0.14, 0.14), rgba(0.96, 0.96, 0.94))
    if ref == "gas_station/meshes/gas_station.dae":
        return gas_station_parts()
    if ref == "house_3/meshes/house_3.dae":
        return combine(
            storefront_parts(6.4, 5.0, 4.0, PALETTE["house_3"], rgba(0.54, 0.28, 0.20), rgba(0.60, 0.22, 0.18)),
            [part("roof_peak", rgba(0.62, 0.24, 0.18), box_triangles((0.0, 0.0, 5.6), (5.6, 4.6, 1.4)))]
        )
    if ref == "law_office/meshes/law_office.dae":
        return building_parts(6.84, 5.43, 13.92, rgba(0.62, 0.34, 0.22), rgba(0.22, 0.22, 0.24), rgba(0.34, 0.56, 0.78))
    if ref == "person_walking/meshes/walking.dae":
        return human_parts()
    if ref == "playground/meshes/playground.dae":
        return playground_parts()
    if ref == "polaris_ranger_ev/meshes/polaris.dae":
        return vehicle_parts(3.5, 1.7, 1.0, 0.7, PALETTE["polaris_ranger_ev"], tint(PALETTE["polaris_ranger_ev"], 0.12), rgba(0.10, 0.10, 0.10))
    if ref == "salon/meshes/salon.dae":
        return storefront_parts(7.2, 5.4, 7.0, rgba(0.24, 0.58, 0.64), rgba(0.96, 0.94, 0.88), rgba(0.18, 0.28, 0.34))
    if ref == "table_marble/meshes/table_lightmap.dae":
        return table_parts()
    if ref == "thrift_shop/meshes/thrift_shop.dae":
        return storefront_parts(7.2, 5.4, 6.6, rgba(0.72, 0.48, 0.22), rgba(0.92, 0.84, 0.58), rgba(0.44, 0.20, 0.14))
    if ref == "tower_crane/meshes/tower_crane.dae":
        return crane_parts()
    if ref == "water_tower/meshes/water_tower.dae":
        return water_tower_parts()
    if ref == "brick_box_3x1x3/meshes/simple_box.dae":
        return [part("brick", rgba(0.72, 0.34, 0.24), box_triangles((0.0, 0.0, 0.0), (1.0, 1.0, 1.0)))]
    if ref == "cessna/meshes/body.dae":
        return plane_body_parts()
    if ref == "cessna/meshes/left_aileron.dae":
        return plane_surface_parts(2.0, 0.36, rgba(0.97, 0.97, 0.99))
    if ref == "cessna/meshes/left_flap.dae":
        return plane_surface_parts(1.6, 0.42, rgba(0.82, 0.18, 0.16))
    if ref == "cessna/meshes/right_aileron.dae":
        return plane_surface_parts(2.0, 0.36, rgba(0.97, 0.97, 0.99))
    if ref == "cessna/meshes/right_flap.dae":
        return plane_surface_parts(1.6, 0.42, rgba(0.82, 0.18, 0.16))
    if ref == "cessna/meshes/elevators.dae":
        return plane_surface_parts(2.0, 0.50, rgba(0.97, 0.97, 0.99))
    if ref == "cessna/meshes/rudder.dae":
        return plane_rudder_parts()
    if ref == "cessna/meshes/cessna_prop.dae":
        return plane_prop_parts()
    if ref == "cessna/meshes/cessna_front_wheel.dae":
        return plane_wheel_parts(0.22, 0.10)
    if ref == "cessna/meshes/cessna_rear_left_wheel.dae":
        return plane_wheel_parts(0.28, 0.12)
    if ref == "cessna/meshes/cessna_rear_right_wheel.dae":
        return plane_wheel_parts(0.28, 0.12)
    color = base_color_for_key(ref)
    return [part("generic", color, box_triangles((0.0, 0.0, 0.0), (1.0, 1.0, 1.0)))]


def triangles_to_dae(path, parts):
    effects = []
    materials = []
    geometries = []
    nodes = []
    for index, item in enumerate(parts):
        effect_id = f"effect_{index}"
        material_id = f"material_{index}"
        geometry_id = f"geometry_{index}"
        color = item["color"]
        positions = []
        indices = []
        counter = 0
        for triangle in item["triangles"]:
            for vertex in triangle:
                positions.extend(vertex)
                indices.append(str(counter))
                counter += 1
        color_text = " ".join(f"{value:.4f}" for value in color)
        effects.append(
            f"""    <effect id="{effect_id}">
      <profile_COMMON>
        <technique sid="common">
          <phong>
            <emission><color>0 0 0 1</color></emission>
            <ambient><color>{color_text}</color></ambient>
            <diffuse><color>{color_text}</color></diffuse>
            <specular><color>0.08 0.08 0.08 1</color></specular>
            <shininess><float>12</float></shininess>
          </phong>
        </technique>
      </profile_COMMON>
    </effect>"""
        )
        materials.append(f"""    <material id="{material_id}" name="{item['name']}"><instance_effect url="#{effect_id}"/></material>""")
        positions_text = " ".join(f"{value:.6f}" for value in positions)
        geometries.append(
            f"""    <geometry id="{geometry_id}" name="{item['name']}">
      <mesh>
        <source id="{geometry_id}-positions">
          <float_array id="{geometry_id}-positions-array" count="{len(positions)}">{positions_text}</float_array>
          <technique_common>
            <accessor source="#{geometry_id}-positions-array" count="{counter}" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="{geometry_id}-vertices">
          <input semantic="POSITION" source="#{geometry_id}-positions"/>
        </vertices>
        <triangles material="{material_id}" count="{counter // 3}">
          <input semantic="VERTEX" source="#{geometry_id}-vertices" offset="0"/>
          <p>{" ".join(indices)}</p>
        </triangles>
      </mesh>
    </geometry>"""
        )
        nodes.append(
            f"""      <node id="node_{index}" name="{item['name']}">
        <instance_geometry url="#{geometry_id}">
          <bind_material>
            <technique_common>
              <instance_material symbol="{material_id}" target="#{material_id}"/>
            </technique_common>
          </bind_material>
        </instance_geometry>
      </node>"""
        )
    dae_text = f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <unit name="meter" meter="1"/>
    <up_axis>Z_UP</up_axis>
  </asset>
  <library_effects>
{chr(10).join(effects)}
  </library_effects>
  <library_materials>
{chr(10).join(materials)}
  </library_materials>
  <library_geometries>
{chr(10).join(geometries)}
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
{chr(10).join(nodes)}
    </visual_scene>
  </library_visual_scenes>
  <scene>
    <instance_visual_scene url="#Scene"/>
  </scene>
</COLLADA>
"""
    write_text(path, dae_text)


def triangles_to_obj(path, parts):
    obj_lines = [f"mtllib {path.with_suffix('.mtl').name}"]
    mtl_lines = []
    vertex_index = 1
    for index, item in enumerate(parts):
        material_name = f"mat_{index}"
        r, g, b, a = item["color"]
        obj_lines.append(f"o {item['name']}")
        obj_lines.append(f"usemtl {material_name}")
        face_start = vertex_index
        for triangle in item["triangles"]:
            for vertex in triangle:
                obj_lines.append(f"v {vertex[0]:.6f} {vertex[1]:.6f} {vertex[2]:.6f}")
                vertex_index += 1
        for offset in range(0, len(item["triangles"]) * 3, 3):
            a_idx = face_start + offset
            b_idx = a_idx + 1
            c_idx = a_idx + 2
            obj_lines.append(f"f {a_idx} {b_idx} {c_idx}")
        mtl_lines.extend(
            [
                f"newmtl {material_name}",
                f"Ka {r:.4f} {g:.4f} {b:.4f}",
                f"Kd {r:.4f} {g:.4f} {b:.4f}",
                "Ks 0.0600 0.0600 0.0600",
                f"d {a:.4f}",
                "illum 2",
                "",
            ]
        )
    write_text(path, "\n".join(obj_lines) + "\n")
    write_text(path.with_suffix(".mtl"), "\n".join(mtl_lines))


def triangles_to_stl(path, parts):
    stl_lines = ["solid placeholder"]
    for item in parts:
        for triangle in item["triangles"]:
            a, b, c = triangle
            ux, uy, uz = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
            vx, vy, vz = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
            nx = uy * vz - uz * vy
            ny = uz * vx - ux * vz
            nz = ux * vy - uy * vx
            norm = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
            stl_lines.append(f"facet normal {nx / norm:.6f} {ny / norm:.6f} {nz / norm:.6f}")
            stl_lines.append(" outer loop")
            stl_lines.append(f"  vertex {a[0]:.6f} {a[1]:.6f} {a[2]:.6f}")
            stl_lines.append(f"  vertex {b[0]:.6f} {b[1]:.6f} {b[2]:.6f}")
            stl_lines.append(f"  vertex {c[0]:.6f} {c[1]:.6f} {c[2]:.6f}")
            stl_lines.append(" endloop")
            stl_lines.append("endfacet")
    stl_lines.append("endsolid placeholder")
    write_text(path, "\n".join(stl_lines) + "\n")


def write_mesh_placeholder(path):
    ref = path.relative_to(MODEL_DIR).as_posix()
    parts = mesh_parts_for_ref(ref)
    suffix = path.suffix.lower()
    if suffix == ".dae":
        triangles_to_dae(path, parts)
    elif suffix == ".obj":
        triangles_to_obj(path, parts)
    elif suffix == ".stl":
        triangles_to_stl(path, parts)
    else:
        triangles_to_dae(path.with_suffix(".dae"), parts)


def material_names_from_worlds():
    names = {}
    script_block = re.compile(r"<script>(.*?)</script>", re.DOTALL)
    uri_pattern = re.compile(r"model://([^<\"'\n\r ]+/materials/scripts/?)[^<]*")
    name_pattern = re.compile(r"<name>([^<]+)</name>")
    for world_path in WORLD_DIR.glob("*.world"):
        text = world_path.read_text(errors="ignore")
        for block in script_block.findall(text):
            uris = uri_pattern.findall(block)
            if not uris:
                continue
            block_names = name_pattern.findall(block) or ["Generated/Default"]
            for uri in uris:
                model_key = uri.split("/materials/scripts", 1)[0]
                names.setdefault(model_key, set()).update(block_names)
    return names


def world_references():
    refs = set()
    for world_path in WORLD_DIR.glob("*.world"):
        text = world_path.read_text(errors="ignore")
        index = 0
        while True:
            index = text.find("model://", index)
            if index < 0:
                break
            cursor = index + 8
            while cursor < len(text) and text[cursor] not in "<\"' \n\r\t":
                cursor += 1
            refs.add(text[index + 8:cursor].rstrip("/"))
            index = cursor
    return refs


def create_direct_reference_assets(refs):
    for ref in sorted(refs):
        path = MODEL_DIR / ref
        suffix = path.suffix.lower()
        if suffix in {".dae", ".obj", ".stl"}:
            write_mesh_placeholder(path)
        elif suffix == ".png":
            write_binary(path, PNG_BYTES)
        elif "/materials/textures" in ref:
            path.mkdir(parents=True, exist_ok=True)
            write_binary(path / "placeholder.png", PNG_BYTES)
        elif "/materials/scripts" in ref:
            path.mkdir(parents=True, exist_ok=True)


def material_color_for_name(model_key, material_name):
    overrides = MATERIAL_OVERRIDES.get(model_key, {})
    if material_name in overrides:
        return overrides[material_name]
    if "white" in material_name.lower() or "net" in material_name.lower():
        return rgba(0.95, 0.95, 0.95)
    if "mud" in material_name.lower():
        return rgba(0.34, 0.22, 0.12)
    return base_color_for_key(model_key)


def create_material_scripts(names_by_model):
    for model_key, material_names in sorted(names_by_model.items()):
        script_path = MODEL_DIR / model_key / "materials" / "scripts" / "generated.material"
        blocks = []
        for material_name in sorted(material_names):
            r, g, b, a = material_color_for_name(model_key, material_name)
            blocks.append(
                f"""material {material_name}
{{
  technique
  {{
    pass
    {{
      ambient {r:.4f} {g:.4f} {b:.4f} {a:.4f}
      diffuse {r:.4f} {g:.4f} {b:.4f} {a:.4f}
      specular 0.0800 0.0800 0.0800 1 8
      emissive 0 0 0 1
    }}
  }}
}}
"""
            )
        write_text(script_path, "\n".join(blocks))
        textures_dir = script_path.parent.parent / "textures"
        textures_dir.mkdir(parents=True, exist_ok=True)
        write_binary(textures_dir / "placeholder.png", PNG_BYTES)


def create_special_models():
    for model_key, sdf_contents in SPECIAL_MODELS.items():
        model_path = MODEL_DIR / model_key
        write_text(model_path / "model.config", material_config_block(model_path.name))
        write_text(model_path / "model.sdf", sdf_contents)


def main():
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    refs = world_references()
    create_direct_reference_assets(refs)
    create_material_scripts(material_names_from_worlds())
    create_special_models()
    print(f"Generated offline Gazebo assets under {MODEL_DIR}")


if __name__ == "__main__":
    main()
