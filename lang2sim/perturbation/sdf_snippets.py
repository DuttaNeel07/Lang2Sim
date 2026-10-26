"""Fallback inline SDF definitions for dynamic obstacle models.

Used when pre-compiled models are not found on the local filesystem or Fuel catalog.
Provides physics collision geometries, inertial tensors, and basic PBR/diffuse visuals.
"""

from __future__ import annotations

_WOODEN_CRATE_SDF = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.6</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.6</iyy><iyz>0</iyz><izz>0.6</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box><size>0.6 0.6 0.6</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>0.6 0.6 0.6</size></box>
        </geometry>
        <material>
          <ambient>0.6 0.4 0.2 1</ambient>
          <diffuse>0.6 0.4 0.2 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

_BARREL_SDF = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>15.0</mass>
        <inertia>
          <ixx>1.2</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1.2</iyy><iyz>0</iyz><izz>0.675</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <cylinder><radius>0.3</radius><length>0.9</length></cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder><radius>0.3</radius><length>0.9</length></cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.3 0.6 1</ambient>
          <diffuse>0.2 0.3 0.6 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

_CARGO_BOX_SDF = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>20.0</mass>
        <inertia>
          <ixx>1.2</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>2.2</iyy><iyz>0</iyz><izz>2.2</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box><size>1.0 0.6 0.6</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>1.0 0.6 0.6</size></box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.7 0.7 0.2 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

MODELS = {
    "wooden_crate": _WOODEN_CRATE_SDF,
    "barrel": _BARREL_SDF,
    "cargo_box": _CARGO_BOX_SDF,
}


def get_inline_sdf(model_name: str, entity_name: str) -> str:
    """Return inline SDF string for known models, formatted with entity name."""
    template = MODELS.get(model_name, _WOODEN_CRATE_SDF)
    return template.format(name=entity_name).strip()
