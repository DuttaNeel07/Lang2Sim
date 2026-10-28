"""
Gazebo Harmonic (SDF 1.9+) World Compiler.
File: lang2sim/compiler/sdf_compiler.py
"""

from pathlib import Path

from jinja2 import Environment, FileSystemLoader

from lang2sim.core.config import config
from lang2sim.core.models import SimSpec


class HarmonicSdfCompiler:
    """
    Compiles canonical SimSpec models into physically accurate Gazebo Harmonic
    SDF 1.9+ XML worlds.
    """

    def __init__(self, templates_dir: Path = config.templates_dir):
        self.templates_dir = templates_dir
        self.env = Environment(
            loader=FileSystemLoader(str(self.templates_dir)),
            autoescape=False,
            trim_blocks=True,
            lstrip_blocks=True,
        )
        self.template = self.env.get_template("world_harmonic.sdf.jinja")

    def compile(
        self,
        sim_spec: SimSpec,
        output_path: Path,
        world_name: str = "default_world",
    ) -> Path:
        """
        Renders the SDF 1.9 template and writes the result to disk.
        """
        output_path.parent.mkdir(parents=True, exist_ok=True)

        context = {
            "world_name": world_name,
            "ambient_light": sim_spec.environment.ambient_light_luminance,
            "surface_friction": sim_spec.environment.surface_friction,
            "sky": sim_spec.environment.sky_weather,
            "room": sim_spec.environment.room_dimensions,
            "wall_height": sim_spec.environment.wall_height,
            "objects": sim_spec.objects,
        }

        rendered_xml = self.template.render(context)

        with open(output_path, "w", encoding="utf-8") as f:
            f.write(rendered_xml)

        return output_path


def compile_world_sdf(
    sim_spec: SimSpec, output_path: Path, world_name: str = "default_world"
) -> Path:
    """Convenience helper to compile a SimSpec into a world.sdf file."""
    compiler = HarmonicSdfCompiler()
    return compiler.compile(sim_spec, output_path, world_name)
