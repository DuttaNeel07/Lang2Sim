"""
Simulation Intermediate Representation (SIR) Generator.
File: sim2lang/compiler/sir_generator.py
"""

from typing import Optional
from sim2lang.core.models import SimSpec, SimulationIR
from sim2lang.world_gen.shapely_solver import ShapelySpatialSolver
from sim2lang.compiler.sir_validator import SIRValidator


class SIRGenerator:
    """
    Compiles raw SimSpec declarations into validated SimulationIR graphs.
    """

    def __init__(self):
        self.solver = ShapelySpatialSolver()
        self.validator = SIRValidator()

    def generate(self, sim_spec: SimSpec) -> SimulationIR:
        """
        Solves placement for unplaced objects and builds the canonical SimulationIR.
        """
        placed_objects, corridors = self.solver.solve_layout(
            environment=sim_spec.environment,
            objects=sim_spec.objects,
            spawn_pose=sim_spec.mission.initial_pose,
            goal_pose=sim_spec.mission.goal_pose,
        )

        resolved_spec = sim_spec.model_copy(update={"objects": placed_objects})
        is_valid, validation_messages = self.validator.validate(resolved_spec)

        return SimulationIR(
            spec=resolved_spec,
            aisle_corridors=corridors,
            is_valid=is_valid,
            validation_messages=validation_messages,
        )


def generate_simulation_ir(sim_spec: SimSpec) -> SimulationIR:
    """Convenience helper to generate validated SimulationIR."""
    generator = SIRGenerator()
    return generator.generate(sim_spec)
