# Developer 1 Guide: My Role, Deliverables & Big Picture (In Plain English)

> **Welcome to Part 1 of Sim2Lang!**  
> If you are Developer 1, this document explains **who you are, what you are building, how it connects to your team, and the exact files you need to write**—without confusing jargon.

---

## 1. Who Am I and What Is My Mission?

### The "Elevator Pitch" of Your Job
You are the **World Architect & Mapmaker**.

If Sim2Lang were a video game:
* **Developer 2** is building the chat window and the AI brain that talks to the player.
* **Developer 3** is building the game launcher that starts the 3D graphics on your screen.
* **Developer 4** is building the obstacle challenges and the high-score counter.
* **YOU (Developer 1)** are the **Level Designer**. You build the 3D room, arrange the furniture, ensure the hallways are wide enough for the robot, draw the floor plan map, and hand those files to Developer 3 so the game can actually load!

### The Real-World Problem You Solve
Normally, in robotics companies, when an engineer wants to test an autonomous robot in Gazebo simulation, they have to:
1. Manually write hundreds of lines of complex XML (`.sdf`) to place walls and shelves (takes 1–2 days).
2. Connect a joystick to the robot and spend 15–30 minutes driving it around the virtual room to scan it with laser sensors (SLAM) just to generate a top-down navigation map.

**Your code replaces those 3 days of manual human labor in 3.5 seconds.**

---

## 2. How My Work Connects to My Teammates

Here is the assembly line of how data flows through the team:

```
[User Types in Terminal]
"Generate a 14x10m warehouse with 8 pallet racks, dim lights, and send robot to rack 5"
                           │
                           ▼
┌────────────────────────────────────────────────────────┐
│ DEVELOPER 2: The Copilot Brain & Terminal App          │
│ • Listens to user prompt.                              │
│ • Calls YOUR Python functions first to build the room! │
└──────────────────────────┬─────────────────────────────┘
                           │
                           ▼ (Calls YOUR functions)
┌────────────────────────────────────────────────────────┐
│ YOU (DEVELOPER 1): The World Architect & Mapmaker      │
│ • Calculates where the 8 racks go without overlapping. │
│ • Makes sure hallways are at least 2.2 meters wide.    │
│ • Draws the black-and-white map (map.yaml, map.pgm).   │
│ • Writes the 3D Gazebo world file (world.sdf).         │
└──────────────────────────┬─────────────────────────────┘
                           │
                           ▼ (Hands world.sdf & map.yaml to Dev 3)
┌────────────────────────────────────────────────────────┐
│ DEVELOPER 3: The Simulation Launcher & Robot Driver    │
│ • Opens Gazebo (3D world) and RViz (robot radar).      │
│ • Loads the map YOU drew so the robot knows the room.  │
│ • Starts the robot driving toward Rack 5.              │
└──────────────────────────┬─────────────────────────────┘
                           │
                           ▼ (While robot is moving...)
┌────────────────────────────────────────────────────────┐
│ DEVELOPER 4: The Chaos Monkey & Telemetry Scorer       │
│ • Drops a surprise wooden crate in the robot's way.    │
│ • Dims the room lights to test the robot's camera.     │
│ • Scores how cleanly the robot navigated.              │
└────────────────────────────────────────────────────────┘
```

---

## 3. Which Files Am I Dependent On Before I Can Code?

### **The Best News: You have ZERO dependencies on other teammates!**

* You are **Phase 1 (The Foundation)**.
* You do **not** need Developer 2’s AI to test your geometry math.
* You do **not** need Developer 3’s Docker or ROS 2 setup to draw your map images.
* You do **not** need Developer 4’s sensors to export 3D world files.
* **Everyone else is waiting on YOU:**
  * Developer 2 needs your data models (`models.py`) so their AI knows what parameters exist.
  * Developer 3 needs your `world.sdf` and `map.yaml` so they have a world to launch!

You can code, run, and test **100% of your work on your local laptop using standard Python (`pytest`)** without even opening Gazebo.

---

## 4. The Exact Files You Have to Create

You have **7 core files** to create across 3 folders:

```
Lang2Sim/
├── pyproject.toml                         # 1. Project setup and library list
├── catalog/
│   ├── scene_catalog.json                 # 2. Pre-made warehouse blueprints
│   └── maps/aws_small_warehouse.yaml/.pgm # 3. Pre-drawn warehouse map
└── sim2lang/
    ├── core/
    │   ├── models.py                      # 4. The Data Rulebook (Pydantic schemas)
    │   └── config.py                      # 5. Folder path manager
    ├── world_gen/
    │   ├── shapely_solver.py              # 6. The Geometry & Hallway Math Brain
    │   └── map_slicer.py                  # 7. Instant Top-Down Map Generator
    └── compiler/
        └── sdf_compiler.py                # 8. 3D Gazebo World File Exporter
```

### What Each File Does in Plain English

#### 1. `sim2lang/core/models.py` (The Data Rulebook)
* **What it does:** It defines the official vocabulary for the whole project.
* **Example:** It states that an `ObjectSpec` must have an ID (like `"rack_1"`), a 3D size (length, width, height), and a position `(x, y, rotation)`. If an AI or user accidentally passes letters instead of numbers, Python stops the error before anything breaks.

#### 2. `sim2lang/core/config.py` (The Folder Manager)
* **What it does:** Keeps your hard drive clean. It tells the program where to save generated files (inside `~/.lang2sim/projects/<project_name>/`) so files don't get scattered all over your computer.

#### 3. `catalog/scene_catalog.json` (The Warehouse Shortcut)
* **What it does:** If a user just says *"Load a warehouse and drive to rack 5"*, why calculate math from scratch? We keep a verified blueprint of standard Amazon-style warehouses with exact pre-verified coordinates (e.g. `dock` is at `(0, 0)`, `rack_5` is at `(7.5, 3.0)`).

#### 4. `sim2lang/world_gen/shapely_solver.py` (The Hallway & Collision Math Brain)
* **What it does:** Solves where furniture goes. If the user asks for *"a custom 18x12m room with 8 racks"*, this script places 2D boxes for the racks inside the room.
* **The Two Golden Rules it enforces:**
  1. **No Overlaps:** Shelves can never touch or overlap with each other or the walls.
  2. **Wide Hallways:** Every aisle between shelves must be at least **$2.2\text{ meters}$ wide** so the robot can drive through without getting stuck!

#### 5. `sim2lang/world_gen/map_slicer.py` (Instant 2D Map Generator)
* **What it does:** Normally, robots take 15 minutes of slow driving to scan a room. This script takes the 2D boxes from `shapely_solver.py` and paints a black-and-white top-down image in **2 seconds**:
  * White pixels = Open floor where the robot can drive.
  * Black pixels = Obstacles (walls, racks).
  * Outputs `map.pgm` (the image) and `map.yaml` (metadata telling the robot: 1 pixel = 5 centimeters).

#### 6. `sim2lang/compiler/sdf_compiler.py` (3D World Exporter)
* **What it does:** Takes the room dimensions, object coordinates, and lighting/weather, and fills in a template to write out `world.sdf`. Gazebo reads this file to display the 3D room, ground floor friction, sky, and fog.

---

## 5. What Libraries & Open-Source Tools Am I Using?

| Tool / Project | What It Is in Plain English | Why We Use It |
| :--- | :--- | :--- |
| **`gazebo_world_generator`** | Open-source project from Robotics-UPO | Gives us the foundational math for placing room objects and the SDF templates for Gazebo. |
| **`godrift.ai`** | Commercial robot simulation platform | Teaches us the coordinate standards (`map` $\to$ `odom` $\to$ `base_link`) so our coordinates never confuse the robot. |
| **`sandraschi/gazebo-mcp`** | AI tool server for Gazebo | We ensure our catalog models match its schema so Dev 4 can dynamically spawn models later. |
| **`acutronicRobotics/gym-gazebo2`** | Fast robot benchmarking toolkit | We calibrate surface friction ($\mu = 0.8$ concrete, $\mu = 0.5$ dirt, $\mu = 0.15$ ice) in our world templates. |
| **`Pydantic v2`** | Python data validation library | Enforces strict data types for all coordinates. |
| **`Shapely 2.0`** | 2D geometry math library | Checks box collisions and adds padding around boxes in microseconds. |
| **`NumPy` & `Pillow` (PIL)** | Array math & image drawing | Draws the black-and-white navigation map (`map.pgm`) in under 2 seconds. |
| **`Jinja2`** | Fill-in-the-blank text generator | Generates clean Gazebo `.sdf` XML files without messy string formatting. |

---

## 6. My Action Checklist (How to Build My Part)

Follow these steps in order to complete your part:

- [ ] **Step 1:** Create `pyproject.toml` with project dependencies (`pydantic`, `shapely`, `jinja2`, `pillow`, `numpy`, `pytest`).
- [ ] **Step 2:** Write `sim2lang/core/models.py` with the Pydantic data schemas.
- [ ] **Step 3:** Write `sim2lang/core/config.py` for path handling.
- [ ] **Step 4:** Create `catalog/scene_catalog.json` and add pre-computed warehouse maps.
- [ ] **Step 5:** Write `sim2lang/world_gen/shapely_solver.py` (2D box placement + 2.2m aisle buffering).
- [ ] **Step 6:** Write `sim2lang/world_gen/map_slicer.py` (instant top-down PGM/YAML map drawing).
- [ ] **Step 7:** Write `sim2lang/compiler/sdf_compiler.py` and Jinja2 template for Gazebo Harmonic (SDF 1.9+ only, per Dev 3 v2 decision).
- [ ] **Step 8:** Run `pytest tests/test_world_gen.py` to prove that boxes never collide and maps are created in $<3.5\text{s}$.

When these 8 steps are done, **Part 1 is 100% complete**, and the other 3 developers can immediately plug their work into yours!
