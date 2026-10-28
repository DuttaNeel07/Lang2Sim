# Contributing to Lang2Sim

Thank you for your interest in contributing to Lang2Sim!
We welcome bug reports, bug fixes, documentation improvements, and new features.

---

## Community & Philosophy

Lang2Sim is developed to bridge natural language specifications and deterministic, executable robot simulations with stress testing and telemetry. We strive to maintain a welcoming, constructive, and engineering-rigorous environment.

- **Ask questions early:** Discuss architectural changes or design choices in an issue before writing code.
- **Keep PRs focused:** One concern or feature per PR makes reviews fast and effective.
- **Be considerate:** Maintain good faith, review thoroughly, and respect fellow contributors' time.

---

## Getting Started

### 1. Find or Open an Issue
Before diving into code, check the issue tracker to ensure someone isn't already tackling the task. For bug reports, please include environment details (OS, Docker version, GPU setup if applicable) and minimal steps to reproduce.

### 2. Fork and Clone
Fork the repository on GitHub and clone your fork locally:

```bash
git clone https://github.com/<your-username>/Lang2Sim.git
cd Lang2Sim
```

### 3. Set Up Your Environment
Python 3.10 or newer is required. Create and activate a virtual environment, then install the package in editable mode with development dependencies:

```bash
python -m venv .venv

# On Linux / macOS:
source .venv/bin/activate

# On Windows (PowerShell):
.\.venv\Scripts\Activate.ps1

# Install package with sim & dev extras
pip install -e ".[sim,dev]"
```

### 4. Install Pre-Commit Hooks
We use `pre-commit` to ensure code formatting, linting, and basic hygiene checks pass automatically:

```bash
pre-commit install
```

---

## Code Standards

### Formatting and Linting
We enforce formatting and linting via [Ruff](https://astral.sh/ruff).
Configurations live in `pyproject.toml`.

- **Format:**
  ```bash
  ruff format .
  ```
- **Lint:**
  ```bash
  ruff check .
  ```
  Auto-fixable rules can be fixed with:
  ```bash
  ruff check --fix .
  ```

### Typing and Docstrings
- Type annotations are required on public functions and core data models (`pydantic` schemas).
- Include clear docstrings explaining purpose, inputs, returns, and any side effects (e.g., launching Docker processes, writing to disk).

---

## Running Tests

Run the unit test suite before pushing changes:

```bash
pytest
```

> **Note on Docker integration tests:** Integration tests located in `tests/integration/` (which require a running Docker daemon and ROS 2 container) are skipped unless `L2S_INTEGRATION=1` is set.

---

## Commit Messages

We adhere to the [Conventional Commits](https://www.conventionalcommits.org/) specification:

```
<type>(<scope>): <short description in imperative mood>
```

Common types:
- `feat`: A new feature
- `fix`: A bug fix
- `docs`: Documentation changes
- `style`: Formatting, missing semicolons, whitespace (no production code change)
- `refactor`: Refactoring code without changing public behavior
- `test`: Adding or updating tests
- `chore`: Maintenance tasks, dependencies, CI configuration

---

## Pull Request Workflow

1. **Branch off `staging`:**
   The default integration branch for Lang2Sim is `staging`. Create your feature or bugfix branch from up-to-date `staging`:
   ```bash
   git checkout staging
   git pull upstream staging
   git checkout -b <type>/<short-description>
   ```

2. **Verify locally:**
   Ensure both pre-commit hooks and tests pass:
   ```bash
   pre-commit run --all-files
   pytest
   ```

3. **Open PR against `staging`:**
   Push your branch to your fork and submit a Pull Request targeting the `staging` branch of the upstream repository.

4. **Fill out the PR Template:**
   Complete the PR template checklist to help reviewers understand the context, changes, and testing performed.
