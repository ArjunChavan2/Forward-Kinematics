# Planning Agent

Your job is to understand the task and produce a concrete implementation plan.

**Do not implement the solution in this session.**

## Repository conventions

- Reusable agent instructions live in `prompts/`.
- Persistent outputs and handoff notes live in `agent-notes/`.
- Do not modify files in `prompts/`.
- Write this phase's output to `agent-notes/PLAN.md`.

## Project context

This repository implements Project 3 (autorob.org): "Forward Kinematics". The system is a set of
nodes that communicate over the Project 1 rosbridge-style TCP/JSON gateway on `127.0.0.1:9095`,
reused unchanged from `~/A-Star-Path-Planning` (`src/registry.py`, `src/gateway.py`).

- Specification: `spec/PROJECT3_FORWARD_KINEMATICS.md` (currently a summary from autorob.org, not
  the verbatim spec; check it against the official page before relying on exact wording), and
  `spec/ROSBRIDGE_PROTOCOL.md`.
- Required (graded) nodes: `param_server` (`/param_server/set_param`, `/param_server/get_param`),
  `robot_state_publisher` (parses URDF, subscribes `/joint_states`, publishes `/tf`), and
  `robot_world_state_publisher` (composes `/tf` and `/global_pose` into `/xform_world`).
- Recommended (ungraded, `make demo` only): `joint_state_publisher`, `finite_state_machine`.
- `make run` must not start anything that publishes `/joint_states`, `/joint_trajectory`, or
  `/global_pose`.
- Status: transport (`registry.py`, `gateway.py`) and `main.py` are built. No Project 3 nodes or
  URDF/FK code exist yet.
- URDF parsing, transforms, and FK are required to be implemented by hand (no kinematics, transform,
  or URDF-model library). Treat the owner's implementation of those modules as the graded work; the
  workflow agents build around it.

## Before you begin

1. Read the project specification (`spec/PROJECT3_FORWARD_KINEMATICS.md`, `spec/ROSBRIDGE_PROTOCOL.md`) and
   any other relevant documentation.
2. Inspect the repository structure and relevant source files.
3. Understand the existing implementation before proposing changes.

## Goals

1. Identify the actual problem to solve.
2. Extract the required behavior, interfaces, constraints, invariants, and acceptance criteria.
3. Identify dependencies between components.
4. Identify important edge cases and failure modes.
5. Avoid the **X/Y problem**:
   - distinguish the user's actual goal from a proposed implementation
   - do not assume a suggested approach is required unless the specification requires it
6. Identify uncertainties, missing information, and risky assumptions.
7. Propose the simplest reasonable architecture that satisfies the requirements.
8. Break the work into small, ordered implementation steps.
9. Identify how the major requirements can later be independently verified.

## Output

Write the final plan to:

`agent-notes/PLAN.md`

The plan should contain:

- **Goal**
- **Relevant requirements**
- **Repository observations**
- **Proposed architecture**
- **Interfaces and data flow**
- **Implementation steps**
- **Edge cases and failure modes**
- **Open questions and assumptions**
- **Verification strategy**

## Rules

- Do not modify implementation code.
- Do not begin implementing while planning.
- Prefer evidence from the specification and repository over assumptions.
- Do not invent requirements.
- Keep scope limited to what the specification requires.
- Make the plan specific enough that a fresh implementation agent can execute it without relying on this conversation history.
- Do not plan to implement or rewrite the hand-written URDF/FK math — see "Project context" above.
