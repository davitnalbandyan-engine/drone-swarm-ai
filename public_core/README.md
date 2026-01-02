# Public Core (Reduced)

This folder contains the **public core** of the Drone Swarm AI project.

It is intentionally reduced to demonstrate the **core engineering logic** while
keeping the full implementation available **by request**.

## What this core proves

- **Hierarchical mission control**: TRAVEL / SWEEP phases
- **Shared swarm goal** (`swarm_goal`) driven by **frontier utility**
- **Connectivity-aware coordination**:
  - detects fragmentation (largest connected component)
  - triggers regroup behavior when disconnected
  - suppresses risky local behaviors when near disconnect limits
- **Mission lifecycle**:
  - coverage progress estimation
  - mission completion detection
  - return-to-home transition
- **Minimal RViz visualization + metrics** for verification

## What is intentionally omitted

The following exist in the full implementation but are not included here:

- extended UI layers and polished visualization
- extra mission modes and experimental behaviors
- automated fault-injection / stress testing scripts
- integration-specific details (launch files, packaging variants, etc.)
- any additional tuning or “edge-case” parameters used during iteration

## Scope note

This is **simulation-only** coordination logic. It does not control hardware,
does not model physical safety, and does not provide collision avoidance.

For licensing and access to the full implementation, see the repository root:
- `LICENSING.md`
- `CONTACT.md`
