# calc_drop_translation

Contains core trajectory calculation logic for drop points.

## Files
- `core_math.py`: contains physics simulation, wind models, and `run_preflight_simulation`.
- `MissionService.py`: mission management wrapper (if used for higher-level mission execution).

## Notes
- `run_preflight_simulation(cfg)` returns `(beacon_offset, bottle_offset)` and does not mutate config directly.
- Keep this folder mostly numeric/simulation code separated from MAVLink and I/O logic.
