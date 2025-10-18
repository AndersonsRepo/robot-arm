# Issue: Review Robot Error Check Helpers (Non-Triangle)

## Summary
The recent expansion of `tests/roboterrorcheck.py` introduced math- and IO-focused
validation helpers beyond the existing triangle routines. We need a focused
review to make sure these additional helpers are correct, maintainable, and
aligned with control system expectations.

## Scope
- `tests/roboterrorcheck.py` (all sections except `codeError.Triangle`)
  - `math_validation.dh_forward_kinematics`
  - `math_validation.jacobian_geometric`
  - `math_validation.inverse_kinematics`
  - `math_validation.trajectory_generation`
  - `math_validation.arm_model_integrity`
  - `io_validation.imu_input_format`
  - `io_validation.command_channel_validation`
- `tests/test_run.py`
- `docs/roboterrorcheck_tests.md`

## Goals
- Verify formulas, tolerances, and fake-data assumptions in the math helpers.
- Ensure IO validators reflect actual IMU packet layouts and command schemas.
- Confirm the smoke script (`tests/test_run.py`) remains helpful and accurate
  for quick diagnostics.
- Keep documentation clear about intended usage and extension patterns.

## Acceptance Criteria
- Reviewer signs off on each scoped helper with notes for any follow-up tasks.
- Any discrepancies or redesign suggestions are captured in separate tickets.
- Triangle routines remain out of scope for this review.

## Suggested Reviewers
- Controls engineer familiar with FK/IK pipelines.
- Firmware or telemetry engineer for IMU/command validation semantics.

## References
- `tests/roboterrorcheck.py`
- `tests/test_run.py`
- `docs/roboterrorcheck_tests.md`
