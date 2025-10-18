"""Quick smoke script that feeds fake data into roboterrorcheck helpers.

Run with ``python tests/test_run.py`` to see simple PASS/FAIL flags for the
sample payloads defined below.
"""

from __future__ import annotations

from typing import Callable, Sequence

from roboterrorcheck import codeError, io_validation, math_validation


class _FakePose:
    def __init__(self, error_value: float) -> None:
        self._error_value = error_value

    def error(self, _solution: Sequence[float]) -> float:
        return self._error_value


def _fake_solver_success(_target_pose: _FakePose, initial_guess: Sequence[float]) -> Sequence[float]:
    return list(initial_guess)


def _fake_solver_fail(
    _target_pose: _FakePose,
    _initial_guess: Sequence[float],
    *,
    max_iterations: int,
) -> dict[str, object]:
    return {"converged": False, "residual": float(max_iterations)}


def _fake_jacobian_provider(_joint_angles: Sequence[float]) -> list[list[float]]:
    return [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]


def _fake_fd_provider(_joint_angles: Sequence[float], _delta: float) -> list[list[float]]:
    return _fake_jacobian_provider(_joint_angles)


def _run(name: str, func: Callable[..., int], *args, **kwargs) -> tuple[str, str]:
    flag = func(*args, **kwargs)
    status = "PASS" if flag else "FAIL"
    return name, status


def main() -> None:
    checks: list[tuple[str, str]] = [
        _run(
            "triangle.from_angles",
            codeError.Triangle.from_angles,
            60.0,
            60.0,
            60.0,
            1.0,
            1.0,
            1.0,
        ),
        _run(
            "triangle.from_langeth",
            codeError.Triangle.from_langeth,
            90.0,
            45.0,
            45.0,
            3.0,
            4.0,
            5.0,
        ),
        _run(
            "dh_forward_kinematics.test_dh_mat_orthonormality",
            math_validation.dh_forward_kinematics.test_dh_mat_orthonormality,
            0.0,
            0.0,
            1.0,
            0.0,
        ),
        _run(
            "jacobian_geometric.test_jacobian_shape",
            math_validation.jacobian_geometric.test_jacobian_shape,
            _fake_jacobian_provider,
            [0.0, 0.0, 0.0],
        ),
        _run(
            "jacobian_geometric.test_jacobian_finite_difference_agreement",
            math_validation.jacobian_geometric.test_jacobian_finite_difference_agreement,
            _fake_jacobian_provider,
            _fake_fd_provider,
            [0.0, 0.0, 0.0],
            1e-3,
        ),
        _run(
            "inverse_kinematics.test_log_so3_small_angles",
            math_validation.inverse_kinematics.test_log_so3_small_angles,
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        ),
        _run(
            "inverse_kinematics.test_ik_converges_reachable_pose",
            math_validation.inverse_kinematics.test_ik_converges_reachable_pose,
            _fake_solver_success,
            _FakePose(0.01),
            [0.0, 0.0, 0.0],
            0.05,
        ),
        _run(
            "inverse_kinematics.test_ik_handles_unreachable_target",
            math_validation.inverse_kinematics.test_ik_handles_unreachable_target,
            _fake_solver_fail,
            _FakePose(10.0),
            [0.0, 0.0, 0.0],
            0.05,
            max_iterations=5,
        ),
        _run(
            "trajectory_generation.test_cubic_time_scaling_endpoints",
            math_validation.trajectory_generation.test_cubic_time_scaling_endpoints,
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
            2.0,
            [0.0, 1.0, 2.0],
        ),
        _run(
            "arm_model_integrity.test_example_model_joint_count",
            math_validation.arm_model_integrity.test_example_model_joint_count,
            {"joints": [1, 2, 3], "dh_params": [(), (), ()]},
        ),
        _run(
            "io_validation.imu_input_format.test_imu_payload_structure",
            io_validation.imu_input_format.test_imu_payload_structure,
            b"IMU\x00\x00\x00",
            {"length": 6, "header": b"IMU"},
        ),
        _run(
            "io_validation.command_channel_validation.test_instruction_schema",
            io_validation.command_channel_validation.test_instruction_schema,
            {"command": "move", "payload": {}},
            {"required": ["command", "payload"]},
        ),
    ]

    print("Test Run Summary:")
    for name, status in checks:
        print(f"- {name}: {status}")


if __name__ == "__main__":
    main()
