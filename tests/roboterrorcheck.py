"""Human-readable validation helpers for robot diagnostics.

Each helper mirrors the outline we discussed previously. Every routine
returns ``1`` for a pass condition and ``0`` otherwise so existing call
sites that expect integer flags continue to work without modification.
"""

from __future__ import annotations

from math import cos, sin, isclose
from numbers import Number
from typing import Iterable, Sequence


# ---------------------------------------------------------------------------
# Shared utilities
# ---------------------------------------------------------------------------

def _are_numbers(values: Iterable[object]) -> bool:
    """Return ``True`` when *values* only contains numeric entries."""

    return all(isinstance(value, Number) for value in values)


def _triangle_inequality(lengths: Sequence[float]) -> bool:
    a, b, c = lengths
    return a + b > c and a + c > b and b + c > a


def _matrix_close(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]], tol: float) -> bool:
    for row_a, row_b in zip(a, b):
        for value_a, value_b in zip(row_a, row_b):
            if not isclose(value_a, value_b, abs_tol=tol, rel_tol=tol):
                return False
    return True


def _identity_4x4() -> list[list[float]]:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _dh_matrix(theta: float, alpha: float, r: float, d: float) -> list[list[float]]:
    return [
        [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), r * cos(theta)],
        [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), r * sin(theta)],
        [0.0, sin(alpha), cos(alpha), d],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _matmul(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]) -> list[list[float]]:
    result: list[list[float]] = []
    for row in a:
        result_row: list[float] = []
        for column in zip(*b):
            result_row.append(sum(value * weight for value, weight in zip(row, column)))
        result.append(result_row)
    return result


# ---------------------------------------------------------------------------
# codeError namespace
# ---------------------------------------------------------------------------


class codeError:
    class Triangle:
        """Triangle checks based on either angles or side lengths."""

        @staticmethod
        def from_angles(
            angle1: float,
            angle2: float,
            angle3: float,
            length1: float,
            length2: float,
            length3: float,
        ) -> int:
            angles = (angle1, angle2, angle3)
            lengths = (length1, length2, length3)

            if not _are_numbers(angles + lengths):
                return 0
            if any(value <= 0 for value in angles + lengths):
                return 0
            if not isclose(sum(angles), 180.0, abs_tol=1e-3):
                return 0
            if not _triangle_inequality(lengths):
                return 0
            return 1

        @staticmethod
        def from_langeth(
            angle1: float,
            angle2: float,
            angle3: float,
            length1: float,
            length2: float,
            length3: float,
        ) -> int:
            angles = (angle1, angle2, angle3)
            lengths = (length1, length2, length3)

            if not _are_numbers(angles + lengths):
                return 0
            if any(value <= 0 for value in lengths):
                return 0
            if not _triangle_inequality(lengths):
                return 0
            return 1


# ---------------------------------------------------------------------------
# math_validation namespace
# ---------------------------------------------------------------------------


class math_validation:
    class dh_forward_kinematics:
        @staticmethod
        def test_dh_mat_orthonormality(theta: float, alpha: float, r: float, d: float) -> int:
            matrix = _dh_matrix(theta, alpha, r, d)
            rotation = [row[:3] for row in matrix[:3]]

            dot_products = [
                sum(rotation[0][i] * rotation[1][i] for i in range(3)),
                sum(rotation[0][i] * rotation[2][i] for i in range(3)),
                sum(rotation[1][i] * rotation[2][i] for i in range(3)),
            ]
            norms = [sum(value * value for value in row) for row in rotation]

            is_orthogonal = all(isclose(value, 0.0, abs_tol=1e-5) for value in dot_products)
            has_unit_length = all(isclose(norm, 1.0, abs_tol=1e-5) for norm in norms)
            return int(is_orthogonal and has_unit_length)

        @staticmethod
        def test_fk_home_pose_identity(
            joint_angles_home: Sequence[float],
            dh_params_home: Sequence[tuple[float, float, float, float]],
        ) -> int:
            if len(joint_angles_home) != len(dh_params_home):
                return 0

            transform = _identity_4x4()
            for joint_angle, (theta, alpha, r, d) in zip(joint_angles_home, dh_params_home):
                transform = _matmul(transform, _dh_matrix(theta + joint_angle, alpha, r, d))

            return int(_matrix_close(transform, _identity_4x4(), tol=1e-4))

        @staticmethod
        def test_fk_random_joint_consistency(
            joint_sets: Iterable[Sequence[float]],
            expected_transforms: Iterable[Sequence[Sequence[float]]],
        ) -> int:
            for joints, expected in zip(joint_sets, expected_transforms):
                transform = _identity_4x4()
                for joint in joints:
                    transform = _matmul(transform, _dh_matrix(joint, 0.0, 1.0, 0.0))
                if not _matrix_close(transform, expected, tol=1e-3):
                    return 0
            return 1

    class jacobian_geometric:
        @staticmethod
        def test_jacobian_shape(jacobian_provider, joint_angles: Sequence[float]) -> int:
            jacobian = jacobian_provider(joint_angles)
            rows = len(jacobian)
            cols = len(jacobian[0]) if rows else 0
            return int(rows == 6 and cols == len(joint_angles))

        @staticmethod
        def test_jacobian_finite_difference_agreement(
            analytic_provider,
            finite_difference_provider,
            joint_angles: Sequence[float],
            delta: float,
        ) -> int:
            analytic = analytic_provider(joint_angles)
            numeric = finite_difference_provider(joint_angles, delta)

            for row_analytic, row_numeric in zip(analytic, numeric):
                for value_a, value_b in zip(row_analytic, row_numeric):
                    if not isclose(value_a, value_b, rel_tol=1e-3, abs_tol=1e-4):
                        return 0
            return 1

    class inverse_kinematics:
        @staticmethod
        def test_log_so3_small_angles(rotation_matrix_small_angle: Sequence[Sequence[float]]) -> int:
            return int(_matrix_close(rotation_matrix_small_angle, _identity_4x4(), tol=1e-4))

        @staticmethod
        def test_ik_converges_reachable_pose(
            solver,
            target_pose,
            initial_guess: Sequence[float],
            tolerance: float,
        ) -> int:
            solution = solver(target_pose, initial_guess)
            error = target_pose.error(solution)
            return int(error <= tolerance)

        @staticmethod
        def test_ik_handles_unreachable_target(
            solver,
            target_pose,
            initial_guess: Sequence[float],
            tolerance: float,
            max_iterations: int,
        ) -> int:
            result = solver(target_pose, initial_guess, max_iterations=max_iterations)
            did_fail = result.get("converged") is False
            residual = result.get("residual", float("inf"))
            return int(did_fail and residual > tolerance)

    class trajectory_generation:
        @staticmethod
        def test_cubic_time_scaling_endpoints(
            p0: Sequence[float],
            pf: Sequence[float],
            total_time: float,
            sample_times: Sequence[float],
        ) -> int:
            if total_time <= 0:
                return 0
            if any(time < 0 or time > total_time for time in sample_times):
                return 0

            for start, end in zip(p0, pf):
                values = [_cubic_time_scaling(start, end, total_time, time) for time in sample_times]
                if not isclose(values[0], start, abs_tol=1e-6):
                    return 0
                if not isclose(values[-1], end, abs_tol=1e-6):
                    return 0
            return 1

        @staticmethod
        def test_cubic_time_scaling_invalid_times(
            p0: Sequence[float],
            pf: Sequence[float],
            total_time: float,
        ) -> int:
            try:
                _cubic_time_scaling(p0[0], pf[0], total_time, 0.0)
            except ValueError:
                return 1
            return 0

    class arm_model_integrity:
        @staticmethod
        def test_example_model_joint_count(example_model_definition: dict) -> int:
            joints = example_model_definition.get("joints", [])
            dh_params = example_model_definition.get("dh_params", [])
            return int(len(joints) == len(dh_params) and len(joints) > 0)

        @staticmethod
        def test_load_from_config_units(config_yaml: dict, expected_units: str) -> int:
            return int(config_yaml.get("units") == expected_units)


# ---------------------------------------------------------------------------
# io_validation namespace
# ---------------------------------------------------------------------------


class io_validation:
    class imu_input_format:
        @staticmethod
        def test_imu_payload_structure(raw_packet_bytes: bytes, schema_definition: dict) -> int:
            expected_length = schema_definition.get("length")
            header = schema_definition.get("header")
            if expected_length is None or header is None:
                return 0
            has_length = len(raw_packet_bytes) == expected_length
            has_header = raw_packet_bytes.startswith(header)
            return int(has_length and has_header)

        @staticmethod
        def test_imu_numeric_ranges(
            accel_vector: Sequence[float],
            gyro_vector: Sequence[float],
            timestamp: float,
        ) -> int:
            if timestamp < 0:
                return 0
            accel_ok = all(-100.0 <= value <= 100.0 for value in accel_vector)
            gyro_ok = all(-2000.0 <= value <= 2000.0 for value in gyro_vector)
            return int(accel_ok and gyro_ok)

        @staticmethod
        def test_imu_missing_fields(raw_packet_bytes_incomplete: bytes) -> int:
            return int(len(raw_packet_bytes_incomplete) < 10)

    class command_channel_validation:
        @staticmethod
        def test_instruction_schema(controller_command_payload: dict, command_schema: dict) -> int:
            required = command_schema.get("required", [])
            return int(set(required).issubset(controller_command_payload))

        @staticmethod
        def test_instruction_value_ranges(
            joint_targets: Sequence[float],
            velocity_limits: Sequence[float],
            safety_limits: Sequence[float],
        ) -> int:
            for target, velocity_limit, safety_limit in zip(joint_targets, velocity_limits, safety_limits):
                if abs(target) > safety_limit:
                    return 0
                if abs(target) > velocity_limit:
                    return 0
            return 1

        @staticmethod
        def test_instruction_ordering(
            command_sequence: Sequence[str],
            expected_sequence_rules: Sequence[str],
        ) -> int:
            rules = {name: index for index, name in enumerate(expected_sequence_rules)}
            last_index = -1
            for name in command_sequence:
                index = rules.get(name)
                if index is None or index < last_index:
                    return 0
                last_index = index
            return 1


# ---------------------------------------------------------------------------
# Small helpers for the trajectory checks
# ---------------------------------------------------------------------------


def _cubic_time_scaling(p0: float, pf: float, total_time: float, time: float) -> float:
    if total_time <= 0:
        raise ValueError("total_time must be positive")
    if time < 0 or time > total_time:
        raise ValueError("time must be within [0, total_time]")

    s = time / total_time
    blend = 3 * s ** 2 - 2 * s ** 3
    return p0 + (pf - p0) * blend

