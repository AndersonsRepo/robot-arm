# Robot Error Check Test Helpers

## Directory Layout

```
tests/
└── roboterrorcheck.py
    ├── codeError
    │   └── Triangle
    │       ├── from_angles(angle1, angle2, angle3, length1, length2, length3)
    │       └── from_langeth(angle1, angle2, angle3, length1, length2, length3)
    ├── math_validation
    │   ├── dh_forward_kinematics
    │   │   ├── test_dh_mat_orthonormality(theta, alpha, r, d)
    │   │   ├── test_fk_home_pose_identity(joint_angles_home, dh_params_home)
    │   │   └── test_fk_random_joint_consistency(random_joint_sets, expected_transforms)
    │   ├── jacobian_geometric
    │   │   ├── test_jacobian_shape(model_definition, joint_angles)
    │   │   └── test_jacobian_finite_difference_agreement(model_definition, joint_angles, delta)
    │   ├── inverse_kinematics
    │   │   ├── test_log_so3_small_angles(rotation_matrix_small_angle)
    │   │   ├── test_ik_converges_reachable_pose(target_pose_reachable, initial_guess, ik_options)
    │   │   └── test_ik_handles_unreachable_target(target_pose_unreachable, initial_guess, ik_options)
    │   ├── trajectory_generation
    │   │   ├── test_cubic_time_scaling_endpoints(p0, pf, total_time, sample_times)
    │   │   └── test_cubic_time_scaling_invalid_times(p0, pf, total_time_invalid)
    │   └── arm_model_integrity
    │       ├── test_example_model_joint_count(example_model_definition)
    │       └── test_load_from_config_units(config_yaml, expected_units)
    └── io_validation
        ├── imu_input_format
        │   ├── test_imu_payload_structure(raw_packet_bytes, schema_definition)
        │   ├── test_imu_numeric_ranges(accel_vector, gyro_vector, timestamp)
        │   └── test_imu_missing_fields(raw_packet_bytes_incomplete)
        └── command_channel_validation
            ├── test_instruction_schema(controller_command_payload, command_schema)
            ├── test_instruction_value_ranges(joint_targets, velocity_limits, safety_limits)
            └── test_instruction_ordering(command_sequence, expected_sequence_rules)
```

## Using the Helpers

- **Standalone smoke run**: execute `python tests/test_run.py` to feed representative
  fake data into every helper and review quick PASS/FAIL flags.
- **Pytest integration**: run `pytest tests/roboterrorcheck.py` to execute the full
  validation suite inside the standard testing harness.
- **Embedding in other tools**: import the functions directly (for example,
  `from tests.roboterrorcheck import math_validation`) and feed real sensor or
  controller data to obtain 1/0 health indicators.
- **Extending coverage**: follow the existing namespace structure when adding new
  checks so high-level categories remain easy to discover.

The helpers intentionally accept primitive inputs so they can be re-used both in
unit tests and in lightweight command-line diagnostics.
