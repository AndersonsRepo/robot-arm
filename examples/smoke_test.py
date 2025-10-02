"""
Telemanipulator Smoke Test (v0)

Basic functionality test using NullServoDriver and example_model.
"""
import numpy as np
from telearm import example_model, NullServoDriver, MotionController

if __name__ == "__main__":
    model = example_model()
    driver = NullServoDriver(model.n())
    ctrl = MotionController(model, driver)
    ctrl.go_home()

    q0 = driver.angles()
    # target: move 5 cm in x, 4 cm in z
    T_goal = np.eye(4)
    T_goal[:3,3] = np.array([0.05, 0.0, 0.04])
    qf = ctrl.move_cartesian(q_start=q0, T_goal=T_goal, seconds=2.0, steps=40)
    print("Final q (rad):", qf)
    print("EE pose:\n", ctrl.kin.fk(qf))
