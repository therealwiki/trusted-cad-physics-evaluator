import pystark


def test_angular_velocity_control_setters_are_named_and_callable():
    methods = dir(pystark.RBCAngularVelocityHandler)
    assert "set_target_angular_velocity_in_rad_per_s" in methods
    assert "set_max_torque" in methods
    assert "set_delay" in methods
