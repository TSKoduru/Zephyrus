"""The rules for entering Offboard after arming. No ROS needed."""
from skybeta_flight_interface.auto_offboard import (
    ARMED, ARMED_IN_POSITION, AutoOffboard, DONE, Inputs, READY, REQUESTING,
    WAIT_DISARMED)


def inputs(now=0.0, link=True, armed=False, in_position=False, landed=True,
           offboard=False, position_valid=True, setpoint_age=0.1, frozen=False):
    return Inputs(now, link, armed, in_position, landed, offboard,
                  position_valid, setpoint_age, frozen)


def ready():
    auto = AutoOffboard(True, 1.0)
    auto.step(inputs())
    assert auto.state == READY
    return auto


def armed():
    """Disarmed, then armed in Altitude: the next switch to Position is the go."""
    auto = ready()
    assert auto.step(inputs(armed=True))[0] is False
    assert auto.state == ARMED
    return auto


def go(auto=None, now=0.0, **kw):
    """The pilot's switch to Position."""
    return (auto or armed()).step(
        inputs(now=now, armed=True, in_position=True, **kw))


def requesting():
    auto = armed()
    assert go(auto)[0] is True
    assert auto.state == REQUESTING
    return auto


def test_arming_then_position_requests_offboard():
    request, message = go()
    assert request is True
    assert 'requesting' in message


def test_arming_alone_starts_nothing():
    auto = armed()
    for t in range(100):
        assert auto.step(inputs(now=float(t), armed=True)) == (False, None)
    assert auto.state == ARMED


def test_arming_already_in_position_is_the_pilot_flying():
    auto = ready()
    request, message = auto.step(inputs(armed=True, in_position=True))
    assert request is False
    assert 'no autonomy' in message
    assert auto.state == ARMED_IN_POSITION
    for t in range(100):
        assert auto.step(
            inputs(now=float(t), armed=True, in_position=True)) == (False, None)


def test_out_of_position_and_back_on_the_ground_is_the_switch():
    auto = ready()
    auto.step(inputs(armed=True, in_position=True))
    assert auto.step(inputs(now=1.0, armed=True)) == (False, None)
    assert auto.state == ARMED
    assert go(auto, now=2.0)[0] is True


def test_position_in_the_air_stays_with_the_pilot():
    # Flying by hand in Altitude, then flipping to Position to hold.
    auto = armed()
    auto.step(inputs(now=5.0, armed=True, landed=False))
    request, message = go(auto, now=6.0, landed=False)
    assert request is False
    assert 'in the air' in message
    assert auto.state == DONE


def test_landing_after_a_flip_in_the_air_does_not_start_it():
    auto = armed()
    go(auto, landed=False)
    auto.step(inputs(now=1.0, armed=True))
    assert go(auto, now=2.0)[0] is False
    assert auto.state == DONE


def test_landed_state_never_received():
    assert_refused('no landed state', landed=None)


def test_disabled_does_nothing():
    auto = AutoOffboard(False, 1.0)
    auto.step(inputs())
    auto.step(inputs(armed=True))
    assert go(auto) == (False, None)


def test_already_armed_at_startup_is_left_alone():
    # A flight interface restarted in flight must not seize the vehicle.
    auto = AutoOffboard(True, 1.0)
    for t in range(5):
        assert auto.step(inputs(now=float(t), armed=True)) == (False, None)
        assert go(auto, now=t + 0.5) == (False, None)
    assert auto.state == WAIT_DISARMED


def test_armed_during_a_link_dropout_is_left_alone():
    # The pilot may have armed and taken off while the link was down.
    auto = ready()
    assert auto.step(inputs(link=False)) == (False, None)
    assert auto.step(inputs(now=1.0, armed=True)) == (False, None)
    assert go(auto, now=2.0) == (False, None)
    assert auto.state == WAIT_DISARMED


def test_a_link_dropout_between_arming_and_the_switch_cancels_it():
    auto = armed()
    auto.step(inputs(now=1.0, armed=True, link=False))
    assert go(auto, now=2.0) == (False, None)
    assert auto.state == WAIT_DISARMED


def test_a_link_dropout_while_requesting_stops_it():
    auto = requesting()
    auto.step(inputs(now=0.1, armed=True, in_position=True, link=False))
    assert go(auto, now=0.6) == (False, None)


def assert_refused(text, **kw):
    auto = armed()
    request, message = go(auto, **kw)
    assert request is False
    assert text in message
    assert auto.state == DONE
    # and it does not get a second chance when the condition clears
    assert go(auto, now=1.0) == (False, None)


def test_no_valid_position():
    assert_refused('valid local position', position_valid=False)


def test_hold_in_force():
    assert_refused('hold', frozen=True)


def test_no_team_node_running():
    assert_refused('no team setpoint', setpoint_age=None)


def test_team_node_went_quiet():
    assert_refused('2.5 s old', setpoint_age=2.5)


def test_repeats_until_px4_answers():
    auto = armed()
    assert go(auto, now=0.0)[0] is True
    assert go(auto, now=0.1)[0] is False
    assert go(auto, now=0.5)[0] is True
    assert go(auto, now=0.6)[0] is False
    assert go(auto, now=1.0)[0] is True


def test_stops_once_in_offboard():
    auto = requesting()
    request, message = auto.step(inputs(now=0.5, armed=True, offboard=True))
    assert request is False
    assert 'active' in message
    assert auto.state == DONE


def test_gives_up_after_the_window():
    auto = requesting()
    request, message = go(auto, now=2.0)
    assert request is False
    assert 'did not enter' in message
    assert auto.state == DONE


def test_pilot_moving_the_switch_stops_the_request():
    auto = requesting()
    request, message = auto.step(inputs(now=0.1, armed=True))
    assert request is False
    assert 'mode changed' in message


def test_never_retakes_the_vehicle_from_the_pilot():
    auto = requesting()
    auto.step(inputs(now=0.2, armed=True, offboard=True))
    # Pilot takes over: Altitude, back to Position, for the rest of the flight.
    for t in range(100):
        assert auto.step(inputs(now=1.0 + t, armed=True))[0] is False
        assert go(auto, now=1.5 + t)[0] is False


def test_each_arming_gets_its_own_chance():
    auto = requesting()
    auto.step(inputs(now=1.0, armed=True, offboard=True))
    auto.step(inputs(now=2.0, armed=False))
    assert auto.state == READY
    auto.step(inputs(now=3.0, armed=True))
    assert go(auto, now=4.0)[0] is True
    assert auto.state == REQUESTING
