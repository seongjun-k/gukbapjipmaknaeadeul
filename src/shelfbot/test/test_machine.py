"""StateMachine 순수 로직 테스트 (ROS 불필요)."""
import json
import os
import shutil
import tempfile

import pytest

from shelfbot.machine import State, StateMachine


@pytest.fixture
def log_dir():
    d = tempfile.mkdtemp()
    yield d
    shutil.rmtree(d, ignore_errors=True)


def test_normal_cycle(log_dir):
    m = StateMachine(log_dir=log_dir)
    assert m.start() is True
    assert m.state == State.NAVIGATING
    m.transition(State.DOCKING)
    m.transition(State.PLACING)
    m.transition(State.DONE)
    assert m.state == State.DONE
    assert set(m.step_times.keys()) == {'NAVIGATING', 'DOCKING', 'PLACING'}


def test_start_rejected_when_not_ready(log_dir):
    m = StateMachine(log_dir=log_dir)
    m.start()
    assert m.start() is False
    assert m.state == State.NAVIGATING


def test_retry_returns_to_failed_state(log_dir):
    m = StateMachine(log_dir=log_dir)
    m.start()
    m.transition(State.DOCKING)
    m.fail(State.DOCKING, 'marker_lost')
    assert m.state == State.FAILED
    assert m.fail_reason == 'marker_lost'

    target = m.retry()
    assert target == State.DOCKING
    assert m.state == State.DOCKING
    assert m.fail_reason is None
    assert m.failed_state is None


def test_retry_noop_when_not_failed(log_dir):
    m = StateMachine(log_dir=log_dir)
    assert m.retry() is None
    assert m.state == State.READY


def test_reset_from_done_and_failed(log_dir):
    m = StateMachine(log_dir=log_dir)
    m.start()
    m.transition(State.DOCKING)
    m.transition(State.PLACING)
    m.transition(State.DONE)
    assert m.reset() is True
    assert m.state == State.READY

    m2 = StateMachine(log_dir=log_dir)
    m2.start()
    m2.fail(State.NAVIGATING, 'nav_timeout')
    assert m2.reset() is True
    assert m2.state == State.READY


def test_reset_rejected_from_mid_cycle(log_dir):
    m = StateMachine(log_dir=log_dir)
    m.start()
    assert m.reset() is False
    assert m.state == State.NAVIGATING


def test_jsonl_written(log_dir):
    m = StateMachine(log_dir=log_dir)
    m.start()
    m.transition(State.DOCKING)

    files = os.listdir(log_dir)
    assert len(files) == 1
    with open(os.path.join(log_dir, files[0])) as f:
        lines = [json.loads(line) for line in f]
    assert lines[0]['state'] == 'NAVIGATING'
    assert lines[1]['state'] == 'DOCKING'
    assert 'ts' in lines[0]


def test_listener_called_on_transition(log_dir):
    m = StateMachine(log_dir=log_dir)
    events = []
    m.add_listener(lambda payload: events.append(payload))
    m.start()
    m.transition(State.DOCKING)
    assert len(events) == 2
    assert events[-1]['state'] == 'DOCKING'


def test_step_times_recorded(log_dir):
    m = StateMachine(log_dir=log_dir)
    m.start()
    m.transition(State.DOCKING)
    assert 'NAVIGATING' in m.step_times
    assert m.step_times['NAVIGATING'] >= 0
