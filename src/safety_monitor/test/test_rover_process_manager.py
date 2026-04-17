import subprocess
from unittest.mock import MagicMock, patch
from safety_monitor.rover_process_manager import RoverProcessManager


def test_is_alive_false_before_spawn():
    mgr = RoverProcessManager('/dev/ttyACM1', 115200)
    assert mgr.is_alive() is False


def test_spawn_calls_popen_with_ros2_run():
    with patch('safety_monitor.rover_process_manager.subprocess.Popen') as mock_popen:
        mock_popen.return_value = MagicMock()
        mgr = RoverProcessManager('/dev/ttyACM1', 115200)
        mgr.spawn()
        args = mock_popen.call_args[0][0]
        assert args[0] == 'ros2'
        assert 'rover_node' in args
        assert any('connection_string:=/dev/ttyACM1' in a for a in args)
        assert any('baud_rate:=115200' in a for a in args)


def test_is_alive_true_when_process_running():
    with patch('safety_monitor.rover_process_manager.subprocess.Popen') as mock_popen:
        mock_proc = MagicMock()
        mock_proc.poll.return_value = None  # still running
        mock_popen.return_value = mock_proc
        mgr = RoverProcessManager('/dev/ttyACM1', 115200)
        mgr.spawn()
        assert mgr.is_alive() is True


def test_is_alive_false_when_process_exited():
    with patch('safety_monitor.rover_process_manager.subprocess.Popen') as mock_popen:
        mock_proc = MagicMock()
        mock_proc.poll.return_value = 1  # exited
        mock_popen.return_value = mock_proc
        mgr = RoverProcessManager('/dev/ttyACM1', 115200)
        mgr.spawn()
        assert mgr.is_alive() is False


def test_kill_calls_terminate_then_waits():
    with patch('safety_monitor.rover_process_manager.subprocess.Popen') as mock_popen:
        mock_proc = MagicMock()
        mock_proc.poll.return_value = None
        mock_proc.wait.return_value = 0
        mock_popen.return_value = mock_proc
        mgr = RoverProcessManager('/dev/ttyACM1', 115200)
        mgr.spawn()
        mgr.kill()
        mock_proc.terminate.assert_called_once()
        mock_proc.wait.assert_called_once_with(timeout=2.0)


def test_kill_sigkills_on_timeout():
    with patch('safety_monitor.rover_process_manager.subprocess.Popen') as mock_popen:
        mock_proc = MagicMock()
        mock_proc.poll.return_value = None
        # First wait() raises TimeoutExpired, second wait() (after kill) succeeds
        mock_proc.wait.side_effect = [
            subprocess.TimeoutExpired(cmd='ros2', timeout=2.0),
            None,
        ]
        mock_popen.return_value = mock_proc
        mgr = RoverProcessManager('/dev/ttyACM1', 115200)
        mgr.spawn()
        mgr.kill()
        mock_proc.kill.assert_called_once()


def test_kill_noop_when_not_spawned():
    mgr = RoverProcessManager('/dev/ttyACM1', 115200)
    mgr.kill()  # must not raise


def test_is_alive_false_after_kill():
    with patch('safety_monitor.rover_process_manager.subprocess.Popen') as mock_popen:
        mock_proc = MagicMock()
        mock_proc.poll.return_value = None
        mock_proc.wait.return_value = 0
        mock_popen.return_value = mock_proc
        mgr = RoverProcessManager('/dev/ttyACM1', 115200)
        mgr.spawn()
        mgr.kill()
        assert mgr.is_alive() is False


def test_spawn_raises_if_already_running():
    with patch('safety_monitor.rover_process_manager.subprocess.Popen') as mock_popen:
        mock_proc = MagicMock()
        mock_proc.poll.return_value = None  # still running
        mock_popen.return_value = mock_proc
        mgr = RoverProcessManager('/dev/ttyACM1', 115200)
        mgr.spawn()
        import pytest
        with pytest.raises(RuntimeError):
            mgr.spawn()
