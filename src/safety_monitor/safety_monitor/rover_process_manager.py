import os
import signal
import subprocess
import threading
from typing import Optional


class RoverProcessManager:
    """Spawns and manages the rover_node OS process."""

    def __init__(self, connection_string: str, baud_rate: int):
        self._connection_string = connection_string
        self._baud_rate = int(baud_rate)
        self._proc: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()

    def spawn(self) -> None:
        """Start rover_node as a child subprocess in its own process group."""
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                raise RuntimeError('rover_node is already running; call kill() first')
            self._proc = subprocess.Popen([
                'ros2', 'run', 'robo_rover', 'rover_node',
                '--ros-args',
                '-p', f'connection_string:={self._connection_string}',
                '-p', f'baud_rate:={self._baud_rate}',
            ], start_new_session=True)

    def kill(self) -> None:
        """Stop rover_node cleanly. SIGINT sent to the entire process group
        (ros2 wrapper + rover_node child) triggers rover's shutdown handler
        which sends zero velocity and disarms. SIGKILL after 3s if needed.

        Safe to call concurrently: only one caller will perform the actual
        termination; subsequent callers see _proc=None and return immediately.
        """
        with self._lock:
            proc = self._proc
            self._proc = None
        if proc is None:
            return
        try:
            pgid = os.getpgid(proc.pid)
        except ProcessLookupError:
            return  # Already dead
        try:
            os.killpg(pgid, signal.SIGINT)  # kills ros2 wrapper + rover_node child
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait()

    def is_alive(self) -> bool:
        """Return True if the subprocess is currently running."""
        with self._lock:
            if self._proc is None:
                return False
            return self._proc.poll() is None
