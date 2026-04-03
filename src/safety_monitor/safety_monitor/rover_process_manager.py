import subprocess
from typing import Optional


class RoverProcessManager:
    """Spawns and manages the rover_node OS process."""

    def __init__(self, connection_string: str, baud_rate: int):
        self._connection_string = connection_string
        self._baud_rate = int(baud_rate)
        self._proc: Optional[subprocess.Popen] = None

    def spawn(self) -> None:
        """Start rover_node as a child subprocess."""
        if self._proc is not None and self._proc.poll() is None:
            raise RuntimeError('rover_node is already running; call kill() first')
        self._proc = subprocess.Popen([
            'ros2', 'run', 'robo_rover', 'rover_node',
            '--ros-args',
            '-p', f'connection_string:={self._connection_string}',
            '-p', f'baud_rate:={self._baud_rate}',
        ])

    def kill(self) -> None:
        """Terminate rover_node. SIGTERM first, SIGKILL after 2s if needed."""
        if self._proc is None:
            return
        self._proc.terminate()
        try:
            self._proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            self._proc.kill()
            self._proc.wait()
        self._proc = None

    def is_alive(self) -> bool:
        """Return True if the subprocess is currently running."""
        if self._proc is None:
            return False
        return self._proc.poll() is None
