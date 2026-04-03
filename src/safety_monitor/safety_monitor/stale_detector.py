import time


class StaleDetector:
    """Tracks last-received timestamps per topic and reports staleness."""

    def __init__(self, topics: list, timeout: float):
        self._timeout = timeout
        self._last_received: dict[str, float] = {}
        self._ever_received: dict[str, bool] = {}
        for topic in topics:
            self._last_received[topic] = 0.0
            self._ever_received[topic] = False

    def record_message(self, topic: str) -> None:
        """Call this in a topic subscription callback."""
        if topic not in self._last_received:
            return
        self._last_received[topic] = time.time()
        self._ever_received[topic] = True

    def check_stale(self) -> str | None:
        """Return the name of the first stale topic, or None if all are healthy."""
        now = time.time()
        for topic, last_time in self._last_received.items():
            if self._ever_received.get(topic) and (now - last_time) > self._timeout:
                return topic
        return None
