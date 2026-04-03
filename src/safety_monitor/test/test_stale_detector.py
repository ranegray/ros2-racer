import time
from safety_monitor.stale_detector import StaleDetector


def test_not_stale_before_first_message():
    """Topics are not stale until the first message arrives (avoids startup false positives)."""
    d = StaleDetector(['/goal_point'], timeout=0.1)
    assert d.check_stale() is None


def test_stale_after_timeout():
    """Topic becomes stale once timeout expires after first message."""
    d = StaleDetector(['/goal_point'], timeout=0.05)
    d.record_message('/goal_point')
    time.sleep(0.1)
    assert d.check_stale() == '/goal_point'


def test_not_stale_before_timeout():
    """Topic with a recent message is not stale."""
    d = StaleDetector(['/goal_point'], timeout=1.0)
    d.record_message('/goal_point')
    assert d.check_stale() is None


def test_reset_on_new_message():
    """Recording a new message resets the stale timer."""
    d = StaleDetector(['/goal_point'], timeout=0.05)
    d.record_message('/goal_point')
    time.sleep(0.1)
    d.record_message('/goal_point')
    assert d.check_stale() is None


def test_empty_topic_list_never_stale():
    """No watched topics means staleness check always returns None."""
    d = StaleDetector([], timeout=0.0)
    assert d.check_stale() is None


def test_multiple_topics_first_stale_returned():
    """Returns the first stale topic when multiple topics are watched."""
    d = StaleDetector(['/a', '/b'], timeout=0.05)
    d.record_message('/a')
    d.record_message('/b')
    time.sleep(0.1)
    result = d.check_stale()
    assert result == '/a'


def test_record_message_unknown_topic_is_noop():
    """Recording a message for an unknown topic does not raise."""
    d = StaleDetector(['/goal_point'], timeout=1.0)
    d.record_message('/unknown_topic')  # must not raise
    assert d.check_stale() is None
