"""Helpers for selecting one live sensor from several conventional topics."""


def should_accept_source(
    active_source: str,
    candidate_source: str,
    active_age: float,
    switch_timeout: float,
) -> bool:
    """Accept the active source, or fail over after it becomes stale."""
    return (
        not active_source
        or candidate_source == active_source
        or active_age > max(0.0, switch_timeout)
    )
