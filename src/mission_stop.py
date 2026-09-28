"""Expected operational stop with a message suitable for the run summary."""


class MissionStop(Exception):
    """Stop a mission safely when a required sensor or robot state is unusable."""
