"""Package-specific errors."""


class OptionalDependencyError(RuntimeError):
    """Report that a requested optional RL feature is unavailable."""
