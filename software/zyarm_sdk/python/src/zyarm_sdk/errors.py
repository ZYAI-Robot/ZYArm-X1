class ZyArmError(Exception):
    """Base error for the ZYArm SDK."""


class ProtocolError(ZyArmError):
    """Raised when a firmware line or command cannot be parsed or formatted."""


class TransportError(ZyArmError):
    """Raised when serial transport cannot read or write as requested."""


class ZyArmTimeoutError(ZyArmError):
    """Raised when an explicit wait reaches its timeout."""


class StaleStateError(ZyArmError):
    """Raised when cached state/action is older than the requested freshness."""


class SafetyError(ZyArmError):
    """Raised before writing an invalid or unsafe command to the arm."""
