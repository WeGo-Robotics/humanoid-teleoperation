"""Device-neutral XR layer.

    XRSource                 the seam: tracking in, prompts out
      |- VuerXRSource        Quest browser + Vuer (ships today)
      '- NativeXRSource      XrLink WebSocket server (Quest app)

`codec` is the published wire format the Quest app is built against.

Nothing here decides whether it is safe to move. That lives in `teleop.safety`,
above this interface, so a new device implementation cannot opt out of it.
"""
from .source import XRSource
from .types import XRFrame

__all__ = ["XRFrame", "XRSource", "VuerXRSource", "NativeXRSource",
           "XrLinkServer", "LinkSnapshot"]


def __getattr__(name):
    # Lazy: VuerXRSource pulls in televuer/vuer and NativeXRSource pulls in
    # websockets. Importing teleop.xr should not require both to be installed.
    if name == "VuerXRSource":
        from .vuer_source import VuerXRSource
        return VuerXRSource
    if name == "NativeXRSource":
        from .native_source import NativeXRSource
        return NativeXRSource
    if name in ("XrLinkServer", "LinkSnapshot"):
        from . import link_server
        return getattr(link_server, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
