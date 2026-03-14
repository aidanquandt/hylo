"""
Host UI module labels and helpers for protocol commands.
Maps HostUiModule enum (from host_options.proto) to display strings.
"""
from __future__ import annotations

import logging

# Optional: only needed when reading host_ui_module option from descriptors
_host_options_pb2 = None
try:
    import host_options_pb2 as _host_options_pb2  # noqa: E402  # from generated/protocol/python
except ImportError:
    pass

# Display labels for each HostUiModule enum value (order matches enum order for module_order).
# Add new modules here when adding a new enum value in host_options.proto.
MODULE_LABELS = {
    0: "Other",   # HOST_UI_MODULE_UNSPECIFIED
    1: "Common",
    2: "System",
    3: "Beacon",
    4: "UWB node config",
    5: "IMU",
    6: "UWB node",
    7: "UWB",
    8: "Error",
    9: "Datalogger",
    10: "TWR",
    11: "TWR manager",
    12: "Stopwatch",
    13: "OTA config",
    14: "Sensor fusion",
}


def get_module_label(enum_value: int) -> str:
    """Return display label for a HostUiModule enum value (int)."""
    return MODULE_LABELS.get(enum_value, "Other")


def get_module_for_message(message_name: str, descriptor_message_types):
    """
    Return (module_label, enum_value) for a request message name.
    descriptor_message_types: pb2.DESCRIPTOR.message_types_by_name.
    Returns ("Other", 0) if option is unset or host_options not available.
    """
    if _host_options_pb2 is None:
        return "Other", 0
    desc = descriptor_message_types.get(message_name)
    if desc is None:
        return "Other", 0
    try:
        opts = desc.GetOptions()
        ext = _host_options_pb2.host_ui_module
        try:
            val = opts.Extensions[ext]
        except (KeyError, TypeError):
            return "Other", 0
        if hasattr(val, "number"):
            val = val.number
        val = int(val)
        label = get_module_label(val)
        if val == 0:
            logging.getLogger(__name__).warning(
                "Request message %s has no host_ui_module option (UNSPECIFIED)", message_name
            )
        return label, val
    except Exception:
        return "Other", 0


def get_module_order() -> list[str]:
    """Return list of module labels in enum order (for API module_order)."""
    return [MODULE_LABELS.get(i, "Other") for i in range(max(MODULE_LABELS.keys()) + 1)]
