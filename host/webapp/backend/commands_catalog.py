"""
/api/commands from protobuf descriptors + host_options.
"""
from __future__ import annotations

from typing import Any, List, Optional, Tuple

import protocol_ids as protocol_ids_mod
import protocol_pb2  # noqa: F401 — register descriptors
import host_options_pb2
from google.protobuf.descriptor_pool import Default
from google.protobuf.message_factory import GetMessageClass
from google.protobuf import message


def _ui_module_for_message(desc) -> int:
    opts = desc.GetOptions()
    if opts.HasExtension(host_options_pb2.host_ui_module):
        return opts.Extensions[host_options_pb2.host_ui_module]
    file_opts = desc.file.GetOptions()
    if file_opts.HasExtension(host_options_pb2.default_host_ui_module):
        return file_opts.Extensions[host_options_pb2.default_host_ui_module]
    return host_options_pb2.HOST_UI_MODULE_UNSPECIFIED


def _module_label(module_enum: int) -> str:
    name = host_options_pb2.HostUiModule.Name(module_enum)
    return name.replace("HOST_UI_MODULE_", "").replace("_", " ").title()


def _module_order_labels() -> List[str]:
    et = host_options_pb2.DESCRIPTOR.enum_types_by_name["HostUiModule"]
    pairs = [(v.number, _module_label(v.number)) for v in et.values if v.number != 0]
    pairs.sort(key=lambda x: x[0])
    return [p[1] for p in pairs]


def _field_to_arg(field_descriptor) -> dict[str, Any]:
    fd = field_descriptor
    if fd.is_repeated:
        return {"name": fd.name, "type": "repeated", "elem": _scalar_type_name(fd)}
    if fd.type == fd.TYPE_ENUM:
        choices = list(fd.enum_type.values_by_name.keys())
        return {"name": fd.name, "type": "choice", "choices": choices}
    if fd.type == fd.TYPE_BOOL:
        return {"name": fd.name, "type": "choice", "choices": ["false", "true"]}
    return {"name": fd.name, "type": _scalar_type_name(fd)}


def _scalar_type_name(fd) -> str:
    if fd.type == fd.TYPE_FLOAT or fd.type == fd.TYPE_DOUBLE:
        return "float"
    if fd.type == fd.TYPE_STRING or fd.type == fd.TYPE_BYTES:
        return "string"
    return "uint32"


def _parse_scalar(fd, val: str) -> Any:
    if fd.type == fd.TYPE_ENUM:
        return fd.enum_type.values_by_name[val].number
    if fd.type == fd.TYPE_BOOL:
        return val.lower() in ("1", "true", "yes", "on")
    if fd.type == fd.TYPE_FLOAT or fd.type == fd.TYPE_DOUBLE:
        return float(val)
    if fd.type == fd.TYPE_STRING:
        return val
    if fd.type == fd.TYPE_BYTES:
        return val.encode("utf-8", errors="replace")
    return int(val, 0)


def build_request(command_name: str, arg_strings: List[str]) -> message.Message:
    pool = Default()
    try:
        desc = pool.FindMessageTypeByName(command_name)
    except KeyError as e:
        raise KeyError(command_name) from e
    cls = GetMessageClass(desc)
    msg = cls()
    fields = sorted(msg.DESCRIPTOR.fields, key=lambda f: f.number)
    idx = 0
    for fd in fields:
        if idx >= len(arg_strings):
            break
        s = arg_strings[idx]
        if fd.is_repeated:
            parts = [p.strip() for p in s.split(",") if p.strip()]
            for p in parts:
                getattr(msg, fd.name).append(_parse_scalar(fd, p))
            idx += 1
        else:
            setattr(msg, fd.name, _parse_scalar(fd, s))
            idx += 1
    return msg


def get_commands_catalog() -> Tuple[List[dict[str, Any]], List[str]]:
    commands: List[dict[str, Any]] = []
    pool = Default()
    for name in protocol_ids_mod.MSG_NAMES:
        if not name.endswith("Request"):
            continue
        try:
            desc = pool.FindMessageTypeByName(name)
        except KeyError:
            continue
        opts = desc.GetOptions()
        if not opts.HasExtension(host_options_pb2.host_message_type):
            continue
        mt = opts.Extensions[host_options_pb2.host_message_type]
        if mt != host_options_pb2.HOST_MESSAGE_TYPE_REQUEST:
            continue
        mod = _ui_module_for_message(desc)
        args = [_field_to_arg(f) for f in sorted(desc.fields, key=lambda x: x.number)]
        desc_txt = name.replace("Request", "")
        commands.append(
            {
                "command": name,
                "description": desc_txt,
                "module": _module_label(mod),
                "args": args,
            }
        )
    module_order = _module_order_labels()
    return commands, module_order
