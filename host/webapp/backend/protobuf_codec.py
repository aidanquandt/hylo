"""
Wire message map, protobuf parse/format, host_options classification (viz / stream / log).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

import protocol_ids as protocol_ids_mod
import protocol_pb2  # noqa: F401 — register generated descriptors in default pool
import host_options_pb2
from google.protobuf.descriptor_pool import Default
from google.protobuf.message_factory import GetMessageClass
from google.protobuf.message import Message


def _build_wire_maps():
    pool = Default()
    names = protocol_ids_mod.MSG_NAMES
    name_to_id: dict[str, int] = {}
    id_to_cls: dict[int, type] = {}
    id_to_msg_type: dict[int, int] = {}
    request_to_response: dict[str, Optional[str]] = {}
    all_names = set(names)

    for i, name in enumerate(names):
        try:
            desc = pool.FindMessageTypeByName(name)
        except KeyError:
            continue
        cls = GetMessageClass(desc)
        name_to_id[name] = i
        id_to_cls[i] = cls
        opts = desc.GetOptions()
        if opts.HasExtension(host_options_pb2.host_message_type):
            id_to_msg_type[i] = opts.Extensions[host_options_pb2.host_message_type]

    for name in names:
        if not name.endswith("Request"):
            continue
        stem = name[: -len("Request")]
        resp = stem + "Response"
        request_to_response[name] = resp if resp in all_names else None

    return name_to_id, id_to_cls, id_to_msg_type, request_to_response


NAME_TO_ID, ID_TO_CLS, ID_TO_MSG_TYPE, REQUEST_TO_RESPONSE = _build_wire_maps()


@dataclass
class ParsedFrame:
    msg_id: int
    name: str
    message: Optional[Message]


class ProtobufCodec:
    def id_from_name(self, name: str) -> int:
        if name not in NAME_TO_ID:
            raise KeyError(f"unknown wire message: {name}")
        return NAME_TO_ID[name]

    def parse(self, msg_id: int, payload: bytes) -> ParsedFrame:
        cls = ID_TO_CLS.get(msg_id)
        if cls is None:
            return ParsedFrame(msg_id, f"UNKNOWN_{msg_id}", None)
        msg = cls()
        msg.ParseFromString(payload)
        return ParsedFrame(msg_id, cls.DESCRIPTOR.name, msg)

    def serialize(self, message: Message) -> tuple[int, bytes]:
        name = message.DESCRIPTOR.name
        mid = NAME_TO_ID.get(name)
        if mid is None:
            raise KeyError(f"message not on wire: {name}")
        return mid, message.SerializeToString()

    def message_type(self, msg_id: int) -> int:
        return ID_TO_MSG_TYPE.get(msg_id, host_options_pb2.HOST_MESSAGE_TYPE_UNSPECIFIED)

    def is_stream(self, msg_id: int) -> bool:
        return self.message_type(msg_id) == host_options_pb2.HOST_MESSAGE_TYPE_STREAM

    def is_event(self, msg_id: int) -> bool:
        return self.message_type(msg_id) == host_options_pb2.HOST_MESSAGE_TYPE_EVENT

    def expected_response_name(self, request_name: str) -> Optional[str]:
        return REQUEST_TO_RESPONSE.get(request_name)

    def matches_command_response(self, request_name: str, resp_msg_id: int) -> bool:
        exp = self.expected_response_name(request_name)
        if exp is not None and NAME_TO_ID.get(exp) == resp_msg_id:
            return True
        if resp_msg_id == protocol_ids_mod.MSG_ID_AckResponse:
            return True
        return False

    def format_log_line(self, pf: ParsedFrame) -> Optional[str]:
        if self.is_stream(pf.msg_id):
            return None
        if pf.message is None:
            return f"[rx] {pf.name} (parse failed or unmapped id)"
        return f"[rx] {pf.name} {pf.message}"

    def to_viz_payload(self, pf: ParsedFrame) -> Optional[dict[str, Any]]:
        if pf.name != "TwrMgrRangeEvent" or pf.message is None:
            return None
        m = pf.message
        return {
            "addr": m.addr,
            "distance_m": m.distance_m,
            "x": m.x,
            "y": m.y,
            "z": m.z,
            "position_unknown": m.position_unknown,
        }

    def to_fusion_payload(self, pf: ParsedFrame) -> Optional[dict[str, Any]]:
        if pf.message is None:
            return None
        if pf.name not in (
            "SensorFusionGetStatusResponse",
            "SensorFusionStreamPayload",
        ):
            return None
        m = pf.message
        return {
            "active": m.active,
            "confidence": m.confidence,
            "x": m.pos_x,
            "y": m.pos_y,
            "z": m.pos_z,
            "yaw": getattr(m, "yaw_rad", 0.0),
        }

codec = ProtobufCodec()
