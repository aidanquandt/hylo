"""
Format protobuf responses for the web UI.

Proto3 text / str() omits scalar fields at default values (e.g. distance_m: 0,
valid: false), which makes results like TwrGetResultResponse look wrong when only
responder_addr is non-zero.
"""
from __future__ import annotations

import json
from typing import Any

from google.protobuf.json_format import MessageToDict
from google.protobuf.message import Message


def format_command_response(msg: Message) -> str:
    d: dict[str, Any] = MessageToDict(
        msg,
        always_print_fields_with_no_presence=True,
        preserving_proto_field_name=True,
    )
    return json.dumps(d, separators=(", ", ": "))
