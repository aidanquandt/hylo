# Protocol: .proto schema, nanopb runtime, and generated C sources.
# Regenerate: python tools/protocol_codegen/run_nanopb_gen.py && python tools/protocol_codegen/codegen_protocol.py
# No venv; see protocol/README.md. Nanopb is vendored in third_party/nanopb (submodule).

cmake_minimum_required(VERSION 3.14)

set(NANOPB_SOURCE_DIR "${CMAKE_SOURCE_DIR}/third_party/nanopb")
if(NOT EXISTS "${NANOPB_SOURCE_DIR}/pb.h")
    message(FATAL_ERROR
        "nanopb not found at ${NANOPB_SOURCE_DIR}. "
        "Run: git submodule update --init third_party/nanopb"
    )
endif()

# ---- Generated C from generated/protocol/c/ (source tree) ----
set(PROTOCOL_GENERATED_SRC "${CMAKE_SOURCE_DIR}/generated/protocol/c")
# uart_framing.h (for protocol_tx.c) lives in src/uart_protocol
set(UART_PROTOCOL_SRC "${CMAKE_SOURCE_DIR}/src/uart_protocol")

# ---- Protocol library: nanopb runtime + generated .pb.c + codegen dispatch/tx ----
# We use only nanopb runtime sources; we do not add_subdirectory(nanopb) to avoid
# building their generator/protoc and to keep our build simple.
add_library(protocol STATIC
    "${NANOPB_SOURCE_DIR}/pb_common.c"
    "${NANOPB_SOURCE_DIR}/pb_encode.c"
    "${NANOPB_SOURCE_DIR}/pb_decode.c"
    "${PROTOCOL_GENERATED_SRC}/host_options.pb.c"
    "${PROTOCOL_GENERATED_SRC}/protocol.pb.c"
    "${PROTOCOL_GENERATED_SRC}/protocol_dispatch.c"
    "${PROTOCOL_GENERATED_SRC}/protocol_tx.c"
)

target_include_directories(protocol PUBLIC
    "${PROTOCOL_GENERATED_SRC}"
    "${NANOPB_SOURCE_DIR}"
    "${UART_PROTOCOL_SRC}"
)

set_target_properties(protocol PROPERTIES FOLDER "protocol")