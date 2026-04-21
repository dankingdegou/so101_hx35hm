"""Transport layer abstractions for async inference."""

from so101_inference.transport.base import PolicyTransport

try:
    from so101_inference.transport.grpc_transport import GrpcTransport

    __all__ = ["PolicyTransport", "GrpcTransport"]
except ModuleNotFoundError:
    # Optional dependency (grpc / lerobot) lives in the Pixi `lerobot` env.
    __all__ = ["PolicyTransport"]
