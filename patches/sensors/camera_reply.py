"""Validate the Unitree videohub sample response without assuming raw-video codecs."""
def image_format(payload):
    if payload.startswith(b"\xff\xd8\xff"):
        return "jpeg"
    if payload.startswith(b"\x89PNG\r\n\x1a\n"):
        return "png"
    return None


def sample_response(msg, request_id):
    if request_id is None or msg.header.identity.id != request_id or msg.header.identity.api_id != 1001:
        return None, None, None
    if msg.header.status.code != 0:
        return "error", None, "videohub status=%s" % msg.header.status.code
    payload = bytes(msg.binary)
    fmt = image_format(payload)
    if fmt is None:
        return "error", None, "sample is not JPEG/PNG: bytes=%d prefix=%s" % (len(payload), payload[:12].hex())
    return fmt, payload, None
