"""Decode compressedDepth PNG with its depth header, not the RGB compressed path.

Wire format follows compressed_depth_image_transport (ConfigHeader + PNG).
The header uses the publisher's native byte order; caller must specify it.
32FC1 transport quantization/max-depth filtering cannot be reversed exactly.
RVL is rejected explicitly. Prefer a standard transport plugin for other codecs.
"""
import math
import re
import struct


def decode_depth_png(message, *, byte_order, bridge):
    import cv2
    import numpy as np
    fmt = message.format
    match = re.fullmatch(r'\s*(16UC1|32FC1);\s*compressedDepth(?:\s+png)?\s*', fmt)
    if not match:
        raise ValueError(f'only 16UC1/32FC1 compressedDepth PNG is supported, got {fmt!r}')
    if byte_order not in ('little', 'big'):
        raise ValueError('depth publisher byte order must be little or big')
    header = struct.Struct(('<' if byte_order == 'little' else '>') + 'iff')
    payload = bytes(message.data)
    if len(payload) <= header.size or payload[header.size:header.size+8] != b'\x89PNG\r\n\x1a\n':
        raise ValueError('compressedDepth header/PNG payload missing')
    mode, factor, offset = header.unpack_from(payload)
    if mode != 0:
        raise ValueError('unsupported compressedDepth header mode')
    depth = cv2.imdecode(np.frombuffer(payload[header.size:], dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if depth is None or depth.ndim != 2 or depth.dtype != np.uint16:
        raise ValueError('compressedDepth PNG must decode to a uint16 single-channel array')
    if match.group(1) == '32FC1':
        if not math.isfinite(factor) or factor <= 0 or not math.isfinite(offset):
            raise ValueError('invalid float-depth reconstruction parameters')
        denominator = depth.astype(np.float32) - offset
        valid = (depth != 0) & (denominator > 0)
        restored = np.full(depth.shape, np.nan, dtype=np.float32)
        restored[valid] = factor / denominator[valid]
        depth = restored
    image = bridge.cv2_to_imgmsg(depth, encoding=match.group(1))
    from copy import deepcopy
    image.header = deepcopy(message.header)
    return image
