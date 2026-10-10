"""Timestamped image detections for the velocity follower; never synthesize pixels."""

import json
import math
import socket
import time


class ImagePixelSubscriber:
    def __init__(self, port=14602, max_age=0.2):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", port))
        self.socket.setblocking(False)
        self.max_age = max_age
        self.latest = None

    def read(self, now=None):
        now = time.monotonic() if now is None else now
        while True:
            try:
                payload, _ = self.socket.recvfrom(2048)
            except BlockingIOError:
                break
            try:
                data = json.loads(payload)
                pixel = tuple(float(value) for value in data["pixel"])
                received = float(data["received_at"])
                stamp = float(data["image_stamp"])
                if (len(pixel) != 2 or not all(math.isfinite(value) for value in (*pixel, received, stamp))
                        or not 0 <= now - received <= self.max_age or stamp <= 0
                        or data.get("source") != "rgbd_detection" or len(data.get("intrinsics", [])) != 3
                        or any(len(row) != 3 for row in data.get("intrinsics", [])) or len(data.get("resolution", [])) != 2
                        or not all(math.isfinite(float(value)) for row in data.get("intrinsics", []) for value in row)):
                    continue
                if self.latest is None or stamp > self.latest["image_stamp"]:
                    self.latest = {"pixel": pixel, "received_at": received, "image_stamp": stamp,
                                   "intrinsics": data.get("intrinsics"), "resolution": data.get("resolution")}
            except (ValueError, TypeError, KeyError, OverflowError):
                continue
        if self.latest is None or not 0 <= now - self.latest["received_at"] <= self.max_age:
            return None
        return self.latest

    def close(self):
        self.socket.close()