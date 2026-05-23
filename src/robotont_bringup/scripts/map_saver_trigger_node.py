#!/usr/bin/env python3

import datetime
import os
import re
import subprocess
from typing import Optional

import rclpy
from rclpy.node import Node
from robotont_bringup.srv import SaveMap


def _default_basename() -> str:
    stamp = datetime.datetime.now(tz=datetime.timezone.utc).strftime("%Y%m%d_%H%M%S")
    return f"map_{stamp}"


def _sanitize_basename(raw: str) -> Optional[str]:
    """Return a single path segment safe for map_saver_cli -f, or None if invalid."""
    s = (raw or "").strip()
    if not s:
        return None
    s = os.path.basename(s.replace("\\", "/"))
    lower = s.lower()
    for ext in (".yaml", ".yml", ".pgm"):
        if lower.endswith(ext):
            s = s[: -len(ext)]
            lower = s.lower()
    s = s.strip()
    if not s or s in (".", "..") or s.startswith("."):
        return None
    safe = re.sub(r"[^0-9A-Za-z._-]+", "_", s).strip("._")
    if not safe or safe in (".", ".."):
        return None
    return safe


class MapSaverTriggerNode(Node):
    """Expose /save_map (robotont_bringup/srv/SaveMap) -> map_saver_cli under save_directory."""

    def __init__(self):
        super().__init__("map_saver_trigger")

        self.save_directory = self.declare_parameter("save_directory", "/ws/saved_maps").value
        self.map_topic = self.declare_parameter("map_topic", "/map").value
        self.service_name = self.declare_parameter("service_name", "save_map").value
        self.use_sim_time = self.declare_parameter("use_sim_time", False).value

        save_root = os.path.abspath(self.save_directory)
        os.makedirs(save_root, exist_ok=True)
        self._save_root = save_root

        srv_topic = self.service_name
        if not srv_topic.startswith("/"):
            srv_topic = f"/{srv_topic}"

        self.create_service(SaveMap, srv_topic, self.handle_save_map)
        self.get_logger().info(
            f"Map save service ready at {srv_topic} -> files under {self._save_root}/"
        )

    def handle_save_map(self, request, response):
        raw = request.basename
        if not (raw or "").strip():
            basename = _default_basename()
        else:
            cleaned = _sanitize_basename(raw)
            if cleaned is None:
                response.success = False
                response.message = (
                    "invalid basename; use letters, digits, ._- only, no path, "
                    "or leave empty for an automatic name"
                )
                self.get_logger().warn(f"{response.message} (got {raw!r})")
                return response
            basename = cleaned

        basepath = os.path.normpath(os.path.join(self._save_root, basename))
        if not basepath.startswith(self._save_root + os.sep) and basepath != self._save_root:
            response.success = False
            response.message = "refusing path outside save_directory"
            self.get_logger().error(response.message)
            return response

        cmd = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", basepath]

        ros_params = []
        if self.use_sim_time:
            ros_params.append("use_sim_time:=true")
        if self.map_topic and self.map_topic != "/map":
            ros_params.append(f"map_topic:={self.map_topic}")

        if ros_params:
            cmd.append("--ros-args")
            for p in ros_params:
                cmd.extend(["-p", p])

        self.get_logger().info(f"Saving map via: {' '.join(cmd)}")

        try:
            proc = subprocess.run(
                cmd,
                check=False,
                capture_output=True,
                text=True,
                timeout=120,
            )
        except subprocess.TimeoutExpired:
            response.success = False
            response.message = "map_saver_cli timed out after 120s"
            self.get_logger().error(response.message)
            return response
        except OSError as exc:
            response.success = False
            response.message = f"failed to spawn map_saver_cli: {exc}"
            self.get_logger().error(response.message)
            return response

        yaml_path = basepath + ".yaml"
        pgm_path = basepath + ".pgm"

        if proc.returncode != 0:
            response.success = False
            tail = (proc.stderr or proc.stdout or "").strip()
            response.message = f"map_saver_cli failed (exit {proc.returncode}): {tail[-2000:]}"
            self.get_logger().error(response.message)
            return response

        if not (os.path.isfile(yaml_path) and os.path.isfile(pgm_path)):
            response.success = False
            response.message = (
                f"map_saver_cli exited 0 but expected outputs missing: {yaml_path}, {pgm_path}"
            )
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = f"saved {basename} ({yaml_path}, {pgm_path})"
        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = MapSaverTriggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
