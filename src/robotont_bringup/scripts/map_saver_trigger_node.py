#!/usr/bin/env python3

import datetime
import json
import os
import re
import shutil
import subprocess
from pathlib import Path
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
    for ext in (".yaml", ".yml", ".pgm", ".posegraph", ".data"):
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
        self.serialize_service = self.declare_parameter(
            "serialize_service", "/slam_toolbox/serialize_map"
        ).value
        self.save_map_timeout = float(self.declare_parameter("save_map_timeout", 10.0).value)
        self.use_sim_time = self._parameter_value("use_sim_time", False)

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

    def _parameter_value(self, name, default):
        if not self.has_parameter(name):
            return self.declare_parameter(name, default).value
        return self.get_parameter(name).value

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

        map_dir = os.path.normpath(os.path.join(self._save_root, basename))
        if not map_dir.startswith(self._save_root + os.sep) and map_dir != self._save_root:
            response.success = False
            response.message = "refusing path outside save_directory"
            self.get_logger().error(response.message)
            return response

        os.makedirs(map_dir, exist_ok=True)
        basepath = os.path.join(map_dir, basename)
        cmd = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", basepath]

        ros_params = []
        ros_params.append(f"save_map_timeout:={self.save_map_timeout}")
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

        checkpoint_message = self.serialize_pose_graph(basepath)
        if checkpoint_message is None:
            response.success = False
            response.message = (
                f"saved occupancy map ({yaml_path}, {pgm_path}) but failed to save "
                "slam_toolbox checkpoint"
            )
            return response

        metadata_file = self.write_bundle_metadata(map_dir, basename)
        response.success = True
        response.message = (
            f"saved map bundle {basename} under {map_dir}/ "
            f"({Path(yaml_path).name}, {Path(pgm_path).name}; "
            f"{checkpoint_message}; {metadata_file})"
        )
        self.get_logger().info(response.message)
        return response

    def serialize_pose_graph(self, basepath):
        cmd = [
            "ros2",
            "service",
            "call",
            self.serialize_service,
            "slam_toolbox/srv/SerializePoseGraph",
            f"{{filename: {basepath}}}",
        ]
        self.get_logger().info(f"Saving slam_toolbox checkpoint via: {' '.join(cmd)}")

        try:
            proc = subprocess.run(
                cmd,
                check=False,
                capture_output=True,
                text=True,
                timeout=120,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().error("slam_toolbox checkpoint save timed out after 120s")
            return None
        except OSError as exc:
            self.get_logger().error(f"failed to call slam_toolbox checkpoint save: {exc}")
            return None

        data_path = basepath + ".data"
        posegraph_path = basepath + ".posegraph"
        output = (proc.stdout or "") + (proc.stderr or "")
        if proc.returncode != 0 or "result=0" not in output:
            self.get_logger().error(
                f"slam_toolbox checkpoint save failed: {output.strip()[-2000:]}"
            )
            return None
        if not (os.path.isfile(data_path) and os.path.isfile(posegraph_path)):
            self.get_logger().error(
                f"slam_toolbox checkpoint outputs missing: {data_path}, {posegraph_path}"
            )
            return None

        return f"{Path(data_path).name}, {Path(posegraph_path).name}"

    def write_bundle_metadata(self, map_dir, basename):
        world_mode = os.environ.get("ROBOTONT_WORLD_MODE", "")
        world_file = os.environ.get("ROBOTONT_WORLD_FILE", "")
        metadata = {
            "version": 1,
            "name": basename,
            "world_mode": world_mode,
            "world_file": world_file,
            "files": {
                "map_yaml": f"{basename}.yaml",
                "map_image": f"{basename}.pgm",
                "posegraph": f"{basename}.posegraph",
                "data": f"{basename}.data",
            },
        }

        if world_mode == "custom" and world_file:
            source = Path(world_file)
            if source.is_file():
                snapshot = Path(map_dir) / "world.json"
                shutil.copyfile(source, snapshot)
                metadata["world_snapshot"] = snapshot.name
                self.get_logger().info(f"Copied custom world snapshot to {snapshot}")
            else:
                metadata["world_snapshot_error"] = f"world_file not found: {world_file}"
                self.get_logger().warn(metadata["world_snapshot_error"])

        metadata_path = Path(map_dir) / "robotont_map_bundle.json"
        with metadata_path.open("w", encoding="utf-8") as stream:
            json.dump(metadata, stream, indent=2, sort_keys=True)
            stream.write("\n")
        return metadata_path.name


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
