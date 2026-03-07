#!/usr/bin/env python3

###############################################################################
# Copyright 2026 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

import argparse
import os
import re
import sys
from dataclasses import dataclass
from typing import List, Optional


CAMERA_TYPES = {"MONOCULAR_CAMERA", "STEREO_CAMERA"}


@dataclass
class SensorMeta:
    name: str
    sensor_type: str


def _resolve_file_path_with_env(path: str, env_var: str) -> Optional[str]:
    if not path:
        return None

    if os.path.isabs(path):
        return path if os.path.exists(path) else None

    if os.path.exists(path):
        return os.path.abspath(path)

    if path.startswith("."):
        return None

    env_val = os.environ.get(env_var, "")
    for entry in env_val.split(":"):
        entry = entry.strip()
        if not entry:
            continue
        candidate = os.path.join(entry, path)
        if os.path.exists(candidate):
            return candidate
    return None


def _map_apollo_path(candidate: str, apollo_root: str) -> str:
    # In repo/dev environments the root may not be mounted at /apollo.
    if os.path.exists(candidate):
        return candidate
    if candidate.startswith("/apollo/"):
        mapped = os.path.join(apollo_root, candidate[len("/apollo/"):])
        if os.path.exists(mapped):
            return mapped
    return candidate


def _resolve_common_config_file(config_file: str) -> Optional[str]:
    relative_config_path = config_file
    if not os.path.isabs(config_file):
        relative_config_path = os.path.join(
            "modules", "perception", "data", "conf", config_file
        )
    return _resolve_file_path_with_env(relative_config_path, "APOLLO_CONF_PATH")


def _parse_sensor_meta(path: str) -> List[SensorMeta]:
    with open(path, "r", encoding="utf-8") as f:
        content = f.read()

    blocks = re.findall(r"sensor_meta\s*\{([^}]*)\}", content, flags=re.DOTALL)
    sensors: List[SensorMeta] = []
    for block in blocks:
        name_match = re.search(r'name\s*:\s*"([^"]+)"', block)
        type_match = re.search(r"type\s*:\s*([A-Z0-9_]+)", block)
        if not name_match or not type_match:
            continue
        sensors.append(SensorMeta(name=name_match.group(1), sensor_type=type_match.group(1)))
    return sensors


def main() -> int:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_apollo_root = os.path.normpath(os.path.join(script_dir, ".."))

    parser = argparse.ArgumentParser(
        description="Preflight checker for Apollo perception startup files."
    )
    parser.add_argument(
        "--apollo_root",
        default=os.environ.get("APOLLO_ROOT_DIR", default_apollo_root),
        help="Apollo root directory.",
    )
    parser.add_argument(
        "--obs_sensor_meta_file",
        default="sensor_meta.pb.txt",
        help="Sensor meta config file (same as FLAGS_obs_sensor_meta_file).",
    )
    parser.add_argument(
        "--obs_sensor_intrinsic_path",
        default="/apollo/modules/perception/data/params",
        help="Intrinsic/extrinsic dir (same as FLAGS_obs_sensor_intrinsic_path).",
    )
    args = parser.parse_args()

    apollo_root = os.path.abspath(args.apollo_root)
    config_file = _resolve_common_config_file(args.obs_sensor_meta_file)
    if not config_file:
        print(
            "[perception preflight] ERROR: cannot resolve sensor meta file "
            f"'{args.obs_sensor_meta_file}' via APOLLO_CONF_PATH or local path.",
            file=sys.stderr,
        )
        return 1

    sensors = _parse_sensor_meta(config_file)
    if not sensors:
        print(
            f"[perception preflight] ERROR: no sensors parsed from {config_file}.",
            file=sys.stderr,
        )
        return 1

    camera_sensors = [s for s in sensors if s.sensor_type in CAMERA_TYPES]
    if not camera_sensors:
        print(
            "[perception preflight] WARNING: no camera sensors defined in sensor meta; "
            "camera intrinsic checks skipped."
        )
        return 0

    intrinsic_root = _map_apollo_path(args.obs_sensor_intrinsic_path, apollo_root)
    missing_files: List[str] = []
    for sensor in camera_sensors:
        intrinsic_file = os.path.join(intrinsic_root, f"{sensor.name}_intrinsics.yaml")
        extrinsic_file = os.path.join(intrinsic_root, f"{sensor.name}_extrinsics.yaml")
        if not os.path.exists(intrinsic_file):
            missing_files.append(intrinsic_file)
        if not os.path.exists(extrinsic_file):
            missing_files.append(extrinsic_file)

    if missing_files:
        print("[perception preflight] ERROR: missing perception calibration files:", file=sys.stderr)
        for path in missing_files:
            print(f"  - {path}", file=sys.stderr)
        print(
            "[perception preflight] Hint: verify APOLLO_CONF_PATH/APOLLO_ROOT_DIR and "
            "vehicle-specific perception config deployment.",
            file=sys.stderr,
        )
        return 1

    print(
        "[perception preflight] OK: validated "
        f"{len(camera_sensors)} camera sensor(s) from {config_file}."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
