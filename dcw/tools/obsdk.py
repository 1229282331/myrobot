"""Small Windows/Linux ctypes binding to the pinned Orbbec SDK v1 C API.

Only enumeration and streaming functions are exposed; no firmware, reset,
calibration, flash, or device-property writes. Frame bytes are copied before
their SDK handles are freed. See vendor SDK/include/libobsensor/h for the ABI.
"""
from __future__ import annotations

import ctypes as C
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
SDK_ROOT = Path(os.environ.get("DCW_SDK_ROOT", str(ROOT / "vendor/sdk-v1.10.37/OrbbecSDK_v1.10.37/SDK")))
SDK_CONFIG = Path(os.environ.get("DCW_SDK_CONFIG", str(SDK_ROOT / "config/OrbbecSDKConfig_v1.0.xml")))
FORMATS = {0: "YUYV", 1: "YUY2", 2: "UYVY", 3: "NV12", 4: "NV21", 5: "MJPG", 8: "Y16", 9: "Y8", 10: "Y10", 11: "Y11", 12: "Y12", 22: "RGB", 23: "BGR", 25: "BGRA", 28: "Z16"}
P, U, I, S = C.c_void_p, C.c_uint32, C.c_int, C.c_char_p


class SDK:
    def __init__(self):
        if sys.platform == "win32":
            self._dll_dirs = [os.add_dll_directory(str(SDK_ROOT / "lib"))]
            library = SDK_ROOT / "lib/OrbbecSDK.dll"
        elif sys.platform == "linux":
            self._dll_dirs = []
            library = SDK_ROOT / "lib/libOrbbecSDK.so"
        else:
            raise RuntimeError(f"Unsupported platform: {sys.platform}")
        self.lib = C.CDLL(str(library))
        self.funcs = {}
        self.lib.ob_error_message.argtypes = [P]
        self.lib.ob_error_message.restype = S
        self.lib.ob_delete_error.argtypes = [P]
        self.lib.ob_delete_error.restype = None
        specs = {
            "create_context_with_config": (P, [S]), "delete_context": (None, [P]),
            "query_device_list": (P, [P]), "delete_device_list": (None, [P]),
            "device_list_device_count": (U, [P]), "device_list_get_device": (P, [P, U]),
            "device_get_device_info": (P, [P]), "delete_device_info": (None, [P]),
            "device_get_bool_property": (C.c_bool, [P, I]),
            "device_get_int_property": (I, [P, I]),
            "device_is_property_supported": (C.c_bool, [P, I, I]),
            "delete_device": (None, [P]),
            "create_pipeline_with_device": (P, [P]), "delete_pipeline": (None, [P]),
            "pipeline_get_stream_profile_list": (P, [P, I]),
            "stream_profile_list_count": (U, [P]),
            "stream_profile_list_get_profile": (P, [P, I]),
            "stream_profile_list_get_video_stream_profile": (P, [P, I, I, I, I]),
            "delete_stream_profile_list": (None, [P]), "delete_stream_profile": (None, [P]),
            "stream_profile_format": (I, [P]),
            "create_config": (P, []), "delete_config": (None, [P]),
            "config_enable_stream": (None, [P, P]),
            "config_set_align_mode": (None, [P, I]),
            "config_set_frame_aggregate_output_mode": (None, [P, I]),
            "pipeline_start_with_config": (None, [P, P]), "pipeline_stop": (None, [P]),
            "pipeline_wait_for_frameset": (P, [P, U]), "delete_frame": (None, [P]),
            "frameset_color_frame": (P, [P]), "frameset_depth_frame": (P, [P]),
            "frameset_ir_frame": (P, [P]),
            "frame_data": (P, [P]), "frame_data_size": (U, [P]),
            "frame_format": (I, [P]), "frame_index": (C.c_uint64, [P]),
            "frame_time_stamp_us": (C.c_uint64, [P]),
            "frame_system_time_stamp_us": (C.c_uint64, [P]),
            "frame_metadata": (P, [P]), "frame_metadata_size": (U, [P]),
            "depth_frame_get_value_scale": (C.c_float, [P]),
            "set_logger_to_file": (None, [I, S]), "set_logger_to_console": (None, [I]),
        }
        for field in ("name", "serial_number", "uid", "firmware_version", "hardware_version", "connection_type"):
            specs[f"device_info_{field}"] = (S, [P])
        for field in ("pid", "vid"):
            specs[f"device_info_{field}"] = (I, [P])
        for field in ("width", "height", "fps"):
            specs[f"video_stream_profile_{field}"] = (U, [P])
        for field in ("width", "height"):
            specs[f"video_frame_{field}"] = (U, [P])
        for name, (result, args) in specs.items():
            f = getattr(self.lib, "ob_" + name)
            f.restype, f.argtypes = result, args + [C.POINTER(P)]
            self.funcs[name] = f
        (ROOT / "logs/sdk").mkdir(parents=True, exist_ok=True)
        self.call("set_logger_to_file", 1, str(ROOT / "logs/sdk").encode())
        self.call("set_logger_to_console", 2)

    def call(self, name, *args):
        error = P()
        value = self.funcs[name](*args, C.byref(error))
        if error.value:
            try:
                message = self.lib.ob_error_message(error).decode("utf-8", "replace")
            finally:
                self.lib.ob_delete_error(error)
            raise RuntimeError(f"{name}: {message}")
        return value.decode("utf-8", "replace") if isinstance(value, bytes) else value

    def release(self, kind, handle):
        if handle:
            self.call("delete_" + kind, handle)

    def context(self):
        return self.call("create_context_with_config", str(SDK_CONFIG).encode())

    def info(self, device):
        info = self.call("device_get_device_info", device)
        try:
            return {field: self.call("device_info_" + field, info) for field in
                    ("name", "serial_number", "uid", "firmware_version", "hardware_version", "connection_type", "pid", "vid")}
        finally:
            self.release("device_info", info)

    def status(self, device):
        result = {}
        for name, prop, kind in (("ldp_enabled", 2, "bool"), ("laser_enabled", 3, "bool"),
                                 ("ldp_status", 32, "bool"), ("laser_mode", 79, "int"),
                                 ("ir_auto_exposure", 2025, "bool"), ("ir_exposure", 2026, "int")):
            try:
                result[name] = (self.call(f"device_get_{kind}_property", device, prop)
                                if self.call("device_is_property_supported", device, prop, 1) else "unsupported")
            except RuntimeError as exc:
                result[name] = {"error": str(exc)}
        return result

    def profiles(self, pipeline, sensor):
        profiles = self.call("pipeline_get_stream_profile_list", pipeline, sensor)
        result = []
        try:
            for index in range(self.call("stream_profile_list_count", profiles)):
                profile = self.call("stream_profile_list_get_profile", profiles, index)
                try:
                    entry = {field: self.call("video_stream_profile_" + field, profile) for field in ("width", "height", "fps")}
                    entry["format_id"] = self.call("stream_profile_format", profile)
                    entry["format"] = FORMATS.get(entry["format_id"], str(entry["format_id"]))
                    result.append(entry)
                finally:
                    self.release("stream_profile", profile)
        finally:
            self.release("stream_profile_list", profiles)
        return result


def probe():
    sdk = SDK()
    ctx = devices = None
    result = {"sdk": "1.10.37", "devices": []}
    try:
        ctx = sdk.context()
        devices = sdk.call("query_device_list", ctx)
        for i in range(sdk.call("device_list_device_count", devices)):
            device = pipeline = None
            try:
                device = sdk.call("device_list_get_device", devices, i)
                entry = sdk.info(device)
                entry["read_only_status"] = sdk.status(device)
                pipeline = sdk.call("create_pipeline_with_device", device)
                entry["profiles"] = {}
                for name, sensor in (("color", 2), ("depth", 3)):
                    try:
                        entry["profiles"][name] = sdk.profiles(pipeline, sensor)
                    except RuntimeError as exc:
                        entry["profiles"][name] = {"error": str(exc)}
                result["devices"].append(entry)
            finally:
                sdk.release("pipeline", pipeline)
                sdk.release("device", device)
    finally:
        sdk.release("device_list", devices)
        sdk.release("context", ctx)
    return result


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Enumerate cameras and profiles without starting streams")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = probe()
    rendered = json.dumps(result, ensure_ascii=False, indent=2)
    print(json.dumps({"sdk": result["sdk"], "devices": [{k: v for k, v in d.items() if k != "profiles"} for d in result["devices"]]}, ensure_ascii=False, indent=2))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
