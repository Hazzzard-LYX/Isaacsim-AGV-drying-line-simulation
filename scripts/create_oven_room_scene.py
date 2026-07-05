#!/usr/bin/env python3
"""Create an Isaac Sim USD scene with four rows of oven URDF instances."""

from __future__ import annotations

import argparse
import math
import os
import struct
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT_DIR = Path(__file__).resolve().parents[1]
URDF_PATH = ROOT_DIR / "oven" / "urdf" / "oven.urdf"
GENERATED_URDF_PATH = ROOT_DIR / "scenes" / "oven.isaacsim.urdf"
DEFAULT_OUTPUT = ROOT_DIR / "scenes" / "oven_room.usd"

OVEN_WIDTH_X = 3.5
OVEN_DEPTH_Y = 2.25
OVEN_HEIGHT_Z = 4.35
OVEN_COLUMN_GAP = 0.3
MIDDLE_BACK_GAP = 2.0
AISLE_WIDTH = 4.5
AISLE_ENTRANCE_EXTENSION = 2.0
OVENS_PER_ROW = 7
ROW_COUNT = 4
ROOM_MARGIN_X = 1.0
ROOM_MARGIN_Y = 2.0
GROUND_LENGTH_X = 1000.0
GROUND_WIDTH_Y = 1000.0
COOLING_ROOM_WIDTH_X = 15.5
COOLING_SLOT_COLUMNS = 9
COOLING_SLOT_ROWS = 7
COOLING_SLOT_SIZE_X = 1.0
COOLING_SLOT_SIZE_Y = 1.5
COOLING_PASSAGE_WIDTH_Y = 1.5
COOLING_LONGITUDINAL_PASSAGE_WIDTH_X = 4.0
COOLING_EAST_WALL_CLEARANCE_X = 1.5
COOLING_AGV_CLEARANCE_X = 1.25
HANDOFF_CART_FRONT_OFFSET_X = 0.75
COOLING_UNLOADED_WAIT_AFTER_HANDOFF = 0.5
WALL_THICKNESS = 0.2
WALL_HEIGHT = 2.0
FLOOR_THICKNESS = 0.1
WALL_FLOOR_OVERLAP = 0.05
DOOR_ANIMATION_FPS = 24
DOOR_ANIMATION_DURATION = 8.0
DOOR_ANIMATION_PERIOD = 2.0
DOOR_PRISMATIC_STROKE = 2.0
SCENE_ANIMATION_DURATION = 1100.0
PIPELINE_CART_APPEAR_INTERVAL = 40.0
PIPELINE_TASKS_PER_AISLE = 8
DRYING_WORK_DURATION = PIPELINE_CART_APPEAR_INTERVAL * 8.0
PRINT_RELATIVE_DEBUG = False
AGV_ROOT_Z = 0.655
AGV_START_X = -11.0
AGV_END_X = 11.0
AGV_FORK_LIFT_HEIGHT = 0.4
AGV_FORK_LIFT_START = 10.0
AGV_FORK_LIFT_END = 14.0
AGV_WHEEL_RADIUS = 0.085
AGV_CART_STANDOFF = 1.25
CARRY_SAMPLE_DT = 1.0 / DOOR_ANIMATION_FPS
CARRY_TRAVEL_SPEED = 1.0
CARRY_RETREAT_DISTANCE = 4.0
TURN_ENTRY_DEPTH = 1.2
CART_ROOT_DIR = ROOT_DIR / "烘车urdf"
CART_LOCAL_FOOTPRINT_CENTER_X = -0.75
CART_LOCAL_FOOTPRINT_CENTER_Y = 0.50
CART_LOCAL_Z_MIN = -0.059989336878061295
CART_IN_OVEN_Z = -CART_LOCAL_Z_MIN
CART_PARKED_Z = -CART_LOCAL_Z_MIN
CART_CARRIED_Z = CART_PARKED_Z + AGV_FORK_LIFT_HEIGHT

# The SolidWorks-exported URDF mesh is not centered on base_link.
OVEN_LOCAL_FOOTPRINT_CENTER_X = 2.1229487
OVEN_LOCAL_FOOTPRINT_CENTER_Y = 1.9719369
DOOR_JOINT_ORIGIN = (2.1229, 3.0969, 2.3219)
SUB_OVEN_LOCAL_X_OFFSETS = {
    "left": -0.75,
    "right": 0.75,
}
DOOR_LINK_FOR_SUB_OVEN_SIDE = {
    "left": "right",
    "right": "left",
}
AGV_ROOT_DIR = ROOT_DIR / "F4-1000C"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--headless", action="store_true", help="Run Isaac Sim headless.")
    parser.add_argument(
        "--proxy-only",
        action="store_true",
        help="Use simple box ovens instead of importing the URDF.",
    )
    parser.add_argument(
        "--strict-urdf",
        action="store_true",
        help="Fail if mesh loading is unavailable instead of using proxy boxes.",
    )
    parser.add_argument(
        "--use-urdf-importer",
        action="store_true",
        help="Use Isaac Sim's URDF importer instead of directly loading STL meshes.",
    )
    return parser.parse_args()


def start_simulation_app(headless: bool):
    ros_paths = [str(ROOT_DIR), str(ROOT_DIR / "oven")]
    existing = os.environ.get("ROS_PACKAGE_PATH")
    if existing:
        ros_paths.append(existing)
    os.environ["ROS_PACKAGE_PATH"] = os.pathsep.join(ros_paths)

    try:
        from isaacsim import SimulationApp
    except ImportError:
        from omni.isaac.kit import SimulationApp

    return SimulationApp({"headless": headless})


def prepared_urdf_path() -> Path:
    GENERATED_URDF_PATH.parent.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(URDF_PATH)
    root = tree.getroot()
    mesh_dir = ROOT_DIR / "oven" / "meshes"
    prefix = "package://oven/meshes/"

    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib.get("filename", "")
        if filename.startswith(prefix):
            mesh.attrib["filename"] = str(mesh_dir / filename[len(prefix):])

    tree.write(GENERATED_URDF_PATH, encoding="utf-8", xml_declaration=True)
    return GENERATED_URDF_PATH


def create_material(stage, path: str, color):
    from pxr import Sdf, UsdShade

    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color[:3])
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(color[3])
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def bind_material(prim, material):
    from pxr import UsdShade

    UsdShade.MaterialBindingAPI(prim).Bind(material)


def add_cube(stage, path: str, center, size, material=None):
    from pxr import Gf, UsdGeom

    cx, cy, cz = center
    sx, sy, sz = size[0] / 2.0, size[1] / 2.0, size[2] / 2.0
    points = [
        (cx - sx, cy - sy, cz - sz),
        (cx + sx, cy - sy, cz - sz),
        (cx + sx, cy + sy, cz - sz),
        (cx - sx, cy + sy, cz - sz),
        (cx - sx, cy - sy, cz + sz),
        (cx + sx, cy - sy, cz + sz),
        (cx + sx, cy + sy, cz + sz),
        (cx - sx, cy + sy, cz + sz),
    ]
    face_indices = [
        0, 3, 2, 1,
        4, 5, 6, 7,
        0, 1, 5, 4,
        1, 2, 6, 5,
        2, 3, 7, 6,
        3, 0, 4, 7,
    ]

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*point) for point in points])
    mesh.CreateFaceVertexCountsAttr([4, 4, 4, 4, 4, 4])
    mesh.CreateFaceVertexIndicesAttr(face_indices)
    prim = mesh.GetPrim()
    if material is not None:
        bind_material(prim, material)
    return prim


def read_binary_stl(path: Path):
    data = path.read_bytes()
    triangle_count = struct.unpack("<I", data[80:84])[0]
    points = []
    indices = []
    offset = 84

    for _ in range(triangle_count):
        values = struct.unpack("<12fH", data[offset:offset + 50])
        for vertex in (values[3:6], values[6:9], values[9:12]):
            indices.append(len(points))
            points.append(vertex)
        offset += 50

    return points, [3] * triangle_count, indices


def add_stl_mesh(stage, path: str, stl_path: Path, material=None):
    from pxr import Gf, UsdGeom

    points, face_counts, face_indices = read_binary_stl(stl_path)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*point) for point in points])
    mesh.CreateFaceVertexCountsAttr(face_counts)
    mesh.CreateFaceVertexIndicesAttr(face_indices)
    prim = mesh.GetPrim()
    if material is not None:
        bind_material(prim, material)
    return prim


def oven_centers():
    column_pitch = OVEN_WIDTH_X + OVEN_COLUMN_GAP
    x0 = -((OVENS_PER_ROW - 1) * column_pitch) / 2.0
    xs = [x0 + i * column_pitch for i in range(OVENS_PER_ROW)]

    total_y = ROW_COUNT * OVEN_DEPTH_Y + 2 * AISLE_WIDTH + MIDDLE_BACK_GAP
    top = total_y / 2.0
    row_a = top - OVEN_DEPTH_Y / 2.0
    row_b = row_a - OVEN_DEPTH_Y - AISLE_WIDTH
    row_c = row_b - OVEN_DEPTH_Y - MIDDLE_BACK_GAP
    row_d = row_c - OVEN_DEPTH_Y - AISLE_WIDTH

    # Local +Y is treated as the oven front. Top and third rows face -Y,
    # second and bottom rows face +Y, making the middle rows back-to-back.
    rows = [
        ("row_1_top", row_a, math.pi),
        ("row_2_upper_middle", row_b, 0.0),
        ("row_3_lower_middle", row_c, math.pi),
        ("row_4_bottom", row_d, 0.0),
    ]

    for row_name, y, yaw in rows:
        for col, x in enumerate(xs, start=1):
            yield row_name, col, x, y, yaw


def room_length_x():
    return OVENS_PER_ROW * OVEN_WIDTH_X + (OVENS_PER_ROW - 1) * OVEN_COLUMN_GAP + 2 * ROOM_MARGIN_X


def room_width_y():
    return ROW_COUNT * OVEN_DEPTH_Y + 2 * AISLE_WIDTH + MIDDLE_BACK_GAP + 2 * ROOM_MARGIN_Y


def aisle_centers_y():
    total_y = ROW_COUNT * OVEN_DEPTH_Y + 2 * AISLE_WIDTH + MIDDLE_BACK_GAP
    top_aisle_y = total_y / 2.0 - OVEN_DEPTH_Y - AISLE_WIDTH / 2.0
    return top_aisle_y, -top_aisle_y


def root_translation_for_center(x: float, y: float, yaw: float):
    c = math.cos(yaw)
    s = math.sin(yaw)
    rotated_cx = c * OVEN_LOCAL_FOOTPRINT_CENTER_X - s * OVEN_LOCAL_FOOTPRINT_CENTER_Y
    rotated_cy = s * OVEN_LOCAL_FOOTPRINT_CENTER_X + c * OVEN_LOCAL_FOOTPRINT_CENTER_Y
    return x - rotated_cx, y - rotated_cy, 0.0


def import_urdf_prototype(stage, proto_path: str):
    import omni.kit.commands
    import omni.usd
    from pxr import UsdGeom

    manager = omni.kit.app.get_app().get_extension_manager()
    for ext in ("isaacsim.asset.importer.urdf",):
        try:
            manager.set_extension_enabled_immediate(ext, True)
        except Exception:
            pass

    _, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    for name, value in {
        "merge_fixed_joints": True,
        "fix_base": True,
        "make_default_prim": False,
        "create_physics_scene": False,
        "import_inertia_tensor": False,
        "distance_scale": 1.0,
    }.items():
        if hasattr(import_config, name):
            setattr(import_config, name, value)

    before = {p.GetPath().pathString for p in stage.Traverse()}
    _, imported_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
            urdf_path=str(prepared_urdf_path()),
        import_config=import_config,
    )

    prim = stage.GetPrimAtPath(proto_path)
    if not prim.IsValid():
        imported_prim = stage.GetPrimAtPath(imported_path)
        created = [imported_prim] if imported_prim.IsValid() else []
        created.extend(
            p for p in stage.Traverse()
            if p.GetPath().pathString not in before and p.GetPath().pathString.count("/") == 1
        )
        if not created:
            raise RuntimeError("URDF importer did not create a root prim.")
        omni.kit.commands.execute(
            "MovePrim",
            path_from=created[0].GetPath().pathString,
            path_to=proto_path,
        )
        prim = stage.GetPrimAtPath(proto_path)

    UsdGeom.Imageable(prim).MakeInvisible()
    return prim


def set_visibility_samples(prim, samples):
    from pxr import UsdGeom

    visibility = UsdGeom.Imageable(prim).CreateVisibilityAttr()
    for seconds, visible in samples:
        visibility.Set(
            UsdGeom.Tokens.inherited if visible else UsdGeom.Tokens.invisible,
            seconds * DOOR_ANIMATION_FPS,
        )


def add_oven_led(stage, path: str, materials, led_samples_by_side=None):
    led_y = DOOR_JOINT_ORIGIN[1] + 0.035
    led_z = OVEN_HEIGHT_Z + 0.18
    led_size = (0.55, 0.08, 0.18)
    default_samples = [(0.0, "green"), (SCENE_ANIMATION_DURATION, "green")]
    led_samples_by_side = led_samples_by_side or {}

    for side_name, x_offset in SUB_OVEN_LOCAL_X_OFFSETS.items():
        status_samples = led_samples_by_side.get(side_name, default_samples)
        green_samples = [(seconds, status == "green") for seconds, status in status_samples]
        red_samples = [(seconds, status == "red") for seconds, status in status_samples]
        led_center = (DOOR_JOINT_ORIGIN[0] + x_offset, led_y, led_z)
        green = add_cube(stage, f"{path}/led_{side_name}_green", led_center, led_size, materials["led_green"])
        red = add_cube(stage, f"{path}/led_{side_name}_red", led_center, led_size, materials["led_red"])
        set_visibility_samples(green, green_samples)
        set_visibility_samples(red, red_samples)


def add_direct_stl_oven(stage, path: str, x: float, y: float, yaw: float, materials, door_samples_by_side=None, led_samples_by_side=None):
    from pxr import Gf, UsdGeom

    root = UsdGeom.Xform.Define(stage, path)
    tx, ty, tz = root_translation_for_center(x, y, yaw)
    root_xform = UsdGeom.Xformable(root.GetPrim())
    root_xform.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
    root_xform.AddRotateZOp().Set(math.degrees(yaw))

    add_stl_mesh(
        stage,
        f"{path}/base_link",
        ROOT_DIR / "oven" / "meshes" / "base_link.STL",
        materials["oven"],
    )

    for link_name, stl_name in (
        ("left", "left.STL"),
        ("right", "right.STL"),
    ):
        link = UsdGeom.Xform.Define(stage, f"{path}/{link_name}")
        link_xform = UsdGeom.Xformable(link.GetPrim())
        translate_op = link_xform.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(*DOOR_JOINT_ORIGIN))
        door_samples = (door_samples_by_side or {}).get(link_name)
        add_prismatic_door_animation(translate_op, Gf.Vec3d(*DOOR_JOINT_ORIGIN), door_samples)
        link_xform.AddRotateZOp().Set(180.0)
        add_stl_mesh(
            stage,
            f"{path}/{link_name}/visual",
            ROOT_DIR / "oven" / "meshes" / stl_name,
            materials["oven"],
        )

    add_oven_led(stage, path, materials, led_samples_by_side)


def add_prismatic_door_animation(translate_op, closed_origin, stroke_samples=None):
    from pxr import Gf

    samples = stroke_samples or [(0.0, 0.0), (SCENE_ANIMATION_DURATION, 0.0)]
    for seconds, stroke in samples:
        translate_op.Set(
            Gf.Vec3d(closed_origin[0], closed_origin[1], closed_origin[2] + stroke),
            seconds * DOOR_ANIMATION_FPS,
        )


def add_oven_instances(stage, use_proxy: bool, strict_urdf: bool, use_urdf_importer: bool, materials, oven_state=None):
    from pxr import Gf, UsdGeom

    ovens_root = UsdGeom.Xform.Define(stage, "/World/Ovens")
    proto_path = "/World/OvenPrototype"
    have_urdf = False

    if not use_proxy and use_urdf_importer:
        try:
            import_urdf_prototype(stage, proto_path)
            have_urdf = True
        except Exception as exc:
            if strict_urdf:
                raise
            print(f"[WARN] URDF import failed, using proxy boxes: {exc}")

    for row_name, col, x, y, yaw in oven_centers():
        path = f"/World/Ovens/{row_name}_oven_{col:02d}"
        state = (oven_state or {}).get(path, {})
        if have_urdf:
            prim = UsdGeom.Xform.Define(stage, path).GetPrim()
            prim.GetReferences().AddInternalReference(proto_path)
            tx, ty, tz = root_translation_for_center(x, y, yaw)
            xform = UsdGeom.Xformable(prim)
            xform.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
            xform.AddRotateZOp().Set(math.degrees(yaw))
            UsdGeom.Imageable(prim).MakeVisible()
            prim.SetInstanceable(True)
        elif use_proxy or use_urdf_importer:
            add_cube(
                stage,
                path,
                (x, y, OVEN_HEIGHT_Z / 2.0),
                (OVEN_WIDTH_X, OVEN_DEPTH_Y, OVEN_HEIGHT_Z),
                materials["oven_proxy"],
            )
        else:
            add_direct_stl_oven(
                stage,
                path,
                x,
                y,
                yaw,
                materials,
                state.get("doors"),
                state.get("leds"),
            )

    return ovens_root.GetPrim()


def add_time_sampled_translate(translate_op, positions):
    from pxr import Gf

    for seconds, position in positions:
        translate_op.Set(Gf.Vec3d(*position), seconds * DOOR_ANIMATION_FPS)


def add_time_sampled_rotate_y(rotate_op, angle_samples):
    for seconds, angle in angle_samples:
        rotate_op.Set(angle, seconds * DOOR_ANIMATION_FPS)


def yaw_delta_degrees(start: float, end: float):
    return (end - start + 180.0) % 360.0 - 180.0


def unwrap_angle_samples(angle_samples):
    unwrapped = []
    previous_angle = None
    for seconds, angle in angle_samples:
        if previous_angle is None:
            unwrapped_angle = angle
        else:
            unwrapped_angle = previous_angle + yaw_delta_degrees(previous_angle, angle)
        unwrapped.append((seconds, unwrapped_angle))
        previous_angle = unwrapped_angle
    return unwrapped


def normalized_yaw(yaw_degrees: float):
    yaw = yaw_degrees % 360.0
    return 0.0 if abs(yaw - 360.0) < 1e-6 else yaw


def cart_visual_yaw(yaw_degrees: float):
    # The drying cart is visually symmetric front-to-back, so 0 and 180 degrees
    # should not produce a visible self-rotation.
    yaw = normalized_yaw(yaw_degrees) % 180.0
    return 0.0 if abs(yaw - 180.0) < 1e-6 else yaw


def add_time_sampled_rotate_z(rotate_op, angle_samples):
    for seconds, angle in unwrap_angle_samples(angle_samples):
        rotate_op.Set(angle, seconds * DOOR_ANIMATION_FPS)


def oven_center_map():
    return {
        f"/World/Ovens/{row_name}_oven_{col:02d}": (x, y, yaw)
        for row_name, col, x, y, yaw in oven_centers()
    }


def aisle_sub_oven_sequence(row_names):
    row_order = {row_name: index for index, row_name in enumerate(row_names)}
    sub_ovens = []
    for row_name, col, x, y, yaw in oven_centers():
        if row_name not in row_order:
            continue
        oven_path = f"/World/Ovens/{row_name}_oven_{col:02d}"
        for door_side in ("left", "right"):
            sub_x, _sub_y = sub_oven_world_center(x, y, yaw, door_side)
            sub_ovens.append(
                {
                    "sort_key": (row_order[row_name], -sub_x),
                    "oven_path": oven_path,
                    "door_side": door_side,
                    "oven": (x, y, yaw),
                }
            )

    sub_ovens.sort(key=lambda item: item["sort_key"])
    return sub_ovens


def process_tasks():
    top_aisle_y, bottom_aisle_y = aisle_centers_y()
    sequences = {
        "top": aisle_sub_oven_sequence(("row_1_top", "row_2_upper_middle")),
        "bottom": aisle_sub_oven_sequence(("row_3_lower_middle", "row_4_bottom")),
    }
    aisle_y = {
        "top": top_aisle_y,
        "bottom": bottom_aisle_y,
    }
    candidates = []
    for index in range(PIPELINE_TASKS_PER_AISLE):
        candidates.append(("top", index, index * 2.0 * PIPELINE_CART_APPEAR_INTERVAL))
        candidates.append(("bottom", index, index * 2.0 * PIPELINE_CART_APPEAR_INTERVAL + PIPELINE_CART_APPEAR_INTERVAL))

    tasks = []
    for slot_index, (side, index, appear_time) in enumerate(sorted(candidates, key=lambda item: item[2]), start=1):
        sub_oven = sequences[side][index]
        task = {
            "name": side,
            "cart_path": f"/World/Carts/WetCart_{side.capitalize()}_{index + 1:02d}",
            "oven_path": sub_oven["oven_path"],
            "door_side": sub_oven["door_side"],
            "aisle_y": aisle_y[side],
            "start": appear_time,
            "service_start": appear_time,
            "slot": slot_index,
            "oven": sub_oven["oven"],
        }
        tasks.append(task)

    return schedule_process_tasks(tasks)


def inbound_travel_distance(task):
    room_x = room_length_x()
    west_feed_x = -room_x / 2.0 - 1.0
    oven_base_x, oven_base_y, oven_base_yaw = task["oven"]
    oven_x, oven_y = sub_oven_world_center(oven_base_x, oven_base_y, oven_base_yaw, task["door_side"])
    aisle_y = task["aisle_y"]
    cart_feed = (west_feed_x, aisle_y)
    cart_retreat = (west_feed_x + CARRY_RETREAT_DISTANCE, aisle_y)
    cart_turn_start = (oven_x, aisle_y)
    cart_turn_end = (
        oven_x,
        aisle_y + math.copysign(min(TURN_ENTRY_DEPTH, abs(oven_y - aisle_y)), oven_y - aisle_y),
    )
    cart_inside = (oven_x, oven_y)
    points = [cart_feed, cart_retreat, cart_turn_start, cart_turn_end, cart_inside]
    return sum(
        math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1])
        for i in range(1, len(points))
    )


def task_times(start: float, task=None):
    if task is not None and "times" in task:
        return task["times"]
    service_start = task.get("service_start", start) if task is not None else start
    if task is not None:
        insert_time = service_start + 6.0 + 0.8 + inbound_travel_distance(task) / CARRY_TRAVEL_SPEED + 1.0
    else:
        insert_time = service_start + 24.0
    work_end = insert_time + DRYING_WORK_DURATION
    return {
        "appear": start,
        "door_open_in_start": service_start + 4.0,
        "door_open_in_end": service_start + 6.0,
        "insert": insert_time,
        "door_close_in_start": insert_time + 1.0,
        "door_close_in_end": insert_time + 3.0,
        "work_end": work_end,
        "door_open_out_start": work_end,
        "door_open_out_end": work_end + 2.0,
        "extract": work_end + 6.0,
        "handoff": work_end + 15.0,
        "cool_pick": work_end + 17.0,
        "park": work_end + 27.0,
        "door_close_out_start": work_end + 16.0,
        "door_close_out_end": work_end + 18.0,
    }


def build_oven_state_for_tasks(tasks):
    state = {}
    for task in tasks:
        times = task_times(task["start"], task)
        oven_state = state.setdefault(task["oven_path"], {"doors": {}, "leds": {}})
        door_side = task["door_side"]
        door_link_side = DOOR_LINK_FOR_SUB_OVEN_SIDE[door_side]
        oven_state["doors"][door_link_side] = [
                (0.0, 0.0),
                (times["door_open_in_start"], 0.0),
                (times["door_open_in_end"], DOOR_PRISMATIC_STROKE),
                (times["door_close_in_start"], DOOR_PRISMATIC_STROKE),
                (times["door_close_in_end"], 0.0),
                (times["door_open_out_start"], 0.0),
                (times["door_open_out_end"], DOOR_PRISMATIC_STROKE),
                (times["door_close_out_start"], DOOR_PRISMATIC_STROKE),
                (times["door_close_out_end"], 0.0),
                (SCENE_ANIMATION_DURATION, 0.0),
            ]
        oven_state["leds"][door_side] = [
                (0.0, "green"),
                (times["door_close_in_end"], "green"),
                (times["door_close_in_end"] + 0.1, "red"),
                (times["work_end"], "red"),
                (times["work_end"] + 0.1, "green"),
                (SCENE_ANIMATION_DURATION, "green"),
            ]
    return state


def agv_fork_samples(load_windows):
    samples = [(0.0, (0.0, 0.0, 0.0))]
    for start, end in load_windows:
        samples.extend(
            [
                (start, (0.0, 0.0, 0.0)),
                (start + 0.8, (0.0, 0.0, AGV_FORK_LIFT_HEIGHT)),
                (end - 0.8, (0.0, 0.0, AGV_FORK_LIFT_HEIGHT)),
                (end, (0.0, 0.0, 0.0)),
            ]
        )
    samples.append((SCENE_ANIMATION_DURATION, samples[-1][1]))
    return samples


def add_agv_instance(stage, materials, name: str, root_positions, fork_windows, yaw_samples=None):
    from pxr import Gf, UsdGeom

    root_path = f"/World/AGV/{name}"
    agv_root = UsdGeom.Xform.Define(stage, root_path)
    root_xform = UsdGeom.Xformable(agv_root.GetPrim())
    root_translate = root_xform.AddTranslateOp()
    add_time_sampled_translate(root_translate, root_positions)
    root_rotate = root_xform.AddRotateZOp()
    add_time_sampled_rotate_z(root_rotate, yaw_samples or [(0.0, 0.0), (SCENE_ANIMATION_DURATION, 0.0)])

    add_stl_mesh(stage, f"{root_path}/base_link", AGV_ROOT_DIR / "meshes" / "base_link.STL", materials["agv_body"])

    wheel_specs = (
        ("left_front_wheel", "left-front-wheel.STL", (-0.15, 0.345, -0.57)),
        ("right_front_wheel", "right-front-wheel.STL", (-0.15, -0.345, -0.57)),
        ("left_back_wheel", "left-back-wheel.STL", (-0.4, 0.345, -0.57)),
        ("right_back_wheel", "right-back-wheel.STL", (-0.4, -0.345, -0.57)),
    )
    for link_name, stl_name, origin in wheel_specs:
        wheel = UsdGeom.Xform.Define(stage, f"{root_path}/{link_name}")
        wheel_xform = UsdGeom.Xformable(wheel.GetPrim())
        wheel_xform.AddTranslateOp().Set(Gf.Vec3d(*origin))
        wheel_rotate = wheel_xform.AddRotateYOp()
        wheel_rotate.Set(0.0, 0.0)
        wheel_rotate.Set(-3600.0, SCENE_ANIMATION_DURATION * DOOR_ANIMATION_FPS)
        add_stl_mesh(stage, f"{root_path}/{link_name}/visual", AGV_ROOT_DIR / "meshes" / stl_name, materials["agv_wheel"])

    fork = UsdGeom.Xform.Define(stage, f"{root_path}/fork")
    fork_xform = UsdGeom.Xformable(fork.GetPrim())
    fork_translate = fork_xform.AddTranslateOp()
    add_time_sampled_translate(fork_translate, agv_fork_samples(fork_windows))
    add_stl_mesh(stage, f"{root_path}/fork/visual", AGV_ROOT_DIR / "meshes" / "fork.STL", materials["agv_body"])

    return agv_root.GetPrim()


def cart_root_position(x: float, y: float, z: float):
    return (x - CART_LOCAL_FOOTPRINT_CENTER_X, y - CART_LOCAL_FOOTPRINT_CENTER_Y, z)


def front_cart_position(agv_x: float, agv_y: float, yaw_degrees: float, z: float, standoff: float = AGV_CART_STANDOFF):
    yaw = math.radians(yaw_degrees)
    return (
        agv_x + math.cos(yaw) * standoff,
        agv_y + math.sin(yaw) * standoff,
        z,
        yaw_degrees,
    )


def sub_oven_world_center(oven_x: float, oven_y: float, oven_yaw: float, door_side: str):
    offset = SUB_OVEN_LOCAL_X_OFFSETS[door_side]
    return (
        oven_x + math.cos(oven_yaw) * offset,
        oven_y + math.sin(oven_yaw) * offset,
    )


def agv_root_for_cart(cart_x: float, cart_y: float, yaw_degrees: float, standoff: float = AGV_CART_STANDOFF):
    yaw = math.radians(yaw_degrees)
    return (
        cart_x - math.cos(yaw) * standoff,
        cart_y - math.sin(yaw) * standoff,
        AGV_ROOT_Z,
    )


def task_motion_geometry(task):
    room_x = room_length_x()
    west_feed_x = -room_x / 2.0 - 1.0
    interface_x = room_x / 2.0
    handoff_x = interface_x - HANDOFF_CART_FRONT_OFFSET_X
    oven_base_x, oven_base_y, oven_base_yaw = task["oven"]
    oven_x, oven_y = sub_oven_world_center(oven_base_x, oven_base_y, oven_base_yaw, task["door_side"])
    aisle_y = task["aisle_y"]
    feed_yaw = 180.0
    oven_yaw = 270.0 if oven_y < aisle_y else 90.0
    outbound_aisle_yaw = 0.0
    cart_feed = (west_feed_x, aisle_y)
    cart_retreat = (west_feed_x + CARRY_RETREAT_DISTANCE, aisle_y)
    cart_turn_start = (oven_x, aisle_y)
    cart_turn_end = (
        oven_x,
        aisle_y + math.copysign(min(TURN_ENTRY_DEPTH, abs(oven_y - aisle_y)), oven_y - aisle_y),
    )
    cart_handoff = (handoff_x, aisle_y)
    return {
        "oven_x": oven_x,
        "oven_y": oven_y,
        "aisle_y": aisle_y,
        "feed_yaw": feed_yaw,
        "oven_yaw": oven_yaw,
        "outbound_aisle_yaw": outbound_aisle_yaw,
        "cart_feed": cart_feed,
        "cart_retreat": cart_retreat,
        "cart_turn_start": cart_turn_start,
        "cart_turn_end": cart_turn_end,
        "cart_handoff": cart_handoff,
        "feed_agv": agv_root_for_cart(*cart_feed, feed_yaw),
        "feed_retreat_agv": agv_root_for_cart(*cart_retreat, feed_yaw),
        "turn_start_agv": agv_root_for_cart(*cart_turn_start, feed_yaw),
        "turn_end_agv": agv_root_for_cart(*cart_turn_end, oven_yaw),
        "oven_inside_agv": agv_root_for_cart(oven_x, oven_y, oven_yaw),
        "handoff_agv": agv_root_for_cart(*cart_handoff, outbound_aisle_yaw),
        "side_retreat_agv": agv_root_for_cart(handoff_x - 1.5, aisle_y, outbound_aisle_yaw),
    }


def travel_seconds(point_a, point_b):
    return math.hypot(point_b[0] - point_a[0], point_b[1] - point_a[1]) / CARRY_TRAVEL_SPEED


def empty_turn_duration(from_yaw: float, to_yaw: float):
    return 0.0 if abs(yaw_delta_degrees(from_yaw, to_yaw)) < 1e-6 else 1.0


def empty_route_samples(start_time: float, start_position, start_yaw: float, target_position, target_yaw: float):
    samples = []
    yaw_samples = []
    seconds = start_time
    x, y, z = start_position
    target_x, target_y, _target_z = target_position
    yaw = start_yaw

    def add_pose():
        upsert_time_sample(samples, (seconds, (x, y, z)))
        upsert_time_sample(yaw_samples, (seconds, yaw))

    def turn_to(next_yaw: float):
        nonlocal seconds, yaw
        if abs(yaw_delta_degrees(yaw, next_yaw)) < 1e-6:
            return
        seconds += 1.0
        yaw = next_yaw
        add_pose()

    def straight_to(next_x: float, next_y: float, next_yaw: float):
        nonlocal seconds, x, y
        distance = math.hypot(next_x - x, next_y - y)
        if distance < 1e-6:
            return
        turn_to(next_yaw)
        seconds += distance / CARRY_TRAVEL_SPEED
        x, y = next_x, next_y
        add_pose()

    add_pose()

    y_first = abs(target_y - y) <= 0.25 and abs(target_y - y) > 1e-6
    if y_first:
        straight_to(x, target_y, 90.0 if target_y > y else 270.0)
    if abs(target_x - x) > 1e-6:
        straight_to(target_x, y, 0.0 if target_x > x else 180.0)
    if abs(target_y - y) > 1e-6:
        straight_to(target_x, target_y, 90.0 if target_y > y else 270.0)
    turn_to(target_yaw)
    return samples, yaw_samples, seconds


def empty_route_duration(start_position, start_yaw: float, target_position, target_yaw: float):
    _samples, _yaw_samples, end_time = empty_route_samples(0.0, start_position, start_yaw, target_position, target_yaw)
    return end_time


def schedule_insert_event(task, current_time, current_position, current_yaw):
    geom = task_motion_geometry(task)
    feed_travel_time = empty_route_duration(current_position, current_yaw, geom["feed_agv"], geom["feed_yaw"])
    feed_pick = max(task["start"] + 6.0, current_time + feed_travel_time)
    feed_depart_time = max(current_time, feed_pick - feed_travel_time)
    prefeed_positions, prefeed_yaws, _feed_arrival_time = empty_route_samples(
        feed_depart_time,
        current_position,
        current_yaw,
        geom["feed_agv"],
        geom["feed_yaw"],
    )
    if feed_depart_time > current_time + 1e-6:
        prefeed_positions.insert(0, (current_time, current_position))
        prefeed_yaws.insert(0, (current_time, current_yaw))
    inbound_lift_done = feed_pick + 0.8

    move_time = inbound_lift_done
    move_time += travel_seconds(geom["cart_feed"], geom["cart_retreat"])
    retreat_done = move_time
    move_time += travel_seconds(geom["cart_retreat"], geom["cart_turn_start"])
    turn_start_time = move_time
    move_time += travel_seconds(geom["cart_turn_start"], geom["cart_turn_end"])
    turn_end_time = move_time
    move_time += travel_seconds(geom["cart_turn_end"], (geom["oven_x"], geom["oven_y"]))
    oven_arrive_time = move_time

    insert_time = oven_arrive_time + 1.0
    oven_lower_done = insert_time + 1.0
    oven_exit_done = insert_time + 3.0
    work_end = insert_time + DRYING_WORK_DURATION

    task["times"] = {
        "appear": task["start"],
        "door_open_in_start": feed_pick - 2.0,
        "door_open_in_end": feed_pick,
        "insert": insert_time,
        "door_close_in_start": insert_time + 1.0,
        "door_close_in_end": insert_time + 3.0,
        "work_end": work_end,
        "door_open_out_start": work_end,
        "door_open_out_end": work_end + 2.0,
        "extract_ready": work_end + 6.0,
        "extract": work_end + 6.0,
        "handoff": work_end + 15.0,
        "cool_pick": work_end + 17.0,
        "park": work_end + 27.0,
        "door_close_out_start": work_end + 16.0,
        "door_close_out_end": work_end + 18.0,
    }
    task["schedule"] = {
        "prefeed_positions": prefeed_positions,
        "prefeed_yaws": prefeed_yaws,
        "retreat_done": retreat_done,
        "turn_start_time": turn_start_time,
        "turn_end_time": turn_end_time,
        "oven_arrive_time": oven_arrive_time,
        "oven_lower_done": oven_lower_done,
        "oven_exit_done": oven_exit_done,
    }
    task["inserted"] = True
    return oven_exit_done, geom["turn_end_agv"], geom["oven_yaw"]


def schedule_extract_event(task, current_time, current_position, current_yaw):
    geom = task_motion_geometry(task)
    times = task["times"]
    travel_to_entry = empty_route_duration(current_position, current_yaw, geom["turn_end_agv"], geom["oven_yaw"])
    outbound_pick = max(times["extract_ready"], current_time + travel_to_entry + 2.0)
    outbound_entry = outbound_pick - 2.0
    outbound_depart = max(current_time, outbound_entry - travel_to_entry)
    preoutbound_positions, preoutbound_yaws, _outbound_entry_time = empty_route_samples(
        outbound_depart,
        current_position,
        current_yaw,
        geom["turn_end_agv"],
        geom["oven_yaw"],
    )
    if outbound_depart > current_time + 1e-6:
        preoutbound_positions.insert(0, (current_time, current_position))
        preoutbound_yaws.insert(0, (current_time, current_yaw))
    outbound_lift_done = outbound_pick + 0.8

    out_time = outbound_lift_done
    out_time += travel_seconds((geom["oven_x"], geom["oven_y"]), geom["cart_turn_end"])
    outbound_turn_end_time = out_time
    out_time += travel_seconds(geom["cart_turn_end"], geom["cart_turn_start"])
    outbound_turn_start_time = out_time
    outbound_aisle_align_time = outbound_turn_start_time + 1.0
    out_time += travel_seconds(geom["cart_turn_start"], geom["cart_handoff"])
    outbound_handoff_time = out_time
    handoff_lower_done = outbound_handoff_time + 0.8
    side_retreat_done = handoff_lower_done + 0.8
    door_close_out_start = outbound_turn_end_time

    times.update(
        {
            "extract": outbound_pick,
            "handoff": outbound_handoff_time,
            "cool_pick": outbound_handoff_time + 2.0,
            "park": outbound_handoff_time + 12.0,
            "door_close_out_start": door_close_out_start,
            "door_close_out_end": door_close_out_start + 2.0,
        }
    )
    task["schedule"].update(
        {
            "outbound_entry": outbound_entry,
            "preoutbound_positions": preoutbound_positions,
            "preoutbound_yaws": preoutbound_yaws,
            "outbound_lift_done": outbound_lift_done,
            "outbound_turn_end_time": outbound_turn_end_time,
            "outbound_turn_start_time": outbound_turn_start_time,
            "outbound_aisle_align_time": outbound_aisle_align_time,
            "outbound_handoff_time": outbound_handoff_time,
            "handoff_lower_done": handoff_lower_done,
            "side_retreat_done": side_retreat_done,
        }
    )
    task["extracted"] = True
    return side_retreat_done, geom["side_retreat_agv"], geom["outbound_aisle_yaw"]


def schedule_process_tasks(tasks):
    tasks_by_side = {
        "top": sorted((task for task in tasks if task["name"] == "top"), key=lambda task: task["start"]),
        "bottom": sorted((task for task in tasks if task["name"] == "bottom"), key=lambda task: task["start"]),
    }
    top_aisle_y, bottom_aisle_y = aisle_centers_y()
    side_home = {
        "top": (0.0, top_aisle_y, AGV_ROOT_Z),
        "bottom": (0.0, bottom_aisle_y, AGV_ROOT_Z),
    }

    for side, side_tasks in tasks_by_side.items():
        pending_insert = list(side_tasks)
        current_time = 0.0
        current_position = side_home[side]
        current_yaw = 180.0

        while pending_insert or any(task.get("inserted") and not task.get("extracted") for task in side_tasks):
            ready_insert = pending_insert and pending_insert[0]["start"] <= current_time + 1e-6
            ready_extracts = [
                task
                for task in side_tasks
                if task.get("inserted")
                and not task.get("extracted")
                and task["times"]["extract_ready"] <= current_time + 1e-6
            ]

            if ready_insert:
                task = pending_insert.pop(0)
                current_time, current_position, current_yaw = schedule_insert_event(
                    task, current_time, current_position, current_yaw
                )
                continue

            if ready_extracts:
                task = min(ready_extracts, key=lambda item: item["times"]["extract_ready"])
                current_time, current_position, current_yaw = schedule_extract_event(
                    task, current_time, current_position, current_yaw
                )
                continue

            next_insert_time = pending_insert[0]["start"] if pending_insert else math.inf
            next_extract = min(
                (
                    task
                    for task in side_tasks
                    if task.get("inserted") and not task.get("extracted")
                ),
                key=lambda item: item["times"]["extract_ready"],
                default=None,
            )
            next_extract_time = next_extract["times"]["extract_ready"] if next_extract is not None else math.inf

            if next_insert_time <= next_extract_time:
                task = pending_insert.pop(0)
                current_time, current_position, current_yaw = schedule_insert_event(
                    task, current_time, current_position, current_yaw
                )
            else:
                current_time, current_position, current_yaw = schedule_extract_event(
                    next_extract, current_time, current_position, current_yaw
                )

    for task in tasks:
        task.pop("inserted", None)
        task.pop("extracted", None)
    return sorted(tasks, key=lambda task: task["start"])


def print_relative_pose_debug(task_name: str, samples):
    if not PRINT_RELATIVE_DEBUG:
        return
    print(f"[REL_DEBUG] {task_name}")
    max_rel_x_error = 0.0
    max_rel_y_error = 0.0
    max_agv_speed = 0.0
    previous = None
    for label, seconds, agv_pos, yaw_degrees, cart_pos in samples:
        yaw = math.radians(yaw_degrees)
        dx = cart_pos[0] - agv_pos[0]
        dy = cart_pos[1] - agv_pos[1]
        rel_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        rel_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        max_rel_x_error = max(max_rel_x_error, abs(rel_x - AGV_CART_STANDOFF))
        max_rel_y_error = max(max_rel_y_error, abs(rel_y))
        if previous is not None and seconds > previous[0]:
            distance = math.hypot(agv_pos[0] - previous[1][0], agv_pos[1] - previous[1][1])
            max_agv_speed = max(max_agv_speed, distance / (seconds - previous[0]))
        previous = (seconds, agv_pos)
        print(
            f"[REL_DEBUG] t={seconds:05.2f} {label:<18} "
            f"agv=({agv_pos[0]: .3f},{agv_pos[1]: .3f}) yaw={yaw_degrees: .1f} "
            f"cart=({cart_pos[0]: .3f},{cart_pos[1]: .3f}) "
            f"rel=({rel_x: .3f},{rel_y: .3f})"
        )
    print(
        f"[REL_DEBUG] {task_name} max_error "
        f"rel_x={max_rel_x_error:.6f} rel_y={max_rel_y_error:.6f} "
        f"max_agv_speed={max_agv_speed:.3f}m/s"
    )


def upsert_time_sample(samples, sample):
    seconds = sample[0]
    if samples and abs(samples[-1][0] - seconds) < 1e-6:
        samples[-1] = sample
    else:
        samples.append(sample)


def dense_carried_samples(waypoints, sample_dt: float = CARRY_SAMPLE_DT):
    agv_positions = []
    agv_yaws = []
    cart_positions = []
    debug_samples = []

    for waypoint_index, waypoint in enumerate(waypoints):
        if len(waypoint) == 5:
            label, seconds, agv_pos, yaw_degrees, cart_z = waypoint
        else:
            label, seconds, agv_pos, yaw_degrees, cart_z, _unused_cart_yaw_degrees = waypoint
        if waypoint_index == 0:
            segment_steps = 0
        else:
            prev = waypoints[waypoint_index - 1]
            duration = max(0.0, seconds - prev[1])
            segment_steps = max(1, int(math.ceil(duration / sample_dt)))

        for step in range(segment_steps + 1):
            if waypoint_index > 0 and step == 0:
                continue
            if waypoint_index == 0:
                alpha = 0.0
                t = seconds
                x, y, z = agv_pos
                yaw = yaw_degrees
                z_cart = cart_z
            else:
                prev = waypoints[waypoint_index - 1]
                if len(prev) == 5:
                    prev_label, prev_seconds, prev_agv_pos, prev_yaw, prev_cart_z = prev
                else:
                    prev_label, prev_seconds, prev_agv_pos, prev_yaw, prev_cart_z, _unused_prev_cart_yaw = prev
                alpha = step / segment_steps
                t = prev_seconds + (seconds - prev_seconds) * alpha
                x = prev_agv_pos[0] + (agv_pos[0] - prev_agv_pos[0]) * alpha
                y = prev_agv_pos[1] + (agv_pos[1] - prev_agv_pos[1]) * alpha
                z = prev_agv_pos[2] + (agv_pos[2] - prev_agv_pos[2]) * alpha
                target_yaw = prev_yaw + yaw_delta_degrees(prev_yaw, yaw_degrees)
                yaw = prev_yaw + (target_yaw - prev_yaw) * alpha
                z_cart = prev_cart_z + (cart_z - prev_cart_z) * alpha

            cart_x, cart_y, cart_z_sample, _unused_cart_yaw = front_cart_position(x, y, yaw, z_cart)
            agv_sample = (t, (x, y, z))
            yaw_sample = (t, yaw)
            cart_sample = (t, cart_x, cart_y, cart_z_sample, yaw)
            upsert_time_sample(agv_positions, agv_sample)
            upsert_time_sample(agv_yaws, yaw_sample)
            upsert_time_sample(cart_positions, cart_sample)
            debug_samples.append((label if step == segment_steps else "carry", t, (x, y, z), yaw, (cart_x, cart_y)))

    return agv_positions, agv_yaws, cart_positions, debug_samples


def add_drying_cart(stage, materials, path: str, positions, visibility_samples=None):
    from pxr import Gf, UsdGeom

    root = UsdGeom.Xform.Define(stage, path)
    root_xform = UsdGeom.Xformable(root.GetPrim())
    translate = root_xform.AddTranslateOp()
    rotate = root_xform.AddRotateZOp()

    normalized = []
    for sample in positions:
        if len(sample) == 4:
            seconds, x, y, z = sample
            normalized.append((seconds, x, y, z, 0.0, False))
        else:
            seconds, x, y, z, yaw = sample
            normalized.append((seconds, x, y, z, cart_visual_yaw(yaw), True))

    if any(sample[-1] for sample in normalized):
        add_time_sampled_translate(translate, [(seconds, (x, y, z)) for seconds, x, y, z, _yaw, _centered in normalized])
        add_time_sampled_rotate_z(rotate, [(seconds, yaw) for seconds, _x, _y, _z, yaw, _centered in normalized])
        mesh_frame = UsdGeom.Xform.Define(stage, f"{path}/mesh_frame")
        mesh_xform = UsdGeom.Xformable(mesh_frame.GetPrim())
        mesh_xform.AddTranslateOp().Set(
            Gf.Vec3d(-CART_LOCAL_FOOTPRINT_CENTER_X, -CART_LOCAL_FOOTPRINT_CENTER_Y, 0.0)
        )
        mesh_path = f"{path}/mesh_frame/base_link"
    else:
        add_time_sampled_translate(
            translate,
            [(seconds, cart_root_position(x, y, z)) for seconds, x, y, z, _yaw, _centered in normalized],
        )
        mesh_path = f"{path}/base_link"

    if visibility_samples is not None:
        set_visibility_samples(root.GetPrim(), visibility_samples)
    add_stl_mesh(stage, mesh_path, CART_ROOT_DIR / "meshes" / "base_link.STL", materials["cart"])
    return root.GetPrim()


def cooling_slot_centers(room_x: float, room_y: float):
    layout = cooling_layout(room_x, room_y)
    x_centers = layout["slot_x_centers"]
    y_centers = layout["slot_y_centers"]

    centers = []
    for y in y_centers:
        for x in x_centers:
            centers.append((x, y))
    return centers


def cooling_passage_width_y(room_y: float):
    if COOLING_SLOT_ROWS <= 1:
        return COOLING_PASSAGE_WIDTH_Y
    usable_y = room_y - 2.0 * WALL_THICKNESS
    distributed = (usable_y - COOLING_SLOT_ROWS * COOLING_SLOT_SIZE_Y) / (COOLING_SLOT_ROWS - 1)
    return max(COOLING_PASSAGE_WIDTH_Y, distributed)


def cooling_layout(room_x: float, room_y: float):
    x_min = room_x / 2.0 + WALL_THICKNESS
    x_max = room_x / 2.0 + COOLING_ROOM_WIDTH_X - WALL_THICKNESS
    slot_x_max = x_max - COOLING_EAST_WALL_CLEARANCE_X
    longitudinal_x_min = x_min
    longitudinal_x_max = x_min + COOLING_LONGITUDINAL_PASSAGE_WIDTH_X
    slot_x_min = longitudinal_x_max
    passage_width_y = cooling_passage_width_y(room_y)
    occupied_y = COOLING_SLOT_ROWS * COOLING_SLOT_SIZE_Y + (COOLING_SLOT_ROWS - 1) * passage_width_y
    y_start = -room_y / 2.0 + (room_y - occupied_y) / 2.0
    slot_y_centers = [
        y_start + COOLING_SLOT_SIZE_Y / 2.0 + row * (COOLING_SLOT_SIZE_Y + passage_width_y)
        for row in range(COOLING_SLOT_ROWS)
    ]
    passage_y_centers = [
        y_start + COOLING_SLOT_SIZE_Y + passage_width_y / 2.0 + passage * (COOLING_SLOT_SIZE_Y + passage_width_y)
        for passage in range(COOLING_SLOT_ROWS - 1)
    ]
    return {
        "x_min": x_min,
        "x_max": x_max,
        "slot_x_max": slot_x_max,
        "longitudinal_x_min": longitudinal_x_min,
        "longitudinal_x_max": longitudinal_x_max,
        "longitudinal_x_center": (longitudinal_x_min + longitudinal_x_max) / 2.0,
        "slot_x_min": slot_x_min,
        "slot_x_centers": evenly_spaced_centers(slot_x_min, slot_x_max, COOLING_SLOT_SIZE_X, COOLING_SLOT_COLUMNS),
        "slot_y_centers": slot_y_centers,
        "passage_y_centers": passage_y_centers,
        "passage_width_y": passage_width_y,
        "y_start": y_start,
    }


def cooling_slot_parking_plan(slot_index: int, slot_x: float, slot_y: float, room_x: float, room_y: float):
    row_index = (slot_index - 1) // COOLING_SLOT_COLUMNS
    layout = cooling_layout(room_x, room_y)
    passage_y_centers = layout["passage_y_centers"]
    if row_index <= 0:
        return passage_y_centers[0], 270.0
    if row_index >= COOLING_SLOT_ROWS - 1:
        return passage_y_centers[-1], 90.0
    if slot_y >= 0.0:
        return passage_y_centers[row_index - 1], 90.0
    return passage_y_centers[row_index], 270.0


def route_position_samples(points, start_time: float, speed: float, yaw_degrees: float):
    samples = []
    yaw_samples = []
    current_time = start_time
    previous = None
    for point in points:
        if previous is not None:
            current_time += math.hypot(point[0] - previous[0], point[1] - previous[1]) / speed
        sample = (current_time, (point[0], point[1], AGV_ROOT_Z))
        upsert_time_sample(samples, sample)
        upsert_time_sample(yaw_samples, (current_time, yaw_degrees))
        previous = point
    return samples, yaw_samples, current_time


def add_pose_key(samples, yaw_samples, seconds: float, x: float, y: float, yaw_degrees: float):
    pose = (x, y, AGV_ROOT_Z)
    yaw = normalized_yaw(yaw_degrees)
    upsert_time_sample(samples, (seconds, pose))
    upsert_time_sample(yaw_samples, (seconds, yaw))
    return seconds, pose, yaw


def add_straight_pose(samples, yaw_samples, seconds: float, x: float, y: float, yaw_degrees: float, target_x: float, target_y: float):
    distance = math.hypot(target_x - x, target_y - y)
    seconds += distance / CARRY_TRAVEL_SPEED
    return add_pose_key(samples, yaw_samples, seconds, target_x, target_y, yaw_degrees)


def add_turn_pose(samples, yaw_samples, seconds: float, x: float, y: float, yaw_degrees: float, target_yaw: float):
    seconds += abs(yaw_delta_degrees(yaw_degrees, target_yaw)) / 90.0
    return add_pose_key(samples, yaw_samples, seconds, x, y, target_yaw)


def build_unloaded_cooling_route(start_pose, pick_agv, aisle_y: float, depart_time: float, longitudinal_x: float):
    samples = []
    yaw_samples = []
    x, y, _z, yaw = start_pose
    seconds, _pose, yaw = add_pose_key(samples, yaw_samples, depart_time, x, y, yaw)

    if abs(x - longitudinal_x) > 1e-3:
        target_yaw = 180.0 if longitudinal_x < x else 0.0
        seconds, _pose, yaw = add_turn_pose(samples, yaw_samples, seconds, x, y, yaw, target_yaw)
        seconds, pose, yaw = add_straight_pose(samples, yaw_samples, seconds, x, y, yaw, longitudinal_x, y)
        x, y, _z = pose

    target_yaw = 90.0 if aisle_y > y else 270.0
    seconds, _pose, yaw = add_turn_pose(samples, yaw_samples, seconds, x, y, yaw, target_yaw)
    seconds, pose, yaw = add_straight_pose(samples, yaw_samples, seconds, x, y, yaw, longitudinal_x, aisle_y)
    x, y, _z = pose

    seconds, _pose, yaw = add_turn_pose(samples, yaw_samples, seconds, x, y, yaw, 180.0)
    seconds, pose, yaw = add_straight_pose(samples, yaw_samples, seconds, x, y, yaw, pick_agv[0], pick_agv[1])
    return samples, yaw_samples, seconds


def add_process_flow(stage, materials):
    room_x = room_length_x()
    room_y = room_width_y()
    west_feed_x = -room_x / 2.0 - 1.0
    interface_x = room_x / 2.0
    handoff_x = interface_x - HANDOFF_CART_FRONT_OFFSET_X
    cooling_slots = cooling_slot_centers(room_x, room_y)
    cooling_map = cooling_layout(room_x, room_y)
    cooling_home = (cooling_map["longitudinal_x_center"], 0.0)
    tasks = sorted(process_tasks(), key=lambda task: task_times(task["start"], task)["handoff"])

    inbound_agv_paths = {"top": [], "bottom": []}
    inbound_fork_windows = {"top": [], "bottom": []}
    inbound_agv_yaws = {"top": [], "bottom": []}
    cooling_positions = []
    cooling_yaws = []
    cooling_fork_windows = []
    cooling_last_pose = None
    cooling_last_time = 0.0

    for task in tasks:
        times = task_times(task["start"], task)
        schedule = task["schedule"]
        oven_base_x, oven_base_y, oven_base_yaw = task["oven"]
        oven_x, oven_y = sub_oven_world_center(oven_base_x, oven_base_y, oven_base_yaw, task["door_side"])
        aisle_y = task["aisle_y"]
        slot_x, slot_y = cooling_slots[task["slot"] - 1]
        feed_pick = times["door_open_in_end"]
        inbound_lift_done = feed_pick + 0.8
        oven_place = times["insert"]
        oven_lower_done = schedule["oven_lower_done"]
        oven_exit_done = schedule["oven_exit_done"]
        outbound_entry = schedule["outbound_entry"]
        outbound_pick = times["extract"]
        outbound_lift_done = schedule["outbound_lift_done"]
        handoff_lower_done = schedule["handoff_lower_done"]
        feed_yaw = 180.0
        oven_yaw = 270.0 if oven_y < aisle_y else 90.0
        cart_feed = (west_feed_x, aisle_y)
        cart_retreat = (west_feed_x + CARRY_RETREAT_DISTANCE, aisle_y)
        cart_turn_start = (oven_x, aisle_y)
        cart_turn_end = (
            oven_x,
            aisle_y + math.copysign(min(TURN_ENTRY_DEPTH, abs(oven_y - aisle_y)), oven_y - aisle_y),
        )
        cart_handoff = (handoff_x, aisle_y)
        feed_agv = agv_root_for_cart(*cart_feed, feed_yaw)
        feed_retreat_agv = agv_root_for_cart(*cart_retreat, feed_yaw)
        turn_yaw = (feed_yaw + oven_yaw) / 2.0
        outbound_aisle_yaw = 0.0
        outbound_turn_yaw = (oven_yaw + outbound_aisle_yaw) / 2.0
        turn_start_agv = agv_root_for_cart(*cart_turn_start, feed_yaw)
        turn_end_agv = agv_root_for_cart(*cart_turn_end, oven_yaw)
        oven_loaded_agv = agv_root_for_cart(*cart_turn_start, oven_yaw)
        oven_inside_agv = agv_root_for_cart(oven_x, oven_y, oven_yaw)
        handoff_agv = agv_root_for_cart(*cart_handoff, outbound_aisle_yaw)
        side = task["name"]
        prefeed_positions = schedule["prefeed_positions"]
        prefeed_yaws = schedule["prefeed_yaws"]
        preoutbound_positions = schedule["preoutbound_positions"]
        preoutbound_yaws = schedule["preoutbound_yaws"]
        cooling_pick_yaw = 180.0
        cooling_pick_agv = agv_root_for_cart(*cart_handoff, cooling_pick_yaw)
        cooling_start_pose = cooling_last_pose or (cooling_home[0], cooling_home[1], AGV_ROOT_Z, 90.0)
        cooling_depart_time = max(cooling_last_time, times["handoff"] + COOLING_UNLOADED_WAIT_AFTER_HANDOFF)
        cooling_pre_pick_positions, cooling_pre_pick_yaws, cooling_pick_time = build_unloaded_cooling_route(
            cooling_start_pose,
            cooling_pick_agv,
            aisle_y,
            cooling_depart_time,
            cooling_map["longitudinal_x_center"],
        )
        cooling_lift_done = cooling_pick_time + 0.8
        slot_passage_y, slot_cart_yaw = cooling_slot_parking_plan(task["slot"], slot_x, slot_y, room_x, room_y)
        slot_passage_agv = (slot_x, slot_passage_y, AGV_ROOT_Z)
        slot_agv = agv_root_for_cart(slot_x, slot_y, slot_cart_yaw)
        cooling_retreat_agv = slot_passage_agv
        cooling_longitudinal_agv = (cooling_map["longitudinal_x_center"], aisle_y, AGV_ROOT_Z)
        cooling_passage_entry_agv = (cooling_map["longitudinal_x_center"], slot_passage_y, AGV_ROOT_Z)
        cooling_y_travel_yaw = 90.0 if slot_passage_y > aisle_y else 270.0
        cooling_to_longitudinal_time = cooling_lift_done + math.hypot(
            cooling_longitudinal_agv[0] - cooling_pick_agv[0],
            cooling_longitudinal_agv[1] - cooling_pick_agv[1],
        ) / CARRY_TRAVEL_SPEED
        cooling_turn_vertical_time = cooling_to_longitudinal_time + 1.0
        cooling_to_passage_time = cooling_turn_vertical_time + math.hypot(
            cooling_passage_entry_agv[0] - cooling_longitudinal_agv[0],
            cooling_passage_entry_agv[1] - cooling_longitudinal_agv[1],
        ) / CARRY_TRAVEL_SPEED
        cooling_turn_east_time = cooling_to_passage_time + 1.0
        cooling_to_slot_approach_time = cooling_turn_east_time + math.hypot(
            slot_passage_agv[0] - cooling_passage_entry_agv[0],
            slot_passage_agv[1] - cooling_passage_entry_agv[1],
        ) / CARRY_TRAVEL_SPEED
        cooling_turn_slot_time = cooling_to_slot_approach_time + 1.0
        cooling_slot_insert_time = cooling_turn_slot_time + math.hypot(
            slot_agv[0] - slot_passage_agv[0],
            slot_agv[1] - slot_passage_agv[1],
        ) / CARRY_TRAVEL_SPEED
        cooling_slot_lower_time = cooling_slot_insert_time + 0.8

        move_time = inbound_lift_done

        def next_move_time(previous_cart, next_cart):
            nonlocal move_time
            move_time += math.hypot(next_cart[0] - previous_cart[0], next_cart[1] - previous_cart[1]) / CARRY_TRAVEL_SPEED
            return move_time

        retreat_done = next_move_time(cart_feed, cart_retreat)
        turn_start_time = next_move_time(cart_retreat, cart_turn_start)
        turn_end_time = next_move_time(cart_turn_start, cart_turn_end)
        oven_arrive_time = next_move_time(cart_turn_end, (oven_x, oven_y))

        out_move_time = outbound_lift_done

        def next_out_time(previous_cart, next_cart):
            nonlocal out_move_time
            out_move_time += math.hypot(next_cart[0] - previous_cart[0], next_cart[1] - previous_cart[1]) / CARRY_TRAVEL_SPEED
            return out_move_time

        outbound_turn_end_time = next_out_time((oven_x, oven_y), cart_turn_end)
        outbound_turn_start_time = next_out_time(cart_turn_end, cart_turn_start)
        outbound_aisle_align_time = outbound_turn_start_time + 1.0
        outbound_handoff_time = next_out_time(cart_turn_start, cart_handoff)
        handoff_lower_done = outbound_handoff_time + 0.8
        cooling_depart_time = max(cooling_last_time, outbound_handoff_time + COOLING_UNLOADED_WAIT_AFTER_HANDOFF)
        cooling_pre_pick_positions, cooling_pre_pick_yaws, cooling_pick_time = build_unloaded_cooling_route(
            cooling_start_pose,
            cooling_pick_agv,
            aisle_y,
            cooling_depart_time,
            cooling_map["longitudinal_x_center"],
        )
        cooling_lift_done = cooling_pick_time + 0.8
        cooling_to_longitudinal_time = cooling_lift_done + math.hypot(
            cooling_longitudinal_agv[0] - cooling_pick_agv[0],
            cooling_longitudinal_agv[1] - cooling_pick_agv[1],
        ) / CARRY_TRAVEL_SPEED
        cooling_turn_vertical_time = cooling_to_longitudinal_time + 1.0
        cooling_to_passage_time = cooling_turn_vertical_time + math.hypot(
            cooling_passage_entry_agv[0] - cooling_longitudinal_agv[0],
            cooling_passage_entry_agv[1] - cooling_longitudinal_agv[1],
        ) / CARRY_TRAVEL_SPEED
        cooling_turn_east_time = cooling_to_passage_time + 1.0
        cooling_to_slot_approach_time = cooling_turn_east_time + math.hypot(
            slot_passage_agv[0] - cooling_passage_entry_agv[0],
            slot_passage_agv[1] - cooling_passage_entry_agv[1],
        ) / CARRY_TRAVEL_SPEED
        cooling_turn_slot_time = cooling_to_slot_approach_time + 1.0
        cooling_slot_insert_time = cooling_turn_slot_time + math.hypot(
            slot_agv[0] - slot_passage_agv[0],
            slot_agv[1] - slot_passage_agv[1],
        ) / CARRY_TRAVEL_SPEED
        cooling_slot_lower_time = cooling_slot_insert_time + 0.8

        inbound_agv_dense, inbound_yaw_dense, inbound_cart_dense, inbound_debug = dense_carried_samples(
            [
                ("feed_pick", feed_pick, feed_agv, feed_yaw, CART_PARKED_Z),
                ("lift_done", inbound_lift_done, feed_agv, feed_yaw, CART_CARRIED_Z),
                ("feed_retreat", retreat_done, feed_retreat_agv, feed_yaw, CART_CARRIED_Z),
                ("turn_start", turn_start_time, turn_start_agv, feed_yaw, CART_CARRIED_Z),
                ("turn_end", turn_end_time, turn_end_agv, oven_yaw, CART_CARRIED_Z),
                ("oven_inside", oven_arrive_time, oven_inside_agv, oven_yaw, CART_CARRIED_Z),
                ("before_lower", oven_lower_done, oven_inside_agv, oven_yaw, CART_IN_OVEN_Z),
            ],
        )
        outbound_agv_dense, outbound_yaw_dense, outbound_cart_dense, outbound_debug = dense_carried_samples(
            [
                ("out_pick", outbound_pick, oven_inside_agv, oven_yaw, CART_IN_OVEN_Z),
                ("out_lift", outbound_lift_done, oven_inside_agv, oven_yaw, CART_CARRIED_Z),
                ("turn_end", outbound_turn_end_time, turn_end_agv, oven_yaw, CART_CARRIED_Z),
                ("turn_start", outbound_turn_start_time, turn_start_agv, outbound_turn_yaw, CART_CARRIED_Z),
                ("out_aisle", outbound_aisle_align_time, turn_start_agv, outbound_aisle_yaw, CART_CARRIED_Z),
                ("handoff", outbound_handoff_time, handoff_agv, outbound_aisle_yaw, CART_CARRIED_Z),
            ],
        )
        cooling_agv_dense, cooling_yaw_dense, cooling_cart_dense, cooling_debug = dense_carried_samples(
            [
                ("cool_pick", cooling_pick_time, cooling_pick_agv, cooling_pick_yaw, CART_PARKED_Z),
                ("cool_lift", cooling_lift_done, cooling_pick_agv, cooling_pick_yaw, CART_CARRIED_Z),
                ("cool_reverse", cooling_to_longitudinal_time, cooling_longitudinal_agv, cooling_pick_yaw, CART_CARRIED_Z),
                ("turn_vertical", cooling_turn_vertical_time, cooling_longitudinal_agv, cooling_y_travel_yaw, CART_CARRIED_Z),
                ("cool_passage_entry", cooling_to_passage_time, cooling_passage_entry_agv, cooling_y_travel_yaw, CART_CARRIED_Z),
                ("turn_east", cooling_turn_east_time, cooling_passage_entry_agv, 0.0, CART_CARRIED_Z),
                ("cool_slot_approach", cooling_to_slot_approach_time, slot_passage_agv, 0.0, CART_CARRIED_Z),
                ("turn_slot", cooling_turn_slot_time, slot_passage_agv, slot_cart_yaw, CART_CARRIED_Z),
                ("slot_insert", cooling_slot_insert_time, slot_agv, slot_cart_yaw, CART_CARRIED_Z),
                ("slot_lower", cooling_slot_lower_time, slot_agv, slot_cart_yaw, CART_PARKED_Z),
            ],
        )
        cooling_park_time = cooling_cart_dense[-1][0]
        print_relative_pose_debug(
            task["name"],
            inbound_debug + outbound_debug,
        )
        appear_visibility = [(0.0, True)] if times["appear"] == 0.0 else [
            (0.0, False),
            (max(0.0, times["appear"] - 0.01), False),
            (times["appear"], True),
        ]

        add_drying_cart(
            stage,
            materials,
            task["cart_path"],
            [
                (0.0, *cart_feed, CART_PARKED_Z, feed_yaw),
                (times["appear"], *cart_feed, CART_PARKED_Z, feed_yaw),
                *inbound_cart_dense,
                (times["extract"], oven_x, oven_y, CART_IN_OVEN_Z, oven_yaw),
                *outbound_cart_dense,
                (handoff_lower_done, *cart_handoff, CART_PARKED_Z, outbound_aisle_yaw),
                (cooling_pick_time, *cart_handoff, CART_PARKED_Z, outbound_aisle_yaw),
                *cooling_cart_dense,
                (SCENE_ANIMATION_DURATION, slot_x, slot_y, CART_PARKED_Z, slot_cart_yaw),
            ],
            appear_visibility,
        )

        side_retreat_agv = agv_root_for_cart(handoff_x - 1.5, aisle_y, outbound_aisle_yaw)
        side_retreat_done = handoff_lower_done + 0.8
        inbound_agv_paths[side].extend(
            [
                *prefeed_positions,
                *inbound_agv_dense,
                (oven_exit_done, turn_end_agv),
                *preoutbound_positions,
                *outbound_agv_dense,
                (handoff_lower_done, handoff_agv),
                (side_retreat_done, side_retreat_agv),
            ]
        )
        inbound_fork_windows[side].extend(
            [
                (feed_pick, oven_lower_done),
                (times["extract"], handoff_lower_done),
            ]
        )
        inbound_agv_yaws[side].extend(
            [
                *prefeed_yaws,
                *inbound_yaw_dense,
                (oven_exit_done, oven_yaw),
                *preoutbound_yaws,
                *outbound_yaw_dense,
                (side_retreat_done, outbound_aisle_yaw),
            ]
        )

        if cooling_last_pose is None:
            cooling_positions.extend(
                [
                    (0.0, cooling_start_pose[:3]),
                    (cooling_depart_time, cooling_start_pose[:3]),
                ]
            )
            cooling_yaws.insert(0, (0.0, cooling_start_pose[3]))
            cooling_yaws.append((cooling_depart_time, cooling_start_pose[3]))
        else:
            cooling_positions.extend(
                [
                    (cooling_last_time, cooling_last_pose[:3]),
                    (cooling_depart_time, cooling_last_pose[:3]),
                ]
            )
            cooling_yaws.extend(
                [
                    (cooling_last_time, cooling_last_pose[3]),
                    (cooling_depart_time, cooling_last_pose[3]),
                ]
            )
        cooling_positions.extend(cooling_pre_pick_positions)
        cooling_yaws.extend(cooling_pre_pick_yaws)
        cooling_positions.extend(cooling_agv_dense)
        cooling_yaws.extend(cooling_yaw_dense)
        cooling_positions.append((cooling_park_time + 1.2, cooling_retreat_agv))
        cooling_yaws.append((cooling_park_time + 1.2, slot_cart_yaw))
        cooling_last_pose = (cooling_retreat_agv[0], cooling_retreat_agv[1], cooling_retreat_agv[2], slot_cart_yaw)
        cooling_last_time = cooling_park_time + 1.2
        print_relative_pose_debug(
            f"cooling_{task['name']}",
            cooling_debug,
        )
        cooling_fork_windows.append((cooling_pick_time, cooling_park_time))

    for side, positions in inbound_agv_paths.items():
        if positions:
            positions = sorted(positions, key=lambda sample: sample[0])
            yaw_samples = sorted(inbound_agv_yaws[side], key=lambda sample: sample[0])
            positions.append((SCENE_ANIMATION_DURATION, positions[-1][1]))
            yaw_samples.append((SCENE_ANIMATION_DURATION, yaw_samples[-1][1]))
            add_agv_instance(stage, materials, f"F4_1000C_{side}", positions, inbound_fork_windows[side], yaw_samples)

    cooling_positions = sorted(cooling_positions, key=lambda sample: sample[0])
    cooling_positions.append((SCENE_ANIMATION_DURATION, cooling_positions[-1][1]))
    add_agv_instance(stage, materials, "F4_1000C_cooling", cooling_positions, cooling_fork_windows, cooling_yaws)


def add_agv(stage, materials):
    from pxr import Gf, UsdGeom

    _, bottom_aisle_y = aisle_centers_y()
    agv_root = UsdGeom.Xform.Define(stage, "/World/AGV/F4_1000C")
    root_xform = UsdGeom.Xformable(agv_root.GetPrim())
    root_translate = root_xform.AddTranslateOp()
    add_time_sampled_translate(
        root_translate,
        [
            (0.0, (AGV_START_X, bottom_aisle_y, AGV_ROOT_Z)),
            (10.0, (AGV_END_X, bottom_aisle_y, AGV_ROOT_Z)),
            (SCENE_ANIMATION_DURATION, (AGV_END_X, bottom_aisle_y, AGV_ROOT_Z)),
        ],
    )

    add_stl_mesh(
        stage,
        "/World/AGV/F4_1000C/base_link",
        AGV_ROOT_DIR / "meshes" / "base_link.STL",
        materials["agv_body"],
    )

    wheel_specs = (
        ("left_front_wheel", "left-front-wheel.STL", (-0.15, 0.345, -0.57)),
        ("right_front_wheel", "right-front-wheel.STL", (-0.15, -0.345, -0.57)),
        ("left_back_wheel", "left-back-wheel.STL", (-0.4, 0.345, -0.57)),
        ("right_back_wheel", "right-back-wheel.STL", (-0.4, -0.345, -0.57)),
    )
    travel = AGV_END_X - AGV_START_X
    wheel_angle = math.degrees(travel / AGV_WHEEL_RADIUS)
    for link_name, stl_name, origin in wheel_specs:
        wheel = UsdGeom.Xform.Define(stage, f"/World/AGV/F4_1000C/{link_name}")
        wheel_xform = UsdGeom.Xformable(wheel.GetPrim())
        wheel_xform.AddTranslateOp().Set(Gf.Vec3d(*origin))
        wheel_rotate = wheel_xform.AddRotateYOp()
        add_time_sampled_rotate_y(
            wheel_rotate,
            [
                (0.0, 0.0),
                (10.0, -wheel_angle),
                (SCENE_ANIMATION_DURATION, -wheel_angle),
            ],
        )
        add_stl_mesh(
            stage,
            f"/World/AGV/F4_1000C/{link_name}/visual",
            AGV_ROOT_DIR / "meshes" / stl_name,
            materials["agv_wheel"],
        )

    fork = UsdGeom.Xform.Define(stage, "/World/AGV/F4_1000C/fork")
    fork_xform = UsdGeom.Xformable(fork.GetPrim())
    fork_translate = fork_xform.AddTranslateOp()
    add_time_sampled_translate(
        fork_translate,
        [
            (0.0, (0.0, 0.0, 0.0)),
            (AGV_FORK_LIFT_START, (0.0, 0.0, 0.0)),
            (AGV_FORK_LIFT_END, (0.0, 0.0, AGV_FORK_LIFT_HEIGHT)),
            (SCENE_ANIMATION_DURATION, (0.0, 0.0, AGV_FORK_LIFT_HEIGHT)),
        ],
    )
    add_stl_mesh(
        stage,
        "/World/AGV/F4_1000C/fork/visual",
        AGV_ROOT_DIR / "meshes" / "fork.STL",
        materials["agv_body"],
    )

    return agv_root.GetPrim()


def evenly_spaced_centers(min_value: float, max_value: float, item_size: float, count: int):
    gap = (max_value - min_value - count * item_size) / (count + 1)
    return [min_value + gap + item_size / 2.0 + i * (item_size + gap) for i in range(count)]


def add_cooling_parking_slots(stage, materials, room_x: float, room_y: float, top_aisle_y: float, bottom_aisle_y: float):
    x_min = room_x / 2.0 + WALL_THICKNESS
    x_max = room_x / 2.0 + COOLING_ROOM_WIDTH_X - WALL_THICKNESS
    slot_x_max = x_max - COOLING_EAST_WALL_CLEARANCE_X
    y_min = -room_y / 2.0
    y_max = room_y / 2.0

    longitudinal_x_min = x_min
    longitudinal_x_max = x_min + COOLING_LONGITUDINAL_PASSAGE_WIDTH_X
    slot_x_min = longitudinal_x_max
    x_centers = evenly_spaced_centers(
        slot_x_min,
        slot_x_max,
        COOLING_SLOT_SIZE_X,
        COOLING_SLOT_COLUMNS,
    )

    passage_width_y = cooling_passage_width_y(room_y)
    occupied_y = COOLING_SLOT_ROWS * COOLING_SLOT_SIZE_Y + (COOLING_SLOT_ROWS - 1) * passage_width_y
    y_start = y_min + (room_y - occupied_y) / 2.0
    y_centers = [
        y_start + COOLING_SLOT_SIZE_Y / 2.0 + row * (COOLING_SLOT_SIZE_Y + passage_width_y)
        for row in range(COOLING_SLOT_ROWS)
    ]

    add_cube(
        stage,
        "/World/CoolingRoom/Passages/Longitudinal",
        ((longitudinal_x_min + longitudinal_x_max) / 2.0, 0, 0.015),
        (COOLING_LONGITUDINAL_PASSAGE_WIDTH_X, room_y, 0.012),
        materials["aisle"],
    )

    for passage_index in range(COOLING_SLOT_ROWS - 1):
        y_center = y_start + COOLING_SLOT_SIZE_Y + passage_width_y / 2.0 + passage_index * (COOLING_SLOT_SIZE_Y + passage_width_y)
        add_cube(
            stage,
            f"/World/CoolingRoom/Passages/Cross_{passage_index + 1:02d}",
            ((slot_x_min + slot_x_max) / 2.0, y_center, 0.015),
            (slot_x_max - slot_x_min, passage_width_y, 0.012),
            materials["aisle"],
        )

    slot_index = 1
    for y in y_centers:
        for x in x_centers:
            add_cube(
                stage,
                f"/World/CoolingRoom/ParkingSlots/Slot_{slot_index:03d}",
                (x, y, 0.02),
                (COOLING_SLOT_SIZE_X, COOLING_SLOT_SIZE_Y, 0.01),
                materials["cooling_slot"],
            )
            slot_index += 1


def add_room(stage, materials):
    room_x = room_length_x()
    room_y = room_width_y()
    top_aisle_y, bottom_aisle_y = aisle_centers_y()
    wall_size_z = WALL_HEIGHT + WALL_FLOOR_OVERLAP
    wall_z = WALL_HEIGHT / 2.0 - WALL_FLOOR_OVERLAP / 2.0

    add_cube(
        stage,
        "/World/Room/Floor",
        (0, 0, -FLOOR_THICKNESS / 2.0),
        (GROUND_LENGTH_X, GROUND_WIDTH_Y, FLOOR_THICKNESS),
        materials["floor"],
    )

    north_south_y = room_y / 2.0 - WALL_THICKNESS / 2.0
    east_west_x = room_x / 2.0 - WALL_THICKNESS / 2.0
    add_cube(stage, "/World/Room/Wall_North", (0, north_south_y, wall_z), (room_x, WALL_THICKNESS, wall_size_z), materials["wall"])
    add_cube(stage, "/World/Room/Wall_South", (0, -north_south_y, wall_z), (room_x, WALL_THICKNESS, wall_size_z), materials["wall"])

    cooling_center_x = room_x / 2.0 + COOLING_ROOM_WIDTH_X / 2.0
    add_cube(
        stage,
        "/World/CoolingRoom/Floor",
        (cooling_center_x, 0, 0.003),
        (COOLING_ROOM_WIDTH_X, room_y, 0.012),
        materials["cooling_floor"],
    )
    add_cube(stage, "/World/CoolingRoom/Wall_North", (cooling_center_x, north_south_y, wall_z), (COOLING_ROOM_WIDTH_X, WALL_THICKNESS, wall_size_z), materials["wall"])
    add_cube(stage, "/World/CoolingRoom/Wall_South", (cooling_center_x, -north_south_y, wall_z), (COOLING_ROOM_WIDTH_X, WALL_THICKNESS, wall_size_z), materials["wall"])
    add_cube(stage, "/World/CoolingRoom/Wall_East", (room_x / 2.0 + COOLING_ROOM_WIDTH_X - WALL_THICKNESS / 2.0, 0, wall_z), (WALL_THICKNESS, room_y, wall_size_z), materials["wall"])
    add_cooling_parking_slots(stage, materials, room_x, room_y, top_aisle_y, bottom_aisle_y)

    opening_ranges = [
        (bottom_aisle_y - AISLE_WIDTH / 2.0, bottom_aisle_y + AISLE_WIDTH / 2.0),
        (top_aisle_y - AISLE_WIDTH / 2.0, top_aisle_y + AISLE_WIDTH / 2.0),
    ]
    y_cursor = -room_y / 2.0
    wall_segments = []
    for opening_min_y, opening_max_y in sorted(opening_ranges):
        if opening_min_y > y_cursor:
            wall_segments.append((y_cursor, opening_min_y))
        y_cursor = max(y_cursor, opening_max_y)
    if y_cursor < room_y / 2.0:
        wall_segments.append((y_cursor, room_y / 2.0))

    for index, (segment_min_y, segment_max_y) in enumerate(wall_segments, start=1):
        segment_center_y = (segment_min_y + segment_max_y) / 2.0
        segment_width_y = segment_max_y - segment_min_y
        add_cube(stage, f"/World/Room/Wall_East_{index}", (east_west_x, segment_center_y, wall_z), (WALL_THICKNESS, segment_width_y, wall_size_z), materials["wall"])
        add_cube(stage, f"/World/Room/Wall_West_{index}", (-east_west_x, segment_center_y, wall_z), (WALL_THICKNESS, segment_width_y, wall_size_z), materials["wall"])

    cooling_west_x = room_x / 2.0 + WALL_THICKNESS / 2.0
    for index, (segment_min_y, segment_max_y) in enumerate(wall_segments, start=1):
        segment_center_y = (segment_min_y + segment_max_y) / 2.0
        segment_width_y = segment_max_y - segment_min_y
        add_cube(stage, f"/World/CoolingRoom/Wall_West_{index}", (cooling_west_x, segment_center_y, wall_z), (WALL_THICKNESS, segment_width_y, wall_size_z), materials["wall"])

    # Slightly tinted floor strips mark the two aisles and extend the wet-material entrance.
    aisle_length_x = room_x + AISLE_ENTRANCE_EXTENSION
    aisle_center_x = -AISLE_ENTRANCE_EXTENSION / 2.0
    add_cube(stage, "/World/Room/Aisle_Top", (aisle_center_x, top_aisle_y, 0.002), (aisle_length_x, AISLE_WIDTH, 0.01), materials["aisle"])
    add_cube(stage, "/World/Room/Aisle_Bottom", (aisle_center_x, bottom_aisle_y, 0.002), (aisle_length_x, AISLE_WIDTH, 0.01), materials["aisle"])


def add_lighting_and_camera(stage):
    from pxr import Gf, UsdGeom, UsdLux

    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/Dome")
    dome.CreateIntensityAttr(350.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/Key")
    sun.CreateIntensityAttr(1500.0)
    sun.CreateAngleAttr(0.5)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(-55.0, 0.0, 35.0))

    camera = UsdGeom.Camera.Define(stage, "/World/Camera")
    camera.AddTranslateOp().Set(Gf.Vec3d(0.0, -18.0, 16.0))
    camera.AddRotateXYZOp().Set(Gf.Vec3f(58.0, 0.0, 0.0))
    camera.CreateFocalLengthAttr(18.0)


def main():
    args = parse_args()
    app = start_simulation_app(args.headless)

    try:
        import omni.usd
        from pxr import UsdGeom

        ctx = omni.usd.get_context()
        ctx.new_stage()
        stage = ctx.get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        stage.SetFramesPerSecond(DOOR_ANIMATION_FPS)
        stage.SetTimeCodesPerSecond(DOOR_ANIMATION_FPS)
        stage.SetStartTimeCode(0)
        stage.SetEndTimeCode(SCENE_ANIMATION_DURATION * DOOR_ANIMATION_FPS)
        UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

        materials = {
            "floor": create_material(stage, "/World/Materials/Floor", (0.005, 0.005, 0.005, 1.0)),
            "wall": create_material(stage, "/World/Materials/Wall", (0.78, 0.78, 0.74, 1.0)),
            "cooling_floor": create_material(stage, "/World/Materials/CoolingFloor", (0.95, 0.78, 0.08, 1.0)),
            "cooling_slot": create_material(stage, "/World/Materials/CoolingSlot", (0.05, 0.70, 0.22, 1.0)),
            "aisle": create_material(stage, "/World/Materials/Aisle", (0.18, 0.42, 0.58, 0.35)),
            "oven": create_material(stage, "/World/Materials/Oven", (0.82, 0.83, 0.80, 1.0)),
            "oven_proxy": create_material(stage, "/World/Materials/OvenProxy", (0.70, 0.72, 0.70, 1.0)),
            "agv_body": create_material(stage, "/World/Materials/AgvBody", (0.86, 0.88, 0.82, 1.0)),
            "agv_wheel": create_material(stage, "/World/Materials/AgvWheel", (0.03, 0.03, 0.03, 1.0)),
            "cart": create_material(stage, "/World/Materials/DryingCart", (0.62, 0.68, 0.64, 1.0)),
            "led_green": create_material(stage, "/World/Materials/LedGreen", (0.02, 0.95, 0.18, 1.0)),
            "led_red": create_material(stage, "/World/Materials/LedRed", (0.95, 0.04, 0.02, 1.0)),
        }

        tasks = process_tasks()
        oven_state = build_oven_state_for_tasks(tasks)

        add_room(stage, materials)
        add_oven_instances(stage, args.proxy_only, args.strict_urdf, args.use_urdf_importer, materials, oven_state)
        add_process_flow(stage, materials)
        add_lighting_and_camera(stage)

        args.output.parent.mkdir(parents=True, exist_ok=True)
        stage.GetRootLayer().Export(str(args.output))
        print(f"Saved scene: {args.output}")
    finally:
        app.close()


if __name__ == "__main__":
    main()
