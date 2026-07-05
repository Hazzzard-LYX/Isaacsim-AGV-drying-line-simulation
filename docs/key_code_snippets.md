# 烘干生产线仿真关键代码片段

本文档作为报告的代码附录，按实现模块整理关键代码片段。完整代码以工程中的源文件为准，本文仅保留和“URDF、USD、房间场景重建、路径规划、调度逻辑、仿真实现”直接相关的片段。代码块中的 `...` 或省略注释表示非关键内容已删去，片段用于说明结构和逻辑，不作为独立脚本直接运行。

## 1. URDF 模型片段

### 1.1 烘箱 URDF：主体与左右门

来源文件：`oven/urdf/oven.urdf`

烘箱 URDF 中包含 `base_link`、`left`、`right` 三个主要 link。其中左右门通过 prismatic joint 和主体连接。当前仿真中，一台烘箱模型对应左右两个门，因此调度时可视作两个子烘箱。

```xml
<robot name="oven">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://oven/meshes/base_link.STL" />
      </geometry>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://oven/meshes/base_link.STL" />
      </geometry>
    </collision>
  </link>

  <link name="left">
    <visual>
      <geometry>
        <mesh filename="package://oven/meshes/left.STL" />
      </geometry>
    </visual>
  </link>

  <joint name="left" type="prismatic">
    <origin xyz="2.1229 3.0969 2.3219" rpy="0 0 3.1416" />
    <parent link="base_link" />
    <child link="left" />
    <axis xyz="0 0 -1" />
    <limit lower="-2" upper="0" effort="0" velocity="0" />
  </joint>

  <link name="right">
    <visual>
      <geometry>
        <mesh filename="package://oven/meshes/right.STL" />
      </geometry>
    </visual>
  </link>

  <joint name="right" type="prismatic">
    <origin xyz="2.1229 3.0969 2.3219" rpy="0 0 3.1416" />
    <parent link="base_link" />
    <child link="right" />
    <axis xyz="0 0 -1" />
    <limit lower="-2" upper="0" effort="0" velocity="0" />
  </joint>
</robot>
```

关键点：

- `left` 和 `right` 是两个可动门。
- 两个门都是 `prismatic` 类型，行程为 `2m`。
- 脚本中没有直接求解 URDF 关节动力学，而是根据该结构重建门的 USD 动画。

### 1.2 AGV URDF：车体、车轮与货叉

来源文件：`F4-1000C/urdf/F4-1000C.urdf`

AGV 模型包含车体、四个车轮和可升降货叉。车轮是连续转动关节，货叉是 prismatic joint。

```xml
<robot name="F4-1000C">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://F4-1000C/meshes/base_link.STL" />
      </geometry>
    </visual>
  </link>

  <link name="left-front-wheel">
    <visual>
      <geometry>
        <mesh filename="package://F4-1000C/meshes/left-front-wheel.STL" />
      </geometry>
    </visual>
  </link>

  <joint name="left-front-wheel" type="continuous">
    <origin xyz="-0.15 0.345 -0.57" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="left-front-wheel" />
    <axis xyz="0 1 0" />
  </joint>

  <link name="fork">
    <visual>
      <geometry>
        <mesh filename="package://F4-1000C/meshes/fork.STL" />
      </geometry>
    </visual>
  </link>

  <joint name="fork-lift" type="prismatic">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="fork" />
    <axis xyz="0 0 -1" />
    <limit lower="-2" upper="0" effort="0" velocity="0" />
  </joint>
</robot>
```

关键点：

- 仿真中使用 AGV 的 STL 网格重建 USD 层级。
- 货叉升降动画使用局部 `Z` 方向平移实现。
- 当前展示中货叉抬升高度设置为 `0.4m`。

### 1.3 烘车 URDF：单一主体模型

来源文件：`烘车urdf/urdf/csrotate.urdf`

烘车为单一 `base_link`，没有独立运动关节。仿真中通过烘车根节点的 time samples 控制位置和姿态。

```xml
<robot name="csrotate">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://csrotate/meshes/base_link.STL" />
      </geometry>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://csrotate/meshes/base_link.STL" />
      </geometry>
    </collision>
  </link>
</robot>
```

关键点：

- 烘车本身没有可动关节。
- 被 AGV 搬运时，烘车位置由 AGV 位姿和固定前向距离计算得到。
- 烘车出现通过 USD visibility 控制，不通过从地下升起实现。

## 2. 场景参数与布局代码

来源文件：`scripts/create_oven_room_scene.py`

### 2.1 关键尺寸参数

```python
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

COOLING_ROOM_WIDTH_X = 15.5
COOLING_SLOT_COLUMNS = 9
COOLING_SLOT_ROWS = 7
COOLING_SLOT_SIZE_X = 1.0
COOLING_SLOT_SIZE_Y = 1.5
COOLING_LONGITUDINAL_PASSAGE_WIDTH_X = 4.0
COOLING_EAST_WALL_CLEARANCE_X = 1.5

PIPELINE_CART_APPEAR_INTERVAL = 40.0
PIPELINE_TASKS_PER_AISLE = 8
DRYING_WORK_DURATION = PIPELINE_CART_APPEAR_INTERVAL * 8.0

AGV_FORK_LIFT_HEIGHT = 0.4
AGV_CART_STANDOFF = 1.25
CARRY_TRAVEL_SPEED = 1.0
CARRY_RETREAT_DISTANCE = 4.0
TURN_ENTRY_DEPTH = 1.2
```

这些参数统一控制烘箱间距、走廊宽度、冷却区车位数量、AGV 抬升高度、生产节拍和搬运速度。

### 2.2 烘箱四排布局

```python
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

    rows = [
        ("row_1_top", row_a, math.pi),
        ("row_2_upper_middle", row_b, 0.0),
        ("row_3_lower_middle", row_c, math.pi),
        ("row_4_bottom", row_d, 0.0),
    ]

    for row_name, y, yaw in rows:
        for col, x in enumerate(xs, start=1):
            yield row_name, col, x, y, yaw
```

该函数生成所有烘箱的中心点和朝向。第 1、3 排朝向负 `Y`，第 2、4 排朝向正 `Y`，从而形成两条面对面的工作走廊。

### 2.3 房间尺寸与走廊中心线

```python
def room_length_x():
    return OVENS_PER_ROW * OVEN_WIDTH_X + (OVENS_PER_ROW - 1) * OVEN_COLUMN_GAP + 2 * ROOM_MARGIN_X


def room_width_y():
    return ROW_COUNT * OVEN_DEPTH_Y + 2 * AISLE_WIDTH + MIDDLE_BACK_GAP + 2 * ROOM_MARGIN_Y


def aisle_centers_y():
    total_y = ROW_COUNT * OVEN_DEPTH_Y + 2 * AISLE_WIDTH + MIDDLE_BACK_GAP
    top_aisle_y = total_y / 2.0 - OVEN_DEPTH_Y - AISLE_WIDTH / 2.0
    return top_aisle_y, -top_aisle_y
```

当前计算结果约为：

- 烘干区尺寸：`28.3m x 24.0m`
- 北侧走廊中心线：`Y = +5.5m`
- 南侧走廊中心线：`Y = -5.5m`

## 3. USD 场景重建代码

### 3.1 创建材质

```python
def create_material(stage, path: str, color):
    from pxr import Sdf, UsdShade

    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color[:3])
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(color[3])
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material
```

材质用于区分地面、墙体、冷却区地面、车位、通道、AGV、烘车和 LED。

### 3.2 使用 Mesh 创建矩形体

```python
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

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*point) for point in points])
    mesh.CreateFaceVertexCountsAttr([4, 4, 4, 4, 4, 4])
    mesh.CreateFaceVertexIndicesAttr([...])
    if material is not None:
        bind_material(mesh.GetPrim(), material)
    return mesh.GetPrim()
```

墙体、地面、通道标记、冷却区车位和 LED 都由该函数生成。

### 3.3 读取 STL 并写入 USD Mesh

```python
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
    if material is not None:
        bind_material(mesh.GetPrim(), material)
    return mesh.GetPrim()
```

该部分绕过 URDF importer，直接把 SolidWorks 导出的 STL 网格写入 USD。这样可以减少不同 Isaac Sim 版本中 URDF importer 行为差异带来的影响。

### 3.4 房间、墙体、冷却区与走廊

```python
def add_room(stage, materials):
    room_x = room_length_x()
    room_y = room_width_y()
    top_aisle_y, bottom_aisle_y = aisle_centers_y()

    add_cube(
        stage,
        "/World/Room/Floor",
        (0, 0, -FLOOR_THICKNESS / 2.0),
        (GROUND_LENGTH_X, GROUND_WIDTH_Y, FLOOR_THICKNESS),
        materials["floor"],
    )

    add_cube(stage, "/World/Room/Wall_North", ..., materials["wall"])
    add_cube(stage, "/World/Room/Wall_South", ..., materials["wall"])

    cooling_center_x = room_x / 2.0 + COOLING_ROOM_WIDTH_X / 2.0
    add_cube(
        stage,
        "/World/CoolingRoom/Floor",
        (cooling_center_x, 0, 0.003),
        (COOLING_ROOM_WIDTH_X, room_y, 0.012),
        materials["cooling_floor"],
    )
    add_cube(stage, "/World/CoolingRoom/Wall_North", ..., materials["wall"])
    add_cube(stage, "/World/CoolingRoom/Wall_South", ..., materials["wall"])
    add_cube(stage, "/World/CoolingRoom/Wall_East", ..., materials["wall"])
    add_cooling_parking_slots(stage, materials, room_x, room_y, top_aisle_y, bottom_aisle_y)

    opening_ranges = [
        (bottom_aisle_y - AISLE_WIDTH / 2.0, bottom_aisle_y + AISLE_WIDTH / 2.0),
        (top_aisle_y - AISLE_WIDTH / 2.0, top_aisle_y + AISLE_WIDTH / 2.0),
    ]

    # 根据两条走廊的开口，把东西两侧墙体切成多段。
    for index, (segment_min_y, segment_max_y) in enumerate(wall_segments, start=1):
        add_cube(stage, f"/World/Room/Wall_East_{index}", ..., materials["wall"])
        add_cube(stage, f"/World/Room/Wall_West_{index}", ..., materials["wall"])

    aisle_length_x = room_x + AISLE_ENTRANCE_EXTENSION
    aisle_center_x = -AISLE_ENTRANCE_EXTENSION / 2.0
    add_cube(stage, "/World/Room/Aisle_Top", ..., materials["aisle"])
    add_cube(stage, "/World/Room/Aisle_Bottom", ..., materials["aisle"])
```

该函数负责重建主房间、冷却区、墙体、无限感地面和两条进料延伸走廊。墙体不是四个简单长方体，而是根据两条走廊开口分段生成。

### 3.5 冷却区车位与通道

```python
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
        "longitudinal_x_center": (longitudinal_x_min + longitudinal_x_max) / 2.0,
        "slot_x_centers": evenly_spaced_centers(slot_x_min, slot_x_max, COOLING_SLOT_SIZE_X, COOLING_SLOT_COLUMNS),
        "slot_y_centers": slot_y_centers,
        "passage_y_centers": passage_y_centers,
        "passage_width_y": passage_width_y,
    }
```

该函数给出冷却区车位、东西向横向通道和西侧南北向主通道的几何关系。

## 4. 路径规划代码

### 4.1 子烘箱位置与 AGV-烘车相对位姿

```python
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


def front_cart_position(agv_x: float, agv_y: float, yaw_degrees: float, z: float, standoff: float = AGV_CART_STANDOFF):
    yaw = math.radians(yaw_degrees)
    return (
        agv_x + math.cos(yaw) * standoff,
        agv_y + math.sin(yaw) * standoff,
        z,
        yaw_degrees,
    )
```

这三个函数是搬运运动的基础：先根据烘箱模型与门侧计算子烘箱中心，再根据烘车目标位置反算 AGV 根节点位置，最后在搬运阶段用 AGV 位姿重新计算烘车位置。

### 4.2 烘干区任务运动几何

```python
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
        "cart_feed": cart_feed,
        "cart_retreat": cart_retreat,
        "cart_turn_start": cart_turn_start,
        "cart_turn_end": cart_turn_end,
        "cart_handoff": cart_handoff,
        "feed_agv": agv_root_for_cart(*cart_feed, feed_yaw),
        "turn_end_agv": agv_root_for_cart(*cart_turn_end, oven_yaw),
        "oven_inside_agv": agv_root_for_cart(oven_x, oven_y, oven_yaw),
        "handoff_agv": agv_root_for_cart(*cart_handoff, outbound_aisle_yaw),
    }
```

这里定义了入炉和出炉路径的关键点。整体路径近似为“小写 r”形状：进料点取车、先倒退、沿走廊行驶、90 度转向、送入烘箱。

### 4.3 空车路径：只允许直行和转向

```python
def empty_route_samples(start_time: float, start_position, start_yaw: float, target_position, target_yaw: float):
    samples = []
    yaw_samples = []
    seconds = start_time
    x, y, z = start_position
    target_x, target_y, _target_z = target_position
    yaw = start_yaw

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
    if abs(target_x - x) > 1e-6:
        straight_to(target_x, y, 0.0 if target_x > x else 180.0)
    if abs(target_y - y) > 1e-6:
        straight_to(target_x, target_y, 90.0 if target_y > y else 270.0)
    turn_to(target_yaw)
    return samples, yaw_samples, seconds
```

空车移动不会直接对 `X` 和 `Y` 同时插值到目标点，而是先转向、再直行、再转向，避免出现 AGV 横向平移。

### 4.4 搬运阶段高密度采样

```python
def dense_carried_samples(waypoints, sample_dt: float = CARRY_SAMPLE_DT):
    agv_positions = []
    agv_yaws = []
    cart_positions = []
    debug_samples = []

    for waypoint_index, waypoint in enumerate(waypoints):
        label, seconds, agv_pos, yaw_degrees, cart_z = waypoint

        # 在相邻关键点之间按 24 FPS 插值。
        for step in range(segment_steps + 1):
            x = prev_agv_pos[0] + (agv_pos[0] - prev_agv_pos[0]) * alpha
            y = prev_agv_pos[1] + (agv_pos[1] - prev_agv_pos[1]) * alpha
            target_yaw = prev_yaw + yaw_delta_degrees(prev_yaw, yaw_degrees)
            yaw = prev_yaw + (target_yaw - prev_yaw) * alpha
            z_cart = prev_cart_z + (cart_z - prev_cart_z) * alpha

            cart_x, cart_y, cart_z_sample, _ = front_cart_position(x, y, yaw, z_cart)
            upsert_time_sample(agv_positions, (t, (x, y, z)))
            upsert_time_sample(agv_yaws, (t, yaw))
            upsert_time_sample(cart_positions, (t, cart_x, cart_y, cart_z_sample, yaw))

    return agv_positions, agv_yaws, cart_positions, debug_samples
```

搬运阶段不是让 AGV 和烘车分别独立走关键帧，而是每一帧都由 AGV 位姿推导烘车位置，保证二者相对位置不变。

### 4.5 冷却区空车接车路径

```python
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
```

冷却区 AGV 先回到西侧南北向主通道，再沿主通道移动到目标走廊入口，最后转向交界处取车。

## 5. 调度逻辑代码

### 5.1 南北走廊轮流产生任务

```python
def process_tasks():
    top_aisle_y, bottom_aisle_y = aisle_centers_y()
    sequences = {
        "top": aisle_sub_oven_sequence(("row_1_top", "row_2_upper_middle")),
        "bottom": aisle_sub_oven_sequence(("row_3_lower_middle", "row_4_bottom")),
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
            "slot": slot_index,
            "oven": sub_oven["oven"],
        }
        tasks.append(task)

    return schedule_process_tasks(tasks)
```

全局每 `40s` 出现一台烘车，北、南两条走廊交替出现。因此同一走廊上的出现间隔为 `80s`。

### 5.2 烘箱门和 LED 状态时间表

```python
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
```

每个任务只影响对应子烘箱的门和 LED。门打开、关门、工作中红灯、完成后绿灯均由任务时间表驱动。

### 5.3 插入任务调度

```python
def schedule_insert_event(task, current_time, current_position, current_yaw):
    geom = task_motion_geometry(task)
    feed_travel_time = empty_route_duration(current_position, current_yaw, geom["feed_agv"], geom["feed_yaw"])
    feed_pick = max(task["start"] + 6.0, current_time + feed_travel_time)
    feed_depart_time = max(current_time, feed_pick - feed_travel_time)

    prefeed_positions, prefeed_yaws, _ = empty_route_samples(
        feed_depart_time,
        current_position,
        current_yaw,
        geom["feed_agv"],
        geom["feed_yaw"],
    )

    inbound_lift_done = feed_pick + 0.8
    move_time = inbound_lift_done
    move_time += travel_seconds(geom["cart_feed"], geom["cart_retreat"])
    move_time += travel_seconds(geom["cart_retreat"], geom["cart_turn_start"])
    move_time += travel_seconds(geom["cart_turn_start"], geom["cart_turn_end"])
    move_time += travel_seconds(geom["cart_turn_end"], (geom["oven_x"], geom["oven_y"]))

    insert_time = move_time + 1.0
    work_end = insert_time + DRYING_WORK_DURATION

    task["times"] = {
        "appear": task["start"],
        "door_open_in_start": feed_pick - 2.0,
        "door_open_in_end": feed_pick,
        "insert": insert_time,
        "work_end": work_end,
        "extract_ready": work_end + 6.0,
    }
    task["schedule"] = {
        "prefeed_positions": prefeed_positions,
        "prefeed_yaws": prefeed_yaws,
    }
    task["inserted"] = True
    return oven_exit_done, geom["turn_end_agv"], geom["oven_yaw"]
```

该函数根据 AGV 当前状态、烘车出现时间和目标烘箱位置计算入炉动作的开始、开门、抬叉、行驶、放下和退出时间。

### 5.4 出炉任务调度

```python
def schedule_extract_event(task, current_time, current_position, current_yaw):
    geom = task_motion_geometry(task)
    times = task["times"]
    travel_to_entry = empty_route_duration(current_position, current_yaw, geom["turn_end_agv"], geom["oven_yaw"])
    outbound_pick = max(times["extract_ready"], current_time + travel_to_entry + 2.0)

    preoutbound_positions, preoutbound_yaws, _ = empty_route_samples(
        outbound_depart,
        current_position,
        current_yaw,
        geom["turn_end_agv"],
        geom["oven_yaw"],
    )

    outbound_lift_done = outbound_pick + 0.8
    out_time = outbound_lift_done
    out_time += travel_seconds((geom["oven_x"], geom["oven_y"]), geom["cart_turn_end"])
    out_time += travel_seconds(geom["cart_turn_end"], geom["cart_turn_start"])
    out_time += travel_seconds(geom["cart_turn_start"], geom["cart_handoff"])
    outbound_handoff_time = out_time

    times.update({
        "extract": outbound_pick,
        "handoff": outbound_handoff_time,
        "cool_pick": outbound_handoff_time + 2.0,
        "park": outbound_handoff_time + 12.0,
    })
    task["schedule"].update({
        "preoutbound_positions": preoutbound_positions,
        "preoutbound_yaws": preoutbound_yaws,
        "outbound_lift_done": outbound_lift_done,
    })
    task["extracted"] = True
    return side_retreat_done, geom["side_retreat_agv"], geom["outbound_aisle_yaw"]
```

该函数计算烘干完成后的取车、出炉、沿走廊送到交界处和烘干区 AGV 后退避让的时间点。

### 5.5 同一走廊内的任务选择策略

```python
def schedule_process_tasks(tasks):
    tasks_by_side = {
        "top": sorted((task for task in tasks if task["name"] == "top"), key=lambda task: task["start"]),
        "bottom": sorted((task for task in tasks if task["name"] == "bottom"), key=lambda task: task["start"]),
    }

    for side, side_tasks in tasks_by_side.items():
        pending_insert = list(side_tasks)
        current_time = 0.0
        current_position = side_home[side]
        current_yaw = 180.0

        while pending_insert or any(task.get("inserted") and not task.get("extracted") for task in side_tasks):
            ready_insert = pending_insert and pending_insert[0]["start"] <= current_time + 1e-6
            ready_extracts = [
                task for task in side_tasks
                if task.get("inserted")
                and not task.get("extracted")
                and task["times"]["extract_ready"] <= current_time + 1e-6
            ]

            if ready_insert:
                task = pending_insert.pop(0)
                current_time, current_position, current_yaw = schedule_insert_event(...)
                continue

            if ready_extracts:
                task = min(ready_extracts, key=lambda item: item["times"]["extract_ready"])
                current_time, current_position, current_yaw = schedule_extract_event(...)
                continue

            if next_insert_time <= next_extract_time:
                current_time, current_position, current_yaw = schedule_insert_event(...)
            else:
                current_time, current_position, current_yaw = schedule_extract_event(...)
```

调度策略是：入口有新烘车时优先入炉；没有新烘车时处理已经烘干完成的烘车；两者都没有时推进到下一任务事件。

## 6. 仿真实现代码

### 6.1 USD 时间采样

```python
def add_time_sampled_translate(translate_op, positions):
    from pxr import Gf

    for seconds, position in positions:
        translate_op.Set(Gf.Vec3d(*position), seconds * DOOR_ANIMATION_FPS)


def add_time_sampled_rotate_z(rotate_op, angle_samples):
    for seconds, angle in unwrap_angle_samples(angle_samples):
        rotate_op.Set(angle, seconds * DOOR_ANIMATION_FPS)
```

所有对象动画最终都写成 USD time samples。脚本中 `DOOR_ANIMATION_FPS = 24`，因此秒数会转换为 24 FPS 下的 time code。

### 6.2 门动画

```python
def add_prismatic_door_animation(translate_op, closed_origin, stroke_samples=None):
    from pxr import Gf

    samples = stroke_samples or [(0.0, 0.0), (SCENE_ANIMATION_DURATION, 0.0)]
    for seconds, stroke in samples:
        translate_op.Set(
            Gf.Vec3d(closed_origin[0], closed_origin[1], closed_origin[2] + stroke),
            seconds * DOOR_ANIMATION_FPS,
        )
```

烘箱门通过局部平移实现 prismatic joint 的可视化效果。门关上时 stroke 为 `0`，打开时 stroke 为 `2.0m`。

### 6.3 AGV 与货叉动画

```python
def agv_fork_samples(load_windows):
    samples = [(0.0, (0.0, 0.0, 0.0))]
    for start, end in load_windows:
        samples.extend([
            (start, (0.0, 0.0, 0.0)),
            (start + 0.8, (0.0, 0.0, AGV_FORK_LIFT_HEIGHT)),
            (end - 0.8, (0.0, 0.0, AGV_FORK_LIFT_HEIGHT)),
            (end, (0.0, 0.0, 0.0)),
        ])
    samples.append((SCENE_ANIMATION_DURATION, samples[-1][1]))
    return samples


def add_agv_instance(stage, materials, name: str, root_positions, fork_windows, yaw_samples=None):
    root_path = f"/World/AGV/{name}"
    agv_root = UsdGeom.Xform.Define(stage, root_path)
    root_xform = UsdGeom.Xformable(agv_root.GetPrim())

    root_translate = root_xform.AddTranslateOp()
    add_time_sampled_translate(root_translate, root_positions)

    root_rotate = root_xform.AddRotateZOp()
    add_time_sampled_rotate_z(root_rotate, yaw_samples)

    add_stl_mesh(stage, f"{root_path}/base_link", AGV_ROOT_DIR / "meshes" / "base_link.STL", materials["agv_body"])

    fork = UsdGeom.Xform.Define(stage, f"{root_path}/fork")
    fork_translate = UsdGeom.Xformable(fork.GetPrim()).AddTranslateOp()
    add_time_sampled_translate(fork_translate, agv_fork_samples(fork_windows))
    add_stl_mesh(stage, f"{root_path}/fork/visual", AGV_ROOT_DIR / "meshes" / "fork.STL", materials["agv_body"])
```

AGV 根节点负责整体运动，货叉作为子节点进行局部升降。

### 6.4 烘车出现与移动

```python
def add_drying_cart(stage, materials, path: str, positions, visibility_samples=None):
    root = UsdGeom.Xform.Define(stage, path)
    root_xform = UsdGeom.Xformable(root.GetPrim())
    translate = root_xform.AddTranslateOp()
    rotate = root_xform.AddRotateZOp()

    add_time_sampled_translate(translate, [(seconds, (x, y, z)) for seconds, x, y, z, yaw, _ in normalized])
    add_time_sampled_rotate_z(rotate, [(seconds, yaw) for seconds, x, y, z, yaw, _ in normalized])

    if visibility_samples is not None:
        set_visibility_samples(root.GetPrim(), visibility_samples)

    add_stl_mesh(stage, mesh_path, CART_ROOT_DIR / "meshes" / "base_link.STL", materials["cart"])
```

烘车进入仿真时通过 `visibility_samples` 控制显隐；搬运和停车位置通过平移、旋转 time samples 控制。

### 6.5 流程装配

```python
def add_process_flow(stage, materials):
    room_x = room_length_x()
    room_y = room_width_y()
    west_feed_x = -room_x / 2.0 - 1.0
    interface_x = room_x / 2.0
    handoff_x = interface_x - HANDOFF_CART_FRONT_OFFSET_X

    cooling_slots = cooling_slot_centers(room_x, room_y)
    cooling_map = cooling_layout(room_x, room_y)
    tasks = sorted(process_tasks(), key=lambda task: task_times(task["start"], task)["handoff"])

    inbound_agv_paths = {"top": [], "bottom": []}
    inbound_fork_windows = {"top": [], "bottom": []}
    inbound_agv_yaws = {"top": [], "bottom": []}
    cooling_positions = []
    cooling_yaws = []
    cooling_fork_windows = []

    for task in tasks:
        # 1. 计算该任务的烘箱位置、走廊位置、交接位置和冷却车位。
        # 2. 生成烘干区 AGV 入炉、出炉和交接路径。
        # 3. 生成冷却区 AGV 接车和停车路径。
        # 4. 写入烘车、AGV、货叉的 time samples。
        add_drying_cart(...)
        inbound_agv_paths[side].extend(...)
        cooling_positions.extend(...)

    add_agv_instance(stage, materials, "F4_1000C_top", ...)
    add_agv_instance(stage, materials, "F4_1000C_bottom", ...)
    add_agv_instance(stage, materials, "F4_1000C_cooling", ...)
```

这是完整流程装配函数，负责把任务调度、路径规划和 USD 动画写入场景。

### 6.6 主程序：生成 USD

```python
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

        materials = {...}
        tasks = process_tasks()
        oven_state = build_oven_state_for_tasks(tasks)

        add_room(stage, materials)
        add_oven_instances(stage, ..., materials, oven_state)
        add_process_flow(stage, materials)
        add_lighting_and_camera(stage)

        stage.GetRootLayer().Export(str(args.output))
    finally:
        app.close()
```

主程序创建 USD stage，设置单位、坐标轴、时间线，之后依次生成房间、烘箱、流程动画、灯光和相机，最后导出 `scenes/oven_room.usd`。

## 7. 可视化启动代码

来源文件：`scripts/launch_process_visualization.py`

```python
def main() -> None:
    scene_path = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else DEFAULT_SCENE
    app = SimulationApp({"headless": False})

    try:
        import omni.timeline
        import omni.usd
        from isaacsim.core.utils.stage import is_stage_loading, open_stage

        if not open_stage(str(scene_path)):
            raise RuntimeError(f"Failed to open stage: {scene_path}")

        while is_stage_loading():
            app.update()

        stage = omni.usd.get_context().get_stage()
        time_codes_per_second = stage.GetTimeCodesPerSecond() or 24.0
        end_time = stage.GetEndTimeCode() / time_codes_per_second

        timeline = omni.timeline.get_timeline_interface()
        timeline.set_start_time(0.0)
        timeline.set_end_time(end_time)
        timeline.set_looping(True)
        timeline.play()

        while app.is_running():
            app.update()
    finally:
        app.close()
```

该脚本负责在 Isaac Sim GUI 中打开生成好的 USD 文件，并让时间线从 `0s` 到 `1100s` 循环播放。

## 8. 对应关系总结

| 报告部分 | 主要代码位置 | 作用 |
| --- | --- | --- |
| URDF 模型 | `oven/urdf/oven.urdf`, `F4-1000C/urdf/F4-1000C.urdf`, `烘车urdf/urdf/csrotate.urdf` | 定义烘箱、AGV、烘车的网格和关节结构 |
| 房间场景重建 | `add_room()`, `add_cooling_parking_slots()`, `cooling_layout()` | 生成地面、墙体、走廊、冷却区和车位 |
| USD 网格生成 | `add_cube()`, `read_binary_stl()`, `add_stl_mesh()` | 将简单几何和 STL 模型写入 USD |
| 路径规划 | `task_motion_geometry()`, `empty_route_samples()`, `dense_carried_samples()`, `build_unloaded_cooling_route()` | 生成 AGV 和烘车的运动路径 |
| 调度逻辑 | `process_tasks()`, `schedule_process_tasks()`, `schedule_insert_event()`, `schedule_extract_event()` | 生成任务顺序和关键时间点 |
| 仿真实现 | `add_process_flow()`, `add_agv_instance()`, `add_drying_cart()`, `build_oven_state_for_tasks()` | 将调度结果转化为 USD 动画 |
| 可视化启动 | `scripts/launch_process_visualization.py` | 打开 USD 并循环播放时间线 |
