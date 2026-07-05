# GitHub 发布目录说明

`github_release/` 是整理后的 GitHub 发布版本。它与原始工作目录分离，方便直接初始化 git 仓库或复制到远程仓库。

## 已包含

- `README.md`：面向 GitHub 的项目首页说明。
- `docs/`：课程报告和关键代码附录。
- `scripts/`：主场景生成和可视化启动脚本。
- `scenes/`：主流程 USD 场景 `oven_room.usd`。
- `oven/`：最终主流程使用的烘箱 URDF/STL 资产。
- `F4-1000C/`：AGV URDF/STL 资产。
- `烘车urdf/`：烘车 URDF/STL 资产。
- `.gitignore`：发布仓库忽略规则。

## 已排除

- `__pycache__/`：Python 缓存。
- `*.zip`：原始压缩包。
- `export.log`：SolidWorks/URDF 导出日志。
- `oven_urdf/`：旧烘箱导出资产，最终主脚本未使用。
- `scenes/agv_cart_lift_test.usd`：AGV 抬叉测试场景，最终主流程未使用。
- `scenes/try_urdf.isaacsim.urdf`：旧 URDF importer 中间文件，包含本机绝对路径，最终主流程未使用。
- `scripts/create_agv_cart_lift_test_scene.py`、`scripts/open_agv_cart_lift_test_in_gui.py`：测试场景脚本。
- `scripts/control_mecanum_chassis_physics.py`、`scripts/open_scene_in_gui.py`：非主流程辅助脚本。
- ROS/Gazebo 导出附属文件：`CMakeLists.txt`、`package.xml`、`launch/`、`config/`、`*.csv`。
- `Final/`：当前主流程未引用的大型模型导出目录。
- `.codex/`、`.agents/`、`.git/`：本地工具和仓库元数据。

## 建议发布方式

进入发布目录后初始化仓库：

```bash
cd github_release
git init
git add .
git commit -m "Initial release"
```

如果远程仓库已创建，再添加 remote 并推送即可。
