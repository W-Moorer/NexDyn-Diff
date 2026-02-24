# NexDyn-Diff

NexDyn-Diff 是一个基于 SimCore 物理引擎核心的动力学仿真项目，专注于齿轮接触仿真和动力学分析。

## 项目概述

本项目是 SimCore 核心库的迁移版本，提供了物理仿真、碰撞检测、齿轮SDF（有符号距离场）插件等功能，支持复杂机械系统的动力学仿真。

## 项目结构

```
NexDyn-Diff/
├── cmake/                  # CMake 构建配置和第三方依赖
│   └── third_party_deps/   # 第三方库配置（Dear ImGui、Implot、Filament等）
├── include/simcore/        # 公共头文件目录
│   ├── simcore.h           # 主头文件，包含所有核心API
│   ├── core_api.h          # 核心API定义
│   ├── dynamics.h          # 动力学相关接口
│   ├── collision.h         # 碰撞检测接口
│   ├── solver.h            # 求解器接口
│   ├── model.h             # 模型定义
│   ├── data.h              # 仿真数据结构
│   ├── math.h              # 数学工具函数
│   ├── xml.h               # XML模型加载接口
│   └── plugin.h            # 插件系统接口
├── src/                    # 源代码目录
│   ├── engine/             # 物理引擎核心实现
│   │   ├── engine_collision_*.c  # 碰撞检测实现（GJK、SDF、凸包等）
│   │   ├── engine_core_*.c       # 核心约束、平滑处理、工具函数
│   │   ├── engine_forward.c      # 前向动力学
│   │   ├── engine_inverse.c      # 逆向动力学
│   │   ├── engine_solver.c       # 约束求解器
│   │   ├── engine_sensor.c       # 传感器系统
│   │   └── engine_util_*.c       # 工具函数（BLAS、稀疏矩阵等）
│   ├── user/               # 用户层API实现
│   │   ├── user_api.cc     # 用户API实现
│   │   ├── user_model.cc   # 模型操作
│   │   ├── user_objects.cc # 对象管理
│   │   └── user_*.cc       # 其他用户功能
│   ├── xml/                # XML解析器
│   │   ├── xml_native_reader.cc  # 原生XML读取
│   │   ├── xml_native_writer.cc  # 原生XML写入
│   │   └── xml_urdf.cc           # URDF格式支持
│   ├── thread/             # 线程池实现
│   │   ├── thread_pool.cc  # 线程池
│   │   └── thread_task.cc  # 任务管理
│   └── plugin/sdf/         # SDF插件
│       └── gear_plugin.cc  # 齿轮SDF插件实现
├── models/                 # 仿真模型文件（XML格式）
│   ├── double_pendulum.xml # 双摆模型
│   ├── gear_selfcontained.xml  # 齿轮接触仿真模型
│   ├── gear_scene.xml      # 齿轮场景
│   └── fuzajiaolian*.xml   # 复杂铰链模型
├── tools/                  # 仿真工具程序
│   ├── double_pendulum_sim.cc    # 双摆仿真工具
│   ├── gear_contact_export_cpp.cc # 齿轮接触力导出工具
│   ├── complex_gear_sim.cc       # 复杂齿轮仿真
│   ├── compare_results.py        # 结果比较脚本
│   └── plot_gear_cpp_csv.py      # 绘图脚本
├── tests/                  # 测试程序
│   ├── p1_smoke.c          # 基础冒烟测试
│   ├── p2_lifecycle_smoke.c # 生命周期测试
│   ├── p3_dynamics_solver_smoke.c # 动力学求解器测试
│   ├── p4_contact_smoke.c  # 接触检测测试
│   ├── p5_modeling_smoke.c # 建模测试
│   ├── p6_extension_smoke.c # 扩展功能测试
│   └── p7_foundation_math_smoke.c # 数学基础测试
├── scripts/                # 辅助脚本
│   ├── check_two_gear_direction.py # 齿轮方向检查
│   └── extract_rmd.py      # 文档提取
├── output/                 # 仿真输出数据（CSV格式）
│   ├── double_pendulum/    # 双摆仿真结果
│   └── *.csv               # 齿轮仿真结果
├── figures/                # 图表和可视化结果
│   ├── double_pendulum/    # 双摆分析图表
│   └── complex_gear/       # 复杂齿轮分析图表
├── CMakeLists.txt          # 主CMake配置
└── LICENSE                 # 许可证文件
```

## 核心功能

### 1. 物理引擎核心 (src/engine/)
- **碰撞检测**: 支持GJK算法、SDF（有符号距离场）、凸包、基本几何体碰撞检测
- **动力学求解**: 前向/逆向动力学、约束求解器
- **传感器系统**: 支持加速度计等传感器
- **内存管理**: 高效的内存分配和错误处理

### 2. 齿轮SDF插件 (src/plugin/sdf/gear_plugin.cc)
- 基于有符号距离场的齿轮几何建模
- 支持参数化齿轮生成（齿数、直径、厚度、压力角等）
- 提供平滑并集/交集/差集操作
- 支持齿轮接触仿真

### 3. XML模型系统 (src/xml/)
- 原生XML模型格式支持
- URDF格式导入支持
- 灵活的模型定义和配置

### 4. 仿真工具 (tools/)
- **double_pendulum_sim**: 双摆动力学仿真，导出位置、速度、加速度数据
- **gear_contact_export_cpp**: 齿轮接触力仿真和导出
- **complex_gear_sim**: 复杂齿轮系统仿真

## 构建说明

### 环境要求
- CMake 3.16+
- C/C++ 编译器（支持C++17）
- 依赖库：ccd、lodepng、qhull、tinyobjloader、tinyxml2

### 构建步骤

```bash
# 创建构建目录
mkdir build && cd build

# 配置
cmake ..

# 构建
cmake --build .
```

### 构建选项
- `SIMCORE_BUILD_SMOKE`: 构建冒烟测试程序
- `SIMCORE_BUILD_GEAR_EXPORT_CPP`: 构建齿轮接触力导出工具（默认开启）
- `SIMCORE_BUILD_DOUBLE_PENDULUM_SIM`: 构建双摆仿真工具（默认开启）
- `SIMCORE_BUILD_COMPLEX_GEAR_SIM`: 构建复杂齿轮仿真工具（默认开启）

## 使用示例

### 双摆仿真
```bash
./double_pendulum_sim
```
输出将保存在 `output/double_pendulum/` 目录，包含各连杆的位置、速度、加速度CSV文件。

### 齿轮接触仿真
```bash
./simcore_gear_export --model models/gear_selfcontained.xml --csv output/gear_example_cpp.csv --ctrl 1.0 --sim-time 5.0
```

参数说明：
- `--model`: 模型文件路径
- `--csv`: 输出CSV文件路径
- `--ctrl`: 驱动器控制信号
- `--sim-time`: 仿真时长（秒）
- `--dt`: 时间步长
- `--debug-contacts-csv`: 接触力调试输出

### 加载插件
齿轮仿真需要加载SDF插件：
```bash
./simcore_gear_export --plugin-lib path/to/simcore_sdf_plugin.dll
```

## 模型格式

### 齿轮模型示例 (gear_selfcontained.xml)
```xml
<simcore model="gear_contact_selfcontained">
  <extension>
    <plugin plugin="simcore.sdf.gear">
      <instance name="gear1">
        <config key="alpha" value="0"/>
      </instance>
    </plugin>
  </extension>

  <worldbody>
    <body name="freewheel_body" pos="0 0 0.2">
      <joint name="freewheel" type="hinge" axis="0 0 1"/>
      <geom name="gear1" type="sdf" mesh="gear1_mesh">
        <plugin instance="gear1"/>
      </geom>
    </body>
  </worldbody>

  <actuator>
    <motor name="drive" joint="drive" ctrllimited="true" ctrlrange="-1 1" gear="1500"/>
  </actuator>
</simcore>
```

## 测试

项目包含7个冒烟测试程序：
- `p1_smoke`: 基础API测试
- `p2_lifecycle_smoke`: 对象生命周期测试
- `p3_dynamics_solver_smoke`: 动力学求解器测试
- `p4_contact_smoke`: 接触检测测试
- `p5_modeling_smoke`: 建模功能测试
- `p6_extension_smoke`: 扩展功能测试
- `p7_foundation_math_smoke`: 数学基础测试

启用测试构建：
```bash
cmake -DSIMCORE_BUILD_SMOKE=ON ..
```

## 输出数据格式

### 齿轮仿真输出 (CSV)
| 列名 | 说明 |
|------|------|
| time | 仿真时间 |
| fx, fy, fz | 接触力各分量 |
| fn_sum | 法向力总和 |
| fworld_mag | 世界坐标系力大小 |
| ncon | 接触点数量 |
| freewheel_vel | 从动轮角速度 |
| drive_vel | 驱动轮角速度 |

### 双摆仿真输出 (CSV)
| 列名 | 说明 |
|------|------|
| Time | 仿真时间 |
| X, Y, Z | 位置/速度/加速度各分量 |

## 许可证

详见 [LICENSE](LICENSE) 文件。

## 致谢

本项目参考了 [MuJoCo](https://github.com/google-deepmind/mujoco) 物理引擎的设计理念和实现思路，特别感谢 MuJoCo 团队为物理仿真领域做出的杰出贡献。齿轮SDF插件的实现也参考了 MuJoCo 的 SDF 齿轮插件设计。
