# C3 Arm Cell · 六轴 PCB 装配与锁付工作站

![根据项目 SDF 与六轴模型离线渲染的工作站](docs/workcell.png)

工业橙色关节壳体、端盖螺钉、双指夹爪、进料段、PCB、锁付工装和良品托盘均由工程内几何生成。上图为模型渲染，真实 Gazebo 窗口需按下文启动。本项目以 3C 小件装配为目标，在 WSL2 / Ubuntu 22.04 上采用 ROS 2 Humble、MoveIt 2、**Gazebo Harmonic（gz-sim 8）** 和可选 RViz2。

## 你将看到的过程

绿色 PCB 位于进料侧。合成视觉计算中心像素，再按相机模型反投影成世界坐标。机械臂规划到预抓位置，沿直线下降，两个手指独立闭合，读取夹爪反馈后在物理引擎中建立工件约束。PCB 随手臂抬升并直线转运到螺丝工位。工艺节点请求压合与锁付，等待压力和扭矩完成位并计时。机械臂把板送往良品托盘，夹具解除连接，视觉再次核对落点，最后退回。多循环时会用同一块板回送上料以复现实验；流水线滚筒是场景几何，当前进料检测由工件位置生成。

**指标边界**：六转轴，设计水平伸展上界 500 mm，目标负载 0.5 kg、目标重复定位精度 ±0.05 mm。本版提供工程原型、参考板级接口和可移植控制核心。0.5 kg 带载性能与 ±0.05 mm 重复定位尚无样机报告。真机验收需使用校准后的 CAD、减速机构、摩擦、伺服参数、工具和独立测量系统。

## 1. Windows 打开压缩包，解压到原来的 A3 路径

下载存为 `D:\A VAM\c3_arm_ws.zip`。在 **Windows PowerShell** 输入：

```powershell
code "D:\A VAM"
```

VS Code 中点击“终端 → 新建终端”。PowerShell 提示符下运行：

```powershell
wsl -d Ubuntu-22.04
```

进入 `wen@...$` 后运行：

```bash
mkdir -p ~/A3
unzip -n "/mnt/d/A VAM/c3_arm_ws.zip" -d ~/A3
cd ~/A3/c3_arm_ws
ls README.md src/c3_cell/models/workcell.sdf tools/container
code -n .
```

在新窗口确认左下角显示 **WSL: Ubuntu-22.04**。当前项目工作区为 `~/A3/c3_arm_ws`；旧版机械臂为 `~/A3/arm_cell_ws`，旧 A2 使用另外的工作区。以后可直接在 WSL 中执行 `cd ~/A3/c3_arm_ws && code .`。

## 2. 首次准备（VS Code 的 WSL 终端 1）

Harmonic 与 Classic 共用过 gz 命令名。项目使用单独的 **rootless Podman 容器**把 Humble 与 Harmonic 安装到一起。WSL 文件系统仍在你的 D 盘；镜像默认写入 WSL 的用户存储。首次联网下载 ROS、MoveIt、Gazebo 和构建组件，请预留足够磁盘空间。宿主只需 Podman，启动不需要 Docker Desktop。容器中世界插件通过 rclcpp 直接与 ROS 通信。

```bash
cd ~/A3/c3_arm_ws
sudo apt update
sudo apt install -y podman uidmap slirp4netns fuse-overlayfs
bash tools/container
```

最后应能看到 Gazebo Sim 的 **8.x** 版本。命令首次构建镜像、创建并启动名为 `c3-harmonic` 的容器，以后复用。然后执行：

```bash
tools/cell python3 tools/smoke.py
tools/cell colcon build --symlink-install --parallel-workers 2
tools/cell ros2 pkg prefix c3_cell
```

离线自测应有四行 PASS，编译应显示 `1 package finished`。tools/cell 在每个命令前自动加载 Humble 与当前安装目录，设置 ROS_DOMAIN_ID=46、ROS_LOCALHOST_ONLY=1 和独立 Gazebo 分区。容器挂载源码目录，VS Code 编辑后仍在原路径。

## 3. 重开 VS Code 后，四个终端看仿真

进入 WSL 窗口，在终端面板点 **＋** 开四个终端，每个都先进入 `~/A3/c3_arm_ws`。上一轮仍在运行时，先在旧的终端 1、2 各按 Ctrl+C。以下都是 Ubuntu 命令。第 4 个终端先做检查，再由第 3 个启动任务。

**终端 1：后台物理世界、运动规划、视觉和任务节点**

```bash
cd ~/A3/c3_arm_ws
bash tools/container
tools/cell ros2 launch c3_cell cell.launch.py recipe:=assembly loops:=1 rviz:=false
```

**终端 2：Harmonic 三维画面**

```bash
cd ~/A3/c3_arm_ws
tools/cell gz sim -g
```

看桌面、橙色关节、绿色板和双指。GUI 若暂停，点击播放。Harmonic 使用 `gz sim -g`。如果你想同时看 MoveIt 的预览，在终端 1 把 `rviz:=false` 改为 `rviz:=true` 再启动。16 GB 机器首次推荐只开 Gazebo 窗口，按需打开 RViz。

**终端 4：检查关节反馈和视觉目标**

```bash
cd ~/A3/c3_arm_ws
tools/cell ros2 topic echo /joint_states --once
tools/cell ros2 topic echo /c3/target --once
tools/cell ros2 service list
tools/cell ros2 topic echo /c3/status
```

应有 j1…j6、left_joint、right_joint；目标 frame_id 为 world，初始 PCB 约在 (0.235, −0.180, 0.079) m；服务列表应有 /plan_kinematic_path、/check_state_validity 和 /c3/auto。最后一条会持续显示状态、关节角、TCP 坐标和故障原因。

**终端 3：启动一次完整抓放**

```bash
cd ~/A3/c3_arm_ws
tools/cell ros2 service call /c3/auto std_srvs/srv/Trigger '{}'
```

success=true 表示开始受理。终端 4 从 observe 走到 close、transfer、screw、open、retreat，最后显示 **DONE** 才表示完成。看终端 2 的 PCB 是否确实随夹爪移动并留在良品托盘。

## 4. 单步、示教、速度和配方

以下在 **终端 3** 操作，仍保持 1、2、4 运行。更方便的人机界面：

```bash
cd ~/A3/c3_arm_ws
bash tools/hmi
```

|操作|终端命令（在工程根目录）|语义|
|---|---|---|
|单步| `tools/cell ros2 service call /c3/step std_srvs/srv/Trigger '{}'` |一个工艺阶段，完成后 PAUSED|
|阶段末暂停| `tools/cell ros2 service call /c3/pause std_srvs/srv/Trigger '{}'` |完成当前原子步骤后保存断点|
|继续| `tools/cell ros2 service call /c3/resume std_srvs/srv/Trigger '{}'` |核对关节及持件状态后继续|
|记录示教点| `tools/cell ros2 service call /c3/teach std_srvs/srv/Trigger '{}'` |任务停止时保存六轴角和 TCP 矩阵|
|更换速度| `tools/cell ros2 param set /c3_station speed 0.40` |下一段轨迹开始采用，rad/s|
|更换加速度| `tools/cell ros2 param set /c3_station accel 0.75` |下一段采用，rad/s²|
|十轮| `tools/cell ros2 param set /c3_station loops 10` |随后调用 auto；包含工件回送|
|软件急停演练| `tools/cell ros2 service call /c3/estop std_srvs/srv/Trigger '{}'` |下发锁存停止，恢复见 docs/state.md|

**写入新工位**：暂停在已确认的示教位姿，运行 `tools/cell ros2 param set /c3_station teach_slot good`，然后 `tools/cell ros2 service call /c3/save_point std_srvs/srv/Trigger '{}'`。可选 good、fixture、infeed。该操作覆盖本用户配方副本中的对应坐标。调整超过夹具实际几何范围时，要同步修改场景夹具与视觉/工装校准值。初始取料仍由视觉输出决定。保留示教记录可调用 teach。

**切换配方**：结束上一轮或在阶段边界暂停，设置 `tools/cell ros2 param set /c3_station recipe soft`，调用 `tools/cell ros2 service call /c3/reload std_srvs/srv/Trigger '{}'`。assembly 是直线转运配方；soft 使用较低速度、较长保压时间，并在前往良品工位时经过三点确定的圆弧。用户 JSON 中可修改 lift、hold_s、speed、accel；恢复断点时采用该断点保存的参数。

**日志在哪里**：容器内 ~/.ros/c3_arm 映射为 WSL 的 `~/.local/share/c3-workcell-home/.ros/c3_arm`。run-*.jsonl 含时间戳、关节目标/反馈、原始 CAN FD 字节、工序和错误；checkpoint.json 记录阶段边界；teach.json 记录示教位姿。终端 4 执行 `ls -lt ~/.local/share/c3-workcell-home/.ros/c3_arm | head` 即可找到。

## 5. 三层架构与交付导航

|层|源文件|实现|
|---|---|---|
|上层应用|scripts/station.py、vision.py、config/|工艺状态机、多配方、在线示教、断点、服务、状态、日志；2D 靶标像素投影/反投影或外部固定相机坐标转换|
|中层通信|scripts/core.py、gateway.py、firmware/wire.c|小端 48 字节 FD 负载、CRC、双向转换、SocketCAN、重连与反馈时序；时钟回绕处理|
|底层驱动|firmware/control.c、kin.c、h743_port.c、servo_fd.c、board_reference.c|位置外环、目标限幅、滤波、六轴缓存后 SYNC，ADC、实时任务和 EXTI 板级参考|
|物理仿真|models/workcell.sdf、plugin/plant.cpp|1 ms 物理步，夹爪双侧接触与到位、动态 PCB、实际约束创建/释放、接触故障、伺服滞后、工件观测与进料 IO|
|机器人描述|models/arm.urdf、config/arm.srdf、config/arm.json|标准 DH、碰撞和质量惯量、相机安装位、MoveIt 规划组|
|启动与验证|launch/cell.launch.py、tools/、.github/workflows/|分后端启动、离线自测、容器、HMI、CI 无界面工艺验收入口|

表中包内路径均以 src/c3_cell 为起点。ROS 包通过 colcon 使用 CMake 构建 Linux 插件与 C 运动学共享库。裸机移植复用同一 C 核心，板卡专用文件不参加 Linux 插件构建。

- [硬件清单、DH 参数、设计指标、移植步骤](docs/hardware.md)
- [质量、质心、惯性张量主对角线](docs/inertia.csv)
- [FD-CAN 双总线协议、CRC、字节布局及工艺 IO](docs/protocol.md)
- [工艺状态图、故障码、锁存与恢复](docs/state.md)
- [验证记录与未验收项](docs/verification.md)

## 6. 仿真真实性与实物迁移

Gazebo 中含自由 PCB、接触传感器、摩擦、质量、惯量、阻尼与夹爪固定约束。示例压合用工位的虚拟弹簧位移计算压力窗口，锁付以到位后计时建立完成反馈；当前没有螺纹啮合、真实电批扭矩曲线、PCB 弹性变形或基于图像的目标检测。实时接触由 Gazebo 判断，MoveIt 使用简化碰撞体做预检。手指触碰被抓 PCB、工件靠到预期支撑面和机构相邻装配面有工艺例外。实时 RViz 显示状态和关节角，Trajectory 显示预览，PlanningScene 查看场景包围几何；可在 Scene Robot 中打开碰撞几何。

两层运动学复用 C 代码；仿真和 MCU 使用同样的 FD 负载。切换硬件后端的入口为 `backend:=hardware interface:=can0`。实际网关 netdev、厂商驱动协议、主板引脚、板级安全使能、真实工具反馈和相机标定必须先完成。[移植与验收步骤](docs/hardware.md)。外部视觉接口为 /camera/pcb_pose，消息 geometry_msgs/PoseStamped，frame_id=camera_optical；配置固定相机外参 camera_xyz 与 camera_xyzw 后输出 /c3/target。腕部相机需要再接入随关节变化的手眼 TF。

## 7. 首次运行常见现象

|现象|在终端 4 检查|处理|
|---|---|---|
|容器镜像构建停在 apt 或拉取镜像|看终端 1 的第一条下载错误|恢复镜像仓库连接后重跑 bash tools/container，已构建层可复用|
|rootless 报 newuidmap / subuid| `podman info`、`grep "^$USER:" /etc/subuid /etc/subgid` |确认当前用户具备系统分配的不重叠 subordinate UID/GID 范围；使用普通 WSL 用户运行上述命令|
|窗口没有出现| `echo "$DISPLAY"`、`ls /tmp/.X11-unix`、`tools/cell glxinfo -B` |使用已有 WSLg 的 Ubuntu 窗口；默认容器使用 Mesa 软件渲染|
|关节反馈缺失|终端 1 搜索 lib c3_plant、Failed to load、SDF 错误|先确认 colcon 成功，`tools/cell ros2 pkg prefix c3_cell` 指向当前 install|
|急停、碰撞或堵转| `tools/cell ros2 topic echo /c3/contact_pair --qos-durability transient_local --once` 或查看 status.error |实际接触对出现在发生接触故障后；核对图形、配方和附着状态|
|重启后报告 checkpoint exists|查看 checkpoint.json 中 pending 和 held|清洁边界可 resume；重新初始化仿真且确认空爪后调用 /c3/reset|
|无法看到 PCB / 状态不变|查 /c3/target、/c3/rx、/clock、status；检查物理是否暂停|先恢复 Gazebo 运行，确认初始进料反馈后再 auto|

## 8. GitHub 独立仓库

建议新建空仓库 `0206vandinh-web/ROS2_C3_ARM_CELL`，在本目录初始化。核对 origin 后沿用你之前的 HTTP/1.1 推送方式：

```bash
cd ~/A3/c3_arm_ws
git init -b main
git add .
git commit -m "Add six-axis PCB assembly cell for ROS 2 Humble and Gazebo Harmonic"
git remote add origin https://github.com/0206vandinh-web/ROS2_C3_ARM_CELL.git
git remote get-url --push origin
git -c http.version=HTTP/1.1 push -u origin main
```

## 技术参考

- [Gazebo 官方 Humble 与 Harmonic 组合说明](https://gazebosim.org/docs/harmonic/ros_installation/)
- [Gazebo 11 与 Harmonic 在 Jammy 的安装说明](https://gazebosim.org/docs/harmonic/install_gz11_side_by_side/)
- [Gazebo Sim 8 API](https://gazebosim.org/api/sim/8/)
- [MoveIt 2 Humble 规划场景](https://moveit.picknik.ai/humble/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html)
- [STM32H743ZI 产品资料](https://www.st.com/en/microcontrollers-microprocessors/stm32h743zi.html)
