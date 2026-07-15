# HANDOFF_REPORT

## 项目整体描述

- 项目用途：用于实现和验证 Unitree 机器人里程计信息获取工具。当前已实现 G1 里程计读取程序，后续计划在同一项目内增加 Go2 机器狗里程计信息获取程序。
- 核心功能：订阅 Unitree 机器人里程计 DDS 话题，打印位置、速度、欧拉角、yaw 角速度和四元数。当前已验证 G1；Go2 的 topic、消息类型和读取程序尚未实现，标记为未确认。
- 主要模块：`src/g1_odometer_probe.cpp` 是当前唯一运行入口；`CMakeLists.txt` 负责构建；`README.md` 说明依赖、构建和运行命令。
- 关键目录结构：
  - `src/`：C++ 源码。
  - `build/`：CMake 构建输出，已在 `.gitignore` 忽略。
  - `unitree_sdk2/`：项目内本地 Unitree SDK2 依赖。
- 主要技术栈：C++17、CMake、Unitree SDK2 DDS channel API。
- 运行入口：当前为 `g1_odometer_probe` 可执行程序；Go2 入口待新增。
- 核心数据流：G1 程序初始化 `ChannelFactory` 并绑定指定网卡，然后订阅 `rt/lf/odommodestate` 或 `rt/odommodestate`，在回调中读取 `unitree_go::msg::dds_::SportModeState_` 并输出字段。
- 重要配置文件：`CMakeLists.txt` 优先使用项目内 `unitree_sdk2`，缺失本地 SDK 时再通过 `find_package(unitree_sdk2 REQUIRED)` 查找 SDK；运行时参数指定网卡、topic、样本数、超时和 DDS domain。
- 外部依赖：Unitree SDK2。项目内已加入 `unitree_sdk2` 子目录；顶层 CMake 已调整为优先使用该本地 SDK，缺失本地 SDK 时再回退到 `find_package(unitree_sdk2 REQUIRED)`。该 SDK 自带 Linux 预编译库，无法用当前 macOS 工具链直接链接。
- 常用命令：
  - `cmake -S . -B build`
  - `cmake --build build`
  - `./build/g1_odometer_probe en11 --topic lf --samples 5 --timeout 10`
- 部署或运行方式：在安装 Unitree SDK2 且网线连接 G1 后，本机建议先用活动有线网口 `en11` 运行低频 topic 探测；已验证的稳定方式是在 `192.168.123.164` 机器人侧直接运行 `/home/unitree/g1_odometer_probe`。
- 本轮修改涉及模块位置：项目目录已从 `/Users/firmiana/Desktop/G1` 重命名为 `/Users/firmiana/Desktop/Odom`，并更新 README、CMake 项目名和本交接报告。

## 当前状态

- 已根据 Unitree 官方网页确认接口：高频 topic 为 `rt/odommodestate`，低频 topic 为 `rt/lf/odommodestate`，消息类型为 `unitree_go::msg::dds_::SportModeState_`。
- 当前目录 `/Users/firmiana/Desktop/Odom` 不是 Git 仓库。
- 已新增并验证 C++ G1 探测程序、README、CMake 文件和交接报告；顶层 CMake 已改为优先使用项目内 `unitree_sdk2`。
- 已在机器人侧实际获取到 G1 里程计数据；持续输出版本已构建、复制到远端并验证可运行。
- `src/g1_odometer_probe.cpp` 已补充中文注释，解释 G1 话题、参数语义、DDS 初始化、订阅回调、等待退出条件和样本打印字段；本次未改变运行逻辑。
- Go2 机器狗里程计获取程序尚未开始实现；相关接口文档、topic、消息类型和机器人连接方式未确认。

## 已验证事实

- 网页文档说明里程计服务发布位置、速度、欧拉角、yaw 角速度和四元数。
- 网页文档说明 `rt/odommodestate` 为 500Hz，`rt/lf/odommodestate` 为 20Hz。
- 本机 `en11` 网口处于 active 状态，IP 为 `169.254.86.94`，链路速率显示 `100baseTX <full-duplex>`。
- 本机 `en0` 也处于 active 状态，但其 IP 为 `192.168.1.31`，更像普通网络连接。
- `/Users/firmiana/Desktop/Odom` 不是 Git 仓库，父级扫描只发现另一个目录 `/Users/firmiana/Desktop/G1机器人感知-可行性分析/.git`。
- 历史工程 `/Users/firmiana/Documents/Codex/2026-06-09/git-github-com-firmiana114-air-robot-3/work/air_robot_gt_projects/unitree_sdk2` 包含 SDK 头文件和 Linux 风格库目录，但 CMake 包配置不完整。
- 执行 `cmake -S . -B build` 失败，原因是找不到 `unitree_sdk2Config.cmake` 或 `unitree_sdk2-config.cmake`。
- 指定历史 SDK 的 `build_docker` 配置目录后，CMake 失败于缺失 `unitree_sdk2Targets.cmake`。
- 使用历史 SDK 头文件做 `-fsyntax-only` 检查时，先需要补充 `thirdparty/include/ddscxx`，随后失败于 macOS 缺少 Linux 头 `sys/sysinfo.h`。
- 项目内新增的 `unitree_sdk2` 包含 `lib/aarch64/libunitree_sdk2.a`、`lib/x86_64/libunitree_sdk2.a` 以及 `thirdparty/lib/*/libddsc.so`、`libddscxx.so`；`file` 检查确认 `.so` 为 Linux ELF 格式。
- 本轮将顶层 `CMakeLists.txt` 改为优先 `add_subdirectory(unitree_sdk2)`，并关闭 SDK examples；重新执行 `cmake -S . -B build-local` 已能进入项目内 SDK，但在 macOS 上失败于 `Unitree SDK library for the architecture is not found`，因为当前 `CMAKE_SYSTEM_PROCESSOR=arm64`，SDK 主库目录只有 `aarch64` 和 `x86_64`。
- 使用项目内 SDK 头文件做 `-fsyntax-only` 仍失败于 `sys/sysinfo.h`，说明当前 macOS 工具链不适合直接编译该 SDK。
- Docker 客户端已安装，但 Docker daemon 未运行，无法在本轮使用 Linux 容器验证构建。
- 用户允许启动 Docker 后，Docker Desktop daemon 已启动成功，`docker info` 显示 Linux `aarch64` 环境。
- 使用 `ubuntu:20.04` arm64 容器挂载项目后，安装 `ca-certificates cmake g++ make`，执行 `cmake -S /work -B /tmp/g1-build && cmake --build /tmp/g1-build -j2` 成功，证明项目内 `unitree_sdk2` 可在 Linux arm64 容器中构建本项目。
- 使用 `--network host` 检查容器网口，容器只看到 Docker 内部 `eth0=192.168.65.3/24`、`services1`、`docker0` 等接口，没有看到 macOS 物理有线网口 `en11`。
- 在同类容器中运行 `/work/build/g1_odometer_probe eth0 --topic lf --samples 1 --timeout 5`，程序成功初始化 DDS channel 并订阅 `rt/lf/odommodestate`，但 5 秒内未收到样本，输出 `received=0`。当前判断主要阻塞在 Docker Desktop 网络无法直通机器人 DDS/物理网口。
- 通过 macOS 管理员授权给 `en11` 临时添加 `192.168.123.222/24` 地址别名后，主机可 ping 通 `192.168.123.161` 和 `192.168.123.164`；`192.168.123.164` 开放 SSH 和 HTTP，HTTP 页面标题为 `unitree-upgrade`。
- 容器在 `--network host` 下也能 ping 通 `192.168.123.161` 和 `192.168.123.164`，但 Unitree SDK DDS 订阅仍无样本，说明 Docker Desktop 网络能做部分 IP 转发，但不能可靠承载 DDS 多播/发现。
- 使用 root 授权短时抓 `en11` 包，确认 `192.168.123.161` 持续向 `239.255.0.1:7401` 发送 RTPS/UDP 数据包，6 秒抓到 11584 个包，物理网口确实能看到机器人 DDS 流量。
- 对 `/tmp/g1_full.pcap` 做 IP 分片重组后，提取 RTPS DATA 中的 CDR payload；使用 Linux arm64 容器内的 Unitree SDK `deserialize_sample_from_buffer` 对 `unitree_go::msg::dds_::SportModeState_` 反序列化成功，输出了 position、velocity、rpy、yaw_speed、quaternion 等字段。部分字段存在 4 字节偏移解释差异，仍需用官方订阅或 ROS 2 topic echo 交叉确认最终字段边界。
- 用户补充：SSH 到 `192.168.123.164` 后执行 `ros2 topic list` 能看到 `/odommodestate`，应为 ROS 2 暴露的里程计话题。
- 用户提供 `unitree@192.168.123.164` 密码后，SSH 确认远端为 `aarch64` Ubuntu，`eth0=192.168.123.164/24`，`/usr/local/lib` 已有 `libunitree_sdk2.a`、`libddsc.so.0`、`libddscxx.so.0`。
- 已将本地 Linux arm64 构建产物 `build/g1_odometer_probe` 复制到远端 `/home/unitree/g1_odometer_probe`，并在远端执行 `LD_LIBRARY_PATH=/usr/local/lib timeout 15s /home/unitree/g1_odometer_probe eth0 --topic both --samples 3 --timeout 12`。
- 远端运行成功订阅 `rt/odommodestate` 高频话题并读取到 3 条样本，例如 position 约 `[0.00101664, -0.00271847, 0.729375]`，velocity 约 `[-9.70085e-08, -5.73386e-08, -2.81513e-06]`，rpy 约 `[0.00158292, 0.026922, 0.0413264]`，yaw_speed 约 `0.00106526`，quaternion 约 `[0.999696, 0.000513101, 0.0134741, 0.0206492]`。主要目标“获取里程计信息”已验证成功。
- 本轮将 `g1_odometer_probe` 改为支持持续输出：`--samples 0` 表示一直运行直到 Ctrl+C，`--timeout 0` 表示禁用超时；新增 `--print-every N` 用于高频 topic 降采样打印，避免 500Hz 持续刷屏。
- 已重新用 Linux arm64 容器构建新版程序并复制到 `192.168.123.164:/home/unitree/g1_odometer_probe`；远端执行 `LD_LIBRARY_PATH=/usr/local/lib timeout 3s /home/unitree/g1_odometer_probe eth0 --topic lf --samples 0 --timeout 0` 成功持续输出低频里程计样本，直到外层 timeout 发送终止信号，程序记录 `stopped by signal`。
- 项目目录已按用户要求从 `/Users/firmiana/Desktop/G1` 重命名为 `/Users/firmiana/Desktop/Odom`；顶层 CMake 项目名从 `g1_odometer_probe` 改为更通用的 `unitree_odometer_tools`，现有可执行文件名仍保持 `g1_odometer_probe`，避免破坏已验证的运行命令。
- 重命名后已在临时 Linux arm64 容器中重新执行 CMake 配置和构建，`g1_odometer_probe` 编译通过。
- 已为 `src/g1_odometer_probe.cpp` 添加中文逻辑注释；变更仅涉及注释，不影响编译产物行为。
- 注释变更后已再次在临时 Linux arm64 容器中执行 CMake 配置和构建，`g1_odometer_probe` 编译通过。

## 阻塞问题

- 未确认本机是否已经安装可在当前 macOS 环境直接构建/链接的 Unitree SDK2；已发现的 SDK 更适合 Linux 环境。
- Docker Desktop 虽能做部分 IP 转发，但不能可靠承载机器人 DDS 多播/发现；当前可行验证路径仍是在机器人侧或可直连物理网口的 Linux 环境运行。
- Go2 机器狗里程计获取程序未实现；Go2 侧接口文档、topic、消息类型和运行目标机器均未确认。

## 下一步

- 在 Linux 主机或能直接访问机器人网口的 Linux 环境中运行本项目；项目内 SDK 已可被顶层 CMake 直接纳入，不再必须先系统安装 SDK。
- 若继续尝试 Docker Desktop，需要解决容器到 `en11` 的二层网络/DDS 多播访问问题；当前 `--network host` 仍未暴露 `en11`。
- 注意：远端 `ros2 topic type /odommodestate` 返回 `unitree_go/msg/SportModeState`，但 `ros2 topic echo /odommodestate` 在非交互环境失败于缺少 Python 包 `unitree_go`；当前可行读取路径是直接运行 `g1_odometer_probe`。
- 构建：`cmake -S . -B build && cmake --build build`。
- 用 `en11` 运行：`./build/g1_odometer_probe en11 --topic lf --samples 5 --timeout 10`。
- 若低频无数据，再尝试 `--topic both` 或检查 G1 侧 State Estimator 服务版本是否满足文档要求。
- 若 CMake 找不到 SDK，可设置 `CMAKE_PREFIX_PATH` 指向 SDK 安装前缀。
- 增加 Go2 程序前，需要先确认 Go2 官方里程计接口、topic 名称、消息类型、SDK2 支持情况和运行位置。

## 注意事项

- 日志级别使用 INFO/WARN/ERROR，没有使用 DEBUG/TRACE。
- 程序只输出必要诊断和结构化里程计字段，不记录密钥、令牌或大体积原始数据。
- 高频 topic 为 500Hz，持续运行时建议配合 `--print-every`，例如 `--topic hf --samples 0 --timeout 0 --print-every 100`。
- 远端持续低频输出命令：`LD_LIBRARY_PATH=/usr/local/lib /home/unitree/g1_odometer_probe eth0 --topic lf --samples 0 --timeout 0`，按 Ctrl+C 停止。
