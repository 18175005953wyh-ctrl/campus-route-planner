# Campus Route Planner — 校园路径规划器

使用 C11、邻接矩阵、Dijkstra 和 A* 算法实现的命令行校园路径规划工具，输出完整路线、总距离及搜索指标对照。

## 项目背景与选择原因

环境设计关注建筑、道路和公共空间之间的联系。校园中的图书馆、广场、宿舍等地点可以抽象为图的顶点，道路可以抽象为边，道路长度就是边的权重。最短路径算法为这些空间联系提供了一种可计算的表达方式。

本项目把环境设计与数据结构学习结合起来：以一个虚构的校园为例，将空间关系录入文件，用程序回答“从某个地点到另一个地点，怎样走总距离更短”。它是算法练习与作品展示项目，示例距离不是实地测量数据，也不是实际导航服务。

这是独立的 `campus-route-planner` 仓库。以下构建命令均从包含 `CMakeLists.txt` 的仓库根目录执行。

## 主要功能

- 显示全部地点的编号、名称和类型。
- 显示所有双向直接道路，每条道路只显示一次。
- 根据起终点编号计算最短路径，显示完整地点顺序与总距离。
- 起终点相同时返回该地点和 0 米；不连通时显示无路径提示。
- 使用不区分大小写的英文名称关键词搜索，返回全部匹配地点。
- 汇总地点数、道路数、道路总长、平均长度、最大度数地点、孤立地点及类型数量。
- 从 CSV 自动读取地图，对错误行输出带行号的警告并跳过。
- 校验空输入、非法整数、过长输入和不存在的编号；输入结束时正常退出。
- 支持 `--data` 显式指定地图目录，以及构建后随可执行文件附带示例数据。
- 支持 `--algorithm dijkstra|astar|compare`，默认 Dijkstra；A* 使用可选示意坐标文件和缩放欧氏启发函数。

## 数据结构与算法

| 图概念 | 在程序中的对应关系 |
| --- | --- |
| 顶点 | 一个 `Location`，包含正整数编号、名称、类型和可选示意坐标 |
| 边 | 两个地点间的一条双向道路 |
| 权重 | 道路长度，正整数，单位米 |
| 邻接矩阵 | `Graph.adjacency[i][j]`，下标对应地点数组的位置 |
| 无直接道路 | `INF_DISTANCE`，值为 1,000,000,000 |
| 自身到自身 | 矩阵对角线为 0 |

地点编号不要求连续，也不等于数组下标。程序通过 `find_location_index_by_id()` 映射，例如测试中的编号 30 对应下标 2。图最多保存 50 个地点，不使用动态内存。

Dijkstra 的处理过程：

1. 起点距离设为 0，其他地点距离设为无穷大。
2. 从尚未确定的地点中选出当前距离最小的一个。
3. 检查它的相邻地点；如果经过它可以走得更短，就更新 `distance[]` 和 `previous[]`。
4. 重复直到终点距离确定，或已经没有可到达的未处理地点。
5. 从终点沿 `previous[]` 回到起点，将所得序列反转，得到完整路线。

`visited[]` 表示已确定最短距离的地点。数组实现的时间复杂度为 O(V²)，邻接矩阵空间复杂度为 O(V²)。非负权是 Dijkstra 的前提；本项目的 CSV 进一步要求道路距离严格大于 0。

相同总距离的候选路线保留先找到的一条，按照地点在 CSV 中的顺序进行选择，因此同一份输入的结果稳定。修改 CSV 中地点的顺序可能改变等长路线的选择，但不会改变最短距离。

为避免“有效路径距离达到无穷大标记”和整数溢出，每条道路最大长度为 `(INF_DISTANCE - 1) / (MAX_LOCATIONS - 1)`，即 **20,408,163 米**。正权图的最短路径无需重复顶点，最多 49 条边，所以合法最短路径始终小于无穷大标记。松弛操作在加法前也检查边界。所有道路总长度使用 `long long` 累加，因为它可能大于单条最短路径距离。

## 项目结构

```text
campus-route-planner/
├── src/
│   ├── main.c                # 菜单、命令行参数和运行流程
│   ├── graph.c               # 图、CSV、显示、搜索和摘要
│   ├── pathfinder.c          # Dijkstra/A*、共享回溯和路径验证
│   ├── geometry.c            # 欧氏距离、缩放系数与启发函数
│   └── input.c               # 行输入和整数解析
├── include/
│   ├── graph.h
│   ├── pathfinder.h
│   ├── geometry.h
│   └── input.h
├── data/
│   ├── locations.csv
│   ├── coordinates.csv       # 可选示意坐标，A*/compare 模式必须完整
│   └── roads.csv
├── tests/
│   ├── test_runner.c         # 独立 C 核心测试程序
│   ├── test_cli.cmake        # 菜单和启动参数的自动化测试
│   └── fixtures/
│       ├── test_locations.csv
│       ├── test_roads.csv
│       └── disconnected_roads.csv
├── screenshots/
│   └── .gitkeep
├── CMakeLists.txt
├── .github/workflows/tests.yml
├── .gitignore
├── LICENSE
└── README.md
```

使用的知识点包括结构体、固定数组、指针参数、邻接矩阵、最短路径、字符串处理、文件读写、模块化编程，以及 CMake、CTest 和 Git。源码为 `.c` 文件，CMake 声明 `LANGUAGES C`，不使用 C++ 或第三方库。

## 示例地图

示例包含 **12 个地点、18 条道路**：Main Gate、Library、Central Square、Teaching Building、Laboratory、Dormitory、Cafeteria、Sports Field、Art Center、Garden、Bus Stop 和 Administration Building。

一个可以验证算法的例子：

```text
Main Gate ---------------------------- Library
          direct road: 600 m

Main Gate --200 m--> Central Square --180 m--> Library
                     total: 380 m
```

图中的道路均可双向通行。程序应选择经过 Central Square 的 380 米路线，而非直接的 600 米道路。示例地图全部连通；孤立点和不连通图由测试数据覆盖。

## CSV 格式

`data/locations.csv` 无表头，每行三列：

```csv
1,Main Gate,Entrance
2,Library,Study
3,Central Square,Public Space
```

`data/roads.csv` 无表头，每行三列为“起点编号、终点编号、距离”：

```csv
1,2,600
1,3,200
3,2,180
```

规则：

- 编号为 `1` 到 `INT_MAX` 的整数，地点编号必须唯一；两端编号必须已经加载。
- 名称和类型为 1–63 字节的非空文本，支持内部空格；首尾空白去除，不支持控制字符。
- 暂不支持字段中的英文逗号，也不实现通用 CSV 引号转义；名称和类型建议使用英文。
- 空行跳过；接受常见 LF/CRLF 换行和首行 UTF-8 BOM。请保存为 UTF-8，程序不做通用编码转换。
- 超长行、格式错误、重复地点编号、未知道路端点、非正或超范围距离、自环均警告并跳过。
- 重复双向道路只保留最小距离，输出警告；不会将两条相同端点的道路计算两次。
- 超过 50 个地点的行警告并跳过；之后依赖这些未加载地点的道路也会跳过。
- 两个文件必须都能打开和读取，否则启动失败并返回非零状态码。文件存在但为空时可以进入菜单。
- 部分坏行不导致整个程序退出：继续使用有效地图数据，警告提示数据可能不完整。
- 程序只读取地图，不修改或保存地图文件。

## Visual Studio 2022 运行方法

需要安装“使用 C++ 的桌面开发”工作负载，其中包含 C 编译器、Windows SDK 和适用于 Windows 的 CMake 工具。工作负载名称中的 C++ 不代表项目使用 C++。

**打开文件夹方式：**

1. 选择“打开本地文件夹”，打开包含 `CMakeLists.txt` 的 `campus-route-planner` 仓库目录。
2. 等待 CMake 配置完成，选择 `x64-Debug`。
3. 在启动项中选择 **`campus_route_planner.exe`**，不要选测试程序 `route_tests.exe`。
4. 按 Ctrl+F5 编译运行，出现英文菜单后输入数字。

CMake 构建后会把示例 `data` 复制到可执行文件旁边。更新原始 CSV 后，如果没有触发可执行文件重新链接，可以执行“重新生成”，或运行时使用 `--data` 指向原始数据目录。

**生成解决方案方式：**在 Developer PowerShell for VS 2022 中进入仓库根目录，执行：

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DSTRICT_WARNINGS=ON
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
.\build\Debug\campus_route_planner.exe --data data
```

也可以打开 `build/CampusRoutePlanner.sln`，将 `campus_route_planner` 设为启动项目后运行。测试程序是独立目标，不会影响正常程序。

## CMake 构建方法

要求 CMake 3.20+ 和支持 C11 的编译器。MSVC 使用 `/W4`；GCC/Clang 使用 `-Wall -Wextra -Wpedantic`。启用 `STRICT_WARNINGS` 会将警告视为错误。

Linux/macOS 中，进入仓库根目录后执行：

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DSTRICT_WARNINGS=ON
cmake --build build
ctest --test-dir build --output-on-failure
./build/campus_route_planner --data data
```

在已加载 MSVC 环境的 Developer PowerShell 中，也可以使用 NMake：

```powershell
cmake -S . -B build-nmake -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Debug -DSTRICT_WARNINGS=ON
cmake --build build-nmake
ctest --test-dir build-nmake --output-on-failure
.\build-nmake\campus_route_planner.exe --data data
```

同一个构建目录不要混用不同生成器。NMake 示例的 `build-nmake/` 同样被忽略。

## 数据目录与启动参数

```text
campus_route_planner
campus_route_planner --data data
campus_route_planner --data "path with spaces"
campus_route_planner --help
```

显式使用 `--data` 时，只读取该目录；相对目录根据当前终端工作目录解析。参数不正确或目录路径太长会返回非零状态码。

不指定目录时，先尝试当前工作目录下的 `data/locations.csv`。找不到时，通过启动命令中的可执行文件路径寻找旁边的 `data`。同一张地图的两个文件始终从同一目录读取，不混用不同目录的文件。

通过 `PATH` 从其他目录仅输入裸命令名启动时，标准 C 无法可靠确定可执行文件的真实位置，建议显式使用 `--data`。数据目录和文件路径缓冲区为 1024 字节，不支持任意长度的路径。源码不包含任何开发机器绝对路径。

## 操作与真实示例

启动后菜单为：

```text
===== Campus Route Planner =====
1. List all locations
2. Show direct roads
3. Find shortest route
4. Search location
5. Show map summary
0. Exit
Select an option:
```

依次输入 `3`、`1`、`2`（每次按回车），查询校门到图书馆，实际输出：

```text
Shortest route:
Main Gate -> Central Square -> Library

Total distance: 380 m
```

输入 `4`，再输入 `building`，会找到 Teaching Building 和 Administration Building。输入 `5` 的实际摘要核心数据为：

```text
Locations: 12
Roads: 18
Total road length: 3690 m
Average road length: 205.00 m
Isolated locations: 0
Most connected location(s):
  Central Square (6 roads)
```

随后还会显示每种地点类型的数量。类型按完整字符串区分大小写统计，名称搜索则忽略英文大小写。没有道路时，平均长度为 0，最大度数地点显示 None；多个地点并列最大度数时全部显示。表格保留完整文本列，终端较窄时可能换行。

输入 `0` 退出，不会修改任何地图数据。

## 原版自动化测试与回归基线

原版基线提交 `ef6c557051e56f96bd0f4f89345a58fdada84d19` 包含两个测试组；2026-10-11 升级前重新验证全部通过：

- `core_tests`：独立的 `tests/test_runner.c`，共 29 项测试。
- `cli_tests`：CMake 驱动主程序的 11 个真实子进程场景，不依赖 Python 或第三方测试框架。

除 CTest 外，可在本项目目录直接运行核心测试（NMake 或单配置构建）：

```powershell
.\build-nmake\route_tests.exe tests/fixtures
```

Visual Studio 多配置构建则使用：

```powershell
.\build\Debug\route_tests.exe tests/fixtures
```

核心测试覆盖：地点和道路加载、编号映射、直接和多中间点最短路径、完整路径起终点与边距离验证、相同起终点、不连通与孤立点、搜索、非法 CSV、重复道路、容量上限、超长输入恢复、BOM、等长路径稳定性、最大合法边权与 50 顶点回溯、64 位道路总长，以及与独立 Floyd-Warshall 参考结果比较的 64 对起终点。

命令行测试覆盖：实际菜单、样例 380 米路线、空输入和非法/超长输入、未知编号、名称搜索、输入中断、缺失文件、空地图、不可达路线、损坏记录和可执行文件旁数据读取。

**2026-09-18 实际验证环境与结果：**

- Windows、MSVC 19.44.35225、CMake/CTest 3.31.6、NMake 生成器。
- C11 编译，启用 `/W4 /WX`；编译成功，无编译警告。
- CTest **2/2 测试组通过**：核心 **29/29**、命令行 **11/11**，无跳过、无失败。
- 单独执行核心测试和主程序演示，得到上述路径与地图摘要结果。
- 最初使用 Ninja 时，本地环境中的编译器探测阶段未完成，改用 NMake 后完成了全部构建测试。没有把 Ninja 的尝试记为通过。
- 未在 GCC、Clang 或 Linux/macOS 实际构建；代码使用标准 C11，但这些环境仍需复验。

测试故意输入坏行，因此测试日志里的 `Warning:` 和缺失测试文件提示属于预期场景，不是编译警告。失败的检查会令测试程序返回非零状态，Release 构建下也不会禁用检查。

核心测试在当前工作目录生成一个临时 CSV，结束时删除；CTest 在构建目录运行，命令行测试的输入及地图副本也位于构建目录。生成的缓存、构建文件和程序不应提交 Git。

## 截图位置

`screenshots/` 已有原版菜单及路线截图。A* 升级可补充 `astar-comparison.png`（校园对照）、`astar-decoy.png`（诱导支路）、`astar-tests.png`（本地测试）和 `astar-ci.png`（远程双平台检查）。这里的终端文本是实际输出摘录，不是截图；不要将本地测试截图当作远程 CI 成功证明。

## 当前限制与改进方向

当前最多 50 个地点，只支持双向、正整数权重道路；地图只读，无编辑和保存功能。不支持带逗号的名称或通用 CSV 引号规则，不处理实时路况、通行时间或实际地理坐标。默认数据路径发现依赖工作目录或启动命令提供的程序路径，复杂启动环境建议使用 `--data`。

邻接矩阵适合这一规模的学习示例，但对大量稀疏地点会浪费空间。A* 已实现；邻接表、区分步行和骑行道路、临时封路、无障碍路线、真实地图和 Web 界面仍未实现。

## 我通过项目练习到的能力

- 将校园空间关系抽象为图，并区分地点编号与数组下标。
- 用结构体和固定数组组织数据，维护双向矩阵的一致性。
- 理解 Dijkstra 的距离更新与前驱回溯。
- 处理输入、CSV 解析、文件错误和数值边界。
- 将界面与算法分离，利用独立测试验证结果。
- 使用 CMake/CTest 构建与测试，并整理 Git 分支、文档和演示数据。

阅读建议：先读 `include/graph.h` 理解数据，再读 `src/pathfinder.c` 理解算法，最后读 `src/main.c` 理解整个流程。

本项目使用 [MIT License](LICENSE)。

## A* Route Search

### Difference from Dijkstra

Dijkstra 选择 `g(n)` 最小的未关闭节点；A* 选择 `f(n) = g(n) + h(n)` 最小的节点。
两者共用邻接矩阵、O(V²)选点循环、整数距离松弛与路径回溯，搜索数组每次查询重新初始化。
相同优先级按地点输入顺序选择。等长最短路线可以不同，compare 检查 found、总距离和各条路径合法性，不要求节点序列相同。
内部一致性检查失败会输出错误并以非零状态退出。

### Schematic Coordinates

`data/coordinates.csv` 无表头，每行是 `location_id,x,y`。地点编号必须已存在，坐标为 `[-1000000,1000000]` 的整数。
这是算法实验的**示意坐标，不是真实 GIS、经纬度或测绘数据**，不改变原有道路权重。
原来的 `locations.csv`、`roads.csv` 均保持三列不变。

未知 ID、重复坐标、缺字段、额外字段、非法整数、超范围数据均按行警告并跳过；重复坐标保留第一条有效记录。
默认 Dijkstra 完全不读取坐标文件。A*/compare 必须能读取坐标文件，且每个已加载地点都有坐标，否则在进入菜单前失败。
空地图搭配空坐标文件允许进入菜单，但没有可查询的地点。

### Scaled Euclidean Heuristic

设 `d(u,v)` 为示意坐标欧氏距离，`w(u,v)` 为整数道路权重：

```text
alpha = min(w(u,v) / d(u,v))，只考虑存在且 d(u,v)>0 的道路
h(n) = alpha * d(n,goal)
```

没有几何长度大于零的道路时取 `alpha=0`；终端会说明 A* 退化为 Dijkstra。
计算坐标差前先转 double，实际路径距离仍累加整数道路权重。
每次查询开始时计算一次 alpha 和所有节点的 h，扩展循环内不重新扫描道路计算 alpha。

为处理 double 舍入，实现将理论 alpha 乘以 `(1-1e-12)` 留出微小余量，再检查每条边的启发一致性。
如果数值检查不满足，则本次查询全部使用零启发；显示的 `Heuristic scale` 是本次实际采用的系数。

### Why the Heuristic Is Admissible

由 alpha 的定义，对每条边都有 `alpha*d(u,v) <= w(u,v)`。
由三角不等式：

```text
h(u) = alpha*d(u,goal)
     <= alpha*d(u,v) + alpha*d(v,goal)
     <= w(u,v) + h(v)
```

这就是一致性。沿任意通往终点的路径累加，结合 `h(goal)=0`，得到 `h(u)` 不超过路径总权重，因此也不超过最短路径长度。
一致启发函数下，关闭节点后无需重新打开；缩小 alpha 或使用零启发也保留上述性质。

### Algorithm Comparison

指标口径：`expanded` 为选中并关闭的节点数，**包括起点和终点**；`relaxed` 只统计成功降低邻居 g 的次数，同一节点可以被成功松弛多次。
起终点相同时是 expanded=1、relaxed=0。
两种算法用同图、同起终点、同数据结构及独立的初始搜索状态，不统计毫秒“加速比”。

### Reproduction Steps

在 VS 2022 的开发者 PowerShell 中，从仓库根目录执行（NMake 单配置）：

```powershell
cmake -S . -B build-astar -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Debug -DSTRICT_WARNINGS=ON
cmake --build build-astar
ctest --test-dir build-astar -C Debug --output-on-failure -V
.\build-astar\campus_route_planner.exe --data data --algorithm compare
```

依次输入 `3`、`1`、`2`，每次按回车，查询校门到图书馆。再输入 `3`、`1`、`8` 查询运动场，或 `3`、`5`、`11` 查询实验室到公交站，最后 `0` 退出。
`--algorithm astar --data data` 的参数顺序也有效。算法值只接受小写；重复参数、缺值、未知参数均返回非零；`--help` 只能单独使用。
只运行原版行为可用 `--algorithm dijkstra` 或不传该参数，菜单和查询输入顺序不变。

诱导支路实验（同一个 PowerShell）：

```powershell
New-Item -ItemType Directory -Force build-astar/decoy | Out-Null
Copy-Item tests/fixtures/astar_decoy_locations.csv build-astar/decoy/locations.csv
Copy-Item tests/fixtures/astar_decoy_roads.csv build-astar/decoy/roads.csv
Copy-Item tests/fixtures/astar_decoy_coordinates.csv build-astar/decoy/coordinates.csv
.\build-astar\campus_route_planner.exe --data build-astar/decoy --algorithm compare
```

依次输入 `3`、`1`、`3`、`0`。所有副本在忽略的构建目录中，原始校园数据不变。
Visual Studio 多配置构建的程序路径为 `build/Debug/campus_route_planner.exe`；Linux 使用 `./build/campus_route_planner`。

### Actual Results

2026-10-11 在代码提交 **`11bec7a688744fff4bf5a48129f63207df0e2b0d`** 上实际运行，Windows x64、MSVC 19.44、NMake Debug、严格警告 `/W4 /WX`。
后续文档提交不改变这组代码和输入；可检出该提交复现。

| Route | Algorithm | Distance (m) | Expanded | Relaxed |
|---|---|---:|---:|---:|
| Main Gate → Library (1→2) | Dijkstra | 380 | 6 | 11 |
| Main Gate → Library (1→2) | A* | 380 | 4 | 9 |
| Main Gate → Sports Field (1→8) | Dijkstra | 600 | 12 | 13 |
| Main Gate → Sports Field (1→8) | A* | 600 | 9 | 12 |
| Laboratory → Bus Stop (5→11) | Dijkstra | 350 | 5 | 7 |
| Laboratory → Bus Stop (5→11) | A* | 350 | 4 | 5 |
| Decoy: Start → Goal (1→3) | Dijkstra | 20 | 6 | 5 |
| Decoy: Start → Goal (1→3) | A* | 20 | 3 | 3 |

校园实际 scale 为 `0.459619407771`。校门到图书馆两者路线均为 `Main Gate -> Central Square -> Library`。
诱导支路图实际 scale 为 `0.999999999999`，两者路线都是 `Start -> Midpoint -> Goal`。
该图西侧三条支路的早期 g 分别为 1、2、3，Dijkstra 会先展开它们；A* 中这些节点的 f 大于主路径的约 20，因此能跳过它们。
校园的缩放值由道路权重约束，不能为了展示更多减少量而随意增大。

实测终端摘录：

```text
Dijkstra: distance=380 m, expanded=6, relaxed=11
A*: distance=380 m, expanded=4, relaxed=9
Heuristic scale: 0.459619407771
Distance agreement: yes
Path validity: yes
```

### Tests and Cross-validation

同一提交本地实际输出：

```text
55 tests passed, 0 tests failed.
38 CLI cases passed
100% tests passed, 0 tests failed out of 2
```

共 **93 项检查**（原有40项全部保留，新增53项），CTest 将它们包装成2组；不是“只有2项测试”。
核心覆盖坐标格式与边界、启发函数一致性与非负性、零启发退化、直达/多段/孤立/相同起终点、错误索引、最大距离50点路径、指标统计、不同等长路线、损坏结果检测及独立重复查询。
独立 Floyd–Warshall 校验原8点图64对、校园12点图144对、20张固定生成12点图2880对，两种算法均核对距离和各自路径。
CLI 另外对校园144对连续查询，检查所有结果一致；还覆盖两种参数顺序、非法参数、缺失/不完整坐标、无坐标时默认Dijkstra、可执行文件旁查找数据、路径空格和旧菜单回归。

GitHub Actions 配置位于 `.github/workflows/tests.yml`，工作流名 **Campus Route Planner tests**，推送及 PR 会分别在 `ubuntu-latest`、`windows-latest` 上严格构建并运行 CTest。
**远程 CI 尚未确认**：此处只记录上述本地验证，不能视为 Linux 或 GitHub Actions 已通过。发布后以最新提交的两个远程任务真实结果为准。

### Limitations

最多50个地点，邻接矩阵和线性选点保持 O(V²) 时间、O(V²) 图空间；启发函数准备也是 O(V²)，因此节点少展开不等于运行时间同比减少。
坐标只是示意数据；alpha=0、坐标区分度弱或大量等价路线时不保证减少节点。
不包含堆、邻接表、实时路况、动态封路或真实导航；比较只改变节点优先级，不混入其他优化。
