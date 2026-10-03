# my_uav_planner — Simuro 无人机对抗赛

赛题：**全球校园人工智能算法精英大赛 · simuro 无人机对抗赛**（仿真，无硬件）。
规则原文（本仓库内副本）：`docs/competition_rules.pdf`。

---

## 一句话任务

1v1 空中对抗。自起飞点**自主起飞**，在 **5 分钟仿真时间**内机动到对方机尾后，
用前视单目相机**识别对方机尾二维码**并上报，**先正确上报者获胜**。
超时则比绕中心障碍柱的**顺时针净圈数**。

关键约束：飞行高度不得低于 **1.2m**；禁止订阅真值话题、禁止访问对方命名空间。

---

## 项目进度

| 模块 | 状态 | 说明 |
|---|---|---|
| `my_uav_planner/rules.py` | ✅ 完成 | 规则→代码：场地几何、飞行包线、计圈、互撞判负、超时判定。纯 stdlib，无仿真依赖 |
| `test/test_rules.py` | ✅ 完成 | 32 个单测全过 |
| `eval/rule_tracker.py` | ✅ 完成 | 离线规则状态机，输入位姿速度时序，输出终局判定 |
| `eval/run_eval.py` | 🟡 冒烟通过 | `--synthetic` 模式 4 项检查通过；`--replay` 待仿真日志格式 |
| `my_uav_planner/perception.py` | ⬜ 未开始 | 相机→二维码解码 + 相对位姿估计。**依赖仿真器** |
| `my_uav_planner/decision.py` | ⬜ 未开始 | 状态机：起飞/搜索/尾随/绕圈/避撞 |
| `my_uav_planner/control.py` | ⬜ 未开始 | 位置/速度控制回路。**依赖模板发布的话题接口** |
| `my_uav_planner/planner_node.py` | ⬜ 未开始 | ROS 2 节点入口。**依赖模板** |
| 仿真器 launcher | ❌ 阻塞 | `tools/simuro_launcher` 是 0 字节空文件，未下载 |
| `libzbar` 原生库 | ❌ 阻塞 | 系统未安装，`pyzbar.decode()` 会运行时崩溃 |

---

## 环境现状

- `uv` 管理的 venv，Python 3.11（`uav_ws/.venv`）
- 已装依赖：`opencv-python 5.0.0`、`numpy 2.4.6`、`pyzbar 0.1.9`
- ⚠️ **`pyzbar` 能 import 但解不了码** —— 原生库 `libzbar` 缺失。
  修复：`sudo apt install libzbar0`，然后解一张测试图确认链路通。
- ⚠️ **`ros2` 不在 PATH**。本赛题几乎确定是 ROS 2 + Gazebo，但需要确认
  仿真器是自带 ROS 环境，还是要求系统装 ROS 2。

---

## 目录结构

```
src/my_uav_planner/
├── my_uav_planner/
│   ├── rules.py          # 规则→代码（无依赖，先写的就是这块）
│   ├── perception.py     # 相机 → 二维码 + 相对位姿        [TODO]
│   ├── decision.py       # 状态机                          [TODO]
│   ├── control.py        # 控制回路                        [TODO]
│   └── planner_node.py   # ROS 2 节点入口                  [TODO]
├── config/params.yaml    # 速度/高度/增益，不硬编码      [TODO]
├── eval/
│   ├── rule_tracker.py   # 离线规则状态机
│   └── run_eval.py       # 批量跑对局 / 回放
├── test/                 # 单测
├── docs/
│   └── competition_rules.pdf   # 规则原文副本
├── resource/             # ament 资源标记
├── package.xml           # ROS 2 包清单                    [TODO]
└── setup.py              # ament_python 入口              [TODO]
```

仓库根 `uav_ws/`：`.venv/`（uv 管理）、`pyproject.toml`、`src/`。

设计原则：把**判罚逻辑全部集中在 `rules.py`**。这类逻辑错一处就丢一局，集中
在一个无依赖文件里才能反复核对、单独测试。

---

## 语言选型：Python 优先

先用 Python 跑通基线。视觉调参、状态机迭代在 Python 里快得多，而 `cv2`/`numpy`
的重活都在 C++ 底层。

**性能注意**：二维码解码（pyzbar）是 10–30ms 级别，而上报频率被限制在 **30Hz**
（超频消息直接丢弃）。所以解码要**降频跑**（如每 3 帧一次）或放独立线程，别塞进
主控制循环。

只有当某节点被 `eval/` 量出真成瓶颈时，再用 C++ 重写**那一个节点**（ROS 2 允许
C++/Python 节点混跑）。不要一开始就全 C++。

---

## 关键规则速查（写代码时对照）

- **场地**：9m×6m×3m 封闭空间，原点在地面中心。起飞点 (±3, 0)。中心障碍柱 0.6×0.6×2.4m
- **高度**：机器人参考点（里程计原点）≥ 1.2m。开始信号后 **15s** 内须首次达标，否则判负
- **低空**：低于 1.2m **连续超 10s** 判负；回到 1.2m 以上后 10s 内再次低于 1.2m 也判负
- **触地**：起飞宽限期结束后触地直接判负
- **计圈门**：(+3,0) 一方在 x>0 一侧，(-3,0) 一方在 x<0 一侧；**俯视顺时针**完整一圈穿过计圈门才计 1 圈；逆时针不计
- **碰撞扣圈**：撞墙/天花板/障碍柱每次扣 1 圈，净圈数可为负。<1s 内连续接触算一次
- **互撞判负**：接触前 0.5s 窗口内，各自速度在"本方→对方"方向投影模长的平均值，**大的一方判负**
- **超时**：净圈数高者胜；相等则比结束时到 z 轴的水平距离，**近者胜**
- **上报**：频率须 < 30Hz，超频丢弃；内容错误不判罚

---

## 待办 / 下一步（按优先级）

1. **【阻塞】拿到仿真器** —— 加 QQ 群 **1087966454** 问要环境模板 + 样例程序；
   下载 launcher：`wget https://static.simuro.liuyaorobot.com/simuro_launcher/latest-linux -O simuro_launcher && chmod +x simuro_launcher`
2. **【阻塞】装 `libzbar0`** 并验证 `pyzbar.decode()` 真能解出一张测试二维码
3. **确认 ROS 2 来源** —— launcher 起来后 `ros2 topic list`，据此定 `package.xml` / `setup.py`
4. 拿到模板后，照抄其话题名与接口，实现 `perception.py` / `control.py` / `planner_node.py`
5. 用 `eval/run_eval.py` 跑自对局，量化胜率与耗时

---

## 开放问题（未确认，先记下）

- [ ] 仿真器是否自带 ROS 2 发行版？版本？
- [ ] 话题名、控制指令接口、里程计输出格式？（等模板）
- [ ] 无人机与二维码板的精确碰撞尺寸？（现在是占位值 `UAV_BODY_HALF_EXTENTS`）
- [ ] 本队固定首发出现在哪一侧？还是赛前随机分配？（代码按两侧镜像设计，未假设）
- [ ] 策略提交方式（"赛前由官方统一发布"）—— 功能包源码？编译目标？

---

## 常用命令

```bash
# 跑单测
cd uav_ws && .venv/bin/python -m unittest discover -s src/my_uav_planner/test -t src/my_uav_planner -v

# 跑 eval 冒烟测试
cd uav_ws && PYTHONPATH=src/my_uav_planner .venv/bin/python src/my_uav_planner/eval/run_eval.py --synthetic
```
