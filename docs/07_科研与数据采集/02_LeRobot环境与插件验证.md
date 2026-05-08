# LeRobot 环境与插件验证

LeRobot 是机器人学习工具链，负责遥操、数据采集、数据集管理、回放、训练和评估流程。ZYArm 通过 `lerobot_robot_zyarm` 插件接入 LeRobot，插件再通过 `zyarm_sdk` 访问串口和固件。

```text
LeRobot 命令
  -> lerobot_robot_zyarm
  -> zyarm_sdk Python
  -> 串口 / 固件
```

## 当前支持版本

第一版固定支持 PyPI 正式版：

```bash
pip install lerobot==0.5.1
```

后续升级 LeRobot 时，需要验证 Robot/Teleoperator API、camera 配置、record/replay 命令和 dataset 写入行为。版本升级属于插件维护工作，见 [LeRobot 开发](../09_开发者指南/07_LeRobot开发/README.md)。

## 安装

建议在仓库根目录执行：

```bash
pip install lerobot==0.5.1
pip install -e software/zyarm_sdk/python
pip install -e software/lerobot_robot_zyarm
```

参数和命令含义：

| 命令 | 含义 |
| --- | --- |
| `pip install lerobot==0.5.1` | 安装当前文档验证过的 LeRobot 版本 |
| `pip install -e software/zyarm_sdk/python` | 以可编辑方式安装 ZYArm Python SDK |
| `pip install -e software/lerobot_robot_zyarm` | 以可编辑方式安装 ZYArm 的 LeRobot 插件 |

`-e` 表示 editable install。它会让 Python 直接使用仓库中的源码，适合本地开发和调试；如果只是使用现有能力，也可以按这里安装。

## 插件导入验证

安装后执行：

```bash
python -c "import lerobot_robot_zyarm; print('zyarm LeRobot plugin ok')"
```

看到下面输出表示插件可以被 Python 找到：

```text
zyarm LeRobot plugin ok
```

这一步只验证 Python 包能导入，不代表机械臂已经连接成功。下一步需要确认设备角色和串口。

## 常见失败

| 现象 | 可能原因 | 处理方向 |
| --- | --- | --- |
| `ModuleNotFoundError: lerobot_robot_zyarm` | 没有安装插件，或当前终端不在同一个 Python 环境 | 重新执行 `pip install -e software/lerobot_robot_zyarm` |
| `ModuleNotFoundError: zyarm_sdk` | 没有安装 Python SDK | 重新执行 `pip install -e software/zyarm_sdk/python` |
| LeRobot 命令不存在 | LeRobot 没有安装到当前环境 | 确认 `pip install lerobot==0.5.1` 成功 |
| Windows 与 Ubuntu 命令表现不同 | Python 环境、串口名和设备权限不同 | 先确认当前终端使用的 Python 环境和串口号 |

## 下一步

继续进入 [设备角色与端口确认](03_设备角色与端口确认.md)，确认 leader/follower 的串口和物理角色。

## 待项目方补充

> 待项目方补充：请提供推荐 Python 版本、推荐虚拟环境创建方式、Windows/Ubuntu 安装截图、LeRobot 安装耗时和常见依赖错误截图。
