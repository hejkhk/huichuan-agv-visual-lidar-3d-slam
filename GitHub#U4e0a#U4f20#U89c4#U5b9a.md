# GitHub 上传规定

本文档用于规范本项目向 GitHub 私有仓库提交和推送代码的流程。

仓库地址：`https://github.com/hejkhk/huichuan-agv-ros2-foxy`

## 1. 基本原则

1. GitHub 只保存可维护的上位机源码、配置、文档和必要的示例地图。
2. 当前仓库必须保持为 **Private**，未经项目负责人确认不得改为 Public，也不得添加无关协作者。
3. 每次只提交本次确实修改的文件，不得顺手提交运行缓存、日志、编译产物或无关测试文件。
4. 已通过实车验证的启动脚本、Cartographer 参数、Nav2 参数和通信协议不得在未验证的情况下覆盖。
5. 禁止使用 `git push --force`、`git reset --hard` 等可能覆盖他人工作或丢失代码的命令。

## 2. 禁止上传的内容

以下内容必须由根目录 `.gitignore` 排除：

| 内容 | 路径或规则 | 原因 |
|---|---|---|
| STM32 下位机工程 | `STM32/` | 下位机工程单独维护，当前私有仓库不上传 |
| SLAM 运行日志 | `SLAM_Log/` | 数据量大，包含大量测试过程文件 |
| 普通日志文件 | `*.log`、`log/`、`log_*/` | 运行时生成，不属于源码 |
| ROS2 编译产物 | `build/`、`build_*/`、`install/`、`install_*/` | 可重新编译生成 |
| Python 缓存 | `__pycache__/`、`*.py[cod]`、`.pytest_cache/` | 运行时生成 |
| 网页依赖与构建产物 | `web/node_modules/`、`web/dist/` | 可通过依赖安装和构建重新生成 |
| Cartographer 状态文件 | `*.pbstream` | 体积可能较大，由 Log 版运行时生成 |

以下敏感信息无论是否被 `.gitignore` 覆盖，都禁止提交：

- GitHub Token、密码、SSH 私钥、API Key、Cookie、个人访问令牌。
- 串口设备的个人权限配置、局域网账号密码和远程登录凭据。
- 包含个人隐私、现场敏感信息或未经授权的第三方源码和 SDK。

如果敏感信息已经进入提交，禁止只删除文件后继续推送；应立即停止上传、撤销或轮换对应凭据，并检查 Git 历史。

## 3. 允许上传的内容

- ROS2 Jazzy 功能包源码、launch、config、URDF/TF 配置和行为树文件。
- 视觉避障、串口通信、Nav2、Cartographer、网页控制台和自动建图源码。
- `open_all.sh`、`open_all_log.sh`、验证脚本及项目使用文档。
- 对复现实车问题确有必要且体积合理的示例地图；历史测试地图上传前应人工确认。
- 经过许可、具备再分发权利的第三方源码或补丁。

单个文件超过 20 MB 时必须先确认是否真的需要上传；大型日志、视频、数据集和完整运行记录不得直接放入 Git 仓库。

## 4. 提交前检查

在项目根目录执行：

```bash
git status --short
git diff
git check-ignore -v STM32 SLAM_Log
```

提交前必须确认：

1. `STM32/` 和 `SLAM_Log/` 显示为被忽略，且没有出现在待提交列表中。
2. 没有 Token、密码、私钥、调试视频、临时截图和大体积日志。
3. 修改没有误碰已定版参数和已通过测试的一键启动链。
4. Python、Shell、YAML、Lua、C++ 和网页文件仍使用项目约定的 UTF-8 编码及行尾格式。
5. 能运行的检查已经执行；无法实车验证时必须在提交说明中明确标注“未实车验证”。

## 5. 标准上传流程

### 5.1 同步远端

多台电脑共同维护时，修改前先执行：

```bash
git switch main
git pull --rebase origin main
```

如出现冲突，必须逐个理解并解决，不得用强制推送覆盖远端。

### 5.2 暂存指定文件

优先明确指定本次修改的文件：

```bash
git add -- 路径/文件1 路径/文件2
git diff --cached
```

只有在确认整个工作区都属于同一次修改时，才允许使用 `git add -A`。暂存后必须执行 `git diff --cached` 复查实际上传内容。

### 5.3 提交和推送

```bash
git commit -m "模块: 简要说明本次修改"
git push origin main
```

推荐提交信息示例：

```text
nav2: 调整四轮差速车局部规划参数
web: 修复地图选点导航状态同步
serial: 适配AA55上行帧解析
docs: 更新项目使用和排查说明
```

一个提交只处理一个明确主题。禁止使用“改一下”“最新版”“测试”等无法追踪内容的提交信息。

## 6. 推送后检查

```bash
git status --short --branch
git rev-parse HEAD
git ls-remote --heads origin main
```

应满足：

- 本地显示 `main...origin/main`，且没有未提交文件。
- 本地 `HEAD` 与远端 `refs/heads/main` 的提交哈希一致。
- GitHub 页面仍显示 `Private`。
- GitHub 文件列表中不存在 `STM32/`、`SLAM_Log/`、密钥或运行日志。

## 7. 版本与回退规定

1. 实车测试前建议为重要稳定版本创建带说明的标签，例如：

   ```bash
   git tag -a v1.0-stable -m "实车验证稳定版"
   git push origin v1.0-stable
   ```

2. 已推送提交需要撤销时，优先使用 `git revert <提交哈希>` 生成反向提交。
3. 不得删除或重写已共享的提交历史。
4. 重大参数实验建议新建分支，验证通过后再合并到 `main`：

   ```bash
   git switch -c test/参数实验名称
   ```

## 8. 特别注意

- `.gitignore` 只会忽略尚未被 Git 跟踪的文件。若禁止上传的文件曾被跟踪，应先使用 `git rm --cached` 从索引移除，再提交修正。
- GitHub 是代码版本库，不代替 SLAM 测试数据归档盘。完整日志、PBStream、视频和传感器原始数据应保存在本地测试目录或专用存储中。
- 更新通信协议、车体尺寸、轮速符号、TF 方向或串口帧格式时，必须同步更新根目录 `ReadMe.md` 和 `修改.md`。
- 推送前发现不确定文件时，先保留在本地并确认，不要抱着“先传上去再说”的想法。
