# 上传到 GitHub

## 方式一：普通 Git（推荐）

先在 GitHub 网站创建一个名为 `robot-web-console` 的 **Private** 空仓库，不要勾选自动创建 README、`.gitignore` 或 License。

在解压后的项目目录打开 PowerShell：

```powershell
git init
git config user.name "Chenyvnuo1"
git config user.email "chenyvnuo@Gmail.com"
git add .
git commit -m "Initial robot web console frontend"
git branch -M main
git remote add origin https://github.com/<你的GitHub用户名>/robot-web-console.git
git push -u origin main
```

首次 `git push` 时，Git Credential Manager 通常会打开浏览器要求登录 GitHub。

## 方式二：GitHub 网页

1. 打开 `https://github.com/new`；
2. Repository name 填 `robot-web-console`；
3. 选择 **Private**；
4. 创建空仓库；
5. 点击 **uploading an existing file**；
6. 将解压目录中的文件和文件夹拖到上传区域；
7. Commit message 填 `Initial robot web console frontend`；
8. 点击 **Commit changes**。

## 上传前复核

不应上传：

- `node_modules/`
- `dist/`
- `.vite/`
- `.env*`
- 视频文件
- 日志文件
- Ubuntu/ROS2 仿真工作区
- 真实机器人 IP
- 密码、Token、API Key 或私钥
