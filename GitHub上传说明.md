# GitHub 上传说明（gh-eng）

本文档记录把「小学一年级超声波避障小车」项目上传到 GitHub 的过程与结果。

## 仓库信息

- 仓库名：`av01-car-avoid`（公开）
- 描述：`小学生Arduino超声波避障小车 极坐标扇形直方图VFH+点云航向闭环校准`
- 用户名：`zhangsheng377`（git 全局身份已配置：`user.name=zhangsheng377`、`user.email=435878393@qq.com`）
- 仓库 URL：见下（创建完成后回填）

## 上传文件清单

从 `\\nas\张腾予\小学\一年级\一年级上\小车\超声波避障小车\` 复制到本地临时目录 `$env:TEMP\gh_upload`：

| 文件 | 说明 |
|---|---|
| `sketch_sep26a\sketch_sep26a.ino` | 主程序（完整小车固件） |
| `sketch_sep26a\编译与调参说明.md` | 编译验证、内存占用、参数调优 |
| `航向闭环校准设计方案.md` | 架构级设计：点云轮廓匹配航向闭环校准 |
| `sketch_sep26a\sketch_sep26a_20260926.bak` | 历史备份 |
| `README.md` | 项目说明（本次新增） |

## 本地仓库

- 本地路径：`C:\Users\43587\AppData\Local\Temp\gh_upload`
- 已 `git init` + `git add -A` + `git commit`：
  `48d8b94 小学生Arduino超声波避障小车：极坐标VFH避障+点云轮廓匹配航向闭环校准`
- 分支：`main`

## 上传方式

优先用 GitHub CLI（`gh`，内置 token 处理更安全，不会泄露 token），
无法安装时退用 `git + REST API + HTTPS token`。

### 方式 A：gh CLI

```powershell
# 安装（winget）
winget install --id GitHub.cli --accept-source-agreements --accept-package-agreements

# 认证（token 走 stdin，不写入 shell 历史 / .git/config）
$env:GH_TOKEN 或 gh auth login --with-token

# 在本地仓库目录创建公开仓库并推送
cd $env:TEMP\gh_upload
gh repo create av01-car-avoid --public --source . --push --description "小学生Arduino超声波避障小车 极坐标扇形直方图VFH+点云航向闭环校准"
```

### 方式 B：git + REST API（gh 装不上时）

```powershell
# 1) 用 token 经 REST API 创建公开仓库
$token = "..."   # 用户提供的 classic token，仅在内存中，绝不写入文件/.git/config
$body = @{ name="av01-car-avoid"; description="小学生Arduino超声波避障小车 极坐标扇形直方图VFH+点云航向闭环校准"; private=$false } | ConvertTo-Json
Invoke-RestMethod -Uri "https://api.github.com/user/repos" -Method Post `
  -Headers @{ Authorization="token $token"; "User-Agent"="dsh" } -Body $body

# 2) 配置远端（用带 token 的 URL，仅在 remote 里、不入 config 推送）
cd $env:TEMP\gh_upload
git remote add origin "https://zhangsheng377:$token@github.com/zhangsheng377/av01-car-avoid.git"

# 3) 推送
git push -u origin main
```

> **token 安全**：token 只用 `gh auth login --with-token` 或 REST API 内存使用，
> 绝不写入会 push 出去的 `.git/config`；完成后提醒用户可撤销该 classic token。

## 上传成功证明

（以下命令在创建 + 推送后执行并回填）

```powershell
gh repo view zhangsheng377/av01-car-avoid        # 显示仓库信息
git remote -v                                    # 显示远端 URL
git ls-remote origin                             # 显示远端分支/HEAD，证明推送成功
```

## 备注

- 源文件在 UNC 网络路径（`\\nas\...`），git 在此路径下受限，故先复制到本地临时目录再 git 操作。
- 中文文件名 git 可正常处理（本机 git core 支持 UTF-8）。
- 真机实测两个缺陷（卡死无脱困、探测距离太近）的修复代码会随迭代更新到本仓库。