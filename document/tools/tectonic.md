可以配置。我实际测试了几类“镜像”，结论如下：

## 1. 官方安装脚本失败的真实原因（不是墙的问题）

`drop-sh` 脚本里架构是硬编码的 `aarch64-unknown-linux-gnu`，而 tectonic 0.17.0 **没有 gnu 版 aarch64 资源**（只有 `aarch64-unknown-linux-musl`），所以 GitHub 返回 404。已改为下载 musl 版并装好：

- `~/.local/bin/tectonic` → `Tectonic 0.17.0`（静态链接 aarch64，已验证可用）

## 2. 中文镜像站：没有

实测结果（直连，未挂代理）：

| 源 | 结果 |
|---|---|
| mirrors.tuna/ustc/sjtu/aliyun 的 `/tectonic/` | 403 / 404，**不存在该镜像** |
| `relay.fullyjustified.net/default_bundle_v33.tar` | 302 → 中转不稳，之前报 TLS `close_notify` |
| `data1b.fullyjustified.net/tlextras-2022.0r0.tar` | ✅ 200，Cloudflare，直连 0.4–1.9 MB/s |

即：res 包（bundle）没有国内镜像，但 **relay 只是个跳转，直连最终 CDN 即可**，这就是最实用的“镜像”。

## 3. 已配置的 bundle 镜像（`~/.config/tectonic/config.toml`）

```toml
[[default_bundles]]
url = "https://data1b.fullyjustified.net/tlextras-2022.0r0.tar"
```

验证有效：
- 指向错误 URL 时 tectonic 直接 panic（证明配置被读取）；
- 清除所有 `*_proxy` 变量后编译 amsmath/tikz/pgfplots/xcolor 文档，120+ 文件正常按需下载，**完全不需要代理**。

⚠️ 必须写成 `[[default_bundles]]`（复数、数组表），写成 `[default_bundle]` 会报 `missing field default_bundles`。

## 4. 二进制更新走 GitHub 加速（`~/.local/bin/tectonic-update`）

回退顺序：`gh-proxy.com` → `ghfast.top` → 官方直连 → 代理 7890 → 代理 7899；并自动按架构选对资源名：

```bash
tectonic-update              # 装最新版
tectonic-update tectonic@0.17.0   # 指定版本
```

实测 gh-proxy 2.4 MB/s，比直连 GitHub（1.5 MB/s）和 7890 代理快，已用它重新验证安装成功。

## 5. 两个注意点

- **全量离线不可行**：bundle 是 2.88 GB 的 `tlextras-2022.0r0.tar`，而 `/` 只剩 1.5 G（95% 占用）。所以只能按需增量下载。
- 缓存位置 `~/.cache/tectonic`（当前 45 M），可用 `TECTONIC_CACHE_DIR=...` 迁移；`tectonic -C` 或 `--only-cached` 可强制离线编译已缓存的包。
- 中文乱码是另一回事（缺 `ctex`/`fontspec` + 字体设置），需要的话我可以按项目要求配 SimSun。