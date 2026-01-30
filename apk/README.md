# APK 安装与卸载命令

包名: `com.xrobotoolkit.client`

## 最新版本 v1.3.0

从 GitHub Releases 下载（推荐 local 本地坐标系 - 更稳定）:
```bash
# 本地坐标系版本（推荐 - 更稳定可靠）⭐
wget https://github.com/lzhu686/XRoboToolkit-Unity-Client/releases/download/v1.3.0/XRoboToolkit-v1.3.0-local.apk
adb install -r -g XRoboToolkit-v1.3.0-local.apk

# 全局坐标系版本（仅多设备空间对齐场景）
wget https://github.com/lzhu686/XRoboToolkit-Unity-Client/releases/download/v1.3.0/XRoboToolkit-v1.3.0-global.apk
adb install -r -g XRoboToolkit-v1.3.0-global.apk
```

或使用本地文件:
```bash
# 推荐使用 local 版本
adb install -r -g v1.3local.apk
```

**坐标系模式说明：**
- **Local（本地坐标系）**：⭐ 更稳定，推荐日常使用
- **Global（全局坐标系）**：可能不稳定，仅特殊场景使用

## 卸载

```bash
adb uninstall com.xrobotoolkit.client
```

## 清除应用数据（不卸载）

```bash
adb shell pm clear com.xrobotoolkit.client
```
