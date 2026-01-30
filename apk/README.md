# APK 安装与卸载命令

包名: `com.xrobotoolkit.client`

## 最新版本 v1.3.0

从 GitHub Releases 下载（推荐）:
```bash
# 全局坐标系版本（推荐）
wget https://github.com/lzhu686/XRoboToolkit-Unity-Client/releases/download/v1.3.0/XRoboToolkit-v1.3.0-global.apk
adb install -r -g XRoboToolkit-v1.3.0-global.apk

# 本地坐标系版本
wget https://github.com/lzhu686/XRoboToolkit-Unity-Client/releases/download/v1.3.0/XRoboToolkit-v1.3.0-local.apk
adb install -r -g XRoboToolkit-v1.3.0-local.apk
```

或使用本地文件:
```bash
adb install -r -g v1.3global.apk
```

## 卸载

```bash
adb uninstall com.xrobotoolkit.client
```

## 清除应用数据（不卸载）

```bash
adb shell pm clear com.xrobotoolkit.client
```
