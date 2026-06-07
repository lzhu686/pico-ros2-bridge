# APK 安装与卸载命令

包名: `com.xrobotoolkit.client`

## 最新版本 v1.4

使用本仓库文件安装：
```bash
# 最新版本
adb install -r -g v1.4.apk

# 如需回退旧版 local 坐标系版本
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
