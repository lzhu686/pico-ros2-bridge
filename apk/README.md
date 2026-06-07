# APK 安装与卸载命令

包名: `com.xrobotoolkit.client`

## 最新版本 v1.4

`v1.4` 是本仓库中的 APK 文件迭代版本。当前 APK 的 Android Manifest 仍显示 `versionName=1.0.0`、`versionCode=1`，请优先以文件名和本文档说明区分版本。

使用本仓库文件安装：
```bash
# 最新版本
adb install -r -g v1.4.apk

# 如需回退旧版 local 坐标系版本
adb install -r -g v1.3local.apk
```

## 双目 RGB 视觉参考方案

本仓库提供的 APK 在官方 XRoboToolkit Client 的数据获取能力基础上，额外整理了一个面向低成本双目 RGB 立体视觉的参考配置。相较于常见的官方/高端双目方案（例如 ZED 等深度相机），这个方案尝试使用更便宜的 USB 双目 RGB 相机完成 VR 端立体视觉显示。`v1.4.apk` 内置的 `assets/video_source.yml` 包含：

- USB 双目 RGB 相机示例配置：2560x720@60fps
- ADB 有线模式：约 23Mbps，适合低延迟有线连接
- WiFi 无线模式：约 11Mbps，适合无线遥操作测试
- 16:9 双目画面显示参数：`contentRatio` 约 `2.276`，用于适配双目画面显示范围
- 双目相机参考参数：`stereoOffset=0.0`

这个方案主要是为了降低双目遥操作的硬件成本，仅供参考。如果你已经有官方推荐或更高端的 ZED 等深度/双目相机方案，也可以继续使用官方方案做数据获取。

## English Notes

`v1.4` is the APK file revision used in this repository. The Android Manifest still reports `versionName=1.0.0` and `versionCode=1`, so use the file name and this README to distinguish the packaged versions.

The APK keeps the official XRoboToolkit Client data-access workflow and adds a reference configuration for a lower-cost stereo RGB setup. Compared with common official/high-end stereo setups such as ZED depth cameras, this reference path targets a cheaper USB stereo RGB camera for VR-side stereo display. The bundled `assets/video_source.yml` includes a 2560x720@60fps USB stereo RGB camera example, ADB/WiFi bitrate presets, 16:9 stereo rendering parameters, and `stereoOffset=0.0` for a stereo camera. ZED or other official/high-end depth/stereo camera setups can still be used if available.

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
