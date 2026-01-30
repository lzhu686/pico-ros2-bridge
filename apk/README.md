# APK 安装与卸载命令

包名: `com.xrobotoolkit.client`

## 安装

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
