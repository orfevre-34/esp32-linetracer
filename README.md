# linetracer

ESP32ベースのライントレースカー。ESP-IDF v5.2で動作。

## 機能

- 6チャンネルセンサーによるライン検出
- PD制御による差動駆動
- 直線加速・急旋回制御
- SSD1306 OLEDディスプレイ表示
- NVSによるキャリブレーション値の保存
- ブザー・RGB LED による状態通知

## ビルド

```bash
. $IDF_PATH/export.sh
idf.py build
idf.py flash monitor
```

ESP-IDF v5.2 のインストール先を `IDF_PATH` 環境変数に設定してください。

## 設定

`idf.py menuconfig` → "Line Tracer Configuration" からピン配置・PD制御パラメータ等を変更可能。

## License

MIT
