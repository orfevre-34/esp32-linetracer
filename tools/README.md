# Tools

スタンドアロンのテスト・デバッグ用プログラム。
それぞれ独自の `app_main()` を持つため、使用時は `main/` のソースと差し替えてビルドしてください。

## 使い方

例: センサーテストを実行する場合

```bash
cp tools/linetrace_sensor.cpp main/linetrace_car.cpp.bak
cp tools/linetrace_sensor.cpp main/linetrace_car.cpp
idf.py build && idf.py flash monitor
# 終わったら元に戻す
mv main/linetrace_car.cpp.bak main/linetrace_car.cpp
```

## ファイル一覧

- `linetrace_sensor.cpp` - 6ch ADCセンサーの生値をシリアル出力するテスト
- `linetrace_display.cpp` - OLEDディスプレイでセンサー値を可視化するテスト
