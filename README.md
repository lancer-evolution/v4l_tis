# V4L for ROS

"DFM22BUC03-ML" カメラをROSでパブリッシュするためのリポジトリ．

[https://github.com/lancer-evolution/v4l2_TIS](https://github.com/lancer-evolution/v4l2_TIS)では，ROSを使わずにUDP通信により映像を配信するソースコードをおいた．

## How to use

直接デバイスを開いてrosで配信
```bash
rosrun v4l_tis send
```

**topic**
```bash
/v4l_tis/image_raw
/v4l_tis/mono_raw
```

udp通信で取得したものをrosで配信
```bash
rosrun v4l_tis udp_reciever
```

## How to custom

調整可能項目

```cpp
init_device(width, height, exposure, gain=63, fps=60)
```

本リポジトリは"DFM22BUC03-ML"用に開発しているが，その他のカメラでも使いたい場合は以下の項目を設定しなおし、ソースコードの改変が必要かもしれない．

`send.cpp`内43，48行目辺りにある

```cpp
init_device(744, 480, 50);
src.create(480, 744, CV_8UC1);
```

を任意のカメラサイズに変更する．




また，カメラに露出設定がなければ`init_device`の第3引数は機能しないので，`v4ldevice.cpp`，`v4ldevice.h`のexposure設定を消したソースコードに変更が必要．
