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

* width
* height
* exposure
* gain=63
* fps=60

これらを`launch`ファイルから設定する．

なお，本リポジトリは"DFM22BUC03-ML"用に開発しているため，その他のカメラでも使いたい場合は、`init_device`関数の引数はやコードを変更する必要があるかもしれない．`v4ldevice.cpp`，`v4ldevice.h`
