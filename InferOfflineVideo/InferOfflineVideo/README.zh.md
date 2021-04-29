中文|[英文](README.md)
# InferOfflineVideo

## 介绍

本开发样例演示 `InferOfflineVideo` 程序，使用芯片进行 `YoloV3` 对视频的目标识别。

该Sample的处理流程为：

```
StreamPuller > VideoDecoder > ObjectDetection > PostProcess > WriteResult
```

## 支持的产品

Atlas 800 (Model 3000), Atlas 800 (Model 3010), Atlas 300 (Model 3010), Atlas 500 (Model 3010)

## 支持的ACL版本

1.73.5.1.B050, 1.73.5.2.B050, 1.75.T11.0.B116, 20.1.0

查询ACL版本号的方法是，在Atlas产品环境下，运行以下命令：
```bash
npu-smi info
```

## 依赖条件

支持YoloV3目标检测模型，示例模型请参考[模型转换说明](data/models/README.zh.md)获取并转换

代码依赖：

版本包中各个Sample都依赖ascendbase目录

编译时如果不是整包拷贝，请确保ascendbase和InferOfflineVideo目录都拷贝到了编译环境的同一路径下，否则会编译失败；如果是整包拷贝，不需要关注。

设置环境变量：
*  `ASCEND_HOME`      Ascend安装的路径，一般为 `/usr/local/Ascend`
*  `LD_LIBRARY_PATH`  指定Sample程序运行时依赖的动态库查找路径

```bash
export ASCEND_HOME=/usr/local/Ascend
export LD_LIBRARY_PATH=${ASCEND_HOME}/ascend-toolkit/latest/acllib/lib64:$LD_LIBRARY_PATH
```

#### FFmpeg 4.2

源码下载地址：https://github.com/FFmpeg/FFmpeg/releases

FFmpeg源码编译和安装配置的方法，可以参考Ascend开发者论坛：https://bbs.huaweicloud.com/forum/thread-25834-1-1.html

如需交叉编译时，则进入到源码包目录下，执行以下命令完成交叉编译，请在--prefix选项自行指定安装路径

```bash
export PATH=${PATH}:${ASCEND_HOME}/ascend-toolkit/latest/toolkit/toolchain/hcc/bin
./configure --prefix=/your/specify/path --target-os=linux --arch=aarch64 --enable-cross-compile --cross-prefix=aarch64-target-linux-gnu- --enable-shared --disable-doc --disable-vaapi --disable-libxcb --disable-libxcb-shm --disable-libxcb-xfixes --disable-libxcb-shape --disable-asm
make -j
make install -j
```

配置FFmpeg4.2库，设置FFmpeg环境变量，如FFmpeg安装路径为/usr/local/ffmpeg。如使用交叉编译，则指定为交叉编译时ffmpeg安装路径
```bash
export FFMPEG_PATH=/usr/local/ffmpeg
export LD_LIBRARY_PATH=$FFMPEG_PATH/lib:$LD_LIBRARY_PATH
```

## 配置

在 `data/config/setup.config` 配置芯片id，模型相关信息，视频流路数和视频流地址等

修改芯片id
```bash
#chip config
SystemConfig.deviceId = 0 #use the device to run the program
```

配置视频流路数和视频流地址

修改视频流路数
```bash
SystemConfig.channelCount = 1
```
修改视频流地址
```bash
#stream url, the number is SystemConfig.channelCount
stream.ch0 = rtsp://xxx.xxx.xxx.xxx:1001/input.264
stream.ch1 = rtsp://xxx.xxx.xxx.xxx:1002/input.264
stream.ch2 = rtsp://xxx.xxx.xxx.xxx:1003/input.264
stream.ch3 = rtsp://xxx.xxx.xxx.xxx:1004/input.264
```

修改视频解码后的缩放分辨率
```bash
VideoDecoder.resizeWidth = 416    # must be equal to ModelInfer.modelWidth
VideoDecoder.resizeHeight = 416   # must be equal to ModelInfer.modelHeight
```

修改模型输入分辨率、模型名称、模型类型以及模型路径
```bash
#yolov3 model input width and height
ModelInfer.modelWidth = 416
ModelInfer.modelHeight = 416
ModelInfer.modelName = YoloV3
ModelInfer.modelType = 0 # 0: YoloV3 Caffe, 1: YoloV3 Tensorflow
ModelInfer.modelPath = ./data/models/yolov3/yolov3_416.om
```

配置跳帧间隔
```bash
skipInterval = 3 # One frame is selected for inference every <skipInterval> frames
```


## 编译

编译Atlas 800 (Model 3000)，Atlas 800 (Model 3010)，Atlas 300 (Model 3010)程序
```bash
bash build.sh
```

编译Atlas 500 (Model 3010)程序
在ARM服务器上编译A500程序时，执行如下命令
```bash
bash build.sh
```

在X86服务器上交叉编译A500程序时，执行如下命令
```bash
bash build.sh A500
```

如果需要将编译结果拷贝到其它环境上运行，拷贝dist目录和ffmpeg动态库即可

## 运行

查看帮助文档
```bash
cd dist
./main -h

------------------------------help information------------------------------
-acl_setup                    ./data/config/acl.json        the config file using for AscendCL init.
-debug_level                  1                             debug level:0-debug, 1-info, 2-warn, 3-error, 4-fatal, 5-off.
-h                            help                          show helps
-help                         help                          show helps
-setup                        ./data/config/setup.config    the config file using for face recognition pipeline
```

对视频流进行目标检测
```bash
cd dist
./main
```

## 约束
支持输入视频格式：H264或H265

## 结果

视频帧推理结果保存在result/result_x_xx_xxx.txt中（x为通道ID, xx为帧ID, xxx为文件生成时的时间戳）
每个result_x_xx_xxx.txt中保存了每帧图片的通道ID，帧ID，检测到的目标数目，每个目标的坐标框，置信度以及标签，格式如下：
```bash
[Channel0-Frame0] Object detected number is 7
#Obj0, box(315, 417.75, 536, 544)  confidence: 0.99707  lable: 17
#Obj1, box(87.5, 412.75, 351.5, 529.5)  confidence: 0.992188  lable: 17
#Obj2, box(1050, 441.5, 1279, 606.5)  confidence: 0.982422  lable: 17
#Obj3, box(535, 409.25, 698, 534)  confidence: 0.975586  lable: 17
#Obj4, box(957.5, 479.5, 1146, 598)  confidence: 0.905762  lable: 17
#Obj5, box(720, 445.75, 1000.5, 591.5)  confidence: 0.869141  lable: 17
#Obj6, box(0, 459.75, 84, 523)  confidence: 0.855469  lable: 17
```


