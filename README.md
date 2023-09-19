
# 多相机内外参联合标定

该项目主要实现针孔相机的内外参数联合标定功能。


## 安装

1. 安装 ros系统 （采集数据，若已完成数据采集跳过该步骤）
参考链接鱼香ros直接安装
```bash
https://fishros.org.cn/forum/topic/20/%E5%B0%8F%E9%B1%BC%E7%9A%84%E4%B8%80%E9%94%AE%E5%AE%89%E8%A3%85%E7%B3%BB%E5%88%97?lang=zh-CN
```
2. 依赖的opencv可以通过步骤1（ros安装）直接完成。(opencv4.0)

    

## 运行
1. 进行标定数据采集，创建ros工作空间，将record_data目录下的laser_rgb_data功能包拷贝至工作空间。

```bash
  mkdir -p catkin_ws/src
  sudo cp -r multi_cam_rgb_data  catkin_ws/src
  cd catkin_ws
  catkin_make 
```

2. 修改代码中的相机图像话题名称，按q键进行标定数据保存，后续标定代码要求图像保存为.png文件，两个相机保存的图像文件名均相同。保存标定文件至少15对，一般20对左右，存放在image文件夹中的camera_1以及camera_2文件夹。
```bash
rosrun  multi_cam_rgb_data multi_cam_rgb_data_node
```

3. 进行相机内参以及外参联合标定。进入项目目录
```bash
mkdir build && cd build
cmake ..
make 
```
4.1 修改config/setting_config.yaml中的路径参数以及标定板的尺寸参数。
```bash
./MuiltiCamCalibration
```