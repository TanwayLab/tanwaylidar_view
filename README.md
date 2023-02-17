# tanwaylidar_view
tanwaylidar_view 是探维科技针对所产雷达系列产品的上位机软件，在Ubuntu18.04环境下开发测试通过。

软件需在ROS环境下使用，ROS安装参见[安装教程](http://wiki.ros.org/ROS/Installation "")。

# 概览

[软件下载与编译](#软件下载与编译)

[点云显示软件使用](#点云显示软件使用)

[IP修改工具使用](#IP修改工具使用)


# 软件下载与编译

1. 打开终端（快捷键：ctrl+alt+T）

1. 创建ROS工作空间

```bash
mkdir -p ~/tanwaylidar_driver/src
cd ~/tanwaylidar_driver/src
```

1. 下载代码

```bash
git clone https://github.com/tanwayLab/tanwaylidar_view.git
```

输入用户名密码，下载成功后，～/tanwaylidar_driver/src文件夹下就会出现程序包。此步骤也可直接在github的项目下直接下载程序的zip压缩包，然后解压到/tanwaylidar_driver/src文件夹下。

1. 编译程序

```bash
cd ~/tanwaylidar_driver && catkin_make
```

1. 设置环境变量

```bash
echo "source ~/tanwaylidar_driver/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

# 点云显示软件使用

1. 修改电脑IP为与雷达通信的IP，默认为"192.168.111.204"

1. 运行程序，正常查看点云 

```bash
【以TensorPro设备为例】roslaunch tanwaylidar_view TensorPro.launch
【以Scope设备为例】roslaunch tanwaylidar_view Scope.launch
【以TSP03-32设备为例】roslaunch tanwaylidar_view TSP03-32.launch
【以Scope-192设备为例】roslaunch tanwaylidar_view Scope-192.launch
【以ScopeMiniA2-192设备为例】roslaunch tanwaylidar_view ScopeMiniA2-192.launch
【以Duetto设备为例】roslaunch tanwaylidar_view Duetto.launch
```

![](./resource/pic/example.png "")


# IP修改工具使用(仅支持TensorPro)

```bash
rosrun tanwaylidar_view tensorpro_interfaces
```

弹出用户交互界面如下,

![](./resource/pic/user_interfaces.png "")

### 参数说明
输入Mac地址、IP地址和端口信息应符合以下规则，否则将会出错。

- IP地址范围为0.0.0.0至255.255.255.255

- 端口的范围为0至65535,建议端口号设置为大于1024的值

- Mac地址范围为00-00-00-00-00-00至FF-FF-FF-FF-FF-FF

- 上位机与激光雷达IP地址应处于同一网段内

- 激光雷达设备出厂时，默认IP和端口号设计如下：

    激光雷达IP：192.168.111.51，激光雷达端口号：5050

    上位机IP：192.168.111.204，上位机端口号：5600

### 验证连接

- 激光雷达供电工作，利用网线将上位机与激光雷达连接，并将上位机IP地址设置为激光雷达数据发送的目的地址。

- 在“验证与雷达的通信”区域输入IP和端口信息（默认值为雷达出厂网络参数），点击“测试雷达连接”。当连接正常且信息输入正确时，会出现如下图所示的状态栏显示，提示源数据来源和数据长度。

![](./resource/pic/connection.png "")

### 修改网络参数

- 激光雷达供电工作，利用网线将上位机与激光雷达连接，并将上位机IP地址设置为激光雷达数据发送的原目的地址。

- 在“验证与雷达的通信”区域输入上位机IP和端口信息，此IP需与上位机IP一致，否则无法发送修改指令。

- 在“修改雷达和接受端信息”区域填写激光雷达的设备Mac信息（一般可从雷达机身的铭牌信息后获得或使用Wireshark抓包工具获取）、新设IP地址和端口信息，点击“确认修改”（因不同操作系统存在权限差异，可能会自动弹出终端要求用户输入密码获取权限，请及时输入用户密码以确保操作成功进行）。

- 当连接正常且信息输入正确时，会出现如下图所示的状态栏显示，提示"【成功】雷达修改操作成功，请重新启动雷达以应用最新配置！"。

- 断开激光雷达电源，重新供电后新配置生效。

- 使用本软件的验证功能或Wireshark等网络调试助手，确认激光雷达IP信息修改生效。

![](./resource/pic/SetIP.png "")

## 历史软件版本描述

| 版本号             | 时间          | 描述     |
| ---------------------| ----------------- | ---------- |
| tanwaylidar_view v1.0.0 | 2021年01月20日 | 增加对Scope设备的支持，用于测试使用 |
| tanwaylidar_view v1.0.1 | 2021年02月01日 | 通道号翻转修改（仅Scope设备）<br />运行参数提示信息错误<br />ReadMe描述错误<br />其他已知BUG |
| tanwaylidar_view v1.0.2 | 2021年03月20日 | 增加Tensor-Pro产品下的GPS协议数据解析功能<br />更新IP修改工具，增加更多的修改反馈提示信息 |
| tanwaylidar_view v1.0.3 | 2021年05月14日 | Scope设备水平角度对齐 |
| tanwaylidar_view v1.0.4 | 2021年06月04日 | IP修改工具可能会失败的BUG |
| tanwaylidar_view v2.0.0 | 2021年06月09日 | 修改view项目使用跨平台基层SDK |
| tanwaylidar_view v2.0.1 | 2021年08月04日 | IP修改工具的易用性输入记录 |
| tanwaylidar_view v2.0.2 | 2022年03月16日 | 增加对TSP03-32、Scope-192设备的支持 |
| tanwaylidar_view v2.0.3 | 2022年03月22日 | 修改TSP03-32的launch文件加载错误bug |
| tanwaylidar_view v2.0.4 | 2022年03月26日 | 修改Scope-192默认组修正角度值，更新跨平台SDK版本到1.0.4 |
| tanwaylidar_view v2.0.5 | 2022年04月20日 | 更新sdk版本，增加对Duetto设备的支持，更新跨平台SDK版本到1.0.5 |
| tanwaylidar_view v2.0.6 | 2022年06月23日 | 更新跨平台SDK版本到1.0.6 |
| tanwaylidar_view v2.0.7 | 2022年09月20日 | 更新跨平台SDK版本到1.0.7，增加ScopeMiniA2-192设备型号的支持 |
| tanwaylidar_view v2.0.8 | 2022年10月15日 | 更新Duetto的launch文件支持标定参数输入；<br />修改ScopeMiniA2-192设备默认的组修正角度值； |
| tanwaylidar_view v2.0.9 | 2022年10月18日 | 更新跨平台SDK版本到1.0.9 |
| tanwaylidar_view v2.0.10 | 2022年10月22日 | 更新跨平台SDK版本到1.0.10 |
| tanwaylidar_view v2.0.11 | 2022年10月24日 | 更新跨平台SDK版本到1.0.11，并扩展点云数据中点的属性信息：<br />通道、水平角度、块标志、时间信息等 |
| tanwaylidar_view v2.0.12 | 2022年11月16日 | 更新跨平台SDK版本到1.0.12；修改Scope-192.launch文件中默认组修正角度值为0、-0.12、-0.24 |
| tanwaylidar_view v2.0.13 | 2022年11月21日 | 更新跨平台SDK版本到1.0.13；修改ROS发布点云时间值取跨平台SDK中点云帧的时间值； |
| tanwaylidar_view v2.0.14 | 2022年11月28日 | 更新跨平台SDK版本到1.0.14； |
| tanwaylidar_view v2.1.0 | 2022年12月06日 | 更新跨平台SDK版本到1.1.0；取消Duetto.launch文件中对标定参数的配置，由SDK解析获取； |
| tanwaylidar_view v2.1.1 | 2022年12月29日 | 更新跨平台SDK版本到1.1.1；增加对IMU数据的接收并推送ROS消息； |
| tanwaylidar_view v2.1.2 | 2023年01月05日 | 修改设备launch文件配置，解决配置参数不生效问题； |
| tanwaylidar_view v2.1.3 | 2023年01月10日 | 更新跨平台SDK版本到1.1.2；修改Duetto设备型号的launch文件配置，增加K、B值设置参数； |
| tanwaylidar_view v2.1.4 | 2023年01月10日 | 更新跨平台SDK版本到1.1.3； |
| tanwaylidar_view v2.1.4.0 | 2023年01月13日 | Duetto设备型号解析时x、y轴对调； |
| tanwaylidar_view v2.1.4.0.1 | 2023年01月14日 | Duetto设备型号修改坐标系方向：X轴向前，Y轴向左，Z轴向上 |
| tanwaylidar_view v2.1.5.0 | 2023年01月30日 | 更新tanwaylidar_view版本到v2.1.5； |
| tanwaylidar_view v2.1.5.0.1 | 2023年02月17日 | 去除Duetto设备解析时的K、B值接口设置，使用原始值计算脉宽数据 |
