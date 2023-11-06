# ROS小车项目

##  1.安装依赖

```bash
# joy功能包
sudo apt-get install ros-melodic-joy
# serial功能包
sudo apt-get install ros-melodic-serial

```



## 2.项目编译

从Github 上面clone该项目到自己的工作空间中

```bash
git clone https://github.com/Freaky1122/car_ros.git
```

在工作空间下编译该项目

```
catkin_make
```



## 3.使用方式

###### 3.1 先开启joy节点，开启手柄遥控

+ 现在只支持XBOX Series手柄，其余手柄对应通道可能不相同，可以自行进行修改

+ 左摇杆上下控制速度，左右控制角速度

  ```
  rosrun joy joy_node
  ```

###### 3.2 开启指令发送Publisher

+ 该节点负责接收手柄的控制信号，并将手柄信号转化成线速度和角速度指令，作为话题发布出去

+ 在后续使用时，需要根据指令来源更改该文件

  ```
  rosrun car_ros cmd_pub_node
  ```

###### 3.3 开启指令接收Subscriber

+ 该节点负责接收其他话题发送的角速度和速度指令，并通过串口发送出去

  ```
  rosrun car_ros cmd_sub_node
  ```

  

