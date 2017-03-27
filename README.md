

# 编译说明


##安装

1. 首先安装Qt5, 默认安装路径在Ubuntu上一般是`/opt/Qt5.7.1`
2. 在环境变量里, `.bashrc`的最后一行加上
    `export QT5_INSTALL_PATH=/opt/Qt5.7.1/5.7/gcc_64`
3. 如果需要支持Kinect2相机，那么需要编译安装freenect2


# 文件目录结构

```
+ cmake                             cmake文件自定义的查找依赖。比如Eigen
+ data                              数据
+ packages                          主要的代码
    + cobotsys                      核心API，基础代码库，只依赖Qt,OpenCV, Boost
        + framework                 所有的抽象代码AbstractXXX
        + include                   其他的基础头文件
        + src                       实现文件
          CMakeLists.txt            cobotsys工程CMake文件
    + cobotsys_ros_wrapper          对ROS的一个简易封装。如果有依赖ROS的部分，依赖这个库就好。
    + cobotsys_ros_extend           基于ROS的应用动态库。ROS有个BUG，不支持dlopen dlclose.程序退出的时候会崩溃
    + plugin_library                扩展支持插件，对cobotsys里各种抽象对象的具体实现
        + Kinect2CameraAdapter      Kinect2相机的适配驱动
        + QtBasedSimpleController   Kinect2相机的简易UI
        + UrRobotDriver             UR机器人适配驱动
        + ...                       如果有，其它
    + xApplications                 实际的应用程序
        + binpicking2               老版本的无序分拣的UI实现
+ samples                           测试例子
+ test_apps                         测试小程序
  CMakeLists.txt                    工程主要的CMakeLists.txt
  README.md                         当前文件
```


# 运行测试程序

在代码库时里， 已经默认提供了一些框架运行程序的默认方法

其中比较有效的一个是`test_std_json_widget`这个程序

对于CLion的IDE，在Build ALL里配置启动程序为

`test_std_json_widget` 程序参数可以是

`data/test_ui_json`目录下的任意一下JSON文件。

程序会根据JSON的描述，来创建共享库里的Widget应用。
