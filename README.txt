该项目利用BooStar UWB模块实现对无人机（ArduPilot copter）的室内定位功能。

编译说明：
依赖库：boost libconfig

mkdir build
cd build 
cmake ..
make

使用说明：
1）硬件搭建
1.将锚点按直角放置。
2.测量直角边距离。
3.直角边为局部坐标系的参考轴。
4.测量搭建局部坐标系下ox与NED坐标系下ox轴夹角。

2）启动UWB系统
1.系统上电会自动启动uwb定位程序；
2.需根据实际搭建位置修改启动配置文件；
3.支持web参数设置；


