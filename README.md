# 项目说明

本项目基于 https://github.com/leo038/hand_eye_calibrate.git 制作；

手眼标定的原理可参考：[机械臂手眼标定方法详解](https://blog.csdn.net/leo0308/article/details/141498200)；

基于源项目的基础上分别编写了眼在手外与眼在手内的标定程序，并根据已有的UR5机械臂进行了专门配置，能够自动连接机械臂；

需要自行安装Intel D435的驱动，安装方法详见百度。

# 使用说明

根据需求选择 “hand_out”（眼在手外）或者 “hand_in”（眼在手内）文件夹，并先运行“data_collect.py”进行图像采集与机器人位姿保存；

获得图像与机器人位姿后，运行“hand_eye_out(in)calibrate.py”进行手眼标定；

手眼标定数据会自动保存于各自的measured_data文件夹中，自行取用；

手眼标定结束后运行各文件夹中的record_realsense.py获取相机深度数据，所获取的深度数据也会自动保存于各自的measured_data文件夹中;

在手眼标定结束后可进入Acc_test文件夹，修改main.py中的各项数据，运行main.py进行手眼标定结果的验证；此验证仅适用于UR5与D435相机结合的项目，如果需要测试自己的项目，请自行替换相机文件（realsenseD435.py）与机器人文件（UR_Robot.py）。
注意：该 UR_Robot.py 与 UR5_control 文件夹中的控制程序不一样！

# Tips

collect_data_in和collect_data_out文件夹中都有示例图片，可以直接运行理解代码。

# 联系方式

如有疑问请联系wangjie03690@163.com


