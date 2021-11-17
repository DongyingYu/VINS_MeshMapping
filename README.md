# system run
source devel/setup.bash
## run dataset
```bash
roslaunch start_process data.launch 
rosbag play /home/ipsg/dataset/Fast.bag  
rosbag play /home/ipsg/dataset/Normal.bag  
```

## run live
```bash
roslaunch start_process live.launch 
/***************************测试用***************************/ 
rosbag play  /media/ipsg/document2/dataset/101_room/101-1-2.bag 
rosbag play /media/ipsg/document2/dataset_ubuntu/outdoor.bag 
/******************************************************/ 

rosbag play  /home/ipsg/dataset/outdoor.bag 
rosbag play  /home/ipsg/dataset/outdoor1.bag 
rosbag play  /media/ipsg/document2/dataset/101_room/success/101-1.bag 
rosbag play  /media/ipsg/document2/dataset/101_room/success/101-1-2.bag 
rosbag play /media/ipsg/document2/dataset/campus_dataset/campus_7.bag 

# 去噪效果展示
sh Denoise
//roslaunch videoshow Denosie_Comparison.launch

#save map
rosservice call /voxgraph_mapper/save_combined_mesh "/home/ipsg/dataset/map_save/101_test.ply"
rosservice call /voxgraph_mapper/save_combined_mesh "/home/ipsg/1295/VINS_MeshMapping/outdoor_with_opti_1.ply"
rosservice call /voxgraph_mapper/save_combined_mesh "/home/ipsg/study/VINS_MeshMapping/v_room_test.ply"
# finish map
rosservice call /voxgraph_mapper/finish_map
```
## matters
```
每次添加都要catkin clean
重新catkin build


编译报错：minkindr_python下的相关文件报错；
解决办法：打开dependencies.rosinstall文件，将里面的两个git链接均下载下来，放在src文件夹下；

编译报错：几个文件的SSL无法连接的问题；
解决办法：将几个SSL链接的文件，单独下载下来，放入Thirdparty文件夹下；
注意：这里卸载CMakeLists.txt中的url路径均是绝对路径，当在不同的路径下编译时需要做改动。
此外，ceres_catkin eigen_catkin文件中均有需要下载git文件的选项，但是编译不影响，应该是系统中已经安装了对应软件的原因。在CMakeLists中有判断语句，首先会查找系统中是否安装了对应版本的库，再决定是否会执行后续下载安装的语句。
eigen_catkin对应的url下载已经做了修改，在Thirdparty中放入了一个对应的安装包。
ceres_catkin也下载了对应版本的Eigen(3.3.4)放入三方库中，留作备用。


做完以上修改时候成功编译；

*定位及重建系统*
功能实现：定位、三维稠密重建、PLY格式地图保存、语义检测、耐辐射去噪、位姿图及轨迹保存。

位姿保存报错：
pose graph path: /home/ipsg/dataset/pose_graph/
pose graph saving... 
[pose_graph-3] process has died [pid 4699, exit code -11, cmd /home/ipsg/1295/VINS_MeshMapping/devel/lib/pose_graph/pose_graph __name:=pose_graph __log:=/home/ipsg/.ros/log/b88a290a-5321-11eb-a34c-0242908cb298/pose_graph-3.log].
log file: /home/ipsg/.ros/log/b88a290a-5321-11eb-a34c-0242908cb298/pose_graph-3*.log

经修改，位姿可以成功保存，最后还是会报错，但是不影响，如下：
pose graph path: /home/ipsg/dataset/pose_graph/
pose graph saving... 
save pose graph time: 2.426240 s
save pose graph finish
you can set 'load_previous_pose_graph' to 1 in the config file to reuse it next time
program shutting down...
terminate called without an active exception
[pose_graph-4] process has died [pid 30673, exit code -6, cmd /home/ipsg/1295/VINS_MeshMapping/devel/lib/pose_graph/pose_graph __name:=pose_graph __log:=/home/ipsg/.ros/log/5d51c2b8-589f-11eb-b03f-244bfe4b549b/pose_graph-4.log].
log file: /home/ipsg/.ros/log/5d51c2b8-589f-11eb-b03f-244bfe4b549b/pose_graph-4*.log
```

## modify
```
<2021-11-15> : 
> 依赖项建立Thirdparty进行存储，编译成功不需要联网；
> 在estimate中发布的里程计位姿，修改为发布关键帧的位姿；
> 在pose_graph中修改了关键帧之间的距离阈(skip_dis)值判断值，在launch文件中修改即可；
> 改为在pose_graph中发布位姿，目前不成功，代码保留了，后续需要再看吧；
> 在keyframe的构造函数中，添加了深度图变量，在保存位姿图时，可以将RGB图对应的深度图一块进行保存，便于后续处理(后面要注意释放，这些数据极为占据内存);
> 在savePoseGraph()函数中做了相关修改，以保存所需格式的rgb depth及位姿数据；

<2021-11-16> : 
> 经过上述修改后，运行室内数据集一直会漂，之前运行没问题。。。 没问题，launch文件运行错误，运行data.launch来跑live....
> skip_dis对建图及优化的效果是有影响的skip_dis越小建图效果越细腻，优化作用也越明显；
> 修改keyframe()构造函数，因之前保存的位姿图存在一个问题，即保存的rgb格式地图为灰度图，这里单独传入一个rgb color进行后续的位姿图保存使用；
> 在对彩色图像的ros数据进行解码时，要解码为BGR8格式，保存下来的彩图显示才是正确的；

<2021-11-17> : 
> 在制作git仓库值，需要将.catkin_tools一并上传上去，因这包含了最初初始化工作空间时的一些信息，不然为提示奇怪的未定义的错误；
> 其余子文件夹中的.gitignore文件均进行删除了，不会有影响；
> github版本未将brief_k10L6.bin上传，在系统运行时要copy一份该文件放与support_files文件夹下；

```









