# vins 回环定位

## 备注
1. 打印坐标纸!! 1210
1. 

1. 再次看回环部分



## 记录

1. vins 本身支持地图保存和加载,显示节点是 /loop_fusion/base_path ,但是因为是直接保存为二进制格式保存和加载时间很长,大概9s,但是protobuf 只需不到1s.
1. 测试是可以定位使用,需要将相机标定好测试用
1. indemind相机有近延迟 -2.8ms,且不符合右手坐标系
## 存在问题

1. 多相机时无法回环
1. ~~保存定位地图~~
1. 运动时开启程序会飞
1. ~~相对位姿nan -> 加载的帧没有loop_info~~, 但是回环时也对其进行了优化和读取,属于读空指针

加载地图时设置loop_index=-1, 则`earliest_loop_index` 会等于新帧检测到回环的帧,且`first_looped_index=earliest_loop_index`将跳过前面部分

5. ~~飞太远时`KeyFrame::findConnection(KeyFrame* old_kf)`  ` loop_info` 全为nan~~
   1. 自己将MIN_LOOP_NUM 分段写了,pnp 没结果但是自己赋了个空的过去
6. ~~权重, 飞太远时仍然回回环到错误的地方~~
   1. `add_data_base: 0         `   #检测回环时是否将当前帧加入到数据库中
7. 将权重设置成100 ,用双目相机,飞到天边也能回来