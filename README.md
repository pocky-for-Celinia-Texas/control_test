主要完成了电机的pid调参，可以通过右拨杆切换电机模式，下拨杆失能，中拨杆电机可以跟随c板运动，并可以跟随另一个电机运动，上拨杆复位。同时，可以通过左拨杆调整两电机的跟随比例。

主要完成了up挡位的自动设置，加速和减速共用一个曲线，通过调参改变斜率和最大速度，开环控制，通过实际轮速算出行进距离，有误差,调参在chassis_spped.hpp中,取四个轮子的平均速度
自动档起步为了让起步顺滑,采用函数10u³ − 15u⁴ + 6u⁵,并让AI生成了smoothstep5()函数供调用,还提供了各项参数便于修改行驶距离,最大速度,中途最大加速度等
S曲线避免最后缓慢动,有死区锁车不动

主要完成了功率计的CAN通信连接,新建类PowerMeter实现接受功率计的数据,电流电压功率分别为power_meter.current, power_meter.voltage, power_meter.power

功率控制由参考资料 https://tj-superpower.feishu.cn/file/ZAOAbMTY4ozT27xI4YacfkX7nac 中的公式提供,建立预测模型,预测超过限制功率时减少力矩,但对四个电机是平均较少相同倍数,未实现 https://www.bilibili.com/video/BV1tu41177wm/?spm_id_from=333.337.search-card.all.click&vd_source=f8ad5b7ee019d4475262656281671b29 中提到的对每个轮子比例减少,同时加上功率反馈闭环,更加精确.



