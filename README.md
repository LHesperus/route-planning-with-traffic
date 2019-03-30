# route planning with traffic
## 队伍：道路千万条请走这一条

# 整体思想：<br>
## 快车上高速，慢车走短路; <br>
## 快慢分批走，谁慢就等谁；<br>
## 道多优先进，车多请绕行；<br>

比赛做的比较晚，做的匆匆忙忙，规则都没来得及看全，28号开始写Dijkstra算法，写完拿1w car的地图测试，
中途不会上传文件，耽搁了一些时间，后来第一次好使两张图跑了2200，成渝赛区排到58名
![Image text](https://github.com/LHesperus/route-planning-with-traffic/blob/master/message/2.png)

[版本1.0](https://github.com/LHesperus/route-planning-with-traffic/commit/971be2f9336369866540bb0331fcb1078f475004#diff-73f6bbf6d297a038d2f7f5071544c23f)
中，定义了基本的Road、Car和Cross类，写了寻路径最短的 Dijkstra 算法，并生成了路径文件。可以算一个最初版本，此版本之前遇到多次段溢出情况，当时 Dijkstra 算法
写的有问题，后来严格按照算法导论书上的算法的流程伪码写的，就运行正常了。
但此版本最初不能在网上运行，是因为没有按照官方的接口写路径，另外也会死锁，因为我没有分散出发时间。

只改plantime不太有效，下一个思路就考虑了死锁情况，大致为：
死锁附近附近一般有较多的车，这样整个地图的车辆分布就很不均匀，如果能给车辆少的路多分配一些车辆，可能就会减缓死锁。
所以在[版本2.0](https://github.com/LHesperus/route-planning-with-traffic/commit/046aaa43c2c81f3a85b359d4d95b9e1cf23d9e0f#diff-73f6bbf6d297a038d2f7f5071544c23f)
中使每条路的总经过车数大概相等，
并且改变了 Dijkstra 算法中每条边的权重计算方法，
path_weight=f(t_path,channel，speed_diff,W_block)<br>
t_path是路的长度 除以 车和路 允许的最小速度，即时间权重，此值大则建议换路 <br>
channel是路的通道数，通道数多，建议进入 <br>
speed_diff 是车速和路速度的差值，即差值越大，车速与路速越不匹配，建议换路 <br>
W_block 是该路的拥堵情况，较多车经过该路时，拥堵系数角度大，建议换路 <br>
测试后 两张图跑了1206，成渝赛区排名34名
![Image text](https://github.com/LHesperus/route-planning-with-traffic/blob/master/message/3.png)
后来发现自己没有考虑好每条路的车流量计算方法，之前只是为了每条路经过的车数量尽可能相同，<br>
忘记每条路的容量是不一样的，所以引入拥堵系数beta
代码中: beta=(car_N+0.01)/((road_isDuplex+1)*road_length*road_channel+0.01);
其中car_N是经过当前道路的车辆总数，注意这里是总数，写的有bug，按常理说应该是实时的，但在当时没有考虑太多，因为官方测试通道快要关了。
beta是变量中唯一一个浮点数，使用时要注意。
测试后只跑通一个图，单图535，比之前的好一些

这时已经29号晚上过10点了，官方测试通道关闭，好多想法都没来得及写：


# 30号：
9点：下载数据，大致车6w+,路250左右，路口150左右，车速，6-16，路长20-60，限速8-15，出发时间0-50，双向路较多
分析可知，与测试地图相比，道路容量增加，车速增加，车辆大概原来的6倍，肯定在6*(测试地图的时间=600)=3600内跑完，

### 1 
最开始设定每发230辆车，发车时间增加10，调度时间大概在6w/230*10=2608附近，
测试结果：地图1时间+地图2时间=2902+2872
代码见：[exam-1.0](https://github.com/LHesperus/route-planning-with-traffic/tree/master/exam/exam-1.0)

### 2
继续增大发车量
每发450辆车，发车时间增加10，测试结果1602+1572，代码见 [exam-2.0](https://github.com/LHesperus/route-planning-with-traffic/tree/master/exam/exam-2.0)

### 3
继续增大发车量
每发600辆车，发车时间增加10，测试结果1262+inf,发生死锁
[exam-3.0](https://github.com/LHesperus/route-planning-with-traffic/tree/master/exam/exam-3.0)

### 4
优化代码，如前所述，beta参数计算的有问题，应该是实时的车流量，故新定义了Answer类，用来记录已经发车的行驶路线，
在第j辆车发车后，减去第(j-200)辆车所经过的路，这里假定第(j-200)辆车已经走过终点，由于程序没有动态规划功能，不能准确知道车是否到达终点，
只能假设在很久之前出发的已经到达终点，每发450辆车，发车时间增加10，测试结果1584+1568，比[exam-2.0效果好了一丁点
代码见：[exam-4.0](https://github.com/LHesperus/route-planning-with-traffic/tree/master/exam/exam-4.0)

### 5
后来其余参数不变，增大发车辆，分别为600，550，但都死锁了,代码见：
[exam-5.0](https://github.com/LHesperus/route-planning-with-traffic/tree/master/exam/exam-5.0)
[exam-6.0](https://github.com/LHesperus/route-planning-with-traffic/tree/master/exam/exam-6.0)

### 6
这时考虑调整发车时间
每3000辆车，发车时间增加50，调度时间估计在：6w/3000*50=1000附近
测试结果：1230+inf，地图2死锁
代码见：[exam-7.0/1-6-0-fialed-1230+inf_3000_50](https://github.com/LHesperus/route-planning-with-traffic/tree/master/exam/exam-7.0/1-6-0-fialed-1230%2Binf_3000_50)

### 7
继续调整参数
2500 50,估计6w/2500*50=1200
测试结果：1430+1418，目前最优结果。
代码见：[exam-7.0/1-6-1-succeed-1430+1418_2500_50](https://github.com/LHesperus/route-planning-with-traffic/tree/master/exam/exam-7.0/1-6-1-succeed-1430%2B1418_2500_50)

### 8
调参
2600 50，测试结果：2地图失败
代码见：[exam-7.0/1-6-2_map2failed-2600_50](https://github.com/LHesperus/route-planning-with-traffic/tree/master/exam/exam-7.0/1-6-2_map2failed-2600_50)

### 9
这时考虑'快慢分批走',思路是：快车，慢车一起走的话，车速低的容易堵住车速高的，可以将其按速度设置为不同的发车时间
但是，在最终发车时间较小的情况下，写了两个都失败了
代码见:
[exam-8.0/1-7-0-failed-1388+inf_550_10-速度过渡](https://github.com/LHesperus/route-planning-with-traffic/tree/master/exam/exam-8.0/1-7-0-failed-1388%2Binf_550_10-%E9%80%9F%E5%BA%A6%E8%BF%87%E6%B8%A1)
[exam-8.0/1-7-1_failed-1299-inf_3000_50速度过渡](https://github.com/LHesperus/route-planning-with-traffic/tree/master/exam/exam-8.0/1-7-1_failed-1299-inf_3000_50%E9%80%9F%E5%BA%A6%E8%BF%87%E6%B8%A1)

### 10
又想到因为速度慢的走的时间长，故应该'谁慢就等谁'，考虑速度不等间隔发车，车快的发车间距进一些，车慢的发车间距远一些
测试结果：1497+1446，感觉没起多少效果
代码见:[exam-8.0/1-7-2-succeed-1497+1446_1500_30_速度不等间隔过渡](https://github.com/LHesperus/route-planning-with-traffic/tree/master/exam/exam-8.0/1-7-2-succeed-1497%2B1446_1500_30_%E9%80%9F%E5%BA%A6%E4%B8%8D%E7%AD%89%E9%97%B4%E9%9A%94%E8%BF%87%E6%B8%A1)

### 11
在此基础上，调整发车数等参数，均失败
代码见：[exam8.0](https://github.com/LHesperus/route-planning-with-traffic/tree/master/exam/exam-8.0)文件夹的除了刚才提到的子文件夹中

### 12
快到截止时间下午5点了，不打算再改了，
最终提交的代码见：
[exam-final](https://github.com/LHesperus/route-planning-with-traffic/tree/master/exam/exam-final)
就是之前最短调度时间的
[exam-7.0/1-6-1-succeed-1430+1418_2500_50](https://github.com/LHesperus/route-planning-with-traffic/tree/master/exam/exam-7.0/1-6-1-succeed-1430%2B1418_2500_50)
最终调度时间：1430+1418=2848。



![调试的各种版本](https://github.com/LHesperus/route-planning-with-traffic/blob/master/message/7.png)

官方的一些文件
[data](https://github.com/LHesperus/route-planning-with-traffic/tree/master/official%20document)

网站调试截图
![Image text](https://github.com/LHesperus/route-planning-with-traffic/blob/master/message/6.png)

