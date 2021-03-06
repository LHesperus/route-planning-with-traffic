#pragma once


#define n_car               61441    //车辆的个数
#define n_road              280      //路的个数，不要和真实值差太多，有个参数需要用它
#define n_cross             170      //路口的个数
#define n_path              1000     //路径长度
#define n_pre_ans           20000    //预置车辆数
#define init_W              1000000000 //初始权重大小
#define init_length			1000000  //路的初始长度
#define n_line              300      //文件每行字符数
#define n_max_length_path   100      //最长路长 
#define n_max_channel		6        //所有路最大通道数
#define preset_last_time    756      //最后一辆预置车的发车时间
//#define pre_car_in50s		800		 //50s内预置车辆数