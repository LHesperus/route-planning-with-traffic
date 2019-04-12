//定义了基本的类，同时定义了读取文件数据的函数
#pragma once
#include "iostream"
#include <sstream>
#include <fstream>
#include <string>

#include <math.h>
#include <cmath>
#include "def_base_value.h"
using namespace std;
class Road
{
	public:
	void set_num(int a,int b,int c,int d,int e,int f,int g);
	int dis_num(int n);
	float dis_beta();
	void set_car_N(int a);
	void set_id(int a);
	void set_beta(int car_N,int road_isDuplex,int road_length,int road_channel);
	void set_Pre_car_N(int a);
	void set_straight(int a);
	void set_min_tree(int a);
    private:
    int road_id=0,road_length=init_length,road_speed=1,road_channel=0,road_from=0,road_to=0,road_isDuplex=1; 
    int car_N=0;//路中车辆计数	
	float beta=0;//拥堵系数，车数/路容量
	int Pre_car_N=0;//有多少辆车经过这条路
	int if_straight=1;//直行就是1
	int if_min_tree=6;//是否是最小树，是就为4

};

//*****************************************************
//#(id,from,to,speed,planTime, priority, preset)
class Car
{
	public:
	void set_num(int a,int b,int c,int d,int e,int f,int g);
	int dis_num(int a);
	void set_path_l(int a);
    private:
    int car_id=0,car_from,car_to,car_speed,car_planTime,priority,preset; 
	int path_legth=0;//	汽车需要走的路径长度		
};
//*****************************************************
//#(id,roadId,roadId,roadId,roadId)没有路则值为-1
	class Cross
    {
		public:
		void set_num(int a,int b,int c,int d,int e);
		void set_id(int a);
		void set_road_id(int a,int b);
        void set_W_Dij(int a);
		void set_pro_cross_num(int a);
		void set_pro_path(int a);
		int dis_num(int n );
        private:
        int cross_id=0,cross_roadId_1=-1,cross_roadId_2=-1,cross_roadId_3=-1,cross_roadId_4=-1,cross_W_Dij= init_W;  //Dijkstra 算法的顶点标号大小，初始尽量大
		int cross_roadId[4]={-1,-1,-1,-1};//数组表示，与上面重复定义，有时间在修改
		int pro_cross_num=-10;//前驱顶点的指针
		int pro_path=-10;//前驱路的id
    };
//*******************************************************
//预置车辆#carid, time, roadId1...
class PresentAnswer
{
	public:
	void set_pre_answer(int a,int b,int c[n_path]);
	int dis_carid();
	int dis_pre_time();
	int dis_pre_path(int a);
	private:
	int pre_carid,pre_time;
	int pre_path[n_path]={0};
};
//*******************************************************
//存储每辆车的路径
class Answer
{
	public:
	void set_path(int i,int road_value);
	void dis_path(int a[n_path]);//输出路径
	private:
	int path[n_path]={0};//车的路径
};

void read_file(string &carPath,string &roadPath,string &crossPath,string &presetAnswerPath,Cross Cross_group[n_cross],Road Road_group[n_road],Car Car_group[n_car],PresentAnswer PresentAnswer_group[n_pre_ans]);