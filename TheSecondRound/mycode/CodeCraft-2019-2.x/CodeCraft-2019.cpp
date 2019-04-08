#include "iostream"
#include <sstream>
#include <fstream>
#include <string>

#include <math.h>
#include <cmath>
#include "def_base_value.h"
#include "def_base_Class.h"
#include "find_path.h"
using namespace std;

//*************************************************************
//车、路、路口数组类
 static Cross Cross_group[n_cross];	//路口
 static Road Road_group[n_road];    //路
 static Car Car_group[n_car];		//车
 static Answer Answer_group[n_car];        //车的路径
 static PresentAnswer PresentAnswer_group[n_pre_ans];//预置车辆

extern int sum_path=0;//所有车总路数,注意车量级，防止溢出

void drive_just_current_road()
{//依次遍历每个方向的每个车道，从车道最前方依次向后遍历道路上的每辆车，将每辆车移动至终止位置或标定为等待状态
	for(int i=1;i<n_path;i++)//每条路
	{
		for(int ii=0;ii<2;ii++)//每个方向
		{
			for(int iii=0;iii<=n_max_channel;iii++)//每个通道
			{
				for(int iiii=0;iiii<n_max_length_path;iiii++)//每辆车
				Road_group[i].car_state[ii][iii][iiii]
			}
		}
	}
}


void run









int main(int argc, char *argv[])
{
     cout << "Begin" <<  endl;
	
	if(argc < 6){
		 cout << "please input args: carPath, roadPath, crossPath, answerPath" <<  endl;
		exit(1);
	}
	
	 string carPath(argv[1]);
	 string roadPath(argv[2]);
	 string crossPath(argv[3]);
	 string presetAnswerPath(argv[4]);
	 string answerPath(argv[5]);
	
	 cout << "carPath is " << carPath <<  endl;
	 cout << "roadPath is " << roadPath <<  endl;
	 cout << "crossPath is " << crossPath <<  endl;
	 cout << "presetAnswerPath is " << presetAnswerPath <<  endl;
	 cout << "answerPath is " << answerPath <<  endl;
	
//*****************************************************************
//*****************************************************************
//从文件中导入数据
read_file(carPath,roadPath,crossPath,presetAnswerPath,Cross_group,Road_group,Car_group,PresentAnswer_group);
//***********************************************************
	// TODO:process
	
	int path_a_b[n_path]={0};
	int p_start=0,p_end=0;
	int time_step=0;//发车时间调整参数

	// TODO:write output file
	ofstream outf; 
	outf.open(answerPath);
	int path_temp[n_path]={0};
	for(int j=1;Car_group[j].dis_num(1)!=0;j++)
	{
		if(Car_group[j].dis_num(8)==1)
		{
			continue;
		}
		p_start=Car_group[j].dis_num(2);
		p_end = Car_group[j].dis_num(3);

		min_time_Dijkstra(p_start,p_end,path_a_b,Car_group[j].dis_num(4),Car_group[j].dis_num(6),Cross_group,Road_group,Car_group);
		
		outf<<'(';
		outf<<Car_group[j].dis_num(1)<<',';
		time_step=int(j/3500);

		outf<<(Car_group[j].dis_num(5)+50* time_step + 750);
		for(int i=0;path_a_b[i]!=0;i++)
		{
				outf<<','<<path_a_b[i];
				Answer_group[j].set_path(i,path_a_b[i]);
		}
		outf<<')'<<endl;
		for(int i=1;i<n_cross;i++)//初始权重
		{
			Cross_group[i].set_W_Dij(init_W);
		}
		//早出发的车 可能已经走完了，去掉早出发车的路径数	
		if(j>200)
		{			
			//早出发车的路径,参数需要修改
			Answer_group[j-100].dis_path(path_temp);
			
			for(int iii=0;path_temp[iii]!=0;iii++)
			{
				for(int ii=1;ii<n_road;ii++)
				{
					if(Road_group[ii].dis_num(1)==path_temp[iii])
					{
						//去掉路径
						Road_group[ii].set_car_N(Road_group[ii].dis_num(8)-1);
						Road_group[ii].set_beta(Road_group[ii].dis_num(8),Road_group[ii].dis_num(7),Road_group[ii].dis_num(2),Road_group[ii].dis_num(4));
					}
				}	
			}		
		}
	}
	outf.close();
	//判题器
	int time=0;
	while(true)//调度所有车辆到达目的地
	{
		time++;
		drive_just_current_road();//第一步，道路内车辆的标定与驱动
		drive_car_init_list();//优先上路车辆
		creat_car_sequence();//创建优先级队列
		
		if(!drive_car_in_wait_state())
		{
			//驱动所有等待车辆进入终止状态，死锁则退出
			return false;
		}
		drive_car_init_list();//所有未上路车辆
		if(isFinish())//所有车辆到达目的地
		{
			break;
		}
	}
	
	
	
	
	return 0;
}