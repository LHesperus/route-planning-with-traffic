#include "iostream"
#include <sstream>
#include <fstream>
#include <string>

#include <math.h>
#include <cmath>
#include "def_base_value.h"
#include "def_base_Class.h"
#include "find_path.h"
#include "Kruskal.h"
using namespace std;

//*************************************************************
//车、路、路口数组类
 static Cross Cross_group[n_cross];	//路口
 static Road Road_group[n_road];    //路
 static Car Car_group[n_car];		//车
 static Answer Answer_group[n_car];        //车的路径
 static PresentAnswer PresentAnswer_group[n_pre_ans];//预置车辆

 static Cross Cross_min_tree_group[n_cross];	//路口
 static Road Road_min_tree_group[n_road];    //路
 
 
extern int sum_path=0;//所有车总路数,注意车量级，防止溢出


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
int time_a[51];//统计发车时间分布
for(int i=1;i<n_car;i++)
{
	time_a[Car_group[i].dis_num(5)]=time_a[Car_group[i].dis_num(5)]+1;
}
for(int i=1;i<51;i++)
{
	cout<<i<<":"<<time_a[i]<<endl;
}
	// TODO:process
//预置车走的路,用来控制先发车的路径，有时间写的话应该考虑分时间段统计，每条路的总预置车大概在[0,4000]范围
//for(int i=1;i<800;i++)
//{
//	for(int ii=0;PresentAnswer_group[i].dis_pre_path(ii)!=0;ii++)//预置车的每个车的每条路
//	{
//		for(int iii=1;iii<n_road;iii++)//找到对应的路id
//		{
//			if(PresentAnswer_group[i].dis_pre_path(ii)==Road_group[iii].dis_num(1))
//			{
//				Road_group[iii].set_Pre_car_N(Road_group[iii].dis_num(9)+1);
//				//Road_group[iii].set_car_N(Road_group[iii].dis_num(8)+1);
//				//Road_group[iii].set_beta(Road_group[iii].dis_num(8),Road_group[iii].dis_num(7),Road_group[iii].dis_num(2),Road_group[iii].dis_num(4));		
//				break;
//			}
//		}
//		
//	}
//}
//for(int i=1;i<n_road;i++)//统计预置车道路
//{
//	cout<<Road_group[i].dis_num(9)<<endl;
//}	
	
	del_loop_gen_min_tree(Cross_min_tree_group,Road_min_tree_group,Cross_group,Road_group);//最小生成树算法


	int path_a_b[n_path]={0};
	int p_start=0,p_end=0;
	int time_step=0;//发车时间调整参数
    int set_time=0;//实际发车时间
	// TODO:write output file
	ofstream outf; 
	outf.open(answerPath);
	int path_temp[n_path]={0};
	
	int pri_car_i[n_car]={0};//存放优先非预置车的地址
	int pri_npre=0;//优先车非预置车总数
	int pri_span=16;//优先车非预置车发车间隔
	int pri_n_group=0;//每批发多少辆优先车
	

	
	for(int i=1,j=0;i<n_car;i++)//优先非预置车
	{
		if(Car_group[i].dis_num(7)==1&&Car_group[i].dis_num(8)!=1)
		{
			pri_car_i[j]=i;
				j++;
		}	
			pri_npre=j;
	}
	
	//pri_n_group=pri_npre*pri_span/preset_last_time;

	//先跑优先非预置车
	pri_n_group=400;
	for(int j=0;pri_car_i[j]!=0;j++)
	{		
		p_start=Car_group[pri_car_i[j]].dis_num(2);
		p_end = Car_group[pri_car_i[j]].dis_num(3);
		
		min_time_Dijkstra(p_start,p_end,path_a_b,Car_group[pri_car_i[j]].dis_num(4),pri_car_i[j],Cross_group,Road_group,Car_group,1);
		
		outf<<'(';
		outf<<Car_group[pri_car_i[j]].dis_num(1)<<',';
		
		
		//time_step=int(j/pri_n_group);
		//set_time=Car_group[pri_car_i[j]].dis_num(5)+pri_span* time_step;
		time_step=int(j/pri_n_group);
		set_time=Car_group[pri_car_i[j]].dis_num(5)+50* time_step;
		//set_time=51+5* time_step;
		outf<<set_time;
		for(int i=0;path_a_b[i]!=0;i++)
		{
				outf<<','<<path_a_b[i];
				Answer_group[pri_car_i[j]].set_path(i,path_a_b[i]);
		}
		outf<<')'<<endl;
		for(int i=1;i<n_cross;i++)//初始权重
		{
			Cross_group[i].set_W_Dij(init_W);
		}
				
		//早出发的车 可能已经走完了，去掉早出发车的路径数	
		if(j>pri_n_group)
		{			
			//早出发车的路径,参数需要修改
			Answer_group[pri_car_i[j-pri_n_group]].dis_path(path_temp);
			
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
	
	//非优先非预置
	int car_n_group=2600;//普通车的发车批量
	//car_n_group=800;
	int car_n_in_pre=8000;//预置车中塞进去的普通车数
	int time_car=756;
	int car_i_temp[n_car]={0};
	for(int j=1,jj=0;Car_group[j].dis_num(1)!=0;j++)
	{	
		if(Car_group[j].dis_num(8)==1||Car_group[j].dis_num(7)==1)//跳过优先车和预置车
		{
			continue;
		}
		jj++;//表示遍历到车
		car_i_temp[jj]=j;
		//考虑将一部分车放在预置车里

			
		p_start=Car_group[j].dis_num(2);
		p_end = Car_group[j].dis_num(3);
    			
		min_time_Dijkstra(p_start,p_end,path_a_b,Car_group[j].dis_num(4),j,Cross_group,Road_group,Car_group,1);
		
		outf<<'(';
		outf<<Car_group[j].dis_num(1)<<',';
		time_step=int(jj/car_n_group);//这里的jj是表示有jj辆车过去了，不能用j
    
		outf<<(Car_group[j].dis_num(5)+50* time_step + time_car);
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
		if(jj>car_n_group)
		{
			
			//早出发车的路径,参数需要修改
			//Answer_group[car_i_temp[jj-car_n_group]].dis_path(path_temp);
			Answer_group[j-car_n_group].dis_path(path_temp);
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
		//if(jj>=car_n_in_pre)//发完预置车
		//{
		//	if(jj/car_n_in_pre==1)//第一批
		//	{
		//		
		//	}
		//	car_n_group=2600;
		//	time_car=800;
		//}
	}
	outf.close();

	
	
	
	
	return 0;
}