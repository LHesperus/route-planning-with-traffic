#include "find_path.h"
//Dijkstra算法找到两点最短路径，返回数组
//涉及汽车速度的Dij算法，并且加入了一些参数
void min_time_Dijkstra(int p_start,int p_end,int path_a_b[n_path],int car_speed,int car_N,Cross Cross_group[n_cross],Road Road_group[n_road],Car Car_group[n_car])
{	
	int Cro_temp[n_cross]={0};//存储遍历的顶点
	bool flag_c=0;
	bool flag_s=0;//判断点是否在S中
	bool flag_r=0;//如果路口相连的边过不去，就置1
	int k=0;//Cro_temp角标，也是最外圈迭代变量
	int cross_N=0,cross_N_next =0,road_i=0;
	int min_speed=1000;//汽车速度和道路限速的最小值
	int path_weight=0;//边的权重，（一些参数变换后的）
	int W_block=1;//边的权重变量
	for (int i=0;i<n_cross;i++) 
	{
		if( Cross_group[i].dis_num(1)== p_start)	//找到对应的路口id
		{
			cross_N = i;
			Cross_group[cross_N].set_W_Dij(0);
			break;
		}
	}
	Cross_group[cross_N].set_pro_cross_num(-100);//参数需要修改，先设置起点无前驱，写成-100
	Cross_group[cross_N].set_pro_path(-100);
	Cro_temp[0]= cross_N;//起点存入S

	int min_W= init_W;//最小顶点权重
	int min_W_N=0;//最小顶点指针
		
	while(cross_N_next ==0||Cross_group[cross_N_next].dis_num(1)!= p_end)
	{
		min_W= init_W;
		min_W_N=0;
		for (int j=0;j<4;j++)   //4条路
		{
			if(Cross_group[cross_N].dis_num(j+2)<0)//该方向无边
			{
				continue;
			}
			for (int i=1;i<n_road;i++)   
			{
				if (Road_group[i].dis_num(1)==Cross_group[cross_N].dis_num(j+2))//找到路口相连边的id
				{	
					if((Road_group[i].dis_num(7)==0&&Road_group[i].dis_num(5)==Cross_group[cross_N].dis_num(1))||(Road_group[i].dis_num(7)==1))//单相且from为路口，或双向
					{
						road_i =i;
						for (int ii=1;ii< n_cross;ii++)
						{
							//找到对应边终点的路口id,排除下一个顶点与当前定点相同的情况
							if((Road_group[i].dis_num(5)==Cross_group[ii].dis_num(1)||Road_group[i].dis_num(6)==Cross_group[ii].dis_num(1))&&Cross_group[ii].dis_num(1)!=Cross_group[cross_N].dis_num(1))
							{			
								cross_N_next =ii;
								break;
							}
						}					
					}
					else
					{
						flag_r=1;
						break;//路口相连的边是单行且不通
					}
					break;
				}
			}
			if(flag_r==1)
			{
				flag_r=0;
				continue;//当前边不通，找下一条边
			}
			for(int i=0;i< n_cross;i++)//参数需要修改，与Cro_temp长度相同
			{
				if(cross_N_next == Cro_temp[i])//避免搜索重复顶点	
				{

					flag_c=1;
					break;	
				}
			}
			if(flag_c==1)//遇到遍历过的顶点，跳过
			{
				flag_c=0;
				continue;
			}
			if(Cross_group[cross_N_next].dis_num(1)==p_end)//判断是否找到终点
			{
				Cross_group[cross_N_next].set_pro_cross_num(Cross_group[cross_N].dis_num(1));//更新前驱
				Cross_group[cross_N_next].set_pro_path(Cross_group[cross_N].dis_num(j+2));
				break;
			}
			//计算该路口该车辆的最大速度
			if(car_speed < Road_group[road_i].dis_num(3))
			{
				min_speed = car_speed;
			}
			else
			{
				min_speed = Road_group[road_i].dis_num(3);
			}
			
			if(Road_group[road_i].dis_num(8)*n_road>sum_path)//判断当前道路汽车数是否大于道路平均汽车数
			{
				W_block=10;
			}
			else
			{
				W_block=1;
			}
			//path_weight=int(Road_group[road_i].dis_num(2)/min_speed/Road_group[road_i].dis_num(4))\
			*(abs(car_speed-Road_group[road_i].dis_num(3))+1)\
			*W_block;//抽象的权重计算公式1
			path_weight=int(100*Road_group[road_i].dis_num(2)/min_speed\
			*(abs(car_speed-Road_group[road_i].dis_num(3))+1)\
			*Road_group[road_i].dis_beta()+1);//抽象的权重计算公式2,拥堵系数

			if((Cross_group[cross_N].dis_num(6)+path_weight)<=Cross_group[cross_N_next].dis_num(6))//判断顶点权重加上边长后 与 边终点路口权重的大小
			{
				Cross_group[cross_N_next].set_W_Dij(Cross_group[cross_N].dis_num(6)+path_weight);////更新路口权重
				Cross_group[cross_N_next].set_pro_cross_num(Cross_group[cross_N].dis_num(1));//更新前驱
				Cross_group[cross_N_next].set_pro_path(Cross_group[cross_N].dis_num(j+2));
			}
		}
		//在V-S中找到权重最小的点，作为下一个定点
		for(int i=1;i< n_cross;i++)
		{
			for(int j=0;j< n_cross;j++)//参数需要修改，与Cro_temp长度相同
			{
				if(i == Cro_temp[j])//去除S中的点	
				{
					flag_s=1;
					break;
				}	
			}
			if(flag_s==1)
			{
				flag_s=0;
				continue;
			}
			if(Cross_group[i].dis_num(6)<=min_W)
			{
				min_W=Cross_group[i].dis_num(6);
				min_W_N=i;
			}
		}

		cross_N=min_W_N;//更新路口id
		k++;
		Cro_temp[k]=min_W_N;
	}	
	//从终点倒推出路径
	int path_b_a[n_path]={0};//路径反顺序
	int p_end_temp=0;
	p_end_temp=p_end;
	flag_s=0;//不新声明了，反正和上面不相互影响
	for(int j=0;j<n_path;j++)//参数需要修改，与path一致
	{
		for(int i=0;i<n_cross;i++)//参数需要修改，与路口数量匹配
		{
			if(Cross_group[i].dis_num(1)==p_end_temp)
			{
				if(Cross_group[i].dis_num(7)!=-10)//推到起点
				{
					p_end_temp=Cross_group[i].dis_num(7);
				}
				else
				{
					flag_s=0;
					break;
				}
				path_b_a[j]=Cross_group[i].dis_num(8);//注意这里在到起点时会存入-100，便于路径反向
				//对应的路增加一辆车
				for(int ii=1;ii<n_road;ii++)
				{
					if(Road_group[ii].dis_num(1)==Cross_group[i].dis_num(8))
					{
						Road_group[ii].set_car_N(Road_group[ii].dis_num(8)+1);
						Road_group[ii].set_beta(Road_group[ii].dis_num(8),Road_group[ii].dis_num(7),Road_group[ii].dis_num(2),Road_group[ii].dis_num(4));
					}
				}
				break;
			}
			if(flag_s==1)//到起点，结束循环
			{
				flag_s=0;
				break;
			}			
		}	
	}
//将路径变成正向
for(int i=0;i< n_path;i++)
{
	path_a_b[i]=0;
}
	for(int i=0;i< n_path;i++)//参数需要修改，与path一致
	{
		if(path_b_a[i]==-100)//搜索到起点 -100 与上面数一致
		{
			for(int j=0;j<i;j++)
			{
				path_a_b[j]=path_b_a[i-j-1];
			}
			Car_group[car_N].set_path_l(i);//该车走的路径长度
			sum_path = sum_path + Car_group[car_N].dis_num(6);
			break;
		}	
	}
	cout<<sum_path<<endl;
}