#include "find_path.h"
//Dijkstra算法找到两点最短路径，返回数组
//涉及汽车速度的Dij算法，并且加入了一些参数
void min_time_Dijkstra(int p_start,int p_end,int path_a_b[n_path],int car_speed,int car_N,Cross Cross_group_temp[n_cross],Road Road_group_temp[n_road],Car Car_group[n_car],int mode)
{	
	Cross Cross_group[n_cross];
	Road Road_group[n_road];
	for(int i=0;i<n_cross;i++)
	{
		Cross_group[i]=Cross_group_temp[i];
	}
	for(int i=0;i<n_road;i++)
	{
		Road_group[i]=Road_group_temp[i];
	}


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
		for (int j=0,pro_i=0;j<4;j++)   //4条路
		{
			if(Cross_group[cross_N].dis_num(j+2)<0||Cross_group[cross_N].dis_num(j+2)==Cross_group[cross_N].dis_num(8))//该方向无边或是前驱边
			{
				continue;
			}
			for(int i=0;i<4;i++)
			{
				if(Cross_group[cross_N].dis_num(8)==Cross_group[cross_N].dis_num(i+2))//找到前驱的方向
				{
					pro_i=i;
				}				
			}

			for (int i=1;i<n_road;i++)   
			{
				if (Road_group[i].dis_num(1)==Cross_group[cross_N].dis_num(j+2))//找到路口相连边的id
				{	
					if((Road_group[i].dis_num(7)==0&&Road_group[i].dis_num(5)==Cross_group[cross_N].dis_num(1))||(Road_group[i].dis_num(7)==1))//单相且from为路口，或双向
					{
						road_i =i;
						if((j+2)%4==pro_i)//当前方向为直行
						{
							Road_group[road_i].set_straight(1);
						}
						else
						{
							Road_group[road_i].set_straight(3);//
						}
						//cout<<Road_group[road_i].dis_num(10)<<endl;;
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
			
			switch(mode)//采取哪种权重计算方法
			{
				case 1:
						path_weight=int(100*Road_group[road_i].dis_num(2)/min_speed\
						*(abs(car_speed-Road_group[road_i].dis_num(3))+1)\
						*Road_group[road_i].dis_beta()+1);//抽象的权重计算公式1,拥堵系数
						//cout<<"1:"<< path_weight<<endl;
						//cout<<"2:"<< abs(car_speed-Road_group[road_i].dis_num(3))+1<<endl;
						//cout<<"3:"<<Road_group[road_i].dis_beta() <<endl;
						break;
				case 2:
						path_weight=int(100*Road_group[road_i].dis_num(2)/min_speed\
						*(abs(car_speed-Road_group[road_i].dis_num(3))+1)\
						*Road_group[road_i].dis_beta()+1)*(Road_group[road_i].dis_num(9)/10+1);//抽象的权重计算公式2,考虑预置车,控制权重大小，防止路口权越界
						//cout<<path_weight<<endl;
						break;	
				case 3://无拥堵系数,非预置优先车用，考虑了每段非预置车的路径
						path_weight=int(10*Road_group[road_i].dis_num(2)/min_speed/Road_group[road_i].dis_num(4)\
						*(abs(car_speed-Road_group[road_i].dis_num(3))+1)*(Road_group[road_i].dis_num(9)+1)+1);//抽象的权重计算公式3
						//cout<<path_weight<<endl;
						break;
				case 4:
						path_weight=int(1000*Road_group[road_i].dis_beta());
						//cout<<path_weight<<endl;
						break;
				case 5://考虑直行和拐弯
						path_weight=Road_group[road_i].dis_num(10);
						break;
				case 6:
						path_weight=int(1.0*Road_group[road_i].dis_num(2)/min_speed/Road_group[road_i].dis_num(4)*(abs(car_speed-Road_group[road_i].dis_num(3))+1));
						break;
				case 7:
						path_weight=int(1.0*Road_group[road_i].dis_num(2)/min_speed/Road_group[road_i].dis_num(4)*(abs(car_speed-Road_group[road_i].dis_num(3))+1))+Road_group[road_i].dis_num(8);
						break;
				case 8://考虑最小生成树
						path_weight=int(100*Road_group[road_i].dis_num(2)/min_speed\
						*(abs(car_speed-Road_group[road_i].dis_num(3))+1)\
						*Road_group[road_i].dis_beta()*Road_group[road_i].dis_num(11)+1);//
						//cout<<"1:"<< path_weight<<endl;
						//cout<<"2:"<< Road_group[road_i].dis_num(11)<<endl;
						//cout<<"3:"<<Road_group[road_i].dis_beta() <<endl;
						break;
				case 9://仅考虑时间
						path_weight=int(100*Road_group[road_i].dis_num(2)/min_speed);
						//cout<<"1:"<< path_weight<<endl;
						break;
				default:
						path_weight=int(100*Road_group[road_i].dis_num(2)/min_speed\
						*(abs(car_speed-Road_group[road_i].dis_num(3))+1)\
						*Road_group[road_i].dis_beta()+1);//抽象的权重计算公式2,拥堵系数
						break;				
			}
		//cout<<"dis_beta"<<Road_group[road_i].dis_beta()<<endl;
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
				if(Cross_group[i].dis_num(7)!=-100)//推到起点
				{
					p_end_temp=Cross_group[i].dis_num(7);
				}
				else
				{
					flag_s=1;
					//flag_s=0;
					//break;
				}
				path_b_a[j]=Cross_group[i].dis_num(8);//注意这里在到起点时会存入-100，便于路径反向
				//对应的路增加一辆车
				for(int ii=1;ii<n_road;ii++)
				{
					if(Road_group[ii].dis_num(1)==Cross_group[i].dis_num(8))
					{
						//Road_group[ii].set_car_N(Road_group[ii].dis_num(8)+1);
						//Road_group[ii].set_beta(Road_group[ii].dis_num(8),Road_group[ii].dis_num(7),Road_group[ii].dis_num(2),Road_group[ii].dis_num(4));
						Road_group_temp[ii].set_car_N(Road_group[ii].dis_num(8)+1);
						Road_group_temp[ii].set_beta(Road_group[ii].dis_num(8),Road_group[ii].dis_num(7),Road_group[ii].dis_num(2),Road_group[ii].dis_num(4));
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

}

//判断有没有环
int Is_have_loop(int p_start,int p_end,Cross Cross_group_temp[n_cross],Road Road_group_temp[n_road])
{	
	Cross Cross_group[n_cross];
	Road Road_group[n_road];
	for(int i=0;i<n_cross;i++)
	{
		Cross_group[i]=Cross_group_temp[i];
	}
	for(int i=0;i<n_road;i++)
	{
		Road_group[i]=Road_group_temp[i];
	}
	int Cro_temp[n_cross]={0};//存储遍历的顶点
	bool flag_c=0;
	bool flag_s=0;//判断点是否在S中
	bool flag_r=0;//如果路口相连的边过不去，就置1
	bool flag_if_can_pass=0;//如果路口的边可以通过，则为1
	bool flag_have_V_S=0;//
	int k=0;//Cro_temp角标，也是最外圈迭代变量
	int cross_N=0,cross_N_next =0,road_i=0;
	int path_weight=0;//边的权重，（一些参数变换后的）
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
						flag_if_can_pass=1;//起点路口能通过
						
						road_i =i;
						for (int ii=1;ii< n_cross;ii++)
						{
							//找到对应边终点的路口id,排除下一个顶点与当前顶点相同的情况
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
				//break;
				return true;//存在环路
			}


			//path_weight=int(100.0*Road_group[road_i].dis_num(2)/Road_group[road_i].dis_num(3)/Road_group[road_i].dis_num(4));//权重计算公式3：路速度比上路限速再比上通道
			path_weight=Road_group[road_i].dis_num(2);
			
			if((Cross_group[cross_N].dis_num(6)+path_weight)<=Cross_group[cross_N_next].dis_num(6))//判断顶点权重加上边长后 与 边终点路口权重的大小
			{
				Cross_group[cross_N_next].set_W_Dij(Cross_group[cross_N].dis_num(6)+path_weight);////更新路口权重
				Cross_group[cross_N_next].set_pro_cross_num(Cross_group[cross_N].dis_num(1));//更新前驱
				Cross_group[cross_N_next].set_pro_path(Cross_group[cross_N].dis_num(j+2));
			}
		}
		//起点没边，或有边但单项不通，路径无环
		if(k==0&&flag_if_can_pass==0)
		{
			return false;
		}
		
		//在V-S中找到权重最小的点，作为下一个定点
		flag_have_V_S=0;
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
			flag_have_V_S=1;
			//如果剩余点与已寻路径不在一个连通分支上，则这些点的权重一直会保持初始值
			//当起点的所有连通点都遍历完之后，可以判定无环			
			if(Cross_group[i].dis_num(6)<=min_W)
			{
				min_W=Cross_group[i].dis_num(6);
				min_W_N=i;
			}
		}
		//V-S中所有点已遍历完，未发现终点
		if(flag_have_V_S==0)
		{
			cout<<"flag_have_V_S"<<flag_have_V_S<<endl;
			return false;
		}
		//V-S中已经没有小权重点可添加，则判定无环
		if(min_W>(init_W-10))//当其大于一个较大的值时，认定其为无穷
		{
			return false;
		}

		cross_N=min_W_N;//更新路口id
		k++;
		Cro_temp[k]=min_W_N;
	}	
}