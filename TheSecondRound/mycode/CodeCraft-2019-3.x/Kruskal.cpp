#include "Kruskal.h"
//图稀疏，用Kruskal方法生成最小生成树
//1 选择边e1，使w1尽可能小
//2 若以选择e1,e2,……,ei,则从E\{e1,e2,……,ei}中选取最小的w(i+1)，且新图无loop
//3 直到任意新增边都会形成loop，程序结束 
void del_loop_gen_min_tree(Cross Cross_min_tree_group[n_cross],Road Road_min_tree_group[n_road],Cross Cross_group[n_cross],Road Road_group[n_road])//最小生成树算法
{
	float road_t=0;//路的抽象权重
	float min_road_t=1.0*init_length;
	int road_id=0;//路的id
	int road_i=0;//路的角标
	int cross_from_i=0;
	int cross_to_i=0;
	int cross_from_dir=0;//路在起点路口的方向，从前到后0123
	int cross_to_dir=0;
	int tree_road[n_road]={0};//记录已找到的生成树中对应路的id
	int loop_road[n_road]={0};//记录会形成环的边id
	bool flag_tl=0;//若扫描到与tree_road和loop_road中相同id的路，则flag_tl=1
	bool flag_add_road=0;//如果找不到新路，则为0，找到新路则为1
	//给路和路口赋初始值， 待写
	
	for(int k=0;k<n_road;k++)
	{
		flag_add_road=0;
		min_road_t=1.0*init_length;
		for(int i=1;i<n_road;i++)//找到最短的路
		{
			//测试，遇到单向道，跳过
			if(Road_group[i].dis_num(7)==0)
			{
				continue;
			}
			
			//去掉已经找到的生成树的路和会形成环的路
			for(int ii=0;ii<n_road;ii++)
			{
				
				if(Road_group[i].dis_num(1)==tree_road[ii]||Road_group[i].dis_num(1)==loop_road[ii])
				{
					flag_tl=1;	
					break;
				}		
			}
			if(flag_tl==1)
			{
				flag_tl=0;
				continue;
			}
//在这里修改可以生成不同种类最小生成树
			road_t=Road_group[i].dis_num(2)*1.0/Road_group[i].dis_num(3);//之后再考虑通道数
			if(road_t<=min_road_t)
			{
				min_road_t=road_t;
				road_id=Road_group[i].dis_num(1);
				road_i=i;
			}
			flag_add_road=1;
		}
		//cout<<"flag_add_road:"<<flag_add_road<<endl;
		//如果无路可再增加，则终止循环
		if(flag_add_road==0)
		{
			break;
		}		
		//找到相应的路口
		for(int i=1;i<n_cross;i++)
		{
			if(Cross_group[i].dis_num(1)==Road_group[road_i].dis_num(5))//路的起点
			{
				cross_from_i=i;
				for(int ii=0;ii<4;ii++)//找到路口方向
				{
					if(Cross_group[i].dis_num(ii+2)==road_id)
					{
						cross_from_dir=ii;
						break;
					}
				}
			}
			if(Cross_group[i].dis_num(1)==Road_group[road_i].dis_num(6))//路的终点
			{
				cross_to_i=i;
				for(int ii=0;ii<4;ii++)//找到路口方向
				{
					if(Cross_group[i].dis_num(ii+2)==road_id)
					{
						cross_to_dir=ii;
						break;
					}
				}
			}
		}
		if(k>2)//由比赛地图的性质，前3次肯定不能形成环，找第4条边的时候再判断有无环
		{
			//判断加入点之后是否生成环，只需要用改进的dijkstra算法看新边是否可以形成环，因为只要形成环，环内肯定有新边
			//在新边加入前，用dij判断新边对应的两个路口有无其他路径
			if(Is_have_loop(Road_group[road_i].dis_num(5),Road_group[road_i].dis_num(6),Cross_min_tree_group,Road_min_tree_group)||\
			   Is_have_loop(Road_group[road_i].dis_num(6),Road_group[road_i].dis_num(5),Cross_min_tree_group,Road_min_tree_group))
			{
				//如果形成环，则不加入新边
				//记录会形成环的边
				loop_road[k]=Road_group[road_i].dis_num(1);//忽略k不是从0开始均匀递增的影响
				continue;				
			}
			//否则，加入新边
		}
		//将对应最小权重边给最小生成树
		Road_min_tree_group[road_i].set_num(Road_group[road_i].dis_num(1),\
											Road_group[road_i].dis_num(2),\
											Road_group[road_i].dis_num(3),\
											Road_group[road_i].dis_num(4),\
											Road_group[road_i].dis_num(5),\
											Road_group[road_i].dis_num(6),\
											Road_group[road_i].dis_num(7));
		//将对应路标记为最小树路
		Road_group[road_i].set_min_tree(1);
		//将对应路的两个路口复制给最小生成树
		Cross_min_tree_group[cross_from_i].set_id(Cross_group[cross_from_i].dis_num(1));
		Cross_min_tree_group[cross_from_i].set_road_id(cross_from_dir,road_id);
		Cross_min_tree_group[cross_from_i].set_W_Dij(init_W);//Is_have_loop可能将路口值改动，这里再初始化一次
		Cross_min_tree_group[cross_from_i].set_pro_cross_num(-100);
		Cross_min_tree_group[cross_from_i].set_pro_path(-100);
		
		Cross_min_tree_group[cross_to_i  ].set_id(Cross_group[cross_to_i].dis_num(1));
		Cross_min_tree_group[cross_to_i  ].set_road_id(cross_to_dir,road_id);
		Cross_min_tree_group[cross_to_i  ].set_W_Dij(init_W);//Is_have_loop可能将路口值改动，这里再初始化一次
		Cross_min_tree_group[cross_to_i  ].set_pro_cross_num(-100);
		Cross_min_tree_group[cross_to_i  ].set_pro_path(-100);
		
		//记录最小生成数的边
		tree_road[k]=Road_group[road_i].dis_num(1);
	}
	//for(int k=0;k<n_road;k++)
	//{
	//	cout<<"k:"<<k<<endl;
	//	cout<<"loop_road[k]:"<<loop_road[k]<<endl;
	//	cout<<"tree_road[k]:"<<tree_road[k]<<endl;
	//}
	
}
