#include "Kruskal.h"
//ͼϡ�裬��Kruskal����������С������
//1 ѡ���e1��ʹw1������С
//2 ����ѡ��e1,e2,����,ei,���E\{e1,e2,����,ei}��ѡȡ��С��w(i+1)������ͼ��loop
//3 ֱ�����������߶����γ�loop��������� 
void del_loop_gen_min_tree(Cross Cross_min_tree_group[n_cross],Road Road_min_tree_group[n_road],Cross Cross_group[n_cross],Road Road_group[n_road])//��С�������㷨
{
	float road_t=0;//·�ĳ���Ȩ��
	float min_road_t=1.0*init_length;
	int road_id=0;//·��id
	int road_i=0;//·�ĽǱ�
	int cross_from_i=0;
	int cross_to_i=0;
	int cross_from_dir=0;//·�����·�ڵķ��򣬴�ǰ����0123
	int cross_to_dir=0;
	int tree_road[n_road]={0};//��¼���ҵ����������ж�Ӧ·��id
	int loop_road[n_road]={0};//��¼���γɻ��ı�id
	bool flag_tl=0;//��ɨ�赽��tree_road��loop_road����ͬid��·����flag_tl=1
	bool flag_add_road=0;//����Ҳ�����·����Ϊ0���ҵ���·��Ϊ1
	//��·��·�ڸ���ʼֵ�� ��д
	
	for(int k=0;k<n_road;k++)
	{
		flag_add_road=0;
		min_road_t=1.0*init_length;
		for(int i=1;i<n_road;i++)//�ҵ���̵�·
		{
			//���ԣ����������������
			if(Road_group[i].dis_num(7)==0)
			{
				continue;
			}
			
			//ȥ���Ѿ��ҵ�����������·�ͻ��γɻ���·
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
//�������޸Ŀ������ɲ�ͬ������С������
			road_t=Road_group[i].dis_num(2)*1.0/Road_group[i].dis_num(3);//֮���ٿ���ͨ����
			if(road_t<=min_road_t)
			{
				min_road_t=road_t;
				road_id=Road_group[i].dis_num(1);
				road_i=i;
			}
			flag_add_road=1;
		}
		//cout<<"flag_add_road:"<<flag_add_road<<endl;
		//�����·�������ӣ�����ֹѭ��
		if(flag_add_road==0)
		{
			break;
		}		
		//�ҵ���Ӧ��·��
		for(int i=1;i<n_cross;i++)
		{
			if(Cross_group[i].dis_num(1)==Road_group[road_i].dis_num(5))//·�����
			{
				cross_from_i=i;
				for(int ii=0;ii<4;ii++)//�ҵ�·�ڷ���
				{
					if(Cross_group[i].dis_num(ii+2)==road_id)
					{
						cross_from_dir=ii;
						break;
					}
				}
			}
			if(Cross_group[i].dis_num(1)==Road_group[road_i].dis_num(6))//·���յ�
			{
				cross_to_i=i;
				for(int ii=0;ii<4;ii++)//�ҵ�·�ڷ���
				{
					if(Cross_group[i].dis_num(ii+2)==road_id)
					{
						cross_to_dir=ii;
						break;
					}
				}
			}
		}
		if(k>2)//�ɱ�����ͼ�����ʣ�ǰ3�ο϶������γɻ����ҵ�4���ߵ�ʱ�����ж����޻�
		{
			//�жϼ����֮���Ƿ����ɻ���ֻ��Ҫ�øĽ���dijkstra�㷨���±��Ƿ�����γɻ�����ΪֻҪ�γɻ������ڿ϶����±�
			//���±߼���ǰ����dij�ж��±߶�Ӧ������·����������·��
			if(Is_have_loop(Road_group[road_i].dis_num(5),Road_group[road_i].dis_num(6),Cross_min_tree_group,Road_min_tree_group)||\
			   Is_have_loop(Road_group[road_i].dis_num(6),Road_group[road_i].dis_num(5),Cross_min_tree_group,Road_min_tree_group))
			{
				//����γɻ����򲻼����±�
				//��¼���γɻ��ı�
				loop_road[k]=Road_group[road_i].dis_num(1);//����k���Ǵ�0��ʼ���ȵ�����Ӱ��
				continue;				
			}
			//���򣬼����±�
		}
		//����Ӧ��СȨ�ر߸���С������
		Road_min_tree_group[road_i].set_num(Road_group[road_i].dis_num(1),\
											Road_group[road_i].dis_num(2),\
											Road_group[road_i].dis_num(3),\
											Road_group[road_i].dis_num(4),\
											Road_group[road_i].dis_num(5),\
											Road_group[road_i].dis_num(6),\
											Road_group[road_i].dis_num(7));
		//����Ӧ·���Ϊ��С��·
		Road_group[road_i].set_min_tree(1);
		//����Ӧ·������·�ڸ��Ƹ���С������
		Cross_min_tree_group[cross_from_i].set_id(Cross_group[cross_from_i].dis_num(1));
		Cross_min_tree_group[cross_from_i].set_road_id(cross_from_dir,road_id);
		Cross_min_tree_group[cross_from_i].set_W_Dij(init_W);//Is_have_loop���ܽ�·��ֵ�Ķ��������ٳ�ʼ��һ��
		Cross_min_tree_group[cross_from_i].set_pro_cross_num(-100);
		Cross_min_tree_group[cross_from_i].set_pro_path(-100);
		
		Cross_min_tree_group[cross_to_i  ].set_id(Cross_group[cross_to_i].dis_num(1));
		Cross_min_tree_group[cross_to_i  ].set_road_id(cross_to_dir,road_id);
		Cross_min_tree_group[cross_to_i  ].set_W_Dij(init_W);//Is_have_loop���ܽ�·��ֵ�Ķ��������ٳ�ʼ��һ��
		Cross_min_tree_group[cross_to_i  ].set_pro_cross_num(-100);
		Cross_min_tree_group[cross_to_i  ].set_pro_path(-100);
		
		//��¼��С�������ı�
		tree_road[k]=Road_group[road_i].dis_num(1);
	}
	//for(int k=0;k<n_road;k++)
	//{
	//	cout<<"k:"<<k<<endl;
	//	cout<<"loop_road[k]:"<<loop_road[k]<<endl;
	//	cout<<"tree_road[k]:"<<tree_road[k]<<endl;
	//}
	
}
