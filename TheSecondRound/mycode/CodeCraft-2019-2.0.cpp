#include "iostream"
#include <sstream>
#include <fstream>
#include <string>

#include <math.h>
#include <cmath>
using namespace std;

//define class
//***********************************************
//************************************************
const int n_car=61441;//车辆的个数
const int n_road=280;//路的个数，不要和真实值差太多，有个参数需要用它
const int n_cross=170;//路口的个数
const int n_path=1000;//路径长度
const int n_pre_ans=20000;//预置车辆数
const int init_W=10000000;//初始权重大小
const int n_line=200;//文件每行字符数
class Road
{
	public:
	void set_num(int a,int b,int c,int d,int e,int f,int g);
	int dis_num(int n);
	float dis_beta();
	void set_car_N(int a);
	float set_beta(int car_N,int road_isDuplex,int road_length,int road_channel);
    private:
    int road_id=0,road_length=10000,road_speed=0,road_channel=0,road_from=0,road_to=0,road_isDuplex=1; 
    int car_N=0;//路中车辆计数	
	float beta=0;//拥堵系数，车数/路容量
};
    
void Road::set_num(int a,int b,int c,int d,int e,int f,int g)
{
	road_id=a;
	road_length=b;
	road_speed=c;
	road_channel=d;
	road_from=e;
	road_to=f;
	road_isDuplex=g;
}
void Road::set_car_N(int a){car_N = a;};//车辆数
float Road::set_beta(int car_N,int road_isDuplex,int road_length,int road_channel)
	{
	beta=(car_N+0.01)/((road_isDuplex+1)*road_length*road_channel+0.01);
	return beta;
	};
int Road::dis_num(int n)//测试用
{
	switch(n)
	{
		case 1:
			return road_id;
		case 2:
			return road_length;
		case 3:
			return road_speed;
		case 4:
			return road_channel;
		case 5:
			return road_from;
		case 6:
			return road_to;
		case 7:
			return road_isDuplex;
		case 8:
			return car_N;
        default:
            return -1;
	}
};
float Road::dis_beta()
{
	return beta;
}
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
    
void Car::set_num(int a,int b,int c,int d,int e,int f,int g)
		{
			car_id=a;
			car_from=b;
			car_to=c;
			car_speed=d;
			car_planTime=e;
			priority=f;
			preset=g;
		}
void Car::set_path_l(int a){path_legth=a;}
int Car::dis_num(int n)//测试用
{
	switch(n)
	{
		case 1:
			return car_id;
		case 2:
			return car_from;
		case 3:
			return car_to;
		case 4:
			return car_speed;
		case 5:
			return car_planTime;
		case 6:
			return path_legth;	
		case 7:
			return priority;
		case 8:
			return preset;
        default:
            return -1;
	}
};

//*****************************************************
//#(id,roadId,roadId,roadId,roadId)没有路则值为-1
	class Cross
    {
		public:
		void set_num(int a,int b,int c,int d,int e);
        void set_W_Dij(int a);
		void set_pro_cross_num(int a);
		void set_pro_path(int a);
		int dis_num(int n );
        private:
        int cross_id=0,cross_roadId_1=0,cross_roadId_2=0,cross_roadId_3=0,cross_roadId_4=0,cross_W_Dij= init_W;  //Dijkstra 算法的顶点标号大小，初始尽量大
		int cross_roadId[4]={0};//数组表示，与上面重复定义，有时间在修改
		int pro_cross_num=-10;//前驱顶点的指针
		int pro_path=-10;//前驱路的id
    };
    
void Cross::set_num(int a,int b,int c,int d,int e)
		{
			cross_id=a;
			cross_roadId_1=b;
			cross_roadId_2=c;
			cross_roadId_3=d;
			cross_roadId_4=e;
			cross_roadId[0]=b;
			cross_roadId[1]=c;
			cross_roadId[2]=d;
			cross_roadId[3]=e;
		}
void Cross::set_W_Dij(int a){cross_W_Dij=a;}
void Cross::set_pro_cross_num(int a){pro_cross_num=a;}
void Cross::set_pro_path(int a){pro_path=a;}

int Cross::dis_num(int n)//测试用
{
	switch(n)
	{
		case 1:
			return cross_id;
		case 2:
			return cross_roadId[0];
		case 3:
			return cross_roadId[1];
		case 4:
			return cross_roadId[2];
		case 5:
			return cross_roadId[3];	
		case 6:
			return cross_W_Dij;
		case 7:
			return pro_cross_num;
		case 8:
			return pro_path;
        default:
            return -1;
	}
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
void PresentAnswer::set_pre_answer(int a,int b,int c[n_path])
{
	pre_carid=a;
	pre_time=b;
	for(int i=0;c[i]!=0;i++)
	{
		pre_path[i]=c[i];
	}
}
int PresentAnswer::dis_carid(){return pre_carid;}
int PresentAnswer::dis_pre_time(){return pre_time;}
int PresentAnswer::dis_pre_path(int a){return pre_path[a];}
//*******************************************************
class Answer
{
	public:
	void set_path(int i,int road_value);
	void dis_path(int a[n_path]);//输出路径
	private:
	int path[n_path]={0};//车的路径
};

void Answer::set_path(int i,int road_value)
{
	path[i]=road_value;
}

void Answer::dis_path(int a[n_path])
{
	for(int i=0;i<n_path;i++)
		{
			a[i]=path[i];
		}
}
//*************************************************************
//车、路、路口数组类
static Cross Cross_group[n_cross];	//路口
static Road Road_group[n_road];    //路
static Car Car_group[n_car];		//车
static Answer Answer_group[n_car];        //车的路径
static PresentAnswer PresentAnswer_group[n_pre_ans];//预置车辆
//Dijkstra算法找到两点最短路径，返回数组
//涉及汽车速度的Dij算法，并且加入了一些参数
int sum_path=0;//总路数,注意车量级，防止溢出
void min_time_Dijkstra(int p_start,int p_end,int path_a_b[n_path],int car_speed,int car_N)
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
}
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
int road_num=0,car_num=0,cross_num=0,pre_ans_num=0;
	// TODO:read input filebuf
//读文件，目前第一行注释也读进去了
ifstream fin(roadPath, ios::in);
char line[n_line]={0};//存储每行字符
string x="" ; 
int a=0,b=0,c=0,d=0,e=0,f=0,g=0,size_x;
char zero='0';//用来字符转数字
int j=0,k=0;
while(fin.getline(line, sizeof(line)))
{
	x=line;
	size_x=x.size();//每行字符数
    j=-1;
    k=0;  
    for(int i=size_x-2;i>=1;i--) //从后向前，方便进位,首尾括号跳过
    {
        if(x[i]==',')//逗号分隔
        {
            k++;
            j=-1;
            continue;
        }
        if(x[i]==' ')//空格跳过
            continue;

        if(isdigit(x[i])==1)//是否有数字，进位
            j++;
        switch(k)
        {
            case 0:           
                a=(x[i]-zero)*int(pow(10,j))+a;
                break;
            case 1:
                b=(x[i]-zero)*int(pow(10,j))+b;
                break;
            case 2:
                c=(x[i]-zero)*int(pow(10,j))+c;
                break;
            case 3:
                d=(x[i]-zero)*int(pow(10,j))+d;
                break;        
            case 4:
                e=(x[i]-zero)*int(pow(10,j))+e;
                break;        
            case 5:
                f=(x[i]-zero)*int(pow(10,j))+f;
                break;        
            case 6:
                g=(x[i]-zero)*int(pow(10,j))+g;
                break;          
        }           
    }
Road_group[road_num].set_num(g,10*f,e,d,c,b,a);//路的长度放大了10倍，为了除以速度得到较精确时间
road_num++;
a=0;b=0;c=0;d=0;e=0;f=0;g=0; 
}
fin.clear();
fin.close();
//Road_group[1].dis_num();//测试用
//导入车数据*****************************************************************
ifstream fin1(carPath, ios::in);
j=0;k=0;
while(fin1.getline(line, sizeof(line)))
{
	x=line;
	size_x=x.size();//每行字符数
    j=-1;
    k=0;  
    for(int i=size_x-2;i>=1;i--) //从后向前，方便进位,首尾括号跳过
    {
        if(x[i]==',')//逗号分隔
        {
            k++;
            j=-1;
            continue;
        }
        if(x[i]==' ')//空格跳过
            continue;

        if(isdigit(x[i])==1)//是否有数字，进位
            j++;
        switch(k)
        {
            case 0:           
                a=(x[i]-zero)*int(pow(10,j))+a;
                break;
            case 1:
                b=(x[i]-zero)*int(pow(10,j))+b;
                break;
            case 2:
                c=(x[i]-zero)*int(pow(10,j))+c;
                break;
            case 3:
                d=(x[i]-zero)*int(pow(10,j))+d;
                break;        
            case 4:
                e=(x[i]-zero)*int(pow(10,j))+e;
                break;        
            case 5:
                f=(x[i]-zero)*int(pow(10,j))+f;
                break;        
            case 6:
                g=(x[i]-zero)*int(pow(10,j))+g;
                break;          
        }           
    }
Car_group[car_num].set_num(g,f,e,d,c,b,a);
car_num++;
a=0;b=0;c=0;d=0;e=0;f=0;g=0; 
}
fin1.clear();
fin1.close();
//Car_group[127].dis_num();//测试用
//**********************************************************
//导入路口数据*****************************************************************
ifstream fin2(crossPath, ios::in);
j=0;k=0;
while(fin2.getline(line, sizeof(line)))
{
	x=line;
	size_x=x.size();//每行字符数
    j=-1;
    k=0;  
    for(int i=size_x-2;i>=1;i--) //从后向前，方便进位,首尾括号跳过
    {
        if(x[i]==',')//逗号分隔
        {
            k++;
            j=-1;
            continue;
        }
        if(x[i]==' ')//空格跳过
            continue;

        if(isdigit(x[i])==1)//是否有数字，进位
            j++;
        switch(k)
        {
          case 0:           
              a=(x[i]-zero)*int(pow(10,j))+a;
              break;
          case 1:
              b=(x[i]-zero)*int(pow(10,j))+b;
              break;
          case 2:
              c=(x[i]-zero)*int(pow(10,j))+c;
              break;
          case 3:
              d=(x[i]-zero)*int(pow(10,j))+d;
              break;        
          case 4:
              e=(x[i]-zero)*int(pow(10,j))+e;
              break;        
          case 5:
              f=(x[i]-zero)*int(pow(10,j))+f;
              break;        
            case 6:
                g=(x[i]-zero)*int(pow(10,j))+g;
                break;          
        }           
    }
Cross_group[cross_num].set_num(e,d,c,b,a);
cross_num++;
a=0;b=0;c=0;d=0;e=0; 
}
fin2.clear();
fin2.close();
//cout<<Cross_group[0].dis_num(1)<<endl;//测试用
//**********************************************************
//导入预置车辆数据
ifstream fin3(presetAnswerPath, ios::in);
j=0;k=0;
int pre_path_forward[n_path]={0};//预置车辆的正向路径
int pre_path[n_path]={0};//其实应该是n_path+2,这个数组把每行的所有数据都存下来了
while(fin3.getline(line, sizeof(line)))
{
	x=line;
	size_x=x.size();//每行字符数
    j=-1;
    k=0;  
    for(int i=size_x-2;i>=1;i--) //从后向前，方便进位,首尾括号跳过
    {
        if(x[i]==',')//逗号分隔
        {
            k++;
            j=-1;
            continue;
        }
        if(x[i]==' ')//空格跳过
            continue;

        if(isdigit(x[i])==1)//是否有数字，进位
            j++;
			
		pre_path[k]=(x[i]-zero)*int(pow(10,j))+pre_path[k];
    }
	for(int ii=0;ii<k-1;ii++)
	{
		pre_path_forward[ii]=pre_path[k-ii-2];
	}
PresentAnswer_group[pre_ans_num].set_pre_answer(pre_path[k],pre_path[k-1],pre_path_forward);
	//清空数据
	for(int ii=0;ii<n_path;ii++)
	{
		pre_path[ii]=0;
		pre_path_forward[ii]=0;
	}
pre_ans_num++;
}
fin3.clear();
fin3.close();
cout<<PresentAnswer_group[2].dis_carid()<<endl;//测试用
cout<<PresentAnswer_group[2].dis_pre_time()<<endl;//测试用
cout<<PresentAnswer_group[2].dis_pre_path(1)<<endl;//测试用
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
		min_time_Dijkstra(p_start,p_end,path_a_b,Car_group[j].dis_num(4),Car_group[j].dis_num(6));
		
		outf<<'(';
		outf<<Car_group[j].dis_num(1)<<',';
		time_step=int(j/3000);

		outf<<(Car_group[j].dis_num(5)+50* time_step + 900);
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
	return 0;
}