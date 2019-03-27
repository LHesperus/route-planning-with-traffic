#include "iostream"
#include <sstream>
#include <fstream>
#include <string>

#include <math.h>
using namespace std;

//define class
//***********************************************
//************************************************
	class Road
    {
		public:
		void set_num(int a,int b,int c,int d,int e,int f,int g);
		int dis_num(int n);
        private:
        int road_id=0,road_length=10000,road_speed=0,road_channel=0,road_from=0,road_to=0,road_isDuplex=1;  
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
int Road::dis_num(int n)//测试用
{
//    cout<<road_id<<endl;
//    cout<<road_length<<endl;
//    cout<<road_speed<<endl;
//    cout<<road_channel<<endl;
//    cout<<road_from<<endl;
//    cout<<road_to<<endl;
//    cout<<road_isDuplex<<endl;
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
        default:
            return -1;
	}
};
//*****************************************************
//#(id,from,to,speed,planTime)
	class Car
    {
		public:
		void set_num(int a,int b,int c,int d,int e);
		void dis_num();
        private:
        int car_id,car_from,car_to,car_speed,car_planTime;  
    };
    
void Car::set_num(int a,int b,int c,int d,int e)
		{
			car_id=a;
			car_from=b;
			car_to=c;
			car_speed=d;
			car_planTime=e;
		}
void Car::dis_num()//测试用
{
	cout<<car_id<<endl;
    cout<<car_from<<endl;
    cout<<car_to<<endl;
    cout<<car_speed<<endl;
    cout<<car_planTime<<endl;
}

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
        int cross_id=0,cross_roadId_1=0,cross_roadId_2=0,cross_roadId_3=0,cross_roadId_4=0,cross_W_Dij=10000;  //Dijkstra 算法的顶点标号大小，初始尽量大
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
//	cout<<cross_id<<endl;
//    cout<<cross_roadId_1<<endl;
//    cout<<cross_roadId_2<<endl;
//    cout<<cross_roadId_3<<endl;
//    cout<<cross_roadId_4<<endl;
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
//*************************************************************
//车、路、路口数组类,参数需要修改
static Cross Cross_group[10000];	//路口
static Road Road_group[10000];    //路
static Car Car_group[10000];		//车
//Dijkstra算法找到两点最短路径，返回数组

int* Common_Dijkstra(int p_start,int p_end)
{	
	int path[100]={0};		//最短路，为路口数据，参数需要修改
	int Cro_temp[100]={0};//存储遍历的顶点
	bool flag_c=0;
	bool flag_s=0;//判断点是否在S中
	int k=0;//path角标
	int cross_N=0,cross_N_next=0,road_i=0;
	for (int i=0;i<10000;i++) //参数需要修改，与路口数量匹配
	{
		if( Cross_group[i].dis_num(1)==p_start)	//找到对应的路口id
		{
			cross_N=i;
			Cross_group[cross_N].set_W_Dij(0);
			break;
		}
	}
	Cross_group[cross_N].set_pro_cross_num(-100);//参数需要修改，先设置起点无前驱，写成-100
	Cross_group[cross_N].set_pro_path(-100);

	int min_W=100000;//最小顶点权重
	int min_W_N=0;//最小顶点指针
		
	//while(Cross_group[cross_N_next].dis_num(1)!=p_end)
	while(k<100)
	{
		cout<<Cross_group[cross_N_next].dis_num(1)<<endl;
		min_W=100000;
		min_W_N=0;
		for (int j=0;j<4;j++)   //4条路
		{
			if(Cross_group[cross_N].dis_num(j+2)<0)//该方向无边
			{
				continue;
			}
			for (int i=1;i<10000;i++)   
			{

				if (Road_group[i].dis_num(1)==Cross_group[cross_N].dis_num(j+2))//找到路口相连边的id
				{	
					road_i=i;
					for (int ii=1;ii<10000;ii++)
					{
						if(Road_group[i].dis_num(6)==Cross_group[ii].dis_num(1))//找到对应边终点的路口id
						{
							cross_N_next=ii;
							break;
						}
					}
					break;
				}
			}
			for(int i=0;i<100;i++)//参数需要修改，与Cro_temp长度相同
			{
				if(cross_N_next==Cro_temp[i])//避免搜索重复顶点	
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
				path[k]=Cross_group[cross_N].dis_num(j+2);
				Cross_group[cross_N_next].set_pro_cross_num(Cross_group[cross_N].dis_num(1));//更新前驱
				Cross_group[cross_N_next].set_pro_path(Cross_group[cross_N].dis_num(j+2));
				cout<<Cross_group[cross_N_next].dis_num(7)<<endl;
				break;
			}
			
			if((Cross_group[cross_N].dis_num(6)+Road_group[road_i].dis_num(2))<=Cross_group[cross_N_next].dis_num(6))//判断顶点权重加上边长后 与 边终点路口权重的大小
			{
				Cross_group[cross_N_next].set_W_Dij(Cross_group[cross_N].dis_num(6)+Road_group[road_i].dis_num(2));////更新路口权重
				Cross_group[cross_N_next].set_pro_cross_num(Cross_group[cross_N].dis_num(1));//更新前驱
				Cross_group[cross_N_next].set_pro_path(Cross_group[cross_N].dis_num(j+2));
			}
			
		//	Cro_temp[k]=min_W_N;//存入已经找到的最小顶点的指针
			//path[k]=(int)Road_group[road_i].dis_num(1);
			//cout<<Cro_temp[k]<<endl;
		}
		//在V-S中找到权重最小的点，作为下一个定点

		for(int i=1;i<10000;i++)
		{
			for(int j=0;j<100;j++)//参数需要修改，与Cro_temp长度相同
			{
				if(i==Cro_temp[j])//出去S中的点	
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
		Cro_temp[k]=min_W_N;
		cross_N=min_W_N;//更新路口id
		cout<<k<<endl;
		k++;
	}	
	//从终点倒推出路径
	int path_b_a[100]={0};//路径反顺序
	int p_end_temp=0;
	p_end_temp=p_end;
	flag_s=0;//不新声明了，反正和上面不相互影响
	for(int j=0;j<100;j++)//参数需要修改，与path一致
	{
		for(int i=0;i<10000;i++)//参数需要修改，与路口数量匹配
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
int path_a_b[100]={0};
for(int i=0;i<100;i++)//参数需要修改，与path一致
{
	if(path_b_a[i]==-100)
	{
		for(int j=0;j<i;j++)
		{
			path_a_b[j]=path_b_a[i-j-1];
		}
		break;
	}	
}
	
//	for(int i=0;i<100;i++)
//		cout<< path_b_a[i] <<endl;
	for(int i=0;path_a_b[i]!=0;i++)
		cout<< path_a_b[i] <<endl;
int a[3]={0};
	return a;
}
int main(int argc, char *argv[])
{
    cout << "Begin" << endl;
	
	if(argc < 5){
		cout << "please input args: carPath, roadPath, crossPath, answerPath" << endl;
		exit(1);
	}
	
	string carPath(argv[1]);
	string roadPath(argv[2]);
	string crossPath(argv[3]);
	string answerPath(argv[4]);
	
	cout << "carPath is " << carPath << endl;
	cout << "roadPath is " << roadPath << endl;
	cout << "crossPath is " << crossPath << endl;
	cout << "answerPath is " << answerPath << endl;

//*****************************************************************
//*****************************************************************


int road_num=0,car_num=0,cross_num=0;

	// TODO:read input filebuf
//读文件，目前第一行注释也读进去了，有时间再处理
ifstream fin("../config/road.txt", ios::in);
char line[100]={0};//存储每行字符
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
Road_group[road_num].set_num(g,f,e,d,c,b,a);
road_num++;
a=0;b=0;c=0;d=0;e=0;f=0;g=0; 
}
fin.clear();
fin.close();
//Road_group[1].dis_num();//测试用
//导入车数据*****************************************************************
ifstream fin1("../config/car.txt", ios::in);
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
Car_group[car_num].set_num(e,d,c,b,a);
car_num++;
a=0;b=0;c=0;d=0;e=0; 
}
fin1.clear();
fin1.close();
//Car_group[127].dis_num();//测试用
//**********************************************************
//导入路口数据*****************************************************************
ifstream fin2("../config/cross.txt", ios::in);
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
//Cross_group[4].dis_num();//测试用
//**********************************************************

//***********************************************************
	// TODO:process
	int* p;
p=Common_Dijkstra(1,9);
	cout<< p <<endl;
	//cout< p->1 <<p->2 << p->3 <<endl;
//	cout<< Cross_group[14].dis_num(8)<<endl;
//	cout<< Cross_group[14].dis_num(7)<<endl;
	// TODO:write output file
	
	return 0;
}



