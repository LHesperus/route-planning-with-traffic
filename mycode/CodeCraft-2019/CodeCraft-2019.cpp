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
        int road_id,road_length,road_speed,road_channel,road_from,road_to,road_isDuplex;  
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
		int dis_num(int n );
        private:
        int cross_id,cross_roadId_1,cross_roadId_2,cross_roadId_3,cross_roadId_4,cross_W_Dij=10000;  //Dijkstra 算法的顶点标号大小，初始尽量大
		int cross_roadId[4];//数组表示，与上面重复定义，有时间在修改
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
	//Cross_group[13].dis_num();	//test
	//int a[10]={0}; //test
	int path[100]={0};		//最短路，为路口数据，参数需要修改
	int k=0;//path角标
	int cross_N=0,cross_N_next=0,road_N=0,road_i=0;
	for (int i=0;i<10000;i++) //参数需要修改，与路口数量匹配
	{
		if( Cross_group[i].dis_num(1)==p_start)	//找到对应的路口id
		{
			cross_N=i;
			Cross_group[cross_N].set_W_Dij(0);
			break;
		}
	}

	int min_W=100000;//最小顶点权重
	int min_W_N=0;//最小顶点指针
		
	while(Cross_group[cross_N_next].dis_num(1)!=p_end)
	{
		min_W=100000;
		min_W_N=0;
		for (int j=0;j<4;j++)   //4条路
		{
			for (int i=0;i<10000;i++)   
			{
				if (Road_group[i].dis_num(1)==Cross_group[cross_N].dis_num(j+2))//找到路口相连边的id
				{	
					road_i=i;
					for (int ii=0;ii<10000;ii++)
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
			if(Cross_group[cross_N_next].dis_num(1)==p_end)//判断是否找到终点
			{
				path[k]=Cross_group[cross_N].dis_num(j+2);
				break;
			}
			
			if((Cross_group[cross_N].dis_num(6)+Road_group[road_i].dis_num(2))<=Cross_group[cross_N_next].dis_num(6))//判断顶点权重加上边长后 与 边终点路口权重的大小
			{
				Cross_group[cross_N_next].set_W_Dij(Cross_group[cross_N].dis_num(6)+Road_group[road_i].dis_num(2));////更新路口权重
			}
			//存储最小权重的顶点指针
			if(Cross_group[cross_N_next].dis_num(6)<=min_W)
			{
				min_W=Cross_group[cross_N_next].dis_num(6);
				min_W_N=cross_N_next;
			}
			path[k]=(int)Road_group[road_i].dis_num(1);
			cross_N=min_W_N;//更新路口id
		}
		k++;
	}	
	for(int i=0;path[i]!=0;i++)
		cout<< path[i] <<endl;
	return path;
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
p=Common_Dijkstra(1,50);
	cout<< p <<endl;
	//cout< p->1 <<p->2 << p->3 <<endl;
	
	// TODO:write output file
	
	return 0;
}



