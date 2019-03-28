#include "iostream"
#include <sstream>
#include <fstream>
#include <string>

#include <math.h>
using namespace std;

//define class
//***********************************************
//************************************************
const int n_car=20000;//车辆的个数
const int n_road=200;//路的个数
const int n_cross=100;//路口的个数
const int n_path=1000;//路径长度
const int init_W=100000;//初始权重大小
const int n_line=200;//文件每行字符数
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
		int dis_num(int a);
        private:
        int car_id=0,car_from,car_to,car_speed,car_planTime;  
    };
    
void Car::set_num(int a,int b,int c,int d,int e)
		{
			car_id=a;
			car_from=b;
			car_to=c;
			car_speed=d;
			car_planTime=e;
		}
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
//*************************************************************
//车、路、路口数组类,参数需要修改
static Cross Cross_group[n_cross];	//路口
static Road Road_group[n_road];    //路
static Car Car_group[n_car];		//车
//Dijkstra算法找到两点最短路径，返回数组

void Common_Dijkstra(int p_start,int p_end,int path_a_b[n_path])
{	
	int Cro_temp[n_cross]={0};//存储遍历的顶点
	bool flag_c=0;
	bool flag_s=0;//判断点是否在S中
	bool flag_r=0;//如果路口相连的边过不去，就置1
	int k=0;//path角标
	int cross_N=0,cross_N_next =0,road_i=0;
	for (int i=0;i<n_cross;i++) //参数需要修改，与路口数量匹配
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
	//cout<<"0:"<<Cro_temp[0]<<endl;

	int min_W= init_W;//最小顶点权重
	int min_W_N=0;//最小顶点指针
		
	while(cross_N_next ==0||Cross_group[cross_N_next].dis_num(1)!= p_end)
	//while(k<100)
	{
		min_W= init_W;
		min_W_N=0;
		for (int j=0;j<4;j++)   //4条路
		{
			if(Cross_group[cross_N].dis_num(j+2)<0)//该方向无边
			{
				continue;
			}
		//	cout<<"4:"<<Cross_group[cross_N].dis_num(j+2)<<endl;
			for (int i=1;i<n_road;i++)   //参数需要修改
			{

		//		cout<<"11:"<<Road_group[i].dis_num(1)<<endl;

				if (Road_group[i].dis_num(1)==Cross_group[cross_N].dis_num(j+2))//找到路口相连边的id
				{	
			//		cout<<"Road_group[i].dis_num(1)"<<Road_group[i].dis_num(1)<<endl;
					if((Road_group[i].dis_num(7)==0&&Road_group[i].dis_num(5)==Cross_group[cross_N].dis_num(1))||(Road_group[i].dis_num(7)==1))//单相且from为路口，或双向
					{
				//		cout<<"Road_group[i].dis_num(1)"<<Road_group[i].dis_num(1)<<endl;
						road_i =i;
						for (int ii=1;ii< n_cross;ii++)
						{
						//	cout<<Cross_group<<Cross_group[ii].dis_num(1)<<endl;
						//	cout<<"Road_group[i].dis_num(6)"<<Road_group[i].dis_num(6)<<endl;
							//找到对应边终点的路口id,排除下一个顶点与当前定点相同的情况
							if((Road_group[i].dis_num(5)==Cross_group[ii].dis_num(1)||Road_group[i].dis_num(6)==Cross_group[ii].dis_num(1))&&Cross_group[ii].dis_num(1)!=Cross_group[cross_N].dis_num(1))
							{			
								cross_N_next =ii;
							//	cout<<"10:"<<cross_N_next<<endl;
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
	//		cout<<"7:"<<cross_N_next<<endl;
			for(int i=0;i< n_cross;i++)//参数需要修改，与Cro_temp长度相同
			{
				//	cout<<"cross_N_next"<<cross_N_next<<endl;
				//	cout<<"Cro_temp[i]"<<Cro_temp[i]<<endl;
				if(cross_N_next == Cro_temp[i])//避免搜索重复顶点	
				{

					flag_c=1;
				//	cout<<"9:"<<flag_c<<endl;
					break;	
				}
			}
			if(flag_c==1)//遇到遍历过的顶点，跳过
			{
				flag_c=0;
				continue;
			}
		//	cout<<"8:"<<cross_N_next<<endl;
					//		cout<<"6:"<<Cross_group[cross_N_next].dis_num(1)<<endl;
			if(Cross_group[cross_N_next].dis_num(1)==p_end)//判断是否找到终点
			{
		//		cout<<"5:"<<Cross_group[cross_N_next].dis_num(1)<<endl;
				Cross_group[cross_N_next].set_pro_cross_num(Cross_group[cross_N].dis_num(1));//更新前驱
				Cross_group[cross_N_next].set_pro_path(Cross_group[cross_N].dis_num(j+2));
				break;
			}
			
			if((Cross_group[cross_N].dis_num(6)+Road_group[road_i].dis_num(2))<=Cross_group[cross_N_next].dis_num(6))//判断顶点权重加上边长后 与 边终点路口权重的大小
			{
				Cross_group[cross_N_next].set_W_Dij(Cross_group[cross_N].dis_num(6)+Road_group[road_i].dis_num(2));////更新路口权重
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
		//cout<<"3:"<< cross_N<<endl;
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
			break;
		}	
	}
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
Road_group[road_num].set_num(g,f,e,d,c,b,a);
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
Car_group[car_num].set_num(e,d,c,b,a);
car_num++;
a=0;b=0;c=0;d=0;e=0; 
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

//***********************************************************
	// TODO:process
	int path_a_b[n_path]={0};
	int p_start=0,p_end=0;
	Common_Dijkstra(14,8,path_a_b);
		for(int i=0;path_a_b[i]!=0;i++)
		cout<< path_a_b[i] <<endl;
	
	// TODO:write output file
	ofstream outf; 
	outf.open(answerPath);
	for(int j=1;Car_group[j].dis_num(1)!=0;j++)
	{
		p_start=Car_group[j].dis_num(2);
		p_end = Car_group[j].dis_num(3);
		Common_Dijkstra(p_start,p_end,path_a_b);
		
		outf<<'(';
		outf<<Car_group[j].dis_num(1)<<',';
		outf<<Car_group[j].dis_num(5);
		for(int i=0;path_a_b[i]!=0;i++)
		{
				outf<<','<<path_a_b[i];
		}
		outf<<')'<<endl;
		for(int i=1;i<n_cross;i++)//初始权重
		{
			Cross_group[i].set_W_Dij(init_W);
		}
	}
	outf.close();

	return 0;
}



