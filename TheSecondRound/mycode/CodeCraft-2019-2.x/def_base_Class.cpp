#include "def_base_Class.h"

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
void Road::set_id(int a){road_id=a;};//设置id
void Road::set_beta(int car_N,int road_isDuplex,int road_length,int road_channel)
	{
	beta=(car_N+0.01)/((road_isDuplex+1)*road_length*road_channel+0.01);
	};
void Road::set_Pre_car_N(int a){Pre_car_N=a;};//预置车辆数
void Road::set_straight(int a){if_straight=a;};
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
		case 9:
			return Pre_car_N;
		case 10:
			return if_straight;
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
void Cross::set_id(int a){cross_id=a;}
void Cross::set_road_id(int a,int b)//设置各个方向路的id，a=0 1 2 3 
{
	cross_roadId[a]=b;
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
//*******************************************************
//read file
void read_file(string &carPath,string &roadPath,string &crossPath,string &presetAnswerPath,Cross Cross_group[n_cross],Road Road_group[n_road],Car Car_group[n_car],PresentAnswer PresentAnswer_group[n_pre_ans])
{
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
//cout<<PresentAnswer_group[2].dis_carid()<<endl;//测试用
//cout<<PresentAnswer_group[2].dis_pre_time()<<endl;//测试用
//cout<<PresentAnswer_group[2].dis_pre_path(1)<<endl;//测试用
}