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
		void dis_num();
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
void Road::dis_num()//测试用
{
    cout<<road_id<<endl;
    cout<<road_length<<endl;
    cout<<road_speed<<endl;
    cout<<road_channel<<endl;
    cout<<road_from<<endl;
    cout<<road_to<<endl;
    cout<<road_isDuplex<<endl;
}
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
		void dis_num();
        private:
        int cross_id,cross_roadId_1,cross_roadId_2,cross_roadId_3,cross_roadId_4;  
    };
    
void Cross::set_num(int a,int b,int c,int d,int e)
		{
			cross_id=a;
			cross_roadId_1=b;
			cross_roadId_2=c;
			cross_roadId_3=d;
			cross_roadId_4=e;
		}
void Cross::dis_num()//测试用
{
	cout<<cross_id<<endl;
    cout<<cross_roadId_1<<endl;
    cout<<cross_roadId_2<<endl;
    cout<<cross_roadId_3<<endl;
    cout<<cross_roadId_4<<endl;
}
//*************************************************************

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
//车、路、路口数组类
//*****************************************************************
//*****************************************************************
Road Road_group[10000];    //路
Car Car_group[10000];		//车
Cross Cross_group[10000];	//路口
int road_num=0,car_num=0,cross_num=0;

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
Cross_group[4].dis_num();//测试用
//**********************************************************




	// TODO:read input filebuf
	// TODO:process
	// TODO:write output file
	
	return 0;
}
