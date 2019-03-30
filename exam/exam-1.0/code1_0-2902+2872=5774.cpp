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
const int n_car=61441;//�����ĸ���
const int n_road=280;//·�ĸ�������Ҫ����ʵֵ��̫�࣬�и�������Ҫ����
const int n_cross=170;//·�ڵĸ���
const int n_path=1000;//·������
const int init_W=10000000;//��ʼȨ�ش�С
const int n_line=200;//�ļ�ÿ���ַ���
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
    int car_N=0;//·�г�������	
	float beta=0;//ӵ��ϵ��������/·����
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
void Road::set_car_N(int a){car_N = a;};//������
float Road::set_beta(int car_N,int road_isDuplex,int road_length,int road_channel)
	{
	beta=(car_N+0.01)/((road_isDuplex+1)*road_length*road_channel+0.01);
	return beta;
	};
int Road::dis_num(int n)//������
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
//#(id,from,to,speed,planTime)
class Car
{
	public:
	void set_num(int a,int b,int c,int d,int e);
	int dis_num(int a);
	void set_path_l(int a);
    private:
    int car_id=0,car_from,car_to,car_speed,car_planTime; 
	int path_legth=0;//	������Ҫ�ߵ�·������		
};
    
void Car::set_num(int a,int b,int c,int d,int e)
		{
			car_id=a;
			car_from=b;
			car_to=c;
			car_speed=d;
			car_planTime=e;
		}
void Car::set_path_l(int a){path_legth=a;}
int Car::dis_num(int n)//������
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
        default:
            return -1;
	}
};

//*****************************************************
//#(id,roadId,roadId,roadId,roadId)û��·��ֵΪ-1
	class Cross
    {
		public:
		void set_num(int a,int b,int c,int d,int e);
        void set_W_Dij(int a);
		void set_pro_cross_num(int a);
		void set_pro_path(int a);
		int dis_num(int n );
        private:
        int cross_id=0,cross_roadId_1=0,cross_roadId_2=0,cross_roadId_3=0,cross_roadId_4=0,cross_W_Dij= init_W;  //Dijkstra �㷨�Ķ����Ŵ�С����ʼ������
		int cross_roadId[4]={0};//�����ʾ���������ظ����壬��ʱ�����޸�
		int pro_cross_num=-10;//ǰ�������ָ��
		int pro_path=-10;//ǰ��·��id
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

int Cross::dis_num(int n)//������
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
//����·��·��������,������Ҫ�޸�
static Cross Cross_group[n_cross];	//·��
static Road Road_group[n_road];    //·
static Car Car_group[n_car];		//��
//Dijkstra�㷨�ҵ��������·������������
//�漰�����ٶȵ�Dij�㷨�����Ҽ�����һЩ����
int sum_path=0;//��·��,ע�⳵��������ֹ���
void min_time_Dijkstra(int p_start,int p_end,int path_a_b[n_path],int car_speed,int car_N)
{	
	int Cro_temp[n_cross]={0};//�洢�����Ķ���
	bool flag_c=0;
	bool flag_s=0;//�жϵ��Ƿ���S��
	bool flag_r=0;//���·�������ı߹���ȥ������1
	int k=0;//Cro_temp�Ǳ꣬Ҳ������Ȧ��������
	int cross_N=0,cross_N_next =0,road_i=0;
	int min_speed=1000;//�����ٶȺ͵�·���ٵ���Сֵ
	int path_weight=0;//�ߵ�Ȩ�أ���һЩ�����任��ģ�
	int W_block=1;//�ߵ�Ȩ�ر���
	for (int i=0;i<n_cross;i++) //������Ҫ�޸ģ���·������ƥ��
	{
		if( Cross_group[i].dis_num(1)== p_start)	//�ҵ���Ӧ��·��id
		{
			cross_N = i;
			Cross_group[cross_N].set_W_Dij(0);
			break;
		}
	}
	Cross_group[cross_N].set_pro_cross_num(-100);//������Ҫ�޸ģ������������ǰ����д��-100
	Cross_group[cross_N].set_pro_path(-100);
	Cro_temp[0]= cross_N;//������S
	//cout<<"0:"<<Cro_temp[0]<<endl;

	int min_W= init_W;//��С����Ȩ��
	int min_W_N=0;//��С����ָ��
		
	while(cross_N_next ==0||Cross_group[cross_N_next].dis_num(1)!= p_end)
	//while(k<100)
	{
		min_W= init_W;
		min_W_N=0;
		for (int j=0;j<4;j++)   //4��·
		{
			if(Cross_group[cross_N].dis_num(j+2)<0)//�÷����ޱ�
			{
				continue;
			}
		//	cout<<"4:"<<Cross_group[cross_N].dis_num(j+2)<<endl;
			for (int i=1;i<n_road;i++)   //������Ҫ�޸�
			{
		//		cout<<"11:"<<Road_group[i].dis_num(1)<<endl;

				if (Road_group[i].dis_num(1)==Cross_group[cross_N].dis_num(j+2))//�ҵ�·�������ߵ�id
				{	
			//		cout<<"Road_group[i].dis_num(1)"<<Road_group[i].dis_num(1)<<endl;
					if((Road_group[i].dis_num(7)==0&&Road_group[i].dis_num(5)==Cross_group[cross_N].dis_num(1))||(Road_group[i].dis_num(7)==1))//������fromΪ·�ڣ���˫��
					{
				//		cout<<"Road_group[i].dis_num(1)"<<Road_group[i].dis_num(1)<<endl;
						road_i =i;
						for (int ii=1;ii< n_cross;ii++)
						{
						//	cout<<Cross_group<<Cross_group[ii].dis_num(1)<<endl;
						//	cout<<"Road_group[i].dis_num(6)"<<Road_group[i].dis_num(6)<<endl;
							//�ҵ���Ӧ���յ��·��id,�ų���һ�������뵱ǰ������ͬ�����
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
						break;//·�������ı��ǵ����Ҳ�ͨ
					}
					break;
				}
			}
			if(flag_r==1)
			{
				flag_r=0;
				continue;//��ǰ�߲�ͨ������һ����
			}
	//		cout<<"7:"<<cross_N_next<<endl;
			for(int i=0;i< n_cross;i++)//������Ҫ�޸ģ���Cro_temp������ͬ
			{
				//	cout<<"cross_N_next"<<cross_N_next<<endl;
				//	cout<<"Cro_temp[i]"<<Cro_temp[i]<<endl;
				if(cross_N_next == Cro_temp[i])//���������ظ�����	
				{

					flag_c=1;
				//	cout<<"9:"<<flag_c<<endl;
					break;	
				}
			}
			if(flag_c==1)//�����������Ķ��㣬����
			{
				flag_c=0;
				continue;
			}
		//	cout<<"8:"<<cross_N_next<<endl;
					//		cout<<"6:"<<Cross_group[cross_N_next].dis_num(1)<<endl;
			if(Cross_group[cross_N_next].dis_num(1)==p_end)//�ж��Ƿ��ҵ��յ�
			{
		//		cout<<"5:"<<Cross_group[cross_N_next].dis_num(1)<<endl;
				Cross_group[cross_N_next].set_pro_cross_num(Cross_group[cross_N].dis_num(1));//����ǰ��
				Cross_group[cross_N_next].set_pro_path(Cross_group[cross_N].dis_num(j+2));
				break;
			}
			//�����·�ڸó���������ٶ�
			if(car_speed < Road_group[road_i].dis_num(3))
			{
				min_speed = car_speed;
			}
			else
			{
				min_speed = Road_group[road_i].dis_num(3);
			}
			
			if(Road_group[road_i].dis_num(8)*n_road>sum_path)//�жϵ�ǰ��·�������Ƿ���ڵ�·ƽ��������
			{
				W_block=10;
			}
			else
			{
				W_block=1;
			}
			//cout<<"Road_group[road_i].dis_num(8)"<<Road_group[road_i].dis_num(8)<<endl;
			//cout<<"sum_path"<<sum_path<<endl;
			//path_weight=int(Road_group[road_i].dis_num(2)/min_speed/Road_group[road_i].dis_num(4))\
			*(abs(car_speed-Road_group[road_i].dis_num(3))+1)\
			*W_block;//�����Ȩ�ؼ��㹫ʽ1
			path_weight=int(100*Road_group[road_i].dis_num(2)/min_speed\
			*(abs(car_speed-Road_group[road_i].dis_num(3))+1)\
			*Road_group[road_i].dis_beta()+1);//�����Ȩ�ؼ��㹫ʽ2,ӵ��ϵ��
			//cout<<Road_group[road_i].dis_beta()<<endl;
			//cout<<path_weight<<endl;
			//path_weight=(abs(car_speed-Road_group[road_i].dis_num(3))+1)*W_block;//������·��
			
			//cout<<"Road_group[road_i].dis_num(2)/min_speed="<<Road_group[road_i].dis_num(2)/min_speed<<endl;
			//cout<<"abs(car_speed-Road_group[road_i].dis_num(3)"<<abs(car_speed-Road_group[road_i].dis_num(3))<<endl;
			//cout<<"channel"<<Road_group[road_i].dis_num(4)<<endl;
			//cout<<"W_block"<<W_block<<endl;
			//cout<<"path_weight="<<path_weight<<endl;
			if((Cross_group[cross_N].dis_num(6)+path_weight)<=Cross_group[cross_N_next].dis_num(6))//�ж϶���Ȩ�ؼ��ϱ߳��� �� ���յ�·��Ȩ�صĴ�С
			{
				Cross_group[cross_N_next].set_W_Dij(Cross_group[cross_N].dis_num(6)+path_weight);////����·��Ȩ��
				Cross_group[cross_N_next].set_pro_cross_num(Cross_group[cross_N].dis_num(1));//����ǰ��
				Cross_group[cross_N_next].set_pro_path(Cross_group[cross_N].dis_num(j+2));
			}
		}
		//��V-S���ҵ�Ȩ����С�ĵ㣬��Ϊ��һ������
		for(int i=1;i< n_cross;i++)
		{
			for(int j=0;j< n_cross;j++)//������Ҫ�޸ģ���Cro_temp������ͬ
			{
				if(i == Cro_temp[j])//ȥ��S�еĵ�	
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

		cross_N=min_W_N;//����·��id
		//cout<<"3:"<< cross_N<<endl;
		k++;
		Cro_temp[k]=min_W_N;
	}	
	//���յ㵹�Ƴ�·��
	int path_b_a[n_path]={0};//·����˳��
	int p_end_temp=0;
	p_end_temp=p_end;
	flag_s=0;//���������ˣ����������治�໥Ӱ��
	for(int j=0;j<n_path;j++)//������Ҫ�޸ģ���pathһ��
	{
		for(int i=0;i<n_cross;i++)//������Ҫ�޸ģ���·������ƥ��
		{
			if(Cross_group[i].dis_num(1)==p_end_temp)
			{
				if(Cross_group[i].dis_num(7)!=-10)//�Ƶ����
				{
					p_end_temp=Cross_group[i].dis_num(7);
				}
				else
				{
					flag_s=0;
					break;
				}
				path_b_a[j]=Cross_group[i].dis_num(8);//ע�������ڵ����ʱ�����-100������·������
				//��Ӧ��·����һ����
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
			if(flag_s==1)//����㣬����ѭ��
			{
				flag_s=0;
				break;
			}			
		}	
	}
//��·���������
for(int i=0;i< n_path;i++)
{
	path_a_b[i]=0;
}
	for(int i=0;i< n_path;i++)//������Ҫ�޸ģ���pathһ��
	{
		if(path_b_a[i]==-100)//��������� -100 ��������һ��
		{
			for(int j=0;j<i;j++)
			{
				path_a_b[j]=path_b_a[i-j-1];
			}
			Car_group[car_N].set_path_l(i);//�ó��ߵ�·������
			sum_path = sum_path + Car_group[car_N].dis_num(6);
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
//���ļ���Ŀǰ��һ��ע��Ҳ����ȥ�ˣ���ʱ���ٴ���
ifstream fin(roadPath, ios::in);
char line[n_line]={0};//�洢ÿ���ַ�
string x="" ; 
int a=0,b=0,c=0,d=0,e=0,f=0,g=0,size_x;
char zero='0';//�����ַ�ת����
int j=0,k=0;
while(fin.getline(line, sizeof(line)))
{
	x=line;
	size_x=x.size();//ÿ���ַ���
    j=-1;
    k=0;  
    for(int i=size_x-2;i>=1;i--) //�Ӻ���ǰ�������λ,��β��������
    {
        if(x[i]==',')//���ŷָ�
        {
            k++;
            j=-1;
            continue;
        }
        if(x[i]==' ')//�ո�����
            continue;

        if(isdigit(x[i])==1)//�Ƿ������֣���λ
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
Road_group[road_num].set_num(g,10*f,e,d,c,b,a);//·�ĳ��ȷŴ���10����Ϊ�˳����ٶȵõ��Ͼ�ȷʱ��
road_num++;
a=0;b=0;c=0;d=0;e=0;f=0;g=0; 
}
fin.clear();
fin.close();
//Road_group[1].dis_num();//������
//���복����*****************************************************************
ifstream fin1(carPath, ios::in);
j=0;k=0;
while(fin1.getline(line, sizeof(line)))
{
	x=line;
	size_x=x.size();//ÿ���ַ���
    j=-1;
    k=0;  
    for(int i=size_x-2;i>=1;i--) //�Ӻ���ǰ�������λ,��β��������
    {
        if(x[i]==',')//���ŷָ�
        {
            k++;
            j=-1;
            continue;
        }
        if(x[i]==' ')//�ո�����
            continue;

        if(isdigit(x[i])==1)//�Ƿ������֣���λ
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
//Car_group[127].dis_num();//������
//**********************************************************
//����·������*****************************************************************
ifstream fin2(crossPath, ios::in);
j=0;k=0;
while(fin2.getline(line, sizeof(line)))
{
	x=line;
	size_x=x.size();//ÿ���ַ���
    j=-1;
    k=0;  
    for(int i=size_x-2;i>=1;i--) //�Ӻ���ǰ�������λ,��β��������
    {
        if(x[i]==',')//���ŷָ�
        {
            k++;
            j=-1;
            continue;
        }
        if(x[i]==' ')//�ո�����
            continue;

        if(isdigit(x[i])==1)//�Ƿ������֣���λ
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

//cout<<Cross_group[0].dis_num(1)<<endl;//������

//**********************************************************

//***********************************************************
	// TODO:process
	
	int path_a_b[n_path]={0};
	int p_start=0,p_end=0;
	int time_step=0;//����ʱ���������

	// TODO:write output file
	ofstream outf; 
	outf.open(answerPath);
	for(int j=1;Car_group[j].dis_num(1)!=0;j++)
	{
		p_start=Car_group[j].dis_num(2);
		p_end = Car_group[j].dis_num(3);
		min_time_Dijkstra(p_start,p_end,path_a_b,Car_group[j].dis_num(4),Car_group[j].dis_num(6));
		
		outf<<'(';
		outf<<Car_group[j].dis_num(1)<<',';
		time_step=int(j/230);
		//�������ĳ���ǰ
		//{��д}
		//·�̵�����
		//{Car_group[j].dis_num(6)}
	//	if(Car_group[j].dis_num(4)<=4)//�쳵����
	//	{
	//		outf<<(10+10* time_step+8);	
	//	}
	//	else if(Car_group[j].dis_num(4)>4&&Car_group[j].dis_num(4)<=6)
	//	{
	//		outf<<(10+10* time_step+4);
	//	}
	//	else
	//	{
	//		outf<<(10+10* time_step+0);
	//	}
		outf<<(Car_group[j].dis_num(5)+10* time_step);
		for(int i=0;path_a_b[i]!=0;i++)
		{
				outf<<','<<path_a_b[i];
		}
		outf<<')'<<endl;
		for(int i=1;i<n_cross;i++)//��ʼȨ��
		{
			Cross_group[i].set_W_Dij(init_W);
		}
	}
	outf.close();

	
	return 0;
}



