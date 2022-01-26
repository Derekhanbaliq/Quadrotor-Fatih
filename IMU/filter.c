/*******************************************************************************
��Ŀ���ƣ�Expeditioner-I: Fatih
�������ߣ�Mingyan Zhou/Derek Zhou/������
��д���ڣ�2020/09/04
�����ܣ��˲��㷨--�����Ƚϼ�
*******************************************************************************/
#include "filter.h"
#include "structconfig.h"
#include "math.h"

#define N 20      //�˲���������Ĵ�С
#define M_PI_F 3.1416f

//****************************************A
// 1 ȷ��һ��Ԫ�ص�λ��->��֪��������Ԫ�ص��±꣬����ֵ��С�Ҵ�����
float FindPos(float *a, int low, int high) //a[9]={9, 8, 7, 6, 5, 4, 3, 2, 1}; low=2; high=6;
{										   //a[2]=7; a[6]=3;
    float val=a[low];
    while(low<high)
    {
        while(low<high && a[high]>=val)
             high--;                    //����ұߵ�������VAL�±���ǰ��
             a[low]=a[high];            //���ұߵ�ֵС��VAL��ֵ��a[low] value��λ
        
		while(low<high && a[low]<=val)
             low++;                     //�����ߵ�������VAL�±�������
             a[high]=a[low];            //����ߵ�ֵ����VAL��ֵ���ұ�a[high]
    }
    a[low]=val;
    return low; //a[9]={9, 8, 3, 6, 5, 4, 7, 2, 1}; pos=low=6(λ); a[2]=3; a[6]=7;
}

// 2 ��������
void QuickSort(float* a, int low, int high) //a:�����׵�ַ��low:������С�±꣬high:��������±�
{											//QuickSort(a, 2, 6) -> QuickSort(a, 0, 8)
    int pos;
    if(low<high)
    {
        pos=FindPos(a, low, high); //����һ��λ��	//pos=6
        QuickSort(a, low, pos-1);  //�ݹ����		//QuickSort(a, 2, 5);
        QuickSort(a, pos+1, high);			    	//QuickSort(a, 6, 6);
    }
} //a[9]={1, 2, 3, 4, 5, 6, 7, 8, 9};

//****************************************B
// 3 ȥ��ֵƽ��ֵ�˲�һ������
void  SortAver_Filter(float value, float *filter, u8 n) //value: ����������
{
	static float buf[N]={0.0}; //�˲���������Ĵ�СN=20
	static u8 cnt=0, flag=1;
	float temp=0;
	u8 i=0;
	
	buf[cnt++]=value;
	
	if(cnt<n && flag)
		return; //������� ������
	else
		flag=0; 
	
	QuickSort(buf, 0, n-1); //buf[0]��buf[n-1]��Ԫ������
	
	for(i=1; i<n-1; i++)
		temp+=buf[i]; //��� ��ȥ���ֵ����Сֵ
	
	if(cnt>=n)
		cnt=0; //����������
	
	*filter=temp/(n-2); //��ƽ��
}

// 4 ȥ��ֵƽ��ֵ�˲���������
void SortAver_FilterXYZ(INT16_XYZ *acc, FLOAT_XYZ *Acc_filt, u8 n)
{
	static float bufx[N], bufy[N], bufz[N];
	static u8 cnt=0, flag=1;
	float temp1=0, temp2=0, temp3=0;
	u8 i;
	
	bufx[cnt]=acc->X;
	bufy[cnt]=acc->Y;
	bufz[cnt]=acc->Z;
	
	cnt++; //�����λ�ñ����ڸ�ֵ���󣬷���bufx[0]���ᱻ��ֵ
	
	if(cnt<n && flag) 
		return; //�������������
	else
		flag=0;
	
	QuickSort(bufx, 0, n-1);
	QuickSort(bufy, 0, n-1);
	QuickSort(bufz, 0, n-1);
	
	for(i=1; i<n-1; i++)
	 {
		temp1+=bufx[i];
		temp2+=bufy[i];
		temp3+=bufz[i];
	 }
	 
	 if(cnt>=n) cnt=0;
	 
	 Acc_filt->X=temp1/(n-2);
	 Acc_filt->Y=temp2/(n-2);
	 Acc_filt->Z=temp3/(n-2);
}

//****************************************C
// 5 ���������˲�һ������
void Aver_Filter(float data, float *filt_data, u8 n)
{
	static float buf[N];
	static u8 cnt =0, flag = 1;
	float temp=0;
	u8 i;
	
	buf[cnt]=data;
	cnt++;
	
	if(cnt<n && flag) 
		return;
	else
		flag = 0;
	
	for(i=0;i<n;i++) //��ǰ��ķ��������������ûȥ��ֵ����
	{
		temp += buf[i];
	}
	
	if(cnt>=n) cnt=0;
	
	*filt_data = temp/n;
}

// 6 ���������˲���������
void Aver_FilterXYZ(INT16_XYZ *acc, FLOAT_XYZ *Acc_filt, u8 n)
{
	static s32 bufax[N], bufay[N], bufaz[N];
	static u8 cnt=0,flag=1;
	s32 temp1=0, temp2=0, temp3=0, i;
	
	bufax[cnt] = acc->X;
	bufay[cnt] = acc->Y;
	bufaz[cnt] = acc->Z;
	cnt++;
	
	if(cnt<n && flag) 
		return;
	else
		flag = 0;
	
	for(i=0;i<n;i++)
	{
		temp1 += bufax[i];
		temp2 += bufay[i];
		temp3 += bufaz[i];
	}
	
	if(cnt>=n)  cnt = 0;
	
	Acc_filt->X = temp1/n;
	Acc_filt->Y = temp2/n;
	Acc_filt->Z = temp3/n;
}
