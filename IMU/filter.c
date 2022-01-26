/*******************************************************************************
项目名称：Expeditioner-I: Fatih
程序作者：Mingyan Zhou/Derek Zhou/周茗岩
编写日期：2020/09/04
程序功能：滤波算法--不过比较简单
*******************************************************************************/
#include "filter.h"
#include "structconfig.h"
#include "math.h"

#define N 20      //滤波缓存数组的大小
#define M_PI_F 3.1416f

//****************************************A
// 1 确定一个元素的位序->已知两个数组元素的下标，将其值左小右大排序
float FindPos(float *a, int low, int high) //a[9]={9, 8, 7, 6, 5, 4, 3, 2, 1}; low=2; high=6;
{										   //a[2]=7; a[6]=3;
    float val=a[low];
    while(low<high)
    {
        while(low<high && a[high]>=val)
             high--;                    //如果右边的数大于VAL下标往前移
             a[low]=a[high];            //当右边的值小于VAL则赋值给a[low] value换位
        
		while(low<high && a[low]<=val)
             low++;                     //如果左边的数大于VAL下标往后移
             a[high]=a[low];            //当左边的值大于VAL则赋值给右边a[high]
    }
    a[low]=val;
    return low; //a[9]={9, 8, 3, 6, 5, 4, 7, 2, 1}; pos=low=6(位); a[2]=3; a[6]=7;
}

// 2 快速排序
void QuickSort(float* a, int low, int high) //a:数组首地址，low:数组最小下标，high:数组最大下标
{											//QuickSort(a, 2, 6) -> QuickSort(a, 0, 8)
    int pos;
    if(low<high)
    {
        pos=FindPos(a, low, high); //排序一个位置	//pos=6
        QuickSort(a, low, pos-1);  //递归调用		//QuickSort(a, 2, 5);
        QuickSort(a, pos+1, high);			    	//QuickSort(a, 6, 6);
    }
} //a[9]={1, 2, 3, 4, 5, 6, 7, 8, 9};

//****************************************B
// 3 去最值平均值滤波一组数据
void  SortAver_Filter(float value, float *filter, u8 n) //value: 采样的数据
{
	static float buf[N]={0.0}; //滤波缓存数组的大小N=20
	static u8 cnt=0, flag=1;
	float temp=0;
	u8 i=0;
	
	buf[cnt++]=value;
	
	if(cnt<n && flag)
		return; //数组填不满 不计算
	else
		flag=0; 
	
	QuickSort(buf, 0, n-1); //buf[0]到buf[n-1]的元素排序
	
	for(i=1; i<n-1; i++)
		temp+=buf[i]; //求和 除去最大值与最小值
	
	if(cnt>=n)
		cnt=0; //计数器清零
	
	*filter=temp/(n-2); //求平均
}

// 4 去最值平均值滤波三组数据
void SortAver_FilterXYZ(INT16_XYZ *acc, FLOAT_XYZ *Acc_filt, u8 n)
{
	static float bufx[N], bufy[N], bufz[N];
	static u8 cnt=0, flag=1;
	float temp1=0, temp2=0, temp3=0;
	u8 i;
	
	bufx[cnt]=acc->X;
	bufy[cnt]=acc->Y;
	bufz[cnt]=acc->Z;
	
	cnt++; //这个的位置必须在赋值语句后，否则bufx[0]不会被赋值
	
	if(cnt<n && flag) 
		return; //数组填不满不计算
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
// 5 滑动窗口滤波一组数据
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
	
	for(i=0;i<n;i++) //和前面的方法的区别仅在于没去极值罢了
	{
		temp += buf[i];
	}
	
	if(cnt>=n) cnt=0;
	
	*filt_data = temp/n;
}

// 6 滑动窗口滤波三组数据
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
