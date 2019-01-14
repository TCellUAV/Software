#include "math_Function.h"

MATH_Integer g_sMathIntegerAnaly;
MATH_Integer *g_psMathIntegerAnaly = &g_sMathIntegerAnaly;

MATH_Floater g_sMathFloaterAnaly;
MATH_Floater *g_psMathFloaterAnaly = &g_sMathFloaterAnaly;

/*求绝对值*/
fp32 math_Abs(fp32 val)
{
	if (val < 0)
	{
		return -val;
	}
	else 
	{
		return val;
	}
}

/*快速开平方取倒数0x5f3759df/ 0x5f375a86*/
fp32 math_InvSqrt(fp32 val)
{
    fp32 halfx = 0.5f * val;
    s32 i = *(int*)&val;
        
	i = 0x5f3759df - (i >> 1);        // 计算第一个近似根
    val = *(fp32*)&i;
    val = val*(1.5f - halfx*val*val);       // 牛顿迭代法
    
	return val;
} 

/*快速反正切*/
fp32 math_fast_atan(fp32 v)
{
    fp32 v2 = v * v;
    return (v * (1.6867629106f + v2 * 0.4378497304f) / (1.6867633134f + v2));
}

/*二次方*/
fp32 power_x_2(fp32 x)
{
	return (x*x);
}

/*三次方*/
fp32 power_x_3(fp32 x)
{
	return (x*x*x);
}

/*2D vector length(二维向量求模长)*/
fp32 pythagorous2(fp32 a, fp32 b)
{
	return (sqrtf(power_x_2(a) + power_x_2(b)));
}

/*3D vector length(三维向量求模长)*/
fp32 pythagorous3(fp32 a, fp32 b, fp32 c)
{
	return (sqrtf(power_x_2(a) + power_x_2(b) + power_x_2(c)));	
}

/*限制一个数的范围(限幅)*/
fp32 math_Constrain(fp32 targValue, fp32 maxValue, fp32 minValue)
{
	if (targValue >= maxValue)		/*极大值限幅*/
	{
		targValue = maxValue;
	}
	else if (targValue <= minValue)	/*极小值限幅*/
	{
		targValue = minValue;	
	}
	
	return targValue;
}

/*对一个unsigned / signed数进行分析*/
void math_Integer_Number_Analy(s32 number, u8 totalWidth, MATH_Integer *integer)
{
	u32 pow_10 = 1;
	
	/*判断该数符号*/
	if (number >= 0)
	{
		integer->NUMBER_SIGN = MATH_NUMBER_SIGN_PLUS; /*+*/
	}
	else
	{
		integer->NUMBER_SIGN = MATH_NUMBER_SIGN_MINUS; /*-*/
	}
	
	/*该数绝对值*/
	integer->numberABS = math_Abs(number);
	
	/*该数的位数*/
	integer->avaBitNumber = 0; /*先清0*/
	
	while((integer->numberABS / pow_10) != 0)
	{
		integer->avaBitNumber++;
		
		pow_10 *= 10;
	}
	
	/*整数等于0时,整数位为1位*/	
	if (integer->numberABS == 0)
	{
		integer->avaBitNumber = 1;
	}
	
	/*显示区域总位宽*/
	integer->totalBitNumber = totalWidth;
}

/*对一个fp32数进行分析*/
void math_Floater_Number_Analy(fp32 db_number, u8 totalWidth, MATH_Floater *floater)
{
	u32  powInt_10 = 1;	
	u32  powFloat_10 = 1;
	u8   i;
	fp64 decimal = 0.0f;
	/*判断该数符号*/
	if (db_number >= 0)
	{
		floater->NUMBER_SIGN = MATH_NUMBER_SIGN_PLUS; /*+*/
	}
	else
	{
		floater->NUMBER_SIGN = MATH_NUMBER_SIGN_MINUS; /*-*/		
	}
	
	/*将数拆分成小数部分+整数*/
	/*小数部分进行高精度保存*/
	decimal = db_number - (s32)db_number;				  /* < 1*/

	/*整数部分保存*/
	floater->integerPartABS = (u32)(math_Abs(db_number));
	
	/*计算该数的整数部分位数*/
	floater->integerPartBitNumber = 0; /*先清0*/
	
	while((floater->integerPartABS / powInt_10) != 0)
	{
		floater->integerPartBitNumber++;
		
		powInt_10 *= 10;
	}
	
	/*小数的整数等于0时,整数位为1位*/
	if (floater->integerPartABS == 0)
	{
		floater->integerPartBitNumber = 1;		
	}
	
	/*小数点在整数部分各位后的位置序号*/
	floater->decimalPointPos = floater->integerPartBitNumber + 1;
	
	/*根据总位宽,显示数据,整数部分显示完毕后,剩余位宽给小数部分*/
	floater->decimalPartTotalBitNumber = totalWidth - floater->integerPartBitNumber - 2; /*一个浮点数显示总位宽 - 整数位宽 - 两个符号('+','-','.')*/
	
	/*根据剩余位宽,来确定小数部分精度*/
	for (i = floater->decimalPartTotalBitNumber; i > 0; i--)
	{
		powFloat_10 *= 10;
	}
	
	/*对小数部分扩大对应10^x倍数,保留整数部分*/
	floater->decimalABS = (u32)(math_Abs(decimal * powFloat_10));
	
	/*复位*/
	powFloat_10 = 1;
	
	/*计算小数部分有效位数*/
	floater->decimalPartAvaBitNumber = 0; /*先清0*/
	
	while((floater->decimalABS / powFloat_10) != 0)
	{
		floater->decimalPartAvaBitNumber++;
		
		powFloat_10 *= 10;
	}

	/*小数部分,非0数开始前0的个数*/
    floater->decimalNotZeroFrontZeroNbr = floater->decimalPartTotalBitNumber - floater->decimalPartAvaBitNumber;
}
