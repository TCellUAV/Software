#ifndef _MATH_FUNCTION_H_
#define _MATH_FUNCTION_H_

#include "sys_Platform.h"

/*求两个数最大值*/
#define MATH_ABS(x)     (((x) > 0) ? (x) : -(x))
#define MATH_MAX(a, b)  ((a) > (b) ? (a) : (b))
#define MATH_MIN(a, b)  ((a) < (b) ? (a) : (b))

/*任意一个数的共有特征*/
/*符号*/
typedef enum
{
	MATH_NUMBER_SIGN_PLUS  = 0, /*+号*/
	MATH_NUMBER_SIGN_MINUS = 1,	/*-号*/
}MATH_NUMBER_SIGN;

typedef enum
{
	MATH_FLOTER_PRECISION_2BIT = 2,
	MATH_FLOTER_PRECISION_3BIT = 3,
	MATH_FLOTER_PRECISION_4BIT = 4,
	MATH_FLOTER_PRECISION_5BIT = 5,	
	MATH_FLOTER_PRECISION_6BIT = 6,
	MATH_FLOTER_PRECISION_7BIT = 7,
}MATH_FLOTER_PRECISION;

/*整数特征*/
typedef struct
{
	MATH_NUMBER_SIGN NUMBER_SIGN;	    /*数据的符号*/
	u8 				 totalBitNumber;	/*显示区域总位宽*/
	u8 			     avaBitNumber; 		/*有效位数*/
	u32				 numberABS;			/*绝对值*/
}MATH_Integer;

/*浮点数特征*/
typedef struct
{
	MATH_NUMBER_SIGN NUMBER_SIGN;	              /*数据的符号*/
	u8 			     integerPartBitNumber;        /*整数部分位数*/
    u8				 decimalPartTotalBitNumber;   /*小数部分总位数:(0.0145)4*/
	u8			     decimalPartAvaBitNumber;     /*小数部分有效位数:(0.0145)3*/
	u8 			     decimalPointPos;	          /*小数点在第一个数字后的位置序号*/
	u8 			     decimalNotZeroFrontZeroNbr;  /*小数部分非零数前有几个0*/
	u32				 integerPartABS;	          /*整数部分绝对值*/	
	u32			     decimalABS;      	          /*小数部分绝对值 x10^7 / x10^6 / x10^3 / x10^2/ */
}MATH_Floater;

/*求绝对值*/
fp32 math_Abs(fp32 val);

/*快速开平方*/
fp32 math_InvSqrt(fp32 val);

/*快速反正切*/
fp32 math_fast_atan(fp32 v);

/*二次方*/
fp32 power_x_2(fp32 x);

/*三次方*/
fp32 power_x_3(fp32 x);

/*2D vector length(二维向量求模长)*/
fp32 pythagorous2(fp32 a, fp32 b);

/*3D vector length(三维向量求模长)*/
fp32 pythagorous3(fp32 a, fp32 b, fp32 c);

/*限制一个数的范围(限幅)*/
fp32 math_Constrain(fp32 targValue, fp32 maxValue, fp32 minValue);

/*对一个unsigned / signed数进行分析*/
void math_Integer_Number_Analy(s32 number, u8 totalWidth, MATH_Integer *integer);

/*对一个fp32数进行分析*/
void math_Floater_Number_Analy(fp32 db_number, u8 totalWidth, MATH_Floater *floater);

extern MATH_Integer g_sMathIntegerAnaly;
extern MATH_Integer *g_psMathIntegerAnaly;

extern MATH_Floater  g_sMathFloaterAnaly;
extern MATH_Floater  *g_psMathFloaterAnaly;

#endif
