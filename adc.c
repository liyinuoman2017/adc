/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

#include "gpio.h"
#include "dma.h"
#include "tim.h"

/* USER CODE BEGIN 0 */
#include "usart.h"
#include<string.h>
#include <math.h>/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

#include "gpio.h"
#include "dma.h"
#include "tim.h"

/* USER CODE BEGIN 0 */
#include "usart.h"
#include<string.h>
#include <math.h>
#include "arm_math.h"
#include "coeff.h"
#include "apd_temp_para.h"
#include "dis_para.h"

#define _2PI    (2*3.14152966F)
#define DIS_MAX_VALUE (15000U)
#define DIS_ANGLE_MAX (360U)

uint32_t adc_converted_value_num; //0或者1
ADC_CONVERTED_VALUE  adc_converted_value_0,adc_converted_value_1;


int32_t adc_buff_add[IF_MAX];
int32_t adc_buff[IF_MAX];
int32_t temp_buff[IF_MAX];

volatile uint8_t adc_cplt_flag=0;
volatile uint8_t adc_valid_flag=0;
volatile uint8_t adc_half_cplt_flag=0;

volatile uint8_t sample_channel=SIGNAL_CHN;
volatile uint8_t temp_flag;


volatile DIS_DTTA dis_buff[DIS_MAX];
volatile uint32_t dis_cnt;
volatile uint32_t dis_cnt_last;
volatile uint32_t dis_cnt_last_save;
__DIS_INT_ANG_DATA dis_int_angle_data[DIS_ANGLE_MAX+1];
extern volatile uint32_t motor_speed_counter;

extern volatile uint32_t angle_trig_flag,angle_trig_counter_record[30],angle_trig_cnt,angle_trig_cnt_save;
uint32_t angle_trig_cnt_save_copy;
extern volatile uint32_t motor_speed;
extern volatile uint32_t motor_speed_rt;
uint8_t debug_temp_num=0;
extern uint16_t restart_num;
extern uint8_t adc_error_ovr;

#define BUFF_MAX_SIZE       5           //队列长度为5        
#define BUFF_MAX_LENGTH     24          //宽度为24



uint8_t queue_data_buff[BUFF_MAX_SIZE][BUFF_MAX_LENGTH];                
uint8_t out_buff[BUFF_MAX_LENGTH]; 
float thershold_buff[15];            
Queue queue;
uint8_t send_flag=0;
                
uint32_t APD_circle_cnt,APD_high_cnt,APD_low_cnt;   

uint32_t temp_default_value=1798;//1.71V
uint32_t vapd_pwm_duty=150;//135//200//230;   

uint16_t apd_therm_vol_ad=1000;   //ad值  
int32_t  t_realtime=0;  
int16_t  temp_threshold=-1000;   //低温不调的阈值 ，默认-10度              
   
int16_t  DP_num=0;  
int8_t   temp_num=0;								
uint16_t apd_bias_vol_ad=0;    //ad值             

uint16_t distance_realtime=0;
uint16_t intensity_realtime=0;              

extern volatile uint8_t angle_num,angle_num_max;
uint8_t angle_num_copy,angle_num_max_copy;    
                
int16_t angle_offset=0;   //必须是6的整数倍                

int32_t b0=-1300; //初始截距，-1.5米。2019.9.29改为-1.8米。
int32_t a1=10000,b1=0,a2=10000,b2=0,a3=10000,b3=0,a4=10000,b4=0,thread_12=0,thread_23=0,thread_34=0;  

int32_t a_i=0,b_i=0,a_dc=0,b_dc=0;
int32_t a_t=0,t0=0;  

int16_t t_25=30;  
uint16_t t_apd_vol_high=28*1120,t_apd_vol_low=28*820;


extern uint16_t work_mode;                
extern volatile uint32_t angle_num_sum;

extern uint16_t cp_value,dp_value;

extern uint8_t temp_vol_flag;

uint16_t distance_intensity_thread=0;


uint16_t dis_1,dis_2,dis_3,dis_4,dis_5,inte_1,inte_2,inte_3,inte_4,inte_5;


//保存距离参数
unsigned short dis_cali[31]={
0,
9,
33,
87,
134,
185,
228,
286,
376,
466,
556,
646,
736,
826,
916,
1006,
1096,
1186,
1276,
1304,
1304,
1312,
1317,
1328,
1337,
1348,
1362,
1389,
1411,
1425,
1450
};

unsigned short dis_standard[31]={
1419,
1424,
1446,
1495,
43,
95,
139,
191,
280,
368,
456,
543,
631,
719,
807,
895,
982,
1070,
1190,
1215,
1221,
1230,
1240,
1257,
1274,
1293,
1313,
1334,
1354,
1378,
1398

};

//保存APD的温度参数
unsigned short temp_ad[100]={
3604,
3579,
3553,
3526,
3499,
3470,
3441,
3411,
3379,
3347,
3314,
3281,
3246,
3210,
3174,
3137,
3099,
3060,
3021,
2981,
2940,
2898,
2856,
2814,
2771,
2727,
2683,
2639,
2594,
2549,
2504,
2458,
2413,
2367,
2321,
2275,
2230,
2184,
2138,
2093,
2048,
2003,
1959,
1914,
1871,
1827,
1784,
1742,
1700,
1658,
1617,
1577,
1537,
1498,
1460,
1422,
1385,
1348,
1312,
1277,
1243,
1209,
1176,
1144,
1112,
1082,
1052,
1022,
993,
965,
938,
912,
886,
860,
836,
812,
788,
766,
744,
722,
701,
681,
661,
642,
624,
605,
588,
571,
554,
538,
523,
507,
493,
479,
465,
451,
438,
426,
413,
402
};

unsigned short temp_duty[100]={
270,
270,
270,
270,
270,
270,
270,
270,
270,
270,
270,
270,
270,
270,
270
};



static uint32_t isFull(void); 
static uint32_t isEmpty(void);
static QueueStatus inQueue(uint8_t buff[]);
static QueueStatus outQueue(uint8_t buff[]);

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
                
                
#define debug               0   //0转，1曲线
#define debug_send_distance 0   //1使能静止报距离

uint8_t temp_cali_enable=1;     //使能温度校准功能   
uint8_t dis_offset_enable=1;    //使能距离补偿功能


/* ADC1 init function */
void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2; //采样频率改为1.4MHz，12+3=15clock ，84M/4/15=1.4M
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_Ext_IT11;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;//DISABLE;//ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sample_channel=SIGNAL_CHN;    //采集距离信号
 
}

//设置ADC的采集通道
void ADC1_SetChannel(uint8_t chn)
{
    ADC_ChannelConfTypeDef sConfig;
    
    if(chn==TEMP_CHN)   //采集温度数据
    {    
        /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
        */
      sConfig.Channel = ADC_CHANNEL_6;
      sConfig.Rank = 1;
      sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }
      sample_channel=TEMP_CHN;    //采集温度
    }    
    else if(chn==SIGNAL_CHN)
    {
        /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
        */
      sConfig.Channel = ADC_CHANNEL_5;
      sConfig.Rank = 1;
      sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }
      sample_channel=SIGNAL_CHN;    //采集距离信号
    }
}    


void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    /**ADC1 GPIO Configuration    
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_5;//|GPIO_PIN_4
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral DMA init*/
  
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_NORMAL;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(ADC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */
   
  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC1 GPIO Configuration    
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4|GPIO_PIN_5);

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(adcHandle->DMA_Handle);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(ADC_IRQn);

  }
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
} 

/* USER CODE BEGIN 1 */

//回调函数
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
//    hadc1.Instance->CR2 &= ~ADC_CR2_CONT;   //停止ADC 的连续转换
//    //HAL_ADC_Stop(&hadc1);
//    adc_half_cplt_flag=1;
}  
//回调函数  只传递标志，不读数据
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
//    uint32_t i=0;
    
    if(sample_channel==SIGNAL_CHN)
    {
        adc_cplt_flag=1;    //每次都要计算距离，如果本次采集的是其它通道，则直接用上次的数据计算距离
        
        //采样结束是高电平，意味着触发是上升沿，此时数据有效。
        //采样结束是低电平，意味着触发是“下降沿”，此时数据无效。
        if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3))
        {
            adc_valid_flag=1;
        }
        else
        {
            adc_valid_flag=0;
        }
        /*--------------------------------*/  //50us
        //LD_PowerControl(1);
        //HAL_ADC_Stop(&hadc1);
        //MX_ADC1_Init();
        //HAL_ADC_Start_DMA(&hadc1,adc_converted_value,ADC_DMA_BUFF_LEN); 
        
        //angle_num_sum++;
        angle_num_copy=angle_num;
        angle_num_max_copy=angle_num_max;
        angle_trig_cnt_save_copy=angle_trig_cnt_save;
    }
    else if(sample_channel==TEMP_CHN)
    {
        temp_flag=1;
    }
     LD_PowerControl(1);   
} 

void DelayUS(uint32_t n)
{
    uint32_t i=0,j=0;
    for(i=0;i<n;i++)
    {
        for(j=0;j<18;j++){;}
    }
} 

//质心计算距离,约200us
//3个周期是480us，不能超过这个时间
//算法原则：可以少报，不能错报
static uint16_t Calculate_Position(uint32_t numSamples,int32_t data[],uint16_t *delta,uint16_t *d_current)
{
//    uint32_t i,j;
	uint32_t i;
    int32_t pos=0,pos_pre;
    uint64_t sum1=0,sum2=0;
    
    uint16_t duty;
//    uint32_t max,min,max_num_cnt,max_pos,low_thre,high_thre,start,end;
	 uint32_t max,min,max_pos,low_thre,high_thre,start,end;
    uint32_t amp_thread,amp_thread_2,amp_thread_3,amplitude,max_n,min_thre_2_n,min_thre_2_nmax,max_cnt[10];
    
//    uint32_t low_cnt=0,high_cnt=0;
    
//    DOT dot1,dot2;

    duty=numSamples/2;         //采样点数为2个周期
    /*------------------------------------------------------- */  
    //在两个周期内，搜索峰的位置，需要考虑可能有多个峰
    max=0;
    min=4096;
    max_pos=0;
    for(i=5;i<duty;i+=4)     //从第一个点开始，跳过有时会异常的第0个点
    {
        if(data[i]>max)
        {
            max=data[i];
            //max_pos=i;
        } 
        if(data[i]<min)
        {
            min=data[i];
        } 
    }
    
    amplitude=max-min;
    amp_thread=max-amplitude*1/2;  
    amp_thread_2=min+amplitude*1/4;
    *d_current=min;

    max=0;
    max_pos=0;
    max_n=0;
    min_thre_2_n=0;
    min_thre_2_nmax=0;
    for(i=5;i<duty;i+=2)
    {
        //找极大值
        if(data[i]>amp_thread)
        {
            if(data[i]>max)
            {
                max=data[i];
                max_pos=i;
            }       
        }
        else
        {
            if(data[i-2]>amp_thread)
            {    
                if(max_n<10)
                {    
                    max_cnt[max_n]=max_pos;
                    max_n++;
                    max=0;
                }
            }            
        }
        //找连续点数，数据在min ~ amp_thread_2 之间的连续点数要大于50
        if(data[i]<amp_thread_2)
        {    
            min_thre_2_n++;
        }
        else
        {
            if(min_thre_2_n>min_thre_2_nmax)
            {
                min_thre_2_nmax=min_thre_2_n;
            }    
            min_thre_2_n=0;
        }           
    }
    
    if(min_thre_2_n>min_thre_2_nmax)
    {
        min_thre_2_nmax=min_thre_2_n;
    }
//    if(min_thre_2_nmax<25)
//    {
//        *delta=0;
//        *d_current=0;
//        return (DIS_MAX_VALUE);
//    }
    
  
    
    //实际可能的峰个数，1、2、3、4；还需要判断峰在两个周期内的分布情况
    if((max_pos<10)||(max_pos>(duty-10)))
    {
        *delta=0;
        *d_current=0;
        return (DIS_MAX_VALUE);
    }    

    //前后各预留60个点，截取
    if(max_pos<=60)
    {
        low_thre=0;
    }
    else
    {
        low_thre=max_pos-60;
    }
    
    if(max_pos>=(duty-1-60))
    {
        high_thre=duty-1;
    }
    else
    {
        high_thre=max_pos+60;
    } 
 
/*------------------------------------------------------- */
    max=0;
    min=4095;
    for(i=low_thre;i<high_thre;i++)
    {
        data[i]=(data[i]+data[i+duty])>>1;      //两个周期累加平均
        data[i]=(data[i]+data[i+1])>>1;         //两点平滑
        if(data[i]>max)                         //在截取的数据段内再重新找最大值和最小值
        {
            max=data[i];
            max_pos=i;
        }
        if(data[i]<min)
        {
            min=data[i];
        }
    }
//    //min要大于两个端点值的最大值，避免出现峰左右两边不对称的情况
//    if(data[low_thre]>min)
//    {
//        min=data[low_thre];
//    }    
//    if(data[high_thre]>min)
//    {
//        min=data[high_thre];
//    } 
/*-----------------------------------------------------------------*/ 
//    //计算直流量
//    j=0;
//    sum1=0;
//    for(i=2;i<low_thre;i++)
//    {
//        sum1+=data[i];
//        j++;
//    }
//    for(i=high_thre;i<duty;i++)
//    {
//        sum1+=data[i];
//        j++;
//    }
//    *d_current=sum1/j;
/*------------------------------------------------------- */  
    if((max>min)&&(max-min<20))   //峰峰值小于100 ，直接返回最大距离。
    {      
        *delta=0;
        *d_current=0;
        return (DIS_MAX_VALUE);
    }
/*-----------------------------------------------------------------*/
    //信号截取
    amp_thread_3=max-(max-min)*50/100;
    
    //amp_thread_3要大于两个端点值的最大值，避免出现峰左右两边不对称的情况
    if(data[low_thre]>amp_thread_3)
    {
        amp_thread_3=data[low_thre];
    }    
    if(data[high_thre]>amp_thread_3)
    {
        amp_thread_3=data[high_thre];
    }
    
    //质心  50us
    start=0;
    end=0;
    sum1=0;
    sum2=0;
    for(i=low_thre;i<=high_thre;i++)
    {
        if(data[i]>amp_thread_3)
        {    
            data[i]-=amp_thread_3;
            sum1+=data[i]*i;
            sum2+=data[i];
            if(start==0)
            {
                start=i;
            }
            else
            {
                end=i;
            }    
        }         
    }
    //判断宽度，限制在60个点
//    if((end-start)>60)  
//    {
//        *delta=0;
//        *d_current=0;
//        return (DIS_MAX_VALUE);
//    }    
    pos=(sum1*1000)/sum2; //计算距离  
    pos=(int64_t)pos*15345/duty/1000;      

    
    //强度压缩
    //*delta=sum2/300;  
    *delta=sum2/100;  //50
    
    /*-----------------------------------------------------------------*/ 
    //强度、距离修正
    if((*delta)<255)
    {    
        //pos=pos+((*d_current+a_t*((int32_t)apd_therm_vol_ad-t0)/10000-1100)*a_dc+b_dc+50)/10000+((*delta)*a_i+b_i+50)/100;
        //pos=pos+((*d_current+a_t*((int32_t)apd_therm_vol_ad-t0)/10000-1100)*a_dc+b_dc+50)/10000*10+((*delta)*a_i+b_i+50)/100*10;
        //pos=pos+((*d_current+a_t*((int32_t)apd_therm_vol_ad-t0)/10000-1100)*a_dc+b_dc+50)/1000+((*delta)*a_i+b_i+50)/10;
        pos=pos+((*d_current-1100)*b_i)/1000+((*delta)*a_i)/10;
    }
    else
    {    
        //pos=pos+((*d_current+a_t*((int32_t)apd_therm_vol_ad-t0)/10000-1100)*a_dc+b_dc+50)/1000;
        pos=pos+((*d_current-1100)*b_dc)/1000+((*delta)*a_dc)/10;
    }
    
    
    //温度修正 2019.3.21
    pos=pos+a_t*((int32_t)apd_therm_vol_ad)/10000;
    
/*-----------------------------------------------------------------*/
    //强度范围限制
    if(*delta>255)
    {
        *delta=255;
    }   
/*-----------------------------------------------------------------*/    
    pos_pre=pos;
    //距离补偿，乘以斜率，加上截距
    if(dis_offset_enable)
    {    
        if(pos_pre>(thread_12*10))
        {    
            pos= (pos*a1+b1*1000)/10000;
        }
        else if(pos_pre>(thread_23*10))
        {
            pos= (pos*a2+b2*1000)/10000;
        } 
        else if(pos_pre>(thread_34*10))
        {
            pos= (pos*a3+b3*1000)/10000;
        }
        else
        {
            pos= (pos*a4+b4*1000)/10000;
        }    
    }
    
    pos+=b0;        //加上初始截距
    
    if((pos<0)||(pos>DIS_MAX_VALUE))
    {
        *delta=0;
        *d_current=0;
        return (DIS_MAX_VALUE);
    } 
    
    
    
//高反修正    
//    if((pos>500)&&(pos<1000))
//    {
//        if(*delta>200)
//        {
//            pos=pos-((*delta)*(-0.066)-74.66);
//        }    
//    }    
//    else
//    {
//        if(*delta>150)
//        {
//            pos=pos-((*delta)*(-0.066)-74.66);
//        }    
//    }    
    
    
    
    if(pos>9000) //8200
    {
        pos=DIS_MAX_VALUE;
    } 
    
    //距离强度限制 
    if((pos<dis_1)&&(pos>=dis_2))
    {
        if(*delta<inte_1)
        {
            *delta=0;
            *d_current=0;
            return (DIS_MAX_VALUE);
        }    
    }
    else if((pos<dis_2)&&(pos>=dis_3))
    {
        if(*delta<inte_2)
        {
            *delta=0;
            *d_current=0;
            return (DIS_MAX_VALUE);
        }    
    }
    else if((pos<dis_3)&&(pos>=dis_4))
    {
        if(*delta<inte_3)
        {
            *delta=0;
            *d_current=0;
            return (DIS_MAX_VALUE);
        }    
    }
    else if((pos<dis_4)&&(pos>=dis_5))
    {
        if(*delta<inte_4)
        {
            *delta=0;
            *d_current=0;
            return (DIS_MAX_VALUE);
        }    
    }
    else if(pos<dis_5)
    {
        if(*delta<inte_5)
        {
            *delta=0;
            *d_current=0;
            return (DIS_MAX_VALUE);
        }    
    }
    
    if(pos<3000)
    {    
        if(min_thre_2_nmax<25)
        {
            *delta=0;
            *d_current=0;
            return (DIS_MAX_VALUE);
        }
    }
    
    return (pos);
}
/**
*********************************************************************************************************
* @名称	: 
* @描述	: 
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/

uint16_t intensity_out_value; 
int16_t dp_out_value;
maxmin_t  user_maxmin;
/**
*********************************************************************************************************
* @名称	: 
* @描述	: user_maxmin.min   0为最小
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
void Save_Maxmin_Min(uint16_t distance_realtime)
{
	uint16_t data_buff[MAXMIN_SIZE+1],temp;
	uint8_t i,j;
	//将最小值复制到maxmin数值0~9 处   最新值复制到10
	memcpy(data_buff,&user_maxmin.min[0],20);
	data_buff[MAXMIN_SIZE]=distance_realtime;
	
   //冒泡排序 0~10   0为最小 
	for(j=0;j<10;j++)
	{
		for(i=0;i<10-j;i++)
		{
			if(data_buff[ i ] > data_buff[i + 1])
			{
				temp = data_buff[i];
				data_buff[i] = data_buff[i + 1];
				data_buff[i + 1] = temp;
			}
		}
	}
	//将1~10数据复制到 maxmin数值
	memcpy(&user_maxmin.min[0],&data_buff[0],20);
}
/**
*********************************************************************************************************
* @名称	: 
* @描述	: user_maxmin.max   0为最大
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
void Save_Maxmin_Max(uint16_t distance_realtime)
{
	uint16_t data_buff[MAXMIN_SIZE+1],temp;
	uint8_t i,j;
	//将最小值复制到maxmin数值0~9 处   最新值复制到10
	memcpy(data_buff,&user_maxmin.max[0],20);
	data_buff[MAXMIN_SIZE]=distance_realtime;
	
   //冒泡排序 0~10   0为最大
	for(j=0;j<10;j++)
	{
		for(i=0;i<10-j;i++)
		{
			if(data_buff[ i ] < data_buff[i + 1])
			{
				temp = data_buff[i];
				data_buff[i] = data_buff[i + 1];
				data_buff[i + 1] = temp;
			}
		}
	}
	//将1~10数据复制到 maxmin数值
	memcpy(&user_maxmin.max[0],&data_buff[0],20);
	
	
}
/**
*********************************************************************************************************
* @名称	: 
* @描述	: 
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
 void Temperature_Coefficient_Find_Maxmin(uint16_t distance_realtime)
{
	//测量值小于 MIN
	if(distance_realtime<user_maxmin.min[MAXMIN_SIZE-1])
	{
		Save_Maxmin_Min(distance_realtime);
	}
	//测量值小于 MAX	
	else if((distance_realtime>user_maxmin.max[MAXMIN_SIZE-1])&&(distance_realtime!=DIS_MAX_VALUE))
	{
		Save_Maxmin_Max(distance_realtime);
	}
}

uint8_t temperature_coefficient_state=PREHEAT_STATE;
/**
*********************************************************************************************************
* @名称	: 
* @描述	: 
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/

uint16_t test_max_data;
uint16_t test_min_data;
uint8_t  temperature_coefficient_save_flag=0;
uint16_t temperature_coefficient_save_num=0;
uint16_t thershold_buff_calc[15];
uint16_t  testttt[15];
void Calc_Thershold_Handle(void)
{
	float max,min;
	float  space;
	uint8_t i;
	max=0;
	min=0;
	//提取max  min后5组数据
	for(i=5;i<=9;i++)
		max += user_maxmin.max[i];
	max=max/5;
	for(i=5;i<=9;i++)
		min += user_maxmin.min[i];
	min=min/5;
	space=(max-min)/11;
	for(i=0;i<10;i++)
	{
		thershold_buff[i]=min+space*(i+1);
		thershold_buff_calc[i]=(uint16_t)thershold_buff[i];
	}
	
	
	test_max_data=max;
	test_min_data=min;
	//切换运行模式
	temperature_coefficient_state=SAVE_DATA_STATE;
	temperature_coefficient_save_num=0;
}
/**
*********************************************************************************************************
* @名称	: 
* @描述	: 
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
temperature_coefficient_group_t user_temperature_coefficient;

void Save_Data_Handle(uint16_t distance_realtime)
{
	uint8_t i;
	int8_t  temperature_value; 
	int8_t  save_handle_flag_buff[10];
	/* 清零 */	
	memset(save_handle_flag_buff,0,sizeof(save_handle_flag_buff));
	/*根据测量值匹配阈值*/	
	for(i=0;i<10;i++)
	{
		if(thershold_buff[i]>3)
		{
			if( ( distance_realtime >= ( thershold_buff[i]-5 ) )&&
				( distance_realtime <= ( thershold_buff[i]+5 ) ) )
				save_handle_flag_buff[i]=1;
		}
	}
	/* 提取温度*/
	temperature_value=(temp_num+15);
	/* 将数据保存到所有符合阈值调节的缓存中   */	
	for(i=0;i<10;i++)
	{
		if((save_handle_flag_buff[i]==1)&&(temperature_value>=0)&&(temperature_value<70))
		{
			user_temperature_coefficient.group[i].data[temperature_value].dis=distance_realtime;
			user_temperature_coefficient.group[i].data[temperature_value].dp=dp_out_value;
			user_temperature_coefficient.group[i].data[temperature_value].intensity=intensity_out_value; 
			user_temperature_coefficient.group[i].data[temperature_value].temp=temp_num;
		}
	}
	/* 保存参数到FLASH */
	if(temperature_coefficient_save_flag==1)
	{
		temperature_coefficient_save_flag=0;
		Temperature_Coefficient_Flash_Write(); 
		/*读取参数用于测试观察*/
		Temperature_Coefficient_Flash_Read() ;
	}
}
/**
*********************************************************************************************************
* @名称	: 
* @描述	: 
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
uint8_t temperature_coefficient_handle_flag=0;

void Save_Temperature_Coefficient_Handle(uint16_t distance_realtime)
{
	debug_temp_num++;
	switch(temperature_coefficient_state)
	{
		/* 预热 不进行操作*/
		case PREHEAT_STATE:
		break;
		/* 寻找最大最小值 */
		case FIND_MIN_MAX_STATE:
			Temperature_Coefficient_Find_Maxmin(distance_realtime);
		break;
		/* 计算10组阈值*/
		case CALC_THRESHOLD_STATE:
			Calc_Thershold_Handle();
		break;
		/*  保存数据*/
		case SAVE_DATA_STATE:
			Save_Data_Handle(distance_realtime);		
		break;
		default:
		break;
	}
}
	
uint16_t angle_max=0;
//存储距离的计算结果
void GetPosition(void)
{
//    uint32_t i=0,j=0;
	uint32_t i=0;
    uint16_t delta=0,direct_current=0;
    uint16_t temp=0;
	uint16_t current_angle=0;
 
    
    uint16_t *psrc_buff=NULL;
    int32_t *pdest_buff;    
    
    if(adc_cplt_flag)
    {
        adc_cplt_flag=0;
        
        if(adc_converted_value_num==0)
        {
            psrc_buff=(uint16_t *)adc_converted_value_0.dat;
            adc_converted_value_num=1;
        }
        else
        {
            psrc_buff=(uint16_t *)adc_converted_value_1.dat;
            adc_converted_value_num=0;
        }    
        pdest_buff=adc_buff_add;
        //采样结束是高电平，意味着触发是上升沿，此时数据有效。
        //采样结束是低电平，意味着触发是“下降沿”，此时数据无效。
        if(adc_valid_flag)
        {    
            for(i=0;i<IF_MAX;i++)
            {
                *pdest_buff++=*psrc_buff++;    //复制数据
            }
        }
        
        { 

            temp=angle_standard[angle_trig_cnt_save_copy]+(angle_num_max_copy-angle_num_copy)+1;
            temp-=8;
			current_angle=temp;
            //角度取值1~360
            dis_buff[temp].trig_angle=temp;
            angle_max=temp;
            if(((angle_trig_cnt_save_copy==29)&&(angle_num_copy==0)) || ((angle_trig_cnt_save_copy==0)&&(angle_num_copy>=5)))  //已经走完最后一个齿的最后一个角度，将角度最大值设为360度 
            {
                angle_max=360;
            }
    
			distance_realtime=Calculate_Position(IF_MAX,adc_buff_add,&delta,&direct_current);
			dis_buff[temp].dis=distance_realtime; 
			/*距离为有效值时 重启计数器清零*/
			if(distance_realtime>0)
				restart_num=0;
			
			/* 当计算到角度1时 保存温循数据 */
			if(current_angle==1)
			{
				//temperature_coefficient_handle_flag=0;
				if(work_mode == MODE_SAVE_DATA)
					Save_Temperature_Coefficient_Handle(dis_buff[1].dis);
				
			}
			
			intensity_realtime=delta;
			intensity_out_value=intensity_realtime;
			dis_buff[temp].intensity=intensity_realtime;
			dis_buff[temp].DPset = DP_num;
			dis_buff[temp].Therm = temp_num;
						
            if(work_mode==0x81)
            {    
                printf("%d;%d;%d;%d\r",dis_buff[temp].dis,dis_buff[temp].intensity,direct_current,t_realtime);   //耗时700us
            }           
            GPIOA4_Blink();     //输出距离的计算频率            
        }
    } 
		
}

extern uint32_t apd_duty;
extern uint16_t temp_user_10;
//计算每一度的距离数据，每满6度发送一帧    发送数据采用度脉冲方式
uint32_t GetDistanceAndAngle(void)
{
    uint32_t i=0,j=0;
    uint8_t *p=cmd_data_buff;
//    static uint32_t i_static=0;
    static uint16_t angle=1,angle_max_copy;
    int16_t angle_send=0,temp=0;
    uint32_t sum=0;
  
    if(angle_max_copy  == angle_max)
    {
        return 0;
    }    
    angle_max_copy=angle_max;   

    for(;angle<=angle_max_copy;)
    {   
        if((angle>=6)&&((angle%6)==0))       //每6度发一次
        {
            {
                *p++=0xef;
                *p++=0xef;
                //组帧，发送
                *p++=motor_speed>>8;        //速度
                *p++=motor_speed;            
                //对角度进行修正180度
                angle_send=angle;
                angle_send-=240;
                angle_send-=angle_offset;
                *p++=(angle-6)/6;;            //角度  
                //测6次相位数据，距离+反射率 
                for(j=1;j<=6;j++)  
                {
                    temp=angle_send+j;
                    if(temp>360)
                    {
                        temp-=360;
                    }
                    else if(temp<=0)
                    {
                        temp+=360;
                    } 
                    									
                    if(angle==6 && j==2)
					{
						*p++ =  dis_buff[temp].DPset>>8;
						*p++ =  dis_buff[temp].DPset;					
						*p++ =  (int8_t)(dis_buff[temp].Therm);	
						
//						*p++ =  thershold_buff_calc[0]>>8;
//						*p++ =  thershold_buff_calc[0];
//						
//						*p++=debug_temp_num; //处理计数						
					}
					else
					{
						if(j==3)
						{
							*p++=temp_user_10>>8;   //距离
							*p++=temp_user_10;
							
							*p++=0; 												
						}
						else
						{
							*p++=dis_buff[temp].dis>>8;   //距离
							*p++=dis_buff[temp].dis;
							*p++=dis_buff[temp].intensity; //反射率
							
					          ///清空距离数组
							dis_buff[temp].dis=DIS_MAX_VALUE;
						}
					}
                }
                sum=0;
                for(i=0;i<23;i++)
                {
                    sum+=cmd_data_buff[i];
                }   
                cmd_data_buff[23]=sum;
                if(sum==0xEF)
                {
                    cmd_data_buff[22]++;
                    cmd_data_buff[23]++;
                } 
                angle++;
#if 1           
                inQueue(cmd_data_buff);
                break;
#else        
                while(!__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)){;}       
                HAL_UART_Transmit_DMA(&huart1,p,24); 
#endif                    
            }
        }
        else
        {
            angle++;
        }    
    }
    if(angle>360)      //已发送完最后一帧
    {
        angle=1;
//        i_static=0;
        
    }
    return 1;
}   

void EnableSendData(void)
{
    send_flag=1;
}
    
void DisableSendData(void)
{
    send_flag=0;
}    

void SendDistanceData(void)
{   
    if(!isEmpty())
    {
        //while(!__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)){;}
        if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC))
        {
            outQueue(out_buff);
            HAL_UART_Transmit_DMA(&huart1,out_buff,BUFF_MAX_LENGTH); 
        }    
    }
}    

//队列管理
QueueStatus init_queue(void)  
{  
    //初始化读写指针  
    queue.wp = 0;
    queue.rp = 0;  
    //初始化队列中元素个数  
    queue.queueCnt = 0; // conuter  
    
    return QueueOK;
}  

static uint32_t isFull(void)  
{  
    return (queue.queueCnt == BUFF_MAX_SIZE ) ? 1 : 0;  
}  

static uint32_t isEmpty(void)  
{  
    return (0==queue.queueCnt)? 1 : 0;  
} 

static QueueStatus inQueue(uint8_t buff[])  
{  
    uint32_t i;
    if (1==isFull())  
    {  
        return QueueFull;  
    }  
    else   
    {  
        //保存数据
        for(i=0;i<BUFF_MAX_LENGTH;i++)
        {
            queue_data_buff[queue.wp][i]=buff[i];
        }
        //回转  
        if (++(queue.wp) == BUFF_MAX_SIZE)  
        {  
            queue.wp = 0;  
        }  
        //元素个数加1  
        (queue.queueCnt)++;  
    }     
    return QueueOK;  
} 

static QueueStatus outQueue(uint8_t buff[])  
{  
    uint32_t i;
    if (1==isEmpty())  
    {  
        return QueueEmpty;  
    }  
    else 
    {  
        //读出数据
        for(i=0;i<BUFF_MAX_LENGTH;i++)
        {
            buff[i]=queue_data_buff[queue.rp][i];
        }
        //回转  
        if (++(queue.rp) == BUFF_MAX_SIZE)  
        {  
            queue.rp = 0;  
        }
        //元素个数减1 
        (queue.queueCnt)--;
    }     
    return QueueOK;  
}  

//启动ADC
void ADC1_Start(void)
{
//    static uint8_t adc_cycle_cnt=0;
//    
//    //6个周期启动一次
//    adc_cycle_cnt++;      
//    if(adc_cycle_cnt==7)    
//    {
//        adc_cycle_cnt=0;
//    }
//    if(adc_cycle_cnt==0)  
//    {
//            
////      HAL_ADC_Stop_DMA(&hadc1); 
////      HAL_ADC_Start_DMA(&hadc1,adc_converted_value,ADC_DMA_BUFF_LEN);    //启动函数     //IF_MAX是4个周期  
//      hadc1.Instance->CR2 |= ADC_CR2_CONT;   
//      hadc1.Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART; 
//        //HAL_ADC_Start(&hadc1);

//            
//      
//    }    
}  
#define TEMP_LOOP   (3000)
extern volatile uint8_t FullCircle_flag,flag_step_dp;
uint16_t temp_user_10;

extern uint16_t temp_out_value; 
int32_t  a_x=167,b_x=19172,c_x=22858;
int32_t  a1_x=167,b1_x=19172,c1_x=22858;
/**
*********************************************************************************************************
* @名称	: 
* @描述	:模式00 自动调节DP值  模式0x88根据温度调节DP
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
int32_t Read_ADC_Calculate_Average_Value(void)
{
	uint16_t *psrc_buff=NULL;
	int32_t *pdest_buff;	
	uint16_t i=0;	
	int32_t sum=0;
		
	if(adc_converted_value_num==0)
	{
		psrc_buff=(uint16_t *)adc_converted_value_0.dat;
		adc_converted_value_num=1;
	}
	else
	{
		psrc_buff=(uint16_t *)adc_converted_value_1.dat;
		adc_converted_value_num=0;
	}    
	pdest_buff=temp_buff;
	for(i=0;i<TEMP_SIGNAL_MAX;i++)
	{
		*pdest_buff++=*psrc_buff++;    //复制数据
	}
	//求平均
	for(i=100;i<TEMP_SIGNAL_MAX;i++)
	{
		sum+=temp_buff[i];
	}
	sum= sum/(TEMP_SIGNAL_MAX-100); 
 	return sum;
}
/**
*********************************************************************************************************
* @名称	: 
* @描述	:通过查表的方式换算成实际温度
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
int32_t Query_Table_Calculate_Temperature(uint16_t vol_ad)
{
	uint16_t i=0;
	DOT dot0,dot1,dot2;	
	int32_t calculate_temperature;
	/*判断输出参数范围*/ 
	if(vol_ad>=temp_ad[0])
	{
			calculate_temperature=-1500;
	}    
	else if(vol_ad<=temp_ad[99])
	{
			calculate_temperature=8400;
	}
	else
	{    
		for(i=0;i<99;i++)
		{
			/*查表计算数据*/ 	
			if((vol_ad<=temp_ad[i])&&(vol_ad>temp_ad[i+1]))
			{
				 
				dot1.x=temp_ad[i];
				dot1.y=((int16_t)(i)+(-15))*100;
				dot2.x=temp_ad[i+1];
				dot2.y=((int16_t)(i+1)+(-15))*100;
				dot0.x=vol_ad;
				dot0.y=LineInsert(dot1,dot2,dot0.x);
				calculate_temperature=dot0.y;
				break;                        
			}    
		}
	}
	return 	calculate_temperature;
}
/**
*********************************************************************************************************
* @名称	: 
* @描述	:自动调节DP 上报数据 寻找系数模式
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
void Adjust_DP_Seek_Coefficient(void)
{
	int16_t  delta_dp=0,DPset0=0;	
	static int8_t dp_cnt=0;	
	int16_t dp=0;
	/*两段内部调DP直线*/
	if(t_realtime<(t_25*100))
	{    
		delta_dp=(t_realtime-(t_25*100))*t_apd_vol_low/10000/100;  
	}    
	else
	{
		delta_dp=(t_realtime-(t_25*100))*t_apd_vol_high/10000/100; 
	}    
	DPset0= (int16_t)dp_value+delta_dp;
	/*每5圈  1秒进入一次调节计算  静态模式不用判断圈数*/
	if((FullCircle_flag>=5)||(work_mode==MODE_STATIC_SET_DP))
	{
		FullCircle_flag=0;
		/*调节10次后 进入10秒的保持阶段*/
		dp_cnt++;
		if(dp_cnt>=11)  //10S 阶梯 
		{
			dp_cnt=0;
			flag_step_dp=1;
		}
		/*调节DP 每次加3*/
		dp = DPset0 + (dp_cnt -5)*3;		
		if(dp<=0)
			dp=0;		
		DP_num = dp;
		dp_out_value=DP_num;
		/*判断DP调节使能   调节DP的PWM控制*/
		if(temp_cali_enable)
		{
			if((dp>=1)&&(dp<=900))
			{
					TIM5_PWM_SetDuty(dp);     
			}
		}
	}
}
/**
*********************************************************************************************************
* @名称	: 
* @描述	:自动调节DP 上报数据 寻找系数模式
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
void Adjust_DP_Static(void)
{
	int16_t  delta_dp=0,DPset0=0;	
	static int8_t dp_cnt=0;	
	int16_t dp=0;
	/*两段内部调DP直线*/
	if(t_realtime<(t_25*100))
	{    
		delta_dp=(t_realtime-(t_25*100))*t_apd_vol_low/10000/100;  
	}    
	else
	{
		delta_dp=(t_realtime-(t_25*100))*t_apd_vol_high/10000/100; 
	}    
	DPset0= (int16_t)dp_value+delta_dp;
	/*每5圈  1秒进入一次调节计算  静态模式不用判断圈数*/
	if((FullCircle_flag>=5)||(work_mode==MODE_STATIC_SET_DP))
	{
		FullCircle_flag=0;
		/*调节10次后 进入持阶段*/

		if(dp_cnt<=10)
		{
			/*调节DP 每次加3*/
			dp = DPset0 + (dp_cnt -5)*3;
		}	
		else
			dp = DPset0-15 ;
		dp_cnt++;
		/*调节10次 保持3次*/
		if(dp_cnt>12)
			dp_cnt=0;
		
		if(dp<=0)
			dp=0;		
		DP_num = dp;
		dp_out_value=DP_num;
		/*判断DP调节使能   调节DP的PWM控制*/
		if(temp_cali_enable)
		{
			if((dp>=1)&&(dp<=900))
			{
					TIM5_PWM_SetDuty(dp);     
			}
		}
	}
}
/**
*********************************************************************************************************
* @名称	: 
* @描述	:带入abc  a1b1c1计算DP 验证系数
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
void Verification_Coefficient(void)
{
	float temp=0.0,a=0.0,b=0.0,c=0.0;	
	uint16_t dp=0;
	if(t_realtime <= TEMP_LOOP)
	{							
		temp = (float)(t_realtime)/100;	
		a = (float)(a_x)/10000;
		b = (float)(b_x)/10000;
		c = (float)(c_x)/100;		
		dp = (uint16_t)(a*temp*temp + b*temp + c);
	}
	else
	{		
		temp = (float)(t_realtime)/100;	
		a = (float)(a1_x)/10000;
		b = (float)(b1_x)/10000;
		c = (float)(c1_x)/100;		
		dp = (uint16_t)(a*temp*temp + b*temp + c);			
	}		
	DP_num = dp;	
	dp_out_value=DP_num;	
	if(temp_cali_enable)
	{
		if((dp >= 1)&&(dp <= 900))
		{
				TIM5_PWM_SetDuty(dp);     
		}
	}
}
/**
*********************************************************************************************************
* @名称	: 
* @描述	: 生产温循工艺控制 不同模式进入不同工作状态
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
void Produce_Temperature_Control(void)
{
	/*进入调节DP找系数模式或者验证系数*/
	switch(work_mode)
	{
		/*找系数动态模式 */
		case MODE_NORMAL:
		case MODE_SAVE_DATA:	
			Adjust_DP_Seek_Coefficient();
		break;	
		/*找系数静态模式*/		
		case MODE_STATIC_SET_DP:
			Adjust_DP_Static();
		break;
		/*验证系数动态模式 和验证系数静态模式*/
		case MODE_VERIFICATION:
		case MODE_STATIC_ABC:
			Verification_Coefficient();
		break;			
		default:
		break;		
	}
}
/**
*********************************************************************************************************
* @名称	: 
* @描述	:模式00 自动调节DP值  模式0x88根据温度调节DP
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
void VAPD_Temperature_Adjust(void)
{
	static int32_t  t_realtime_last=-2000;
	/*判断DMA温度数据采样完成*/
    if(temp_flag)
    {
        temp_flag=0;
        /*停止ADC*/ 
        HAL_ADC_Stop(&hadc1);           
		/*通道恢复成采集信号的通道*/ 
        ADC1_SetChannel(SIGNAL_CHN);           
        /*读取ADC值 并计算平局值*/    
		apd_therm_vol_ad=Read_ADC_Calculate_Average_Value();
		/*通过查表的方式换算成实际温度 */  
		t_realtime=Query_Table_Calculate_Temperature(apd_therm_vol_ad); 		
        /*判断温度变化 不能超过2℃ */ 
        if(t_realtime_last>(-2000))
        {
            if(fabs((float)(t_realtime-t_realtime_last))>100)
            {
                t_realtime=t_realtime_last;
            }
            else
            {
                t_realtime=(t_realtime+t_realtime_last)>>1;
            }    
        } 
				
		/*将温度值复制到其他温度变量中*/		
        t_realtime_last=t_realtime; 	
        temp_out_value=t_realtime;
		temp_num = (int8_t)(t_realtime/100);
		temp_user_10=(int16_t)(t_realtime);
		/*温循控制*/
		Produce_Temperature_Control();	   
    } 
} 
/**
*********************************************************************************************************
* @名称	: 
* @描述	:模式00 自动调节DP值  模式0x88根据温度调节DP
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
int32_t Static_Read_ADC_Calculate_Average_Value(void)
{
	uint16_t *psrc_buff=NULL;
	int32_t *pdest_buff;	
	uint16_t i=0;	
	int32_t sum=0;
		

	psrc_buff=(uint16_t *)adc_converted_value_1.dat; 
	pdest_buff=temp_buff;
	for(i=0;i<TEMP_SIGNAL_MAX;i++)
	{
		*pdest_buff++=*psrc_buff++;    //复制数据
	}
	//求平均
	for(i=100;i<TEMP_SIGNAL_MAX;i++)
	{
		sum+=temp_buff[i];
	}
	sum= sum/(TEMP_SIGNAL_MAX-100);  
    return  sum;	
}
/**
*********************************************************************************************************
* @名称	: 
* @描述	:静态上报DP和距离
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
void VAPD_Temp_Static(void)
{
	static int32_t  t_realtime_last=-2000;
	/*判断DMA温度数据采样完成*/
    if(temp_flag)
    {
        temp_flag=0;
        /*停止ADC*/ 
        HAL_ADC_Stop(&hadc1);           
		/*通道恢复成采集信号的通道*/ 
        ADC1_SetChannel(SIGNAL_CHN);           
        /*读取ADC值 并计算平局值               静态*/    
		apd_therm_vol_ad=Static_Read_ADC_Calculate_Average_Value();
		/*通过查表的方式换算成实际温度 */  
		t_realtime=Query_Table_Calculate_Temperature(apd_therm_vol_ad);         
        /*判断温度变化 不能超过2℃ */ 
        if(t_realtime_last>(-2000))
        {
            if(fabs((float)(t_realtime-t_realtime_last))>100)
            {
                t_realtime=t_realtime_last;
            }
            else
            {
                t_realtime=(t_realtime+t_realtime_last)>>1;
            }    
        }    
        t_realtime_last=t_realtime; 
		/*将温度值复制到其他温度变量中*/
        temp_out_value=t_realtime;
  		temp_num = (int8_t)(t_realtime/100);
		temp_user_10=(int16_t)(t_realtime);
		/*温循控制*/
		Produce_Temperature_Control();	        
    } 
}

/*-------------------------------------------*/
//温度校准部分函数
void Temp_senddata(void)
{

    uint8_t *p=cmd_data_buff;


    *p++=0xef;
    *p++=0xef;
    //组帧，发送
    *p++=apd_therm_vol_ad>>8;   
    *p++=apd_therm_vol_ad;
    
    *p++=distance_realtime>>8;   //距离
    *p++=distance_realtime;
    
    *p++=intensity_realtime;

    while(!__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)){;}       
    HAL_UART_Transmit_DMA(&huart1,cmd_data_buff,7); 
} 

uint16_t distance_out_value;
uint16_t temp_out_value; 
void Temp_GetPosition(void)
{
    uint32_t i=0;
    uint16_t delta=0,direct_current=0;

 
    
    uint16_t *psrc_buff=NULL;
    int32_t *pdest_buff;    
    
    if(adc_cplt_flag)
    {
        adc_cplt_flag=0;
        

            psrc_buff=(uint16_t *)adc_converted_value_0.dat;

  
        pdest_buff=adc_buff_add;
        //采样结束是高电平，意味着触发是上升沿，此时数据有效。
        //采样结束是低电平，意味着触发是“下降沿”，此时数据无效。
        if(adc_valid_flag)
        {    
            for(i=0;i<IF_MAX;i++)
            {
                *pdest_buff++=*psrc_buff++;    //复制数据
            }
        }
   
        distance_realtime=Calculate_Position(IF_MAX,adc_buff_add,&delta,&direct_current);
		/*距离为有效值时 重启计数器清零*/
		if(distance_realtime>0)
			restart_num=0;
			
        intensity_realtime=delta;
		intensity_out_value=intensity_realtime;
		distance_out_value=distance_realtime;
		
		/*  保存温度系数  静态测试 */
		//Save_Temperature_Coefficient_Handle(distance_realtime);
    }   
}

void Temp_GetTemp(void)
{
    uint32_t i=0;
    int32_t sum=0;
    uint16_t *psrc_buff=NULL;
    int32_t *pdest_buff;  
    

   
    if(temp_flag)
    {
        temp_flag=0;
        

            psrc_buff=(uint16_t *)adc_converted_value_1.dat;
  
  
        pdest_buff=temp_buff;
        for(i=0;i<TEMP_SIGNAL_MAX;i++)
        {
            *pdest_buff++=*psrc_buff++;    //复制数据
        }
        
        for(i=0;i<TEMP_SIGNAL_MAX;i++)
        {
            sum+=temp_buff[i];
        }
        sum= sum/TEMP_SIGNAL_MAX;
        apd_therm_vol_ad=sum;
              
    } 
}  


/*-------------------------------------------*/

//设置APD偏压，即PWM 占空比
void VPAD_SetPWMDuty(uint32_t duty)
{
    TIM2_PWM_SetDuty(duty);
}    





//GPIOA4用于测试刷新速度
void GPIOA4_Blink(void)
{


}

/*************************************************************************************
* 函数名称: LineInsert
*
* 功能描述: 线性插值,2点画线，根据第3点X计算Y
*				             
* 入口参数:  dot1，第一点；		注意：为避免第一点必须是左边的点
*			 dot2，第二点；
*			 x，   待计算点的X值	
* 出口参数:  
*
************************************************************************************/
int32_t LineInsert(DOT dot1, DOT dot2,int32_t x)
{
	float a,b;
	a=(dot2.y-dot1.y)/(dot2.x-dot1.x);
	b=dot1.y-a*dot1.x;
	return(int32_t)(a*x+b);
}
uint8_t static_data_buff[50]; 
void Static_senddata(void)
{
    uint32_t i=0;
    uint8_t *p=static_data_buff;
    uint32_t sum=0;

	*p++=0xef;
	*p++=0xef;
	//组帧，发送
	*p++=0;        //速度
	*p++=0; 
	*p++=0;;            //角度  

	*p++ =  distance_out_value>>8;
	*p++ =  distance_out_value;
	*p++ =  intensity_out_value;		

	*p++ =  dp_out_value>>8;
	*p++ =  dp_out_value;
	*p++ =  (int8_t)(temp_out_value/100);	

	*p++ =  temp_out_value>>8;
	*p++ =  temp_out_value;
	*p++ =  0;	
	
	for (i=0;i<3;i++)
		*p++=0;
	sum=0;
	for(i=0;i<23;i++)
	{
		sum+=cmd_data_buff[i];
	}   
	cmd_data_buff[23]=sum;
	if(sum==0xEF)
	{
		cmd_data_buff[22]++;
		cmd_data_buff[23]++;
	} 
				
    while(!__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)){;}       
    HAL_UART_Transmit_DMA(&huart1,static_data_buff,24); 
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

#include "arm_math.h"
#include "coeff.h"
#include "apd_temp_para.h"
#include "dis_para.h"

#define _2PI    (2*3.14152966F)
#define DIS_MAX_VALUE (15000U)
#define DIS_ANGLE_MAX (360U)

uint32_t adc_converted_value_num; //0或者1
ADC_CONVERTED_VALUE  adc_converted_value_0,adc_converted_value_1;


int32_t adc_buff_add[IF_MAX];
int32_t adc_buff[IF_MAX];
int32_t temp_buff[IF_MAX];

volatile uint8_t adc_cplt_flag=0;
volatile uint8_t adc_valid_flag=0;
volatile uint8_t adc_half_cplt_flag=0;

volatile uint8_t sample_channel=SIGNAL_CHN;
volatile uint8_t temp_flag;


volatile DIS_DTTA dis_buff[DIS_MAX];
volatile uint32_t dis_cnt;
volatile uint32_t dis_cnt_last;
volatile uint32_t dis_cnt_last_save;
__DIS_INT_ANG_DATA dis_int_angle_data[DIS_ANGLE_MAX+1];
extern volatile uint32_t motor_speed_counter;

extern volatile uint32_t angle_trig_flag,angle_trig_counter_record[30],angle_trig_cnt,angle_trig_cnt_save;
uint32_t angle_trig_cnt_save_copy;
extern volatile uint32_t motor_speed;
extern volatile uint32_t motor_speed_rt;
uint8_t debug_temp_num=0;
extern uint16_t restart_num;
extern uint8_t adc_error_ovr;

#define BUFF_MAX_SIZE       5           //队列长度为5        
#define BUFF_MAX_LENGTH     24          //宽度为24

uint8_t cmd_data_buff[100]={
                    0xEF,0xEF,                                      //帧头
                    0x00,0x05,                                      //转速
                    0x00,                                           //角度
                    0x00,0x01,0x20,
	                  0x00,0x01,0x20,
	                  0x00,0x01,0x20,   //（距离+反射率）*6
                    0x00,0x01,0x20,
	                  0x00,0x01,0x20,
	                  0x00,0x01,0x20,
                    0x00                                            //校验和
                };

uint8_t queue_data_buff[BUFF_MAX_SIZE][BUFF_MAX_LENGTH];                
uint8_t out_buff[BUFF_MAX_LENGTH]; 
float thershold_buff[15];            
Queue queue;
uint8_t send_flag=0;
                
uint32_t APD_circle_cnt,APD_high_cnt,APD_low_cnt;   

uint32_t temp_default_value=1798;//1.71V
uint32_t vapd_pwm_duty=150;//135//200//230;   

uint16_t apd_therm_vol_ad=1000;   //ad值  
int32_t  t_realtime=0;  
int16_t  temp_threshold=-1000;   //低温不调的阈值 ，默认-10度              
   
int16_t  DP_num=0;  
int8_t   temp_num=0;								
uint16_t apd_bias_vol_ad=0;    //ad值             

uint16_t distance_realtime=0;
uint16_t intensity_realtime=0;              

extern volatile uint8_t angle_num,angle_num_max;
uint8_t angle_num_copy,angle_num_max_copy;    
                
int16_t angle_offset=0;   //必须是6的整数倍                

int32_t b0=-1300; //初始截距，-1.5米。2019.9.29改为-1.8米。
int32_t a1=10000,b1=0,a2=10000,b2=0,a3=10000,b3=0,a4=10000,b4=0,thread_12=0,thread_23=0,thread_34=0;  

int32_t a_i=0,b_i=0,a_dc=0,b_dc=0;
int32_t a_t=0,t0=0;  

int16_t t_25=30;  
uint16_t t_apd_vol_high=28*1120,t_apd_vol_low=28*820;


extern uint16_t work_mode;                
extern volatile uint32_t angle_num_sum;

extern uint16_t cp_value,dp_value;

extern uint8_t temp_vol_flag;

uint16_t distance_intensity_thread=0;


uint16_t dis_1,dis_2,dis_3,dis_4,dis_5,inte_1,inte_2,inte_3,inte_4,inte_5;


//保存距离参数
unsigned short dis_cali[31]={
0,
9,
33,
87,
134,
185,
228,
286,
376,
466,
556,
646,
736,
826,
916,
1006,
1096,
1186,
1276,
1304,
1304,
1312,
1317,
1328,
1337,
1348,
1362,
1389,
1411,
1425,
1450
};

unsigned short dis_standard[31]={
1419,
1424,
1446,
1495,
43,
95,
139,
191,
280,
368,
456,
543,
631,
719,
807,
895,
982,
1070,
1190,
1215,
1221,
1230,
1240,
1257,
1274,
1293,
1313,
1334,
1354,
1378,
1398

};

//保存APD的温度参数
unsigned short temp_ad[100]={
3604,
3579,
3553,
3526,
3499,
3470,
3441,
3411,
3379,
3347,
3314,
3281,
3246,
3210,
3174,
3137,
3099,
3060,
3021,
2981,
2940,
2898,
2856,
2814,
2771,
2727,
2683,
2639,
2594,
2549,
2504,
2458,
2413,
2367,
2321,
2275,
2230,
2184,
2138,
2093,
2048,
2003,
1959,
1914,
1871,
1827,
1784,
1742,
1700,
1658,
1617,
1577,
1537,
1498,
1460,
1422,
1385,
1348,
1312,
1277,
1243,
1209,
1176,
1144,
1112,
1082,
1052,
1022,
993,
965,
938,
912,
886,
860,
836,
812,
788,
766,
744,
722,
701,
681,
661,
642,
624,
605,
588,
571,
554,
538,
523,
507,
493,
479,
465,
451,
438,
426,
413,
402
};

unsigned short temp_duty[100]={
270,
270,
270,
270,
270,
270,
270,
270,
270,
270,
270,
270,
270,
270,
270
};



static uint32_t isFull(void); 
static uint32_t isEmpty(void);
static QueueStatus inQueue(uint8_t buff[]);
static QueueStatus outQueue(uint8_t buff[]);

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
                
                
#define debug               0   //0转，1曲线
#define debug_send_distance 0   //1使能静止报距离

uint8_t temp_cali_enable=1;     //使能温度校准功能   
uint8_t dis_offset_enable=1;    //使能距离补偿功能


/* ADC1 init function */
void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2; //采样频率改为1.4MHz，12+3=15clock ，84M/4/15=1.4M
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_Ext_IT11;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;//DISABLE;//ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sample_channel=SIGNAL_CHN;    //采集距离信号
 
}

//设置ADC的采集通道
void ADC1_SetChannel(uint8_t chn)
{
    ADC_ChannelConfTypeDef sConfig;
    
    if(chn==TEMP_CHN)   //采集温度数据
    {    
        /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
        */
      sConfig.Channel = ADC_CHANNEL_6;
      sConfig.Rank = 1;
      sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }
      sample_channel=TEMP_CHN;    //采集温度
    }    
    else if(chn==SIGNAL_CHN)
    {
        /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
        */
      sConfig.Channel = ADC_CHANNEL_5;
      sConfig.Rank = 1;
      sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }
      sample_channel=SIGNAL_CHN;    //采集距离信号
    }
}    


void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    /**ADC1 GPIO Configuration    
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_5;//|GPIO_PIN_4
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral DMA init*/
  
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_NORMAL;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(ADC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */
   
  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC1 GPIO Configuration    
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4|GPIO_PIN_5);

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(adcHandle->DMA_Handle);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(ADC_IRQn);

  }
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
} 

/* USER CODE BEGIN 1 */

//回调函数
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
//    hadc1.Instance->CR2 &= ~ADC_CR2_CONT;   //停止ADC 的连续转换
//    //HAL_ADC_Stop(&hadc1);
//    adc_half_cplt_flag=1;
}  
//回调函数  只传递标志，不读数据
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
//    uint32_t i=0;
    
    if(sample_channel==SIGNAL_CHN)
    {
        adc_cplt_flag=1;    //每次都要计算距离，如果本次采集的是其它通道，则直接用上次的数据计算距离
        
        //采样结束是高电平，意味着触发是上升沿，此时数据有效。
        //采样结束是低电平，意味着触发是“下降沿”，此时数据无效。
        if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3))
        {
            adc_valid_flag=1;
        }
        else
        {
            adc_valid_flag=0;
        }
        /*--------------------------------*/  //50us
        //LD_PowerControl(1);
        //HAL_ADC_Stop(&hadc1);
        //MX_ADC1_Init();
        //HAL_ADC_Start_DMA(&hadc1,adc_converted_value,ADC_DMA_BUFF_LEN); 
        
        //angle_num_sum++;
        angle_num_copy=angle_num;
        angle_num_max_copy=angle_num_max;
        angle_trig_cnt_save_copy=angle_trig_cnt_save;
    }
    else if(sample_channel==TEMP_CHN)
    {
        temp_flag=1;
    }
     LD_PowerControl(1);   
} 

void DelayUS(uint32_t n)
{
    uint32_t i=0,j=0;
    for(i=0;i<n;i++)
    {
        for(j=0;j<18;j++){;}
    }
} 

//质心计算距离,约200us
//3个周期是480us，不能超过这个时间
//算法原则：可以少报，不能错报
static uint16_t Calculate_Position(uint32_t numSamples,int32_t data[],uint16_t *delta,uint16_t *d_current)
{
//    uint32_t i,j;
	uint32_t i;
    int32_t pos=0,pos_pre;
    uint64_t sum1=0,sum2=0;
    
    uint16_t duty;
//    uint32_t max,min,max_num_cnt,max_pos,low_thre,high_thre,start,end;
	 uint32_t max,min,max_pos,low_thre,high_thre,start,end;
    uint32_t amp_thread,amp_thread_2,amp_thread_3,amplitude,max_n,min_thre_2_n,min_thre_2_nmax,max_cnt[10];
    
//    uint32_t low_cnt=0,high_cnt=0;
    
//    DOT dot1,dot2;

    duty=numSamples/2;         //采样点数为2个周期
    /*------------------------------------------------------- */  
    //在两个周期内，搜索峰的位置，需要考虑可能有多个峰
    max=0;
    min=4096;
    max_pos=0;
    for(i=5;i<duty;i+=4)     //从第一个点开始，跳过有时会异常的第0个点
    {
        if(data[i]>max)
        {
            max=data[i];
            //max_pos=i;
        } 
        if(data[i]<min)
        {
            min=data[i];
        } 
    }
    
    amplitude=max-min;
    amp_thread=max-amplitude*1/2;  
    amp_thread_2=min+amplitude*1/4;
    *d_current=min;

    max=0;
    max_pos=0;
    max_n=0;
    min_thre_2_n=0;
    min_thre_2_nmax=0;
    for(i=5;i<duty;i+=2)
    {
        //找极大值
        if(data[i]>amp_thread)
        {
            if(data[i]>max)
            {
                max=data[i];
                max_pos=i;
            }       
        }
        else
        {
            if(data[i-2]>amp_thread)
            {    
                if(max_n<10)
                {    
                    max_cnt[max_n]=max_pos;
                    max_n++;
                    max=0;
                }
            }            
        }
        //找连续点数，数据在min ~ amp_thread_2 之间的连续点数要大于50
        if(data[i]<amp_thread_2)
        {    
            min_thre_2_n++;
        }
        else
        {
            if(min_thre_2_n>min_thre_2_nmax)
            {
                min_thre_2_nmax=min_thre_2_n;
            }    
            min_thre_2_n=0;
        }           
    }
    
    if(min_thre_2_n>min_thre_2_nmax)
    {
        min_thre_2_nmax=min_thre_2_n;
    }
//    if(min_thre_2_nmax<25)
//    {
//        *delta=0;
//        *d_current=0;
//        return (DIS_MAX_VALUE);
//    }
    
    if(max_n>2)     //每个周期最多2个峰，超过直接返回最大距离
    {
        *delta=0;
        *d_current=0;
        return (DIS_MAX_VALUE);
    }    
    
    //实际可能的峰个数，1、2、3、4；还需要判断峰在两个周期内的分布情况
    if((max_pos<10)||(max_pos>(duty-10)))
    {
        *delta=0;
        *d_current=0;
        return (DIS_MAX_VALUE);
    }    

    //前后各预留60个点，截取
    if(max_pos<=60)
    {
        low_thre=0;
    }
    else
    {
        low_thre=max_pos-60;
    }
    
    if(max_pos>=(duty-1-60))
    {
        high_thre=duty-1;
    }
    else
    {
        high_thre=max_pos+60;
    } 
 
/*------------------------------------------------------- */
    max=0;
    min=4095;
    for(i=low_thre;i<high_thre;i++)
    {
        data[i]=(data[i]+data[i+duty])>>1;      //两个周期累加平均
        data[i]=(data[i]+data[i+1])>>1;         //两点平滑
        if(data[i]>max)                         //在截取的数据段内再重新找最大值和最小值
        {
            max=data[i];
            max_pos=i;
        }
        if(data[i]<min)
        {
            min=data[i];
        }
    }
//    //min要大于两个端点值的最大值，避免出现峰左右两边不对称的情况
//    if(data[low_thre]>min)
//    {
//        min=data[low_thre];
//    }    
//    if(data[high_thre]>min)
//    {
//        min=data[high_thre];
//    } 
/*-----------------------------------------------------------------*/ 
//    //计算直流量
//    j=0;
//    sum1=0;
//    for(i=2;i<low_thre;i++)
//    {
//        sum1+=data[i];
//        j++;
//    }
//    for(i=high_thre;i<duty;i++)
//    {
//        sum1+=data[i];
//        j++;
//    }
//    *d_current=sum1/j;
/*------------------------------------------------------- */  
    if((max>min)&&(max-min<20))   //峰峰值小于100 ，直接返回最大距离。
    {      
        *delta=0;
        *d_current=0;
        return (DIS_MAX_VALUE);
    }
/*-----------------------------------------------------------------*/
    //信号截取
    amp_thread_3=max-(max-min)*50/100;
    
    //amp_thread_3要大于两个端点值的最大值，避免出现峰左右两边不对称的情况
    if(data[low_thre]>amp_thread_3)
    {
        amp_thread_3=data[low_thre];
    }    
    if(data[high_thre]>amp_thread_3)
    {
        amp_thread_3=data[high_thre];
    }
    
    //质心  50us
    start=0;
    end=0;
    sum1=0;
    sum2=0;
    for(i=low_thre;i<=high_thre;i++)
    {
        if(data[i]>amp_thread_3)
        {    
            data[i]-=amp_thread_3;
            sum1+=data[i]*i;
            sum2+=data[i];
            if(start==0)
            {
                start=i;
            }
            else
            {
                end=i;
            }    
        }         
    }
    //判断宽度，限制在60个点
//    if((end-start)>60)  
//    {
//        *delta=0;
//        *d_current=0;
//        return (DIS_MAX_VALUE);
//    }    
    pos=(sum1*1000)/sum2; //计算距离  
    pos=(int64_t)pos*15345/duty/1000;      

    
    //强度压缩
    //*delta=sum2/300;  
    *delta=sum2/100;  //50
    
    /*-----------------------------------------------------------------*/ 
    //强度、距离修正
    if((*delta)<255)
    {    
        //pos=pos+((*d_current+a_t*((int32_t)apd_therm_vol_ad-t0)/10000-1100)*a_dc+b_dc+50)/10000+((*delta)*a_i+b_i+50)/100;
        //pos=pos+((*d_current+a_t*((int32_t)apd_therm_vol_ad-t0)/10000-1100)*a_dc+b_dc+50)/10000*10+((*delta)*a_i+b_i+50)/100*10;
        //pos=pos+((*d_current+a_t*((int32_t)apd_therm_vol_ad-t0)/10000-1100)*a_dc+b_dc+50)/1000+((*delta)*a_i+b_i+50)/10;
        pos=pos+((*d_current-1100)*b_i)/1000+((*delta)*a_i)/10;
    }
    else
    {    
        //pos=pos+((*d_current+a_t*((int32_t)apd_therm_vol_ad-t0)/10000-1100)*a_dc+b_dc+50)/1000;
        pos=pos+((*d_current-1100)*b_dc)/1000+((*delta)*a_dc)/10;
    }
    
    
    //温度修正 2019.3.21
    pos=pos+a_t*((int32_t)apd_therm_vol_ad)/10000;
    
/*-----------------------------------------------------------------*/
    //强度范围限制
    if(*delta>255)
    {
        *delta=255;
    }   
/*-----------------------------------------------------------------*/    
    pos_pre=pos;
    //距离补偿，乘以斜率，加上截距
    if(dis_offset_enable)
    {    
        if(pos_pre>(thread_12*10))
        {    
            pos= (pos*a1+b1*1000)/10000;
        }
        else if(pos_pre>(thread_23*10))
        {
            pos= (pos*a2+b2*1000)/10000;
        } 
        else if(pos_pre>(thread_34*10))
        {
            pos= (pos*a3+b3*1000)/10000;
        }
        else
        {
            pos= (pos*a4+b4*1000)/10000;
        }    
    }
    
    pos+=b0;        //加上初始截距
    
    if((pos<0)||(pos>DIS_MAX_VALUE))
    {
        *delta=0;
        *d_current=0;
        return (DIS_MAX_VALUE);
    } 
    
    
    
//高反修正    
//    if((pos>500)&&(pos<1000))
//    {
//        if(*delta>200)
//        {
//            pos=pos-((*delta)*(-0.066)-74.66);
//        }    
//    }    
//    else
//    {
//        if(*delta>150)
//        {
//            pos=pos-((*delta)*(-0.066)-74.66);
//        }    
//    }    
    
    
    
    if(pos>9000) //8200
    {
        pos=DIS_MAX_VALUE;
    } 
    
    //距离强度限制 
    if((pos<dis_1)&&(pos>=dis_2))
    {
        if(*delta<inte_1)
        {
            *delta=0;
            *d_current=0;
            return (DIS_MAX_VALUE);
        }    
    }
    else if((pos<dis_2)&&(pos>=dis_3))
    {
        if(*delta<inte_2)
        {
            *delta=0;
            *d_current=0;
            return (DIS_MAX_VALUE);
        }    
    }
    else if((pos<dis_3)&&(pos>=dis_4))
    {
        if(*delta<inte_3)
        {
            *delta=0;
            *d_current=0;
            return (DIS_MAX_VALUE);
        }    
    }
    else if((pos<dis_4)&&(pos>=dis_5))
    {
        if(*delta<inte_4)
        {
            *delta=0;
            *d_current=0;
            return (DIS_MAX_VALUE);
        }    
    }
    else if(pos<dis_5)
    {
        if(*delta<inte_5)
        {
            *delta=0;
            *d_current=0;
            return (DIS_MAX_VALUE);
        }    
    }
    
    if(pos<3000)
    {    
        if(min_thre_2_nmax<25)
        {
            *delta=0;
            *d_current=0;
            return (DIS_MAX_VALUE);
        }
    }
    
    return (pos);
}
/**
*********************************************************************************************************
* @名称	: 
* @描述	: 
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/

uint16_t intensity_out_value; 
int16_t dp_out_value;
maxmin_t  user_maxmin;
/**
*********************************************************************************************************
* @名称	: 
* @描述	: user_maxmin.min   0为最小
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
void Save_Maxmin_Min(uint16_t distance_realtime)
{
	uint16_t data_buff[MAXMIN_SIZE+1],temp;
	uint8_t i,j;
	//将最小值复制到maxmin数值0~9 处   最新值复制到10
	memcpy(data_buff,&user_maxmin.min[0],20);
	data_buff[MAXMIN_SIZE]=distance_realtime;
	
   //冒泡排序 0~10   0为最小 
	for(j=0;j<10;j++)
	{
		for(i=0;i<10-j;i++)
		{
			if(data_buff[ i ] > data_buff[i + 1])
			{
				temp = data_buff[i];
				data_buff[i] = data_buff[i + 1];
				data_buff[i + 1] = temp;
			}
		}
	}
	//将1~10数据复制到 maxmin数值
	memcpy(&user_maxmin.min[0],&data_buff[0],20);
}
/**
*********************************************************************************************************
* @名称	: 
* @描述	: user_maxmin.max   0为最大
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
void Save_Maxmin_Max(uint16_t distance_realtime)
{
	uint16_t data_buff[MAXMIN_SIZE+1],temp;
	uint8_t i,j;
	//将最小值复制到maxmin数值0~9 处   最新值复制到10
	memcpy(data_buff,&user_maxmin.max[0],20);
	data_buff[MAXMIN_SIZE]=distance_realtime;
	
   //冒泡排序 0~10   0为最大
	for(j=0;j<10;j++)
	{
		for(i=0;i<10-j;i++)
		{
			if(data_buff[ i ] < data_buff[i + 1])
			{
				temp = data_buff[i];
				data_buff[i] = data_buff[i + 1];
				data_buff[i + 1] = temp;
			}
		}
	}
	//将1~10数据复制到 maxmin数值
	memcpy(&user_maxmin.max[0],&data_buff[0],20);
	
	
}
/**
*********************************************************************************************************
* @名称	: 
* @描述	: 
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
 void Temperature_Coefficient_Find_Maxmin(uint16_t distance_realtime)
{
	//测量值小于 MIN
	if(distance_realtime<user_maxmin.min[MAXMIN_SIZE-1])
	{
		Save_Maxmin_Min(distance_realtime);
	}
	//测量值小于 MAX	
	else if((distance_realtime>user_maxmin.max[MAXMIN_SIZE-1])&&(distance_realtime!=DIS_MAX_VALUE))
	{
		Save_Maxmin_Max(distance_realtime);
	}
}

uint8_t temperature_coefficient_state=PREHEAT_STATE;
/**
*********************************************************************************************************
* @名称	: 
* @描述	: 
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/

uint16_t test_max_data;
uint16_t test_min_data;
uint8_t  temperature_coefficient_save_flag=0;
uint16_t temperature_coefficient_save_num=0;
uint16_t thershold_buff_calc[15];
uint16_t  testttt[15];
void Calc_Thershold_Handle(void)
{
	float max,min;
	float  space;
	uint8_t i;
	max=0;
	min=0;
	//提取max  min后5组数据
	for(i=5;i<=9;i++)
		max += user_maxmin.max[i];
	max=max/5;
	for(i=5;i<=9;i++)
		min += user_maxmin.min[i];
	min=min/5;
	space=(max-min)/11;
	for(i=0;i<10;i++)
	{
		thershold_buff[i]=min+space*(i+1);
		thershold_buff_calc[i]=(uint16_t)thershold_buff[i];
	}
	
	
	test_max_data=max;
	test_min_data=min;
	//切换运行模式
	temperature_coefficient_state=SAVE_DATA_STATE;
	temperature_coefficient_save_num=0;
}
/**
*********************************************************************************************************
* @名称	: 
* @描述	: 
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
temperature_coefficient_group_t user_temperature_coefficient;

void Save_Data_Handle(uint16_t distance_realtime)
{
	uint8_t i;
	int8_t  temperature_value; 
	int8_t  save_handle_flag_buff[10];
	/* 清零 */	
	memset(save_handle_flag_buff,0,sizeof(save_handle_flag_buff));
	/*根据测量值匹配阈值*/	
	for(i=0;i<10;i++)
	{
		if(thershold_buff[i]>3)
		{
			if( ( distance_realtime >= ( thershold_buff[i]-5 ) )&&
				( distance_realtime <= ( thershold_buff[i]+5 ) ) )
				save_handle_flag_buff[i]=1;
		}
	}
	/* 提取温度*/
	temperature_value=(temp_num+15);
	/* 将数据保存到所有符合阈值调节的缓存中   */	
	for(i=0;i<10;i++)
	{
		if((save_handle_flag_buff[i]==1)&&(temperature_value>=0)&&(temperature_value<70))
		{
			user_temperature_coefficient.group[i].data[temperature_value].dis=distance_realtime;
			user_temperature_coefficient.group[i].data[temperature_value].dp=dp_out_value;
			user_temperature_coefficient.group[i].data[temperature_value].intensity=intensity_out_value; 
			user_temperature_coefficient.group[i].data[temperature_value].temp=temp_num;
		}
	}
	/* 保存参数到FLASH */
	if(temperature_coefficient_save_flag==1)
	{
		temperature_coefficient_save_flag=0;
		Temperature_Coefficient_Flash_Write(); 
		/*读取参数用于测试观察*/
		Temperature_Coefficient_Flash_Read() ;
	}
}
/**
*********************************************************************************************************
* @名称	: 
* @描述	: 
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
uint8_t temperature_coefficient_handle_flag=0;

void Save_Temperature_Coefficient_Handle(uint16_t distance_realtime)
{
	debug_temp_num++;
	switch(temperature_coefficient_state)
	{
		/* 预热 不进行操作*/
		case PREHEAT_STATE:
		break;
		/* 寻找最大最小值 */
		case FIND_MIN_MAX_STATE:
			Temperature_Coefficient_Find_Maxmin(distance_realtime);
		break;
		/* 计算10组阈值*/
		case CALC_THRESHOLD_STATE:
			Calc_Thershold_Handle();
		break;
		/*  保存数据*/
		case SAVE_DATA_STATE:
			Save_Data_Handle(distance_realtime);		
		break;
		default:
		break;
	}
}
	
uint16_t angle_max=0;
//存储距离的计算结果
void GetPosition(void)
{
//    uint32_t i=0,j=0;
	uint32_t i=0;
    uint16_t delta=0,direct_current=0;
    uint16_t temp=0;
	uint16_t current_angle=0;
 
    
    uint16_t *psrc_buff=NULL;
    int32_t *pdest_buff;    
    
    if(adc_cplt_flag)
    {
        adc_cplt_flag=0;
        
        if(adc_converted_value_num==0)
        {
            psrc_buff=(uint16_t *)adc_converted_value_0.dat;
            adc_converted_value_num=1;
        }
        else
        {
            psrc_buff=(uint16_t *)adc_converted_value_1.dat;
            adc_converted_value_num=0;
        }    
        pdest_buff=adc_buff_add;
        //采样结束是高电平，意味着触发是上升沿，此时数据有效。
        //采样结束是低电平，意味着触发是“下降沿”，此时数据无效。
        if(adc_valid_flag)
        {    
            for(i=0;i<IF_MAX;i++)
            {
                *pdest_buff++=*psrc_buff++;    //复制数据
            }
        }
        
        { 

            temp=angle_standard[angle_trig_cnt_save_copy]+(angle_num_max_copy-angle_num_copy)+1;
            temp-=8;
			current_angle=temp;
            //角度取值1~360
            dis_buff[temp].trig_angle=temp;
            angle_max=temp;
            if(((angle_trig_cnt_save_copy==29)&&(angle_num_copy==0)) || ((angle_trig_cnt_save_copy==0)&&(angle_num_copy>=5)))  //已经走完最后一个齿的最后一个角度，将角度最大值设为360度 
            {
                angle_max=360;
            }
    
			distance_realtime=Calculate_Position(IF_MAX,adc_buff_add,&delta,&direct_current);
			dis_buff[temp].dis=distance_realtime; 
			/*距离为有效值时 重启计数器清零*/
			if(distance_realtime>0)
				restart_num=0;
			
			/* 当计算到角度1时 保存温循数据 */
			if(current_angle==1)
			{
				//temperature_coefficient_handle_flag=0;
				if(work_mode == MODE_SAVE_DATA)
					Save_Temperature_Coefficient_Handle(dis_buff[1].dis);
				
			}
			
			intensity_realtime=delta;
			intensity_out_value=intensity_realtime;
			dis_buff[temp].intensity=intensity_realtime;
			dis_buff[temp].DPset = DP_num;
			dis_buff[temp].Therm = temp_num;
						
            if(work_mode==0x81)
            {    
                printf("%d;%d;%d;%d\r",dis_buff[temp].dis,dis_buff[temp].intensity,direct_current,t_realtime);   //耗时700us
            }           
            GPIOA4_Blink();     //输出距离的计算频率            
        }
    } 
		
}

extern uint32_t apd_duty;
extern uint16_t temp_user_10;
//计算每一度的距离数据，每满6度发送一帧    发送数据采用度脉冲方式
uint32_t GetDistanceAndAngle(void)
{
    uint32_t i=0,j=0;
    uint8_t *p=cmd_data_buff;
//    static uint32_t i_static=0;
    static uint16_t angle=1,angle_max_copy;
    int16_t angle_send=0,temp=0;
    uint32_t sum=0;
  
    if(angle_max_copy  == angle_max)
    {
        return 0;
    }    
    angle_max_copy=angle_max;   

    for(;angle<=angle_max_copy;)
    {   
        if((angle>=6)&&((angle%6)==0))       //每6度发一次
        {
            {
                *p++=0xef;
                *p++=0xef;
                //组帧，发送
                *p++=motor_speed>>8;        //速度
                *p++=motor_speed;            
                //对角度进行修正180度
                angle_send=angle;
                angle_send-=240;
                angle_send-=angle_offset;
                *p++=(angle-6)/6;;            //角度  
                //测6次相位数据，距离+反射率 
                for(j=1;j<=6;j++)  
                {
                    temp=angle_send+j;
                    if(temp>360)
                    {
                        temp-=360;
                    }
                    else if(temp<=0)
                    {
                        temp+=360;
                    } 
                    									
                    if(angle==6 && j==2)
					{
						*p++ =  dis_buff[temp].DPset>>8;
						*p++ =  dis_buff[temp].DPset;					
						*p++ =  (int8_t)(dis_buff[temp].Therm);	
						
//						*p++ =  thershold_buff_calc[0]>>8;
//						*p++ =  thershold_buff_calc[0];
//						
//						*p++=debug_temp_num; //处理计数						
					}
					else
					{
						if(j==3)
						{
							*p++=temp_user_10>>8;   //距离
							*p++=temp_user_10;
							
							*p++=0; 												
						}
						else
						{
							*p++=dis_buff[temp].dis>>8;   //距离
							*p++=dis_buff[temp].dis;
							*p++=dis_buff[temp].intensity; //反射率
							
					          ///清空距离数组
							dis_buff[temp].dis=DIS_MAX_VALUE;
						}
					}
                }
                sum=0;
                for(i=0;i<23;i++)
                {
                    sum+=cmd_data_buff[i];
                }   
                cmd_data_buff[23]=sum;
                if(sum==0xEF)
                {
                    cmd_data_buff[22]++;
                    cmd_data_buff[23]++;
                } 
                angle++;
#if 1           
                inQueue(cmd_data_buff);
                break;
#else        
                while(!__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)){;}       
                HAL_UART_Transmit_DMA(&huart1,p,24); 
#endif                    
            }
        }
        else
        {
            angle++;
        }    
    }
    if(angle>360)      //已发送完最后一帧
    {
        angle=1;
//        i_static=0;
        
    }
    return 1;
}   

void EnableSendData(void)
{
    send_flag=1;
}
    
void DisableSendData(void)
{
    send_flag=0;
}    

void SendDistanceData(void)
{   
    if(!isEmpty())
    {
        //while(!__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)){;}
        if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC))
        {
            outQueue(out_buff);
            HAL_UART_Transmit_DMA(&huart1,out_buff,BUFF_MAX_LENGTH); 
        }    
    }
}    

//队列管理
QueueStatus init_queue(void)  
{  
    //初始化读写指针  
    queue.wp = 0;
    queue.rp = 0;  
    //初始化队列中元素个数  
    queue.queueCnt = 0; // conuter  
    
    return QueueOK;
}  

static uint32_t isFull(void)  
{  
    return (queue.queueCnt == BUFF_MAX_SIZE ) ? 1 : 0;  
}  

static uint32_t isEmpty(void)  
{  
    return (0==queue.queueCnt)? 1 : 0;  
} 

static QueueStatus inQueue(uint8_t buff[])  
{  
    uint32_t i;
    if (1==isFull())  
    {  
        return QueueFull;  
    }  
    else   
    {  
        //保存数据
        for(i=0;i<BUFF_MAX_LENGTH;i++)
        {
            queue_data_buff[queue.wp][i]=buff[i];
        }
        //回转  
        if (++(queue.wp) == BUFF_MAX_SIZE)  
        {  
            queue.wp = 0;  
        }  
        //元素个数加1  
        (queue.queueCnt)++;  
    }     
    return QueueOK;  
} 

static QueueStatus outQueue(uint8_t buff[])  
{  
    uint32_t i;
    if (1==isEmpty())  
    {  
        return QueueEmpty;  
    }  
    else 
    {  
        //读出数据
        for(i=0;i<BUFF_MAX_LENGTH;i++)
        {
            buff[i]=queue_data_buff[queue.rp][i];
        }
        //回转  
        if (++(queue.rp) == BUFF_MAX_SIZE)  
        {  
            queue.rp = 0;  
        }
        //元素个数减1 
        (queue.queueCnt)--;
    }     
    return QueueOK;  
}  

//启动ADC
void ADC1_Start(void)
{
//    static uint8_t adc_cycle_cnt=0;
//    
//    //6个周期启动一次
//    adc_cycle_cnt++;      
//    if(adc_cycle_cnt==7)    
//    {
//        adc_cycle_cnt=0;
//    }
//    if(adc_cycle_cnt==0)  
//    {
//            
////      HAL_ADC_Stop_DMA(&hadc1); 
////      HAL_ADC_Start_DMA(&hadc1,adc_converted_value,ADC_DMA_BUFF_LEN);    //启动函数     //IF_MAX是4个周期  
//      hadc1.Instance->CR2 |= ADC_CR2_CONT;   
//      hadc1.Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART; 
//        //HAL_ADC_Start(&hadc1);

//            
//      
//    }    
}  
#define TEMP_LOOP   (3000)
extern volatile uint8_t FullCircle_flag,flag_step_dp;
uint16_t temp_user_10;

extern uint16_t temp_out_value; 
int32_t  a_x=167,b_x=19172,c_x=22858;
int32_t  a1_x=167,b1_x=19172,c1_x=22858;
/**
*********************************************************************************************************
* @名称	: 
* @描述	:模式00 自动调节DP值  模式0x88根据温度调节DP
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
int32_t Read_ADC_Calculate_Average_Value(void)
{
	uint16_t *psrc_buff=NULL;
	int32_t *pdest_buff;	
	uint16_t i=0;	
	int32_t sum=0;
		
	if(adc_converted_value_num==0)
	{
		psrc_buff=(uint16_t *)adc_converted_value_0.dat;
		adc_converted_value_num=1;
	}
	else
	{
		psrc_buff=(uint16_t *)adc_converted_value_1.dat;
		adc_converted_value_num=0;
	}    
	pdest_buff=temp_buff;
	for(i=0;i<TEMP_SIGNAL_MAX;i++)
	{
		*pdest_buff++=*psrc_buff++;    //复制数据
	}
	//求平均
	for(i=100;i<TEMP_SIGNAL_MAX;i++)
	{
		sum+=temp_buff[i];
	}
	sum= sum/(TEMP_SIGNAL_MAX-100); 
 	return sum;
}
/**
*********************************************************************************************************
* @名称	: 
* @描述	:通过查表的方式换算成实际温度
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
int32_t Query_Table_Calculate_Temperature(uint16_t vol_ad)
{
	uint16_t i=0;
	DOT dot0,dot1,dot2;	
	int32_t calculate_temperature;
	/*判断输出参数范围*/ 
	if(vol_ad>=temp_ad[0])
	{
			calculate_temperature=-1500;
	}    
	else if(vol_ad<=temp_ad[99])
	{
			calculate_temperature=8400;
	}
	else
	{    
		for(i=0;i<99;i++)
		{
			/*查表计算数据*/ 	
			if((vol_ad<=temp_ad[i])&&(vol_ad>temp_ad[i+1]))
			{
				 
				dot1.x=temp_ad[i];
				dot1.y=((int16_t)(i)+(-15))*100;
				dot2.x=temp_ad[i+1];
				dot2.y=((int16_t)(i+1)+(-15))*100;
				dot0.x=vol_ad;
				dot0.y=LineInsert(dot1,dot2,dot0.x);
				calculate_temperature=dot0.y;
				break;                        
			}    
		}
	}
	return 	calculate_temperature;
}
/**
*********************************************************************************************************
* @名称	: 
* @描述	:自动调节DP 上报数据 寻找系数模式
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
void Adjust_DP_Seek_Coefficient(void)
{
	int16_t  delta_dp=0,DPset0=0;	
	static int8_t dp_cnt=0;	
	int16_t dp=0;
	/*两段内部调DP直线*/
	if(t_realtime<(t_25*100))
	{    
		delta_dp=(t_realtime-(t_25*100))*t_apd_vol_low/10000/100;  
	}    
	else
	{
		delta_dp=(t_realtime-(t_25*100))*t_apd_vol_high/10000/100; 
	}    
	DPset0= (int16_t)dp_value+delta_dp;
	/*每5圈  1秒进入一次调节计算  静态模式不用判断圈数*/
	if((FullCircle_flag>=5)||(work_mode==MODE_STATIC_SET_DP))
	{
		FullCircle_flag=0;
		/*调节10次后 进入10秒的保持阶段*/
		dp_cnt++;
		if(dp_cnt>=11)  //10S 阶梯 
		{
			dp_cnt=0;
			flag_step_dp=1;
		}
		/*调节DP 每次加3*/
		dp = DPset0 + (dp_cnt -5)*3;		
		if(dp<=0)
			dp=0;		
		DP_num = dp;
		dp_out_value=DP_num;
		/*判断DP调节使能   调节DP的PWM控制*/
		if(temp_cali_enable)
		{
			if((dp>=1)&&(dp<=900))
			{
					TIM5_PWM_SetDuty(dp);     
			}
		}
	}
}
/**
*********************************************************************************************************
* @名称	: 
* @描述	:自动调节DP 上报数据 寻找系数模式
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
void Adjust_DP_Static(void)
{
	int16_t  delta_dp=0,DPset0=0;	
	static int8_t dp_cnt=0;	
	int16_t dp=0;
	/*两段内部调DP直线*/
	if(t_realtime<(t_25*100))
	{    
		delta_dp=(t_realtime-(t_25*100))*t_apd_vol_low/10000/100;  
	}    
	else
	{
		delta_dp=(t_realtime-(t_25*100))*t_apd_vol_high/10000/100; 
	}    
	DPset0= (int16_t)dp_value+delta_dp;
	/*每5圈  1秒进入一次调节计算  静态模式不用判断圈数*/
	if((FullCircle_flag>=5)||(work_mode==MODE_STATIC_SET_DP))
	{
		FullCircle_flag=0;
		/*调节10次后 进入持阶段*/

		if(dp_cnt<=10)
		{
			/*调节DP 每次加3*/
			dp = DPset0 + (dp_cnt -5)*3;
		}	
		else
			dp = DPset0-15 ;
		dp_cnt++;
		/*调节10次 保持3次*/
		if(dp_cnt>12)
			dp_cnt=0;
		
		if(dp<=0)
			dp=0;		
		DP_num = dp;
		dp_out_value=DP_num;
		/*判断DP调节使能   调节DP的PWM控制*/
		if(temp_cali_enable)
		{
			if((dp>=1)&&(dp<=900))
			{
					TIM5_PWM_SetDuty(dp);     
			}
		}
	}
}
/**
*********************************************************************************************************
* @名称	: 
* @描述	:带入abc  a1b1c1计算DP 验证系数
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
void Verification_Coefficient(void)
{
	float temp=0.0,a=0.0,b=0.0,c=0.0;	
	uint16_t dp=0;
	if(t_realtime <= TEMP_LOOP)
	{							
		temp = (float)(t_realtime)/100;	
		a = (float)(a_x)/10000;
		b = (float)(b_x)/10000;
		c = (float)(c_x)/100;		
		dp = (uint16_t)(a*temp*temp + b*temp + c);
	}
	else
	{		
		temp = (float)(t_realtime)/100;	
		a = (float)(a1_x)/10000;
		b = (float)(b1_x)/10000;
		c = (float)(c1_x)/100;		
		dp = (uint16_t)(a*temp*temp + b*temp + c);			
	}		
	DP_num = dp;	
	dp_out_value=DP_num;	
	if(temp_cali_enable)
	{
		if((dp >= 1)&&(dp <= 900))
		{
				TIM5_PWM_SetDuty(dp);     
		}
	}
}
/**
*********************************************************************************************************
* @名称	: 
* @描述	: 生产温循工艺控制 不同模式进入不同工作状态
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
void Produce_Temperature_Control(void)
{
	/*进入调节DP找系数模式或者验证系数*/
	switch(work_mode)
	{
		/*找系数动态模式 */
		case MODE_NORMAL:
		case MODE_SAVE_DATA:	
			Adjust_DP_Seek_Coefficient();
		break;	
		/*找系数静态模式*/		
		case MODE_STATIC_SET_DP:
			Adjust_DP_Static();
		break;
		/*验证系数动态模式 和验证系数静态模式*/
		case MODE_VERIFICATION:
		case MODE_STATIC_ABC:
			Verification_Coefficient();
		break;			
		default:
		break;		
	}
}
/**
*********************************************************************************************************
* @名称	: 
* @描述	:模式00 自动调节DP值  模式0x88根据温度调节DP
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
void VAPD_Temperature_Adjust(void)
{
	static int32_t  t_realtime_last=-2000;
	/*判断DMA温度数据采样完成*/
    if(temp_flag)
    {
        temp_flag=0;
        /*停止ADC*/ 
        HAL_ADC_Stop(&hadc1);           
		/*通道恢复成采集信号的通道*/ 
        ADC1_SetChannel(SIGNAL_CHN);           
        /*读取ADC值 并计算平局值*/    
		apd_therm_vol_ad=Read_ADC_Calculate_Average_Value();
		/*通过查表的方式换算成实际温度 */  
		t_realtime=Query_Table_Calculate_Temperature(apd_therm_vol_ad); 		
        /*判断温度变化 不能超过2℃ */ 
        if(t_realtime_last>(-2000))
        {
            if(fabs((float)(t_realtime-t_realtime_last))>100)
            {
                t_realtime=t_realtime_last;
            }
            else
            {
                t_realtime=(t_realtime+t_realtime_last)>>1;
            }    
        } 
				
		/*将温度值复制到其他温度变量中*/		
        t_realtime_last=t_realtime; 	
        temp_out_value=t_realtime;
		temp_num = (int8_t)(t_realtime/100);
		temp_user_10=(int16_t)(t_realtime);
		/*温循控制*/
		Produce_Temperature_Control();	   
    } 
} 
/**
*********************************************************************************************************
* @名称	: 
* @描述	:模式00 自动调节DP值  模式0x88根据温度调节DP
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
int32_t Static_Read_ADC_Calculate_Average_Value(void)
{
	uint16_t *psrc_buff=NULL;
	int32_t *pdest_buff;	
	uint16_t i=0;	
	int32_t sum=0;
		

	psrc_buff=(uint16_t *)adc_converted_value_1.dat; 
	pdest_buff=temp_buff;
	for(i=0;i<TEMP_SIGNAL_MAX;i++)
	{
		*pdest_buff++=*psrc_buff++;    //复制数据
	}
	//求平均
	for(i=100;i<TEMP_SIGNAL_MAX;i++)
	{
		sum+=temp_buff[i];
	}
	sum= sum/(TEMP_SIGNAL_MAX-100);  
    return  sum;	
}
/**
*********************************************************************************************************
* @名称	: 
* @描述	:静态上报DP和距离
* @参数	: 
* @返回	: 
*********************************************************************************************************
**/
void VAPD_Temp_Static(void)
{
	static int32_t  t_realtime_last=-2000;
	/*判断DMA温度数据采样完成*/
    if(temp_flag)
    {
        temp_flag=0;
        /*停止ADC*/ 
        HAL_ADC_Stop(&hadc1);           
		/*通道恢复成采集信号的通道*/ 
        ADC1_SetChannel(SIGNAL_CHN);           
        /*读取ADC值 并计算平局值               静态*/    
		apd_therm_vol_ad=Static_Read_ADC_Calculate_Average_Value();
		/*通过查表的方式换算成实际温度 */  
		t_realtime=Query_Table_Calculate_Temperature(apd_therm_vol_ad);         
        /*判断温度变化 不能超过2℃ */ 
        if(t_realtime_last>(-2000))
        {
            if(fabs((float)(t_realtime-t_realtime_last))>100)
            {
                t_realtime=t_realtime_last;
            }
            else
            {
                t_realtime=(t_realtime+t_realtime_last)>>1;
            }    
        }    
        t_realtime_last=t_realtime; 
		/*将温度值复制到其他温度变量中*/
        temp_out_value=t_realtime;
  		temp_num = (int8_t)(t_realtime/100);
		temp_user_10=(int16_t)(t_realtime);
		/*温循控制*/
		Produce_Temperature_Control();	        
    } 
}

/*-------------------------------------------*/
//温度校准部分函数
void Temp_senddata(void)
{

    uint8_t *p=cmd_data_buff;


    *p++=0xef;
    *p++=0xef;
    //组帧，发送
    *p++=apd_therm_vol_ad>>8;   
    *p++=apd_therm_vol_ad;
    
    *p++=distance_realtime>>8;   //距离
    *p++=distance_realtime;
    
    *p++=intensity_realtime;

    while(!__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)){;}       
    HAL_UART_Transmit_DMA(&huart1,cmd_data_buff,7); 
} 

uint16_t distance_out_value;
uint16_t temp_out_value; 
void Temp_GetPosition(void)
{
    uint32_t i=0;
    uint16_t delta=0,direct_current=0;

 
    
    uint16_t *psrc_buff=NULL;
    int32_t *pdest_buff;    
    
    if(adc_cplt_flag)
    {
        adc_cplt_flag=0;
        

            psrc_buff=(uint16_t *)adc_converted_value_0.dat;

  
        pdest_buff=adc_buff_add;
        //采样结束是高电平，意味着触发是上升沿，此时数据有效。
        //采样结束是低电平，意味着触发是“下降沿”，此时数据无效。
        if(adc_valid_flag)
        {    
            for(i=0;i<IF_MAX;i++)
            {
                *pdest_buff++=*psrc_buff++;    //复制数据
            }
        }
   
        distance_realtime=Calculate_Position(IF_MAX,adc_buff_add,&delta,&direct_current);
		/*距离为有效值时 重启计数器清零*/
		if(distance_realtime>0)
			restart_num=0;
			
        intensity_realtime=delta;
		intensity_out_value=intensity_realtime;
		distance_out_value=distance_realtime;
		
		/*  保存温度系数  静态测试 */
		//Save_Temperature_Coefficient_Handle(distance_realtime);
    }   
}

void Temp_GetTemp(void)
{
    uint32_t i=0;
    int32_t sum=0;
    uint16_t *psrc_buff=NULL;
    int32_t *pdest_buff;  
    

   
    if(temp_flag)
    {
        temp_flag=0;
        

            psrc_buff=(uint16_t *)adc_converted_value_1.dat;
  
  
        pdest_buff=temp_buff;
        for(i=0;i<TEMP_SIGNAL_MAX;i++)
        {
            *pdest_buff++=*psrc_buff++;    //复制数据
        }
        
        for(i=0;i<TEMP_SIGNAL_MAX;i++)
        {
            sum+=temp_buff[i];
        }
        sum= sum/TEMP_SIGNAL_MAX;
        apd_therm_vol_ad=sum;
              
    } 
}  


/*-------------------------------------------*/

//设置APD偏压，即PWM 占空比
void VPAD_SetPWMDuty(uint32_t duty)
{
    TIM2_PWM_SetDuty(duty);
}    





//GPIOA4用于测试刷新速度
void GPIOA4_Blink(void)
{
    static uint8_t flag;
    flag=~flag;
    if(flag)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        }  

}

/*************************************************************************************
* 函数名称: LineInsert
*
* 功能描述: 线性插值,2点画线，根据第3点X计算Y
*				             
* 入口参数:  dot1，第一点；		注意：为避免第一点必须是左边的点
*			 dot2，第二点；
*			 x，   待计算点的X值	
* 出口参数:  
*
************************************************************************************/
int32_t LineInsert(DOT dot1, DOT dot2,int32_t x)
{
	float a,b;
	a=(dot2.y-dot1.y)/(dot2.x-dot1.x);
	b=dot1.y-a*dot1.x;
	return(int32_t)(a*x+b);
}
uint8_t static_data_buff[50]; 
void Static_senddata(void)
{
    uint32_t i=0;
    uint8_t *p=static_data_buff;
    uint32_t sum=0;

	*p++=0xef;
	*p++=0xef;
	//组帧，发送
	*p++=0;        //速度
	*p++=0; 
	*p++=0;;            //角度  

	*p++ =  distance_out_value>>8;
	*p++ =  distance_out_value;
	*p++ =  intensity_out_value;		

	*p++ =  dp_out_value>>8;
	*p++ =  dp_out_value;
	*p++ =  (int8_t)(temp_out_value/100);	

	*p++ =  temp_out_value>>8;
	*p++ =  temp_out_value;
	*p++ =  0;	
	
	for (i=0;i<3;i++)
		*p++=0;
	sum=0;
	for(i=0;i<23;i++)
	{
		sum+=cmd_data_buff[i];
	}   
	cmd_data_buff[23]=sum;
	if(sum==0xEF)
	{
		cmd_data_buff[22]++;
		cmd_data_buff[23]++;
	} 
				
    while(!__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)){;}       
    HAL_UART_Transmit_DMA(&huart1,static_data_buff,24); 
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
