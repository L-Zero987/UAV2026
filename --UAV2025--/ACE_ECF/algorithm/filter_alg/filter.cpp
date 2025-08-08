/**
 * @file filter.cpp
 * @author Lann 梁健蘅 (rendezook@qq.com)
 * @brief 
 * @version 0.1
 * @date 2024-10-10
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "filter.hpp"
/**
 * @brief 二阶陷波滤波器结构体
 */

/**
 * @brief 初始化陷波滤波器参数
 * @param filter 滤波器对象
 * @param center_freq 需要滤除的中心频率（单位：Hz）
 * @param bandwidth   陷波带宽（单位：Hz）
 * @param sample_rate 采样频率（单位：Hz）
 */
void filter_alg_n::NotchFilter_Init(NotchFilter *filter,
                                    float center_freq,
                                    float bandwidth,
                                    float sample_rate)
{
  // 计算角频率参数
  float omega = 2 * PI * center_freq / sample_rate;
  float bw = 2 * PI * bandwidth / sample_rate;
  float alpha = tanf(bw / 2);

  // 计算极点/零点位置
  float cos_omega = cosf(omega);
  // float sin_omega = sinf(omega);

  // 计算滤波器系数
  float common_term = 1 / (1 + alpha);
  filter->b0 = common_term;
  filter->b1 = -2 * cos_omega * common_term;
  filter->b2 = common_term;

  filter->a1 = filter->b1;
  filter->a2 = (1 - alpha) * common_term;

  // 初始化状态变量
  filter->x1 = filter->x2 = 0;
  filter->y1 = filter->y2 = 0;
}

/**
 * @brief 执行陷波滤波
 * @param filter 滤波器对象
 * @param input 原始输入信号
 * @return 滤波后的输出信号
 */
float filter_alg_n::NotchFilter_Update(NotchFilter *filter, float input)
{
  // 计算当前输出
  float output = filter->b0 * input +
                 filter->b1 * filter->x1 +
                 filter->b2 * filter->x2 -
                 filter->a1 * filter->y1 -
                 filter->a2 * filter->y2;

  // 更新状态变量
  filter->x2 = filter->x1;
  filter->x1 = input;
  filter->y2 = filter->y1;
  filter->y1 = output;

  return output;
}

//----------------------------------------------------------------
// 使用示例（假设50Hz电源噪声需要滤除）
//----------------------------------------------------------------


// void setup()
// {
//   // 初始化滤波器参数
//   // 参数说明：中心频率=50Hz，带宽=5Hz，采样率=200Hz
//   NotchFilter_Init(&pitch_notch, 50.0f, 5.0f, 200.0f);
// }

/***************************一阶低通滤波********************************************** */
/**
 * @brief 一阶低通滤波初始化
 * @param num 一阶滤波系数
 * @note 滤波系数在 0~1.0 区间, 超出这个范围则默认为 1
 * @note 系数越小, 得到的波形越平滑, 但是也更加迟钝
*/
filter_alg_n::first_order_filter_c::first_order_filter_c(float num)
{
    if(num > 0.0 && num < 1.0)
    {
        this->measure.input = 0;
        this->measure.last_input = 0;
        this->measure.num = num;
        this->measure.out = 0;
    }
    else
    {
        this->measure.input = 0;
        this->measure.last_input = 0;
        this->measure.num = 1;
        this->measure.out = 0;
    }
}


/**
 * @brief 一阶低通滤波计算
 * @param input 采样值
 * @return 输出值
*/
float filter_alg_n::first_order_filter_c::first_order_filter(float input)        
{
  this->measure.input = input;
  this->measure.out = this->measure.input * this->measure.num + (1 - this->measure.num) * this->measure.last_input;
  this->measure.last_input = this->measure.out;

  return this->measure.out;
}

/**
 * @brief 一阶低通滤波初始化
 * @param num 一阶滤波系数
 * @note 滤波系数在 0~1.0 区间, 超出这个范围则默认为 1
 * @note 系数越小, 得到的波形越平滑, 但是也更加迟钝
*/
void filter_alg_n::first_order_filter_c::first_order_filter_init(float num)
{
    if(num > 0.0 && num < 1.0)
    {
        this->measure.input = 0;
        this->measure.last_input = 0;
        this->measure.num = num;
        this->measure.out = 0;
    }
    else
    {
        this->measure.input = 0;
        this->measure.last_input = 0;
        this->measure.num = 1;
        this->measure.out = 0;
    }
}


/**
 * @brief          一阶低通滤波清除
 * @retval         none
 * @attention      只是清除所有计算的数据，不会清除系数
 */
void filter_alg_n::first_order_filter_c::first_order_filter_clear(void)
{
    this->measure.last_input = 0;
    this->measure.input = 0;
    this->measure.out = 0;
}


/***************************滑动均值滤波**************************************** */

/**
 * @brief 滑动均值滤波初始化
 * 
 */
filter_alg_n::sliding_mean_filter_c::sliding_mean_filter_c()
{
    this->measure.count_num = 0;
    for (int i = 0; i < 20; i++)
        this->measure.FIFO[i] = 0.0f;
    this->measure.Input = 0.0f;
    this->measure.Output = 0.0f;
    this->measure.Sum = 0.0f;
    this->measure.sum_flag = 0;
}


/*
 *功能：滑动均值滤波参数初始化(浮点型)
 *输入：滤波对象结构体
 */
void filter_alg_n::sliding_mean_filter_c::sliding_mean_filter_init()
{
  this->measure.count_num = 0;
  for (int i = 0; i < 20; i++)
    this->measure.FIFO[i] = 0.0f;
  this->measure.Input = 0.0f;
  this->measure.Output = 0.0f;
  this->measure.Sum = 0.0f;
  this->measure.sum_flag = 0;
}

/*
 *功能：滑动均值滤波（浮点型）------抑制小幅度高频噪声
 *传入：1.滤波对象结构体  2.更新值 3.均值数量
 *传出：滑动滤波输出值（250次）
 */
float filter_alg_n::sliding_mean_filter_c::sliding_mean_filter(float Input, int num)
{
  // 更新
  this->measure.Input = Input;
  this->measure.FIFO[this->measure.count_num] = this->measure.Input;
  this->measure.count_num++;

  if (this->measure.count_num == num)
  {
    this->measure.count_num = 0;
    this->measure.sum_flag = 1;
  }
  // 求和
  if (this->measure.sum_flag == 1)
  {
    for (int count = 0; count < num; count++)
    {
      this->measure.Sum += this->measure.FIFO[count];
    }
  }
  // 均值
  this->measure.Output = this->measure.Sum / num;
  this->measure.Sum = 0;

  return this->measure.Output;
}
// 递推平均滤波初始化
float filter_alg_n::Recursive_ave_filter_init(Recursive_ave_filter_type_t *filter)
{
  filter->count_num = 0;
  filter->sum = 0;
  return 0;
}

/*
 *功能：递推平均滤波（浮点型）------抑制大幅度高频噪声
 *传入：1.滤波对象结构体  2.更新值 3.均值数量
 *传出：滑动滤波输出值（max = 100次）
 */
float filter_alg_n::Recursive_ave_filter(Recursive_ave_filter_type_t *filter, float input, int num)
{
  int i;
  for (i = num - 1; i > 0; i--)
  {
    filter->fifo[i] = filter->fifo[i - 1];
  }
  filter->fifo[0] = input;

  if (filter->count_num == num)
  {
    filter->count_num = 0;
  }

  filter->sum = 0;
  for (i = 0; i < num; i++)
  {
    filter->sum += filter->fifo[i];
  }

  return (filter->sum / num);
}