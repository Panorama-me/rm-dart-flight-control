#include "drivers_statistic.h"

/*不使用Init初始化时，Rate_Do_Execute数据来源为程序执行次数。
使用Init初始化时，该函数数据来源为伪系统时间(数据精度差,可能不精准)*/
void Statistic::Statistic_Init(uint8_t Control_Time_)
{
    Control_Time = Control_Time_;
}
//统计数据更新
void Statistic::Statistic_Update(uint32_t Time_)
{
    if(Start_Time == 0)
    {
        Start_Time = Time_;
    }

    Last_Time = Time;
    Time = Time_;
    Count++;
    FPS = 1 / (float)(Time - Last_Time) * 1000;
    FPS_Count = Count / (float)(Time - Start_Time) * 1000;
}
//到达指定周期返回true
bool Statistic::Rate_Do_Execute(uint16_t per)
{
    if(Control_Time != 0)
    {
        if((per != 0) && (Count % (uint8_t)(per / Control_Time) == 0))
        {
            return true;
        }
        return false;
    }
    else
    {
        if((per != 0) && (Count % per == 0))
        {
            return true;
        }
        return false;
    }
}

