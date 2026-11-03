#ifndef __DRIVERS_STATISTICS_H
#define __DRIVERS_STATISTICS_H

#ifdef __cplusplus
extern "C"
{
#endif

//#include "platform.h"
#include "stdint.h"

#ifdef __cplusplus
}
#endif
#include <cmath>

// 事件统计类（ms级）
class Statistic
{
public:
    Statistic() {}

    void Statistic_Init(uint8_t Control_Time_);
    void Statistic_Update(uint32_t Time_);

    bool Rate_Do_Execute(uint16_t per);
private:
    uint8_t Control_Time;
    uint32_t Count;

    uint32_t Time;
    uint32_t Start_Time;
    uint32_t Last_Time;

    float FPS;
    float FPS_Count;
};

extern "C++"
{
    template<typename T>
    struct is_pointer
    {
        static const bool value = false;
    };

    template<typename T>
    struct is_pointer<T *>
    {
        static const bool value = true;
    };

    template<typename T1, typename T2>
    struct is_same
    {
        static const bool value = false;
    };

    template<typename T>
    struct is_same<T, T>
    {
        static const bool value = true;
    };

    // 数据非法判断，包括空指针、nan、inf
    class ValidData
    {
    public:
        ValidData() {}
        //判断合法
        template <typename T>
        static bool IsValid(T arg)
        {
            if(is_pointer<T>::value && arg == 0)
            {
                return false;
            }
            uint8_t buf = 0;//buf用来规避编译时所报类型转换错误
            if(is_same<T, const void *>::value)
            {
                buf = 0;
            }
            else
            {
                T buf = arg;
            }
            if(isnan(buf) || isinf(buf))
            {
                return false;
            }
            return true;
        }
        //判断非法
        template <typename T>
        static bool IsInvalid(T arg)
        {
            return !IsValid(arg);
        }
    };
}
#endif
