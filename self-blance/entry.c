#include <stdlib.h>
#include <rtthread.h>
#include <components.h>


void rt_init_thread_entry(void* parameter)
{
    mpu6050_init();

    
}


int self_blance_entry()
{
    rt_thread_t tid;

    tid = rt_thread_create("init",
        rt_init_thread_entry, RT_NULL,
        2048, RT_THREAD_PRIORITY_MAX/3, 20);

    if (tid != RT_NULL)
        rt_thread_startup(tid);

    return 0;
}
