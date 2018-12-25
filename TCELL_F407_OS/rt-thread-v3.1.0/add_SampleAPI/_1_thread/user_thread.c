#include "user_thread.h"

static rt_thread_t led2_thread = RT_NULL, led1_thread = RT_NULL;

static void led1_thread_entry(void *parameter)
{
	while(1)
	{
		rt_kprintf("led1_thread, led1 on\n");
		rt_thread_delay(500);
		
		rt_kprintf("led1_thread, led1 off\n");
		rt_thread_delay(500);
	}
}

static void led1_thread_cleanup(struct rt_thread *tid)
{
    if (tid != led1_thread)
    {
        return ;
    }
	
    rt_kprintf("led1_thread end\n");
}

static void led2_thread_entry(void *parameter)
{
	while(1)
	{
		rt_kprintf("led2_thread, led2 on\n");
		rt_thread_delay(250);
		
		rt_kprintf("led2_thread, led2 off\n");		
		rt_thread_delay(250);
	}
}

static void led2_thread_cleanup(struct rt_thread *tid)
{
    if (tid != led2_thread)
    {
        return ;
    }
	
    rt_kprintf("led2_thread end\n");
}

void user_thread_init(void)
{
	led1_thread = rt_thread_create("led1",
                                   led1_thread_entry,
								   RT_NULL,
								   512,
								   3,
								   20);
	
	if (led1_thread != RT_NULL)
    {
        led1_thread->cleanup = led1_thread_cleanup;
        rt_thread_startup(led1_thread);
    }
	
	led2_thread = rt_thread_create("led2",
                                   led2_thread_entry,
								   RT_NULL,
								   512,
								   4,
								   20);
	
	if (led2_thread != RT_NULL)
    {
        led2_thread->cleanup = led2_thread_cleanup;
        rt_thread_startup(led2_thread);
    }
}
