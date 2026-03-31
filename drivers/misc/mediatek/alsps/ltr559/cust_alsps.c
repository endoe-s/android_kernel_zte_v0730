#include <linux/types.h>
#include <cust_alsps.h>

u16 ltr559_als_raw2lux(u16 als)
{
    u16 lux = 0;

	if (als < 400)
    {
        lux = als * 4 / 10;
    }
	
    else if (als < 1500)
    {
        lux = als * 26 / 100 + 58;
    }
	else
	{
        lux = als * 2 / 10 + 150;
		//has defined LIGHT_RANGE = 10240.0f in hwmsen_chip_info.c
		if (lux > 10240)
		{
		    lux = 10240;
		}
	}

    return lux;
}
