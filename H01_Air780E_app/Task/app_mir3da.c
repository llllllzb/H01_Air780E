/******************** (C) COPYRIGHT 2018 MiraMEMS *****************************
* File Name     : mir3da.c
* Author        : ycwang@miramems.com
* Version       : V1.0
* Date          : 05/18/2018
* Description   : Demo for configuring mir3da
*******************************************************************************/
#include "app_mir3da.h"
#include "app_sys.h"
#include "app_port.h"
#include "app_param.h"
u8_m i2c_addr = 0x26;

s16_m offset_x=0, offset_y=0, offset_z=0;


s8_m mir3da_register_read(u8_m addr, u8_m *data_m, u8_m len)
{
    //To do i2c read api
    iicReadData(i2c_addr<<1, addr, data_m, len);
    //LogPrintf(DEBUG_ALL, "iicRead %s",ret?"Success":"Fail");
    return 0;

}

s8_m mir3da_register_write(u8_m addr, u8_m data_m)
{
    //To do i2c write api
    iicWriteData(i2c_addr<<1, addr, data_m);
    //LogPrintf(DEBUG_ALL, "iicWrite %s",ret?"Success":"Fail");
    return 0;

}

s8_m mir3da_register_mask_write(u8_m addr, u8_m mask, u8_m data)
{
    unsigned char tmp_data=0;
    mir3da_register_read(addr, &tmp_data, 1);
    tmp_data &= ~mask;
    tmp_data |= data & mask;
    mir3da_register_write(addr, tmp_data);
	
	return 0;
}


//Initialization
s8_m mir3da_init(void)
{
	s8_m res = 0;
	u8_m data_m = 0;

	res = mir3da_register_read(REG_CHIP_ID,&data_m,1);
	if(data_m != 0x13)
	{
		res = mir3da_register_read(REG_CHIP_ID,&data_m,1);
		if(data_m != 0x13)
		{
			res = mir3da_register_read(REG_CHIP_ID,&data_m,1);
			if(data_m != 0x13)
			{
				//LogPrintf(DEBUG_FACTORY,"Read gsensor chip id error =%x",data_m);
			}
		}
	}
   	if(data_m != 0x13)
    {
    	LogPrintf(DEBUG_ALL, "mir3da_init==>Read chip error=0x%X", data_m);
		return -1;
	}
	else
	{
		LogPrintf(DEBUG_ALL, "mir3da_init==>Read chip id=0x%X", data_m);
	}
	//MUST do softreset(POR)
	mir3da_register_write(REG_SPI_CONFIG, 0x24);
	DelayMs(20);
	mir3da_register_write(REG_RESOLUTION_RANGE, 0x60);	//+/-4G,16bit
	mir3da_register_write(REG_MODE_ODR, 0x07);			//lowpower mode, ODR 100hz

	mir3da_register_write(REG_INT_SET1, 0x80);			//INT source, filtered data
	mir3da_register_write(REG_INT_CONFIG, 0x00);		//PP, active high
	//load_offset_from_filesystem(offset_x, offset_y, offset_z);	//pseudo-code

	//private register, key to close internal pull up.
	mir3da_register_write(0x7F, 0x83);
	mir3da_register_write(0x7F, 0x69);
	mir3da_register_write(0x7F, 0xBD);


	//mir3da_register_mask_write(0x8F, 0x02, 0x00);	//don't pull up i2c sda/scl pin if needed
	//mir3da_register_mask_write(0x8C, 0x80, 0x00);	//don't pull up cs pin if needed

	if(i2c_addr == 0x26){
		mir3da_register_mask_write(0x8C, 0x40, 0x00);	//must close pin1/sdo pull up
	}

    return res;
}

//enable/disable the chip
s8_m mir3da_set_enable(u8_m enable)
{
    s8_m res = 0;
	if(enable)
		mir3da_register_mask_write(REG_MODE_AXIS, 0x80, 0x00);
	else	
		mir3da_register_mask_write(REG_MODE_AXIS, 0x80, 0x80);

	DelayMs(100);
    return res;
}

//Read three axis data, 1024 LSB = 1 g
s8_m mir3da_read_data(s16_m *x, s16_m *y, s16_m *z)
{
    u8_m tmp_data[6] = {0};

    mir3da_register_read(REG_ACC_X_LSB, tmp_data, 6);

    *x = (short)(tmp_data[1] << 8 | tmp_data[0]) - offset_x;
    *y = (short)(tmp_data[3] << 8 | tmp_data[2]) - offset_y;
    *z = (short)(tmp_data[5] << 8 | tmp_data[4]) - offset_z;


    return 0;
}

//open active interrupt
s8_m mir3da_open_interrupt(u8_m th)
{
    s8_m   res = 0;

    res = mir3da_register_write(REG_INT_SET1, 0x87);
    res = mir3da_register_write(REG_ACTIVE_DUR, 0x00);
   	mir3da_register_write(REG_ACTIVE_X_THS, 0x24);
	mir3da_register_write(REG_ACTIVE_Y_THS, 0x24);
	mir3da_register_write(REG_ACTIVE_Z_THS, 0x24);
    res = mir3da_register_write(REG_INT_MAP1, 0x04);

    return res;
}

//close active interrupt
s8_m mir3da_close_interrupt(void)
{
    s8_m   res = 0;

    res = mir3da_register_write(REG_INT_SET1, 0x00);
    res = mir3da_register_write(REG_INT_MAP1, 0x00);

    return res;
}

u8_m read_gsensor_id(void)
{
    u8_m data_m = 0;
    //Retry 3 times
    mir3da_register_read(REG_CHIP_ID, &data_m, 1);
    if (data_m != 0x13)
    {
        mir3da_register_read(REG_CHIP_ID, &data_m, 1);
        if (data_m != 0x13)
        {
           	mir3da_register_read(REG_CHIP_ID, &data_m, 1);
            if (data_m != 0x13)
            {
                LogPrintf(DEBUG_ALL, "Read gsensor chip id error =%x\r\n", data_m);
                return 0;
            }
        }
    }
    LogPrintf(DEBUG_FACTORY, "GSENSOR Chk. ID=0x%X\r\nGSENSOR CHK OK\r\n", data_m);
    return data_m;
}

s8_m readInterruptConfig(void)
{
    uint8_t data_m;
    mir3da_register_read(REG_INT_SET1, &data_m, 1);
    if (data_m != 0x87)
    {
        mir3da_register_read(REG_INT_SET1, &data_m, 1);
        if (data_m != 0x87)
        {
            return -1;
        }
    }
    LogPrintf(DEBUG_ALL, "Gsensor OK", data_m);
    return 0;
}

void startStep(void)
{
    LogMessage(DEBUG_ALL, "Gsensor enable step counting\r\n");
	mir3da_register_write(0x2F,   0x78);
	mir3da_register_write(0x30,   0x5D);
	mir3da_register_write(0x31,   0x5D);
	mir3da_register_write(0x32,   0x00);
	mir3da_register_write(0x33,   0xAF);
	mir3da_register_write(0x35,   0x77);
	mir3da_register_write(0x37,   0x41);

}

void stopStep(void)
{
    mir3da_register_write(0x33, 0x22);
}

u16_m getStep(void)
{
	s16_m f_step=0;
	u8_m tmp_data[2] = {0};
	mir3da_register_read(REG_STEPS_MSB, tmp_data, 2);
	f_step = tmp_data[0] << 8 | tmp_data[1];
	return f_step;
}

void clearStep(void)
{
   	mir3da_register_mask_write(REG_RESET_STEP, 0x80, 0x80);
    LogMessage(DEBUG_ALL, "Clear step\r\n");
}

