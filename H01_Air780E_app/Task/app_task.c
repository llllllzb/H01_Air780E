#include <app_protocol.h>
#include "app_task.h"
#include "app_mir3da.h"
#include "app_atcmd.h"
#include "app_gps.h"
#include "app_instructioncmd.h"
#include "app_kernal.h"
#include "app_net.h"
#include "app_net.h"
#include "app_param.h"
#include "app_port.h"
#include "app_sys.h"
#include "app_socket.h"
#include "app_server.h"
#include "app_jt808.h"
#include "app_central.h"

#define SYS_LED1_ON       LED1_ON
#define SYS_LED1_OFF      LED1_OFF


static SystemLEDInfo sysledinfo;
motionInfo_s motionInfo;
static bleScanTry_s bleTry;
static int8_t wifiTimeOutId = -1;

/**************************************************
@bref		bit0 置位，布防
@param
@return
@note
**************************************************/
void terminalDefense(void)
{
    sysinfo.terminalStatus |= 0x01;
}

/**************************************************
@bref		bit0 清除，撤防
@param
@return
@note
**************************************************/
void terminalDisarm(void)
{
    sysinfo.terminalStatus &= ~0x01;
}
/**************************************************
@bref		获取运动或静止状态
@param
@return
	>0		运动
	0		静止
@note
**************************************************/

uint8_t getTerminalAccState(void)
{
    return (sysinfo.terminalStatus & 0x02);

}

/**************************************************
@bref		bit1 置位，运动，accon
@param
@return
@note
**************************************************/

void terminalAccon(void)
{
    sysinfo.terminalStatus |= 0x02;
    jt808UpdateStatus(JT808_STATUS_ACC, 1);
}

/**************************************************
@bref		bit1 清除，静止，accoff
@param
@return
@note
**************************************************/
void terminalAccoff(void)
{
    sysinfo.terminalStatus &= ~0x02;
    jt808UpdateStatus(JT808_STATUS_ACC, 0);
}

/**************************************************
@bref		bit2 置位，充电
@param
@return
@note
**************************************************/

void terminalCharge(void)
{
    sysinfo.terminalStatus |= 0x04;
}
/**************************************************
@bref		bit2 清除，未充电
@param
@return
@note
**************************************************/

void terminalunCharge(void)
{
    sysinfo.terminalStatus &= ~0x04;
}

/**************************************************
@bref		获取充电状态
@param
@return
	>0		充电
	0		未充电
@note
**************************************************/

uint8_t getTerminalChargeState(void)
{
    return (sysinfo.terminalStatus & 0x04);
}

/**************************************************
@bref		bit 3~5 报警信息
@param
@return
@note
**************************************************/

void terminalAlarmSet(TERMINAL_WARNNING_TYPE alarm)
{
    sysinfo.terminalStatus &= ~(0x38);
    sysinfo.terminalStatus |= (alarm << 3);
}

/**************************************************
@bref		bit6 置位，已定位
@param
@return
@note
**************************************************/

void terminalGPSFixed(void)
{
    sysinfo.terminalStatus |= 0x40;
}

/**************************************************
@bref		bit6 清除，未定位
@param
@return
@note
**************************************************/

void terminalGPSUnFixed(void)
{
    sysinfo.terminalStatus &= ~0x40;
}

/**************************************************
@bref		LED1 运行任务
@param
@return
@note
**************************************************/

static void sysLed1Run(void)
{
    static uint8_t tick = 0;


    if (sysledinfo.sys_led1_on_time == 0)
    {
        SYS_LED1_OFF;
        return;
    }
    else if (sysledinfo.sys_led1_off_time == 0)
    {
        SYS_LED1_ON;
        return;
    }

    tick++;
    if (sysledinfo.sys_led1_onoff == 1) //on status
    {
        SYS_LED1_ON;
        if (tick >= sysledinfo.sys_led1_on_time)
        {
            tick = 0;
            sysledinfo.sys_led1_onoff = 0;
        }
    }
    else   //off status
    {
        SYS_LED1_OFF;
        if (tick >= sysledinfo.sys_led1_off_time)
        {
            tick = 0;
            sysledinfo.sys_led1_onoff = 1;
        }
    }
}

/**************************************************
@bref		设置灯的闪烁频率
@param
@return
@note
**************************************************/

static void ledSetPeriod(uint8_t ledtype, uint8_t on_time, uint8_t off_time)
{
    if (ledtype == GPSLED1)
    {
        //系统信号灯
        sysledinfo.sys_led1_on_time = on_time;
        sysledinfo.sys_led1_off_time = off_time;
    }
}

/**************************************************
@bref		更新系统灯状态
@param
@return
@note
**************************************************/

void ledStatusUpdate(uint8_t status, uint8_t onoff)
{
    if (onoff == 1)
    {
        sysinfo.sysLedState |= status;
    }
    else
    {
        sysinfo.sysLedState &= ~status;
    }
    if ((sysinfo.sysLedState & SYSTEM_LED_RUN) == SYSTEM_LED_RUN)
    {
        //慢闪
        ledSetPeriod(GPSLED1, 10, 10);
        if ((sysinfo.sysLedState & SYSTEM_LED_NETOK) == SYSTEM_LED_NETOK)
        {
            //常亮
            ledSetPeriod(GPSLED1, 1, 9);
            if ((sysinfo.sysLedState & SYSTEM_LED_GPSOK) == SYSTEM_LED_GPSOK)
            {
                //普通灯常亮
                ledSetPeriod(GPSLED1, 1, 0);
            }
        }
    }
    else
    {
        SYS_LED1_OFF;
        ledSetPeriod(GPSLED1, 0, 1);
    }
}

/**************************************************
@bref		灯控任务
@param
@return
@note
**************************************************/

static void ledTask(void)
{
	if (sysinfo.ledTick <= 0)
	{
		SYS_LED1_OFF;
		return;
	}


    sysLed1Run();
}

/**************************************************
@bref		gps开启请求
@param
@return
@note
**************************************************/
void gpsRequestSet(uint32_t flag)
{
    LogPrintf(DEBUG_ALL, "gpsRequestSet==>0x%04X", flag);
    sysinfo.gpsRequest |= flag;
}

/**************************************************
@bref		gps清除请求
@param
@return
@note
**************************************************/

void gpsRequestClear(uint32_t flag)
{
    LogPrintf(DEBUG_ALL, "gpsRequestClear==>0x%04X", flag);
    sysinfo.gpsRequest &= ~flag;
}

uint32_t gpsRequestGet(uint32_t flag)
{
    return sysinfo.gpsRequest & flag;
}

/**************************************************
@bref		gps任务状态机切换
@param
@return
@note
**************************************************/

static void gpsChangeFsmState(uint8_t state)
{
    sysinfo.gpsFsm = state;
}


/**************************************************
@bref		gps数据接收
@param
@return
@note
**************************************************/

void gpsUartRead(uint8_t *msg, uint16_t len)
{
    static uint8_t gpsRestore[UART_RECV_BUFF_SIZE + 1];
    static uint16_t size = 0;
    uint16_t i, begin;
    if (len + size > UART_RECV_BUFF_SIZE)
    {
        size = 0;
    }
    memcpy(gpsRestore + size, msg, len);
    size += len;
    begin = 0;
    for (i = 0; i < size; i++)
    {
        if (gpsRestore[i] == '\n')
        {
            if (sysinfo.nmeaOutPutCtl)
            {
                LogWL(DEBUG_GPS, gpsRestore + begin, i - begin);
                LogWL(DEBUG_GPS, "\r\n", 2);
                
            }
            nmeaParser(gpsRestore + begin, i - begin);
            begin = i + 1;
        }
    }
    if (begin != 0)
    {
        memmove(gpsRestore, gpsRestore + begin, size - begin);
        size -= begin;
    }
}

static void gpsCfg(void)
{
    char param[50];
    //关闭GSV
    //sprintf(param, "$CCMSG,GSV,1,0,*1A\r\n");
//    sprintf(param, "$PCAS03,1,0,1,0,1,0,0,0,0,0,0,0,0,0*03\r\n");
//    portUartSend(&usart3_ctl, (uint8_t *)param, strlen(param));
    sprintf(param, "$PCAS03,,,,,,,,,,,1*1F\r\n");
    portUartSend(&usart3_ctl, (uint8_t *)param, strlen(param));
    LogMessage(DEBUG_ALL, "gps config nmea output");
}
/**************************************************
@bref       切换中科微波特率为115200
@param
@return
@note
**************************************************/

//$PCAS03,1,0,1,1,1,0,0,0,0,0,0,0,0,0*02
static void changeGPSBaudRate(void)
{
    char param[50];
    sprintf(param, "$PCAS01,5*19\r\n");
    portUartSend(&usart3_ctl, (uint8_t *)param, strlen(param));
    portUartCfg(APPUSART3, 1, 115200, gpsUartRead);
    LogMessage(DEBUG_ALL, "gps config baudrate to 115200");
    startTimer(10, gpsCfg, 0);
}
/**************************************************
@bref       中科微热启动配置
@param
@return
@note
**************************************************/

static void gpsWarmStart(void)
{
    char param[50];
    //热启动
    sprintf(param, "$PCAS10,0*1C\r\n");
    portUartSend(&usart3_ctl, (uint8_t *)param, strlen(param));
    LogMessage(DEBUG_ALL, "Gps config warm start");
    startTimer(10, changeGPSBaudRate, 0);
}
/**************************************************
@bref       开启gps
@param
@return
@note
**************************************************/

void gpsOpen(void)
{
    GPSPWR_ON;
    GPSLNA_ON;
    portUartCfg(APPUSART3, 1, 9600, gpsUartRead);
    startTimer(10, changeGPSBaudRate, 0);
    sysinfo.gpsUpdatetick = sysinfo.sysTick;
    sysinfo.gpsOnoff = 1;
    gpsChangeFsmState(GPSWATISTATUS);
    gpsClearCurrentGPSInfo();
    ledStatusUpdate(SYSTEM_LED_GPSOK, 0);
    moduleSleepCtl(0);
    LogMessage(DEBUG_ALL, "gpsOpen");
    sysinfo.ephemerisFlag = 0;

}
/**************************************************
@bref       等待gps稳定
@param
@return
@note
**************************************************/

static void gpsWait(void)
{
    static uint8_t runTick = 0;
    if (++runTick >= 4)
    {
        runTick = 0;
        gpsChangeFsmState(GPSOPENSTATUS);
        if (sysinfo.ephemerisFlag == 0 || sysinfo.agpsTick == 0 || sysinfo.sysTick >= sysinfo.agpsTick)
        {
          	agpsRequestSet();
          	sysinfo.agpsTick = sysinfo.sysTick + 120;
        }
    }
}

/**************************************************
@bref       关闭gps
@param
@return
@note
**************************************************/

void gpsClose(void)
{
    GPSPWR_OFF;
    GPSLNA_OFF;
    portUartCfg(APPUSART3, 0, 115200, NULL);
    sysinfo.rtcUpdate = 0;
    sysinfo.gpsOnoff = 0;
    gpsClearCurrentGPSInfo();
    terminalGPSUnFixed();
    gpsChangeFsmState(GPSCLOSESTATUS);
    ledStatusUpdate(SYSTEM_LED_GPSOK, 0);
    if (primaryServerIsReady())
    {
        moduleSleepCtl(1);
    }
    LogMessage(DEBUG_ALL, "gpsClose");
}

/**************************************************
@bref		保存上一次gps位置
@param
@return
@note
**************************************************/
void saveGpsHistory(void)
{
	gpsinfo_s *gpsinfo;
    int16_t latitude, longtitude;
    gpsinfo = getLastFixedGPSInfo();
    if (gpsinfo->fixstatus != 0)
    {
        latitude   = (int16_t)gpsinfo->latitude;
        longtitude = (int16_t)gpsinfo->longtitude;
        if (gpsinfo->NS == 'S')
        {
            if (latitude > 0)
            {
                latitude *= -1;
            }
        }
        if (gpsinfo->EW == 'W')
        {
            if (longtitude > 0)
            {
                longtitude *= -1;
            }
        }
        dynamicParam.saveLat = (int32_t)latitude;
        dynamicParam.saveLon = (int32_t)longtitude;
        LogPrintf(DEBUG_ALL, "Save Latitude:%d,Longtitude:%d", dynamicParam.saveLat, dynamicParam.saveLon);
		dynamicParamSaveAll();
    }
}


/**************************************************
@bref		gps控制任务
@param
@return
@note
**************************************************/

static void gpsRequestTask(void)
{
    gpsinfo_s *gpsinfo;
    static uint16_t gpsInvalidTick = 0;
	static uint8_t gpsInvalidFlag = 0, gpsInvalidFlagTick = 0;
	uint16_t gpsInvalidparam;
	/* 同步flag */
	static uint8_t flag = 0;

    switch (sysinfo.gpsFsm)
    {
        case GPSCLOSESTATUS:
            //有设备请求开关
            if (sysinfo.canRunFlag != 1)
            {
            	break;
            }
            if (sysinfo.gpsRequest != 0)
            {
                gpsOpen();
            }
            break;
        case GPSWATISTATUS:
            gpsWait();
            break;
        case GPSOPENSTATUS:
            gpsinfo = getCurrentGPSInfo();
            if (gpsinfo->fixstatus)
            {
                ledStatusUpdate(SYSTEM_LED_GPSOK, 1);
               	lbsRequestClear();
				wifiRequestClear();
				//GPS运行的过程中才改变这个同步的flag，如果定位后下一秒 关gps之前又不定位的话，那么又会重新开始计时上报wifi
				if (sysinfo.gpsRequest != 0)
					flag = 0;
            }
            else
            {
                ledStatusUpdate(SYSTEM_LED_GPSOK, 0);    
                if (sysinfo.gpsRequest != 0)
                	flag = 1;
            }
            if (sysinfo.gpsRequest == 0 || (sysinfo.sysTick - sysinfo.gpsUpdatetick) >= 20)
            {
            	if (sysinfo.gpsRequest == 0)
            	{
					saveGpsHistory();
					agpsRequestClear();
					
            	}
                gpsClose();
            }
            break;
        default:
            gpsChangeFsmState(GPSCLOSESTATUS);
            break;
    }

//    /* 只要ACCON就认为是开启GPS， 所以要忽略GPS的开关与否 */
//    if (getTerminalAccState() == 0)
//    {
//        gpsInvalidTick = 0;
//        gpsInvalidFlag = 0;
//        gpsInvalidFlagTick = 0;
//        return;
//    }
//    if (sysparam.gpsuploadgap == 0)
//    {
//        gpsInvalidTick = 0;
//        gpsInvalidFlag = 0;
//        gpsInvalidFlagTick = 0;
//        return;
//    }
//    if (flag == 0)
//    {
//        gpsInvalidTick = 0;
//        gpsInvalidFlag = 0;
//        gpsInvalidFlagTick = 0;
//		return;
//    }
//    gpsInvalidparam = (sysparam.gpsuploadgap < 60) ? 60 : sysparam.gpsuploadgap;
//    LogPrintf(DEBUG_ALL, "gpsInvalidTick:%d  gpsInvalidparam:%d", gpsInvalidTick, gpsInvalidparam);
//    gpsinfo = getCurrentGPSInfo();
//    if (gpsinfo->fixstatus == 0)
//    {
//        if (++gpsInvalidTick >= gpsInvalidparam)
//        {
//            gpsInvalidTick = 0;
//            gpsInvalidFlag = 1;
//    		wifiRequestSet(DEV_EXTEND_OF_MY);
//        }
//    }
//    else
//    {
//        gpsInvalidTick = 0;
//        gpsInvalidFlag = 0;
//        gpsInvalidFlagTick = 0;
//    }

}



/**************************************************
@bref		上送一个gps位置
@param
@return
@note
**************************************************/

static void gpsUplodOnePointTask(void)
{
    gpsinfo_s *gpsinfo;
    static uint16_t runtick = 0;
    static uint8_t uploadtick = 0;
    //判断是否有请求该事件
    if (sysinfo.gpsOnoff == 0)
        return;
    if (gpsRequestGet(GPS_REQUEST_UPLOAD_ONE) == 0)
    {
        runtick = 0;
        uploadtick = 0;
        return;
    }
    gpsinfo = getCurrentGPSInfo();
    runtick++;
    if (gpsinfo->fixstatus == 0)
    {
        uploadtick = 0;
        if (runtick >= 180)
        {
            runtick = 0;
            uploadtick = 0;
            LogPrintf(DEBUG_ALL, "gps fix time out");
            gpsRequestClear(GPS_REQUEST_UPLOAD_ONE);
            if (getTerminalAccState() == 0)
            {
            	wifiRequestSet(DEV_EXTEND_OF_MY);
            }
        }
        return;
    }
    runtick = 0;
    if (++uploadtick >= 10)
    {
        uploadtick = 0;
        if (sysinfo.flag123)
        {
            dorequestSend123();
        }
        protocolSend(NORMAL_LINK, PROTOCOL_12, getCurrentGPSInfo());
        if (!(sysparam.gpsuploadgap < GPS_UPLOAD_GAP_MAX && sysparam.gpsuploadgap != 0 && 
        	(sysparam.MODE == MODE2 || sysparam.MODE == MODE21 || sysparam.MODE == MODE23)))
        {
			if (primaryServerIsReady())
    		{
        		protocolInfoResiter(getBatteryLevel(), sysinfo.outsidevoltage > 5.0 ? sysinfo.outsidevoltage : sysinfo.insidevoltage,
                            dynamicParam.startUpCnt, dynamicParam.runTime);
        		protocolSend(NORMAL_LINK, PROTOCOL_13, NULL);
        		privateHbtTickReset();
        	}
        }
        jt808SendToServer(TERMINAL_POSITION, getCurrentGPSInfo());
        gpsRequestClear(GPS_REQUEST_UPLOAD_ONE);
    }

}

/**************************************************
@bref		报警上送请求
@param
@return
@note
**************************************************/
void alarmRequestSet(uint16_t request)
{
    LogPrintf(DEBUG_ALL, "alarmRequestSet==>0x%04X", request);
    sysinfo.alarmRequest |= request;
}
/**************************************************
@bref		清除报警上送
@param
@return
@note
**************************************************/

void alarmRequestClear(uint16_t request)
{
    LogPrintf(DEBUG_ALL, "alarmRequestClear==>0x%04X", request);
    sysinfo.alarmRequest &= ~request;
}

/**************************************************
@bref		报警任务
@param
@return
@note
**************************************************/

void alarmRequestTask(void)
{
    uint8_t alarm;
    if (primaryServerIsReady() == 0 || sysinfo.alarmRequest == 0)
    {
        return;
    }
    if (getTcpNack() != 0)
    {
        return;
    }
    //感光报警
    if (sysinfo.alarmRequest & ALARM_LIGHT_REQUEST)
    {
        alarmRequestClear(ALARM_LIGHT_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>Light Alarm");
        terminalAlarmSet(TERMINAL_WARNNING_LIGHT);
        alarm = 0;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //低电报警
    if (sysinfo.alarmRequest & ALARM_LOWV_REQUEST)
    {
        alarmRequestClear(ALARM_LOWV_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>LowVoltage Alarm");
        terminalAlarmSet(TERMINAL_WARNNING_LOWV);
        alarm = 0;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //断电报警
    if (sysinfo.alarmRequest & ALARM_LOSTV_REQUEST)
    {
        alarmRequestClear(ALARM_LOSTV_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>lostVoltage Alarm");
        terminalAlarmSet(TERMINAL_WARNNING_LOSTV);
        alarm = 0;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }
    
    //震动报警
    if (sysinfo.alarmRequest & ALARM_SHUTTLE_REQUEST)
    {
        alarmRequestClear(ALARM_SHUTTLE_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>shuttle Alarm");
        terminalAlarmSet(TERMINAL_WARNNING_SHUTTLE);
        alarm = 0;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //SOS报警
    if (sysinfo.alarmRequest & ALARM_SOS_REQUEST)
    {
        alarmRequestClear(ALARM_SOS_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>SOS Alarm");
        terminalAlarmSet(TERMINAL_WARNNING_SOS);
        alarm = 0;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //急加速报警
    if (sysinfo.alarmRequest & ALARM_ACCLERATE_REQUEST)
    {
        alarmRequestClear(ALARM_ACCLERATE_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>Rapid Accleration Alarm");
        alarm = 9;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }
    //急减速报警
    if (sysinfo.alarmRequest & ALARM_DECELERATE_REQUEST)
    {
        alarmRequestClear(ALARM_DECELERATE_REQUEST);
        LogMessage(DEBUG_ALL,
                   "alarmRequestTask==>Rapid Deceleration Alarm");
        alarm = 10;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //急左转报警
    if (sysinfo.alarmRequest & ALARM_RAPIDLEFT_REQUEST)
    {
        alarmRequestClear(ALARM_RAPIDLEFT_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>Rapid LEFT Alarm");
        alarm = 11;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //急右转报警
    if (sysinfo.alarmRequest & ALARM_RAPIDRIGHT_REQUEST)
    {
        alarmRequestClear(ALARM_RAPIDRIGHT_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>Rapid RIGHT Alarm");
        alarm = 12;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //守卫报警
    if (sysinfo.alarmRequest & ALARM_GUARD_REQUEST)
    {
        alarmRequestClear(ALARM_GUARD_REQUEST);
        LogMessage(DEBUG_ALL, "alarmUploadRequest==>Guard Alarm\n");
        alarm = 1;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }
    //关机报警
    if (sysinfo.alarmRequest & ALARM_SHUTDOWN_REQUEST)
    {
        alarmRequestClear(ALARM_SHUTDOWN_REQUEST);
        LogMessage(DEBUG_ALL, "alarmUploadRequest==>关机报警");
        alarm = 0x30;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

}



/**************************************************
@bref		更新运动或静止状态
@param
	src 		检测来源
	newState	新状态
@note
**************************************************/

static void motionStateUpdate(motion_src_e src, motionState_e newState)
{
    char type[20];


    if (motionInfo.motionState == newState)
    {
        return;
    }
    motionInfo.motionState = newState;
    switch (src)
    {
        case ACC_SRC:
            strcpy(type, "acc");
            break;
        case VOLTAGE_SRC:
            strcpy(type, "voltage");
            break;
        case GSENSOR_SRC:
            strcpy(type, "gsensor");
            break;
       	case SYS_SRC:
       		strcpy(type, "sys");
            break;
        case STEP_SRC:
			strcpy(type, "step");
        	break;
        default:
            strcpy(type, "unknow");
            break;
    }
    LogPrintf(DEBUG_ALL, "Device %s , detected by %s", newState == MOTION_MOVING ? "moving" : "static", type);

    if (newState)
    {
        netResetCsqSearch();
        if (src != SYS_SRC)
        {
	        if (sysparam.gpsuploadgap != 0)
	        {
	            gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
	            if (sysparam.gpsuploadgap < GPS_UPLOAD_GAP_MAX)
	            {
	                gpsRequestSet(GPS_REQUEST_ACC_CTL);
	            }
	        }
	        //没网络时
	        if (isModeDone())
	        {
				gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
	        }    
	        if (sysparam.bf)
	        {
				alarmRequestSet(ALARM_GUARD_REQUEST);
	        }
        }
        terminalAccon();
        hiddenServerCloseClear();
    }
    else
    {
    	if (src != SYS_SRC)
    	{
	        if (sysparam.gpsuploadgap != 0)
	        {
	            gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
	            gpsRequestClear(GPS_REQUEST_ACC_CTL);
	        }
        }
        terminalAccoff();
        sysinfo.accStep = sysinfo.step;		//同步acc步数
        updateRTCtimeRequest();
    }
    if (primaryServerIsReady() && src != SYS_SRC)
    {
        protocolInfoResiter(getBatteryLevel(), sysinfo.outsidevoltage > 5.0 ? sysinfo.outsidevoltage : sysinfo.insidevoltage,
                            dynamicParam.startUpCnt, dynamicParam.runTime);
        protocolSend(NORMAL_LINK, PROTOCOL_13, NULL);
        jt808SendToServer(TERMINAL_POSITION, getLastFixedGPSInfo());
        privateHbtTickReset();
    }
}


/**************************************************
@bref       震动中断
@param
@note
**************************************************/

void motionOccur(void)
{
    motionInfo.tapInterrupt++;
}

/**************************************************
@bref       tapCnt 大小
@param
@note
**************************************************/

uint8_t motionGetSize(void)
{
    return sizeof(motionInfo.tapCnt);
}
/**************************************************
@bref		统计每一秒的中断次数
@param
@note
**************************************************/

static void motionCalculate(void)
{
    motionInfo.ind = (motionInfo.ind + 1) % sizeof(motionInfo.tapCnt);
    motionInfo.tapCnt[motionInfo.ind] = motionInfo.tapInterrupt;
    motionInfo.tapInterrupt = 0;
}
/**************************************************
@bref		获取这最近n秒的震动次数
@param
@note
**************************************************/

static uint16_t motionGetTotalCnt(uint8_t n)
{
    uint16_t cnt;
    uint8_t i;
    cnt = 0;
    for (i = 0; i < n; i++)
    {
        cnt += motionInfo.tapCnt[(motionInfo.ind + sizeof(motionInfo.tapCnt) - i) % sizeof(motionInfo.tapCnt)];
    }
    return cnt;
}

/**************************************************
@bref       检测单位时间内振动频率，判断是否运动
@param
@note
**************************************************/

static uint16_t motionCheckOut(uint8_t sec)
{
    uint8_t i;
    uint16_t validCnt;

    validCnt = 0;
    if (sec == 0 || sec > sizeof(motionInfo.tapCnt))
    {
        return 0;
    }
    for (i = 0; i < sec; i++)
    {
        if (motionInfo.tapCnt[(motionInfo.ind + sizeof(motionInfo.tapCnt) - i) % sizeof(motionInfo.tapCnt)] != 0)
        {
            validCnt++;
        }
    }
    return validCnt;
}

void motionClear(void)
{
	LogMessage(DEBUG_ALL, "motionClear==>OK");
	memset(&motionInfo, 0, sizeof(motionInfo_s));
}

/**************************************************
@bref		获取运动状态
@param
@note
**************************************************/

motionState_e motionGetStatus(void)
{
    return motionInfo.motionState;
}
#define ACC_ON_DETECT_TIME			6

static uint8_t accOnStep[ACC_ON_DETECT_TIME]   = { 0 };
static uint8_t acc_on_step_ind  = 0;
static uint32_t totalStep = 0;

static void accStepInit(void)
{
	totalStep = 0;
	tmos_memset(accOnStep, 0, ACC_ON_DETECT_TIME);
	acc_on_step_ind  = 0;
}

uint8_t accOnOffByGsensorStep(void)
{
	uint32_t stepgap = 0;
	uint8_t i = 0;
	static uint8_t runtick = 0;
//	if (getTerminalAccState())
//	{
//		acc_on_step_ind = 0;
//		runtick = 0;
//		tmos_memset(accOnStep, 0, ACC_ON_DETECT_TIME);
//		//处在运动时不判断步数
//		return 0;
//	}
	runtick++;
	if (runtick <= 10)
	{
		if (totalStep >= sysparam.motionstep)
			return 1;
		else
			return 0;
	}
	runtick = 0;
	totalStep = 0;
	portUpdateStep();
	stepgap = abs(sysinfo.step - sysinfo.accStep);
	accOnStep[acc_on_step_ind] = stepgap;
	acc_on_step_ind = (acc_on_step_ind + 1) % ACC_ON_DETECT_TIME;
	for (i = 0; i < ACC_ON_DETECT_TIME; i++)
	{
		totalStep += accOnStep[i];
	}
	sysinfo.accStep = sysinfo.step;
	LogPrintf(DEBUG_ALL, "accOnOffByGsensorStep==>%d step", totalStep);
	if (totalStep >= sysparam.motionstep)
	{
		//tmos_memset(accOnStep, 0, ACC_ON_DETECT_TIME);
		return 1;
	}
	return 0;
}

/**************************************************
@bref		运动和静止的判断
@param
@note
**************************************************/

static void motionCheckTask(void)
{
    static uint16_t gsStaticTick = 0;
    static uint16_t autoTick = 0;
    
    static uint8_t motionState = 0;
    gpsinfo_s *gpsinfo;
	uint8_t accState = 0;
    uint16_t totalCnt, staticTime, stepCnt = 0;
    if (sysinfo.gsensorOnoff == 0)
    {
		motionStateUpdate(SYS_SRC, MOTION_STATIC);
        gsStaticTick = 0;
        motionState = 0;
        totalCnt = 0;
        accState = 0;
        accStepInit();
        return ;
    }
    
    motionCalculate();
    staticTime = 120;
    
    if (sysparam.MODE == MODE1 || sysparam.MODE == MODE3 || sysparam.MODE == MODE4)
    {
        motionStateUpdate(SYS_SRC, MOTION_STATIC);
        gsStaticTick = 0;
        motionState = 0;
        accState = 0;
        return ;
    }

    //保持运动状态时，如果gap大于Max，则周期性上报gps
    if (getTerminalAccState() && sysparam.gpsuploadgap >= GPS_UPLOAD_GAP_MAX)
    {
        if (++autoTick >= sysparam.gpsuploadgap)
        {
            autoTick = 0;
            gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
        }
    }
    else
    {
        autoTick = 0;
    }
    //判断震动
    totalCnt = motionCheckOut(sysparam.gsdettime);
	stepCnt  = accOnOffByGsensorStep();
    //Acc off时,同步判断震动与步数
    if (getTerminalAccState() == 0)
    {
		if (totalCnt >= sysparam.gsValidCnt && sysparam.gsValidCnt != 0)
		{
			motionState = GSENSOR_SRC;
			gsStaticTick = 0;
			accState = 1;
		}
		else if (stepCnt)
		{
			motionState = STEP_SRC;
			gsStaticTick = 0;
			accState = 1;
		}
    }
    //Acc on时,要0震动和120s内步数少于sysparam.motionstep 同时满足才认为是静止
    else
    {
		if (totalCnt <= sysparam.gsInvalidCnt && stepCnt == 0)
			accState = 0;
		else
			accState = 1;
    }
    
//    LogPrintf(DEBUG_ALL, "motionCheckOut=%d,%d,%d,%d,%d,%d,%d,%d", 
//    			totalCnt, sysparam.gsdettime, sysparam.gsValidCnt,
//                          sysparam.gsInvalidCnt, motionState, autoTick, gsStaticTick, accState);
    
    //gsensor控制(步数或震动)
    if (accState)
    {
        motionStateUpdate(motionState, MOTION_MOVING);
    }
    if (accState == 0)
    {
        gsStaticTick++;
        if (gsStaticTick >= staticTime)
        {
            motionStateUpdate(GSENSOR_SRC, MOTION_STATIC);
        }
    }
    else
    {
        gsStaticTick = 0;
    }
}




/**************************************************
@bref		电压检测任务
@param
@return
@note
**************************************************/

static void voltageCheckTask(void)
{
    static uint16_t lowpowertick = 0;
    static uint8_t  lowwflag = 0;
    float x;
    static uint8_t protectTick = 0;
    uint8_t ret = 0;
    if (sysinfo.adcOnoff == 0)
	{
		return;
	}
	/* 设备运行的过程中可能会复位模组，也就是给模组断电，这个时候先暂停电压检测，以免误判电压 */
	if (sysinfo.moduleSupplyStatus == 0)
	{
		LogPrintf(DEBUG_ALL, "Temporarily stop voltage detection..");
		return;
	}
    x = portGetAdcVol(ADC_CHANNEL);
    sysinfo.outsidevoltage = x * sysparam.adccal;
    sysinfo.insidevoltage = sysinfo.outsidevoltage;

    //LogPrintf(DEBUG_ALL, "x:%.2f, outsidevoltage:%.2f bat:%d%%", x, sysinfo.outsidevoltage ,getBatteryLevel());

	//电池保护
    if (sysinfo.outsidevoltage < 3.0 && sysinfo.canRunFlag == 1)
    {
		protectTick++;
		if (protectTick >= 5)
		{
			protectTick = 0;
			sysinfo.canRunFlag = 0;
			LogPrintf(DEBUG_ALL, "Batvoltage is lowwer than %.2f", sysinfo.outsidevoltage);

		}
    }
	else if (sysinfo.outsidevoltage >= 3.1 && sysinfo.canRunFlag == 0)
	{
		protectTick++;
		if (protectTick >= 5)
		{
			protectTick = 0;
			sysinfo.canRunFlag = 1;
			LogPrintf(DEBUG_ALL, "Batvoltage is more than %.2f", sysinfo.outsidevoltage);
		}
		
	}
	else
	{
		protectTick = 0;
	}
    
    //低电报警
    if (sysinfo.outsidevoltage < sysinfo.lowvoltage)
    {
        lowpowertick++;
        if (lowpowertick >= 30)
        {
            if (lowwflag == 0)
            {
                lowwflag = 1;
                LogPrintf(DEBUG_ALL, "power supply too low %.2fV", sysinfo.outsidevoltage);
                //低电报警
                jt808UpdateAlarm(JT808_LOWVOLTAE_ALARM, 1);
                alarmRequestSet(ALARM_LOWV_REQUEST);
                //lbsRequestSet(DEV_EXTEND_OF_MY);
                wifiRequestSet(DEV_EXTEND_OF_MY);
                gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
            }

        }
    }
    else
    {
        lowpowertick = 0;
    }


    if (sysinfo.outsidevoltage >= sysinfo.lowvoltage + 0.5)
    {
        lowwflag = 0;
        jt808UpdateAlarm(JT808_LOWVOLTAE_ALARM, 0);
    }


}

/**************************************************
@bref		模式状态机切换
@param
@return
@note
**************************************************/

static void changeModeFsm(uint8_t fsm)
{
    sysinfo.runFsm = fsm;
    LogPrintf(DEBUG_ALL, "changeModeFsm==>%d", fsm);
}

/**************************************************
@bref		快速关闭
@param
@return
@note
**************************************************/

static void modeShutDownQuickly(void)
{
    static uint16_t delaytick = 0;
    //存在一种情况是GPS在一开始就定位了，清除了wifi基站的标志位，而4G上线慢导致运行了30秒就关机了，因此要加上primaryServerIsReady判断
    if (sysinfo.gpsRequest == 0 && sysinfo.alarmRequest == 0 && sysinfo.wifiRequest == 0 && sysinfo.lbsRequest == 0 && primaryServerIsReady())
    {
    	//sysparam.gpsuploadgap>=60时，由于gpsrequest==0会关闭kernal,导致一些以1s为时基的任务不计时，会导致gps上报不及时，acc状态无法切换等
    	//比如运动的情况下，由于没有GPS_REQUEST_ACC_CTL，这里会导致关闭kernal
    	if ((sysparam.MODE == MODE21 || sysparam.MODE == MODE23) && getTerminalAccState() && sysparam.gpsuploadgap >= GPS_UPLOAD_GAP_MAX)
    	{
			delaytick = 0;
    	}
        delaytick++;
        if (delaytick >= 20)
        {
            LogMessage(DEBUG_ALL, "modeShutDownQuickly==>shutdown");
            delaytick = 0;
            changeModeFsm(MODE_STOP); //执行完毕，关机
        }
    }
    else
    {
        delaytick = 0;
    }
}

/**************************************************
@bref		mode4切回在网模式
@param
@return
@note
**************************************************/

static void mode4CloseSocketQuickly(void)
{
	static uint16_t tick = 0;
	
	if (isModuleRunNormal())
	{
		if (sysinfo.gpsRequest == 0 && sysinfo.alarmRequest == 0 && sysinfo.wifiExtendEvt == 0 && sysinfo.lbsRequest == 0 && sysinfo.netRequest == 0)
		{
			tick++;
			if (tick >= 15)
			{
				LogMessage(DEBUG_ALL, "mode4CloseSocketQuickly==>On net");
				changeMode4Callback();
				tick = 0;
			}
		}
		else
		{
			tick = 0;
		}
		sysinfo.nonetTick = 0;
	}
	else if (isModuleRunNormal() == 0 && isModuleOfflineStatus() == 0)
	{
		sysinfo.nonetTick++;
		tick = 0;
		LogPrintf(DEBUG_ALL, "sysinfo.nonetTick:%d", sysinfo.nonetTick);
		if (sysinfo.nonetTick >= 270)
		{
			sysinfo.nonetTick = 0;
			LogMessage(DEBUG_ALL, "mode4CloseSocketQuickly==>Shut down");
			modeTryToStop();
		}
	}
	else
	{
		sysinfo.nonetTick = 0;
		tick = 0;
	}
}

/**************************************************
@bref		运行-》关机
@param
@return
@note
**************************************************/

void modeTryToStop(void)
{
    sysinfo.gpsRequest = 0;
    sysinfo.alarmRequest = 0;
    sysinfo.wifiRequest = 0;
    sysinfo.lbsRequest = 0;
    sysinfo.wifiExtendEvt = 0;
    sysinfo.lbsExtendEvt = 0;
    volCheckRequestClear();
    netRequestClear();
    changeModeFsm(MODE_STOP);
    LogMessage(DEBUG_ALL, "modeTryToStop");
}

/**************************************************
@bref		待机
@param
@return
@note
**************************************************/

void modeTryToDone(void)
{
	sysinfo.gpsRequest = 0;
    sysinfo.alarmRequest = 0;
    sysinfo.wifiRequest = 0;
    sysinfo.lbsRequest = 0;
    sysinfo.wifiExtendEvt = 0;
    sysinfo.lbsExtendEvt = 0;
    volCheckRequestClear();
    netRequestClear();
    changeModeFsm(MODE_DONE);
    LogMessage(DEBUG_ALL, "modeTryToDone");
}


/**************************************************
@bref		启动扫描
@param
@return
@note
**************************************************/

static void modeScan(void)
{
//    static uint8_t runTick = 0;
//    scanList_s  *list;
//    if (sysparam.leavealm == 0 || (sysparam.MODE != MODE1 && sysparam.MODE != MODE3))
//    {
//        runTick = 0;
//        changeModeFsm(MODE_START);
//        return;
//    }
//    if (runTick == 1)
//    {
//        portFsclkChange(1);
//        bleCentralDiscovery();
//    }
//    else if (runTick >= 20)
//    {
//        runTick = 0;
//        list = scanListGet();
//        if (list->cnt == 0)
//        {
//            alarmRequestSet(ALARM_LEAVE_REQUEST);
//        }
//        changeModeFsm(MODE_START);
//    }
//    runTick++;
}


/**************************************************
@bref		蓝牙状态机切换
@param
@return
@note
**************************************************/

static void bleChangeFsm(modeChoose_e fsm)
{
    bleTry.runFsm = fsm;
    bleTry.runTick = 0;
}

/**************************************************
@bref		扫描完成回调
@param
@return
@note
**************************************************/

void bleScanCallBack(deviceScanList_s *list)
{
    uint8_t i;
    for (i = 0; i < list->cnt; i++)
    {
        if (my_strpach(list->list[i].broadcaseName, "AUTO"))
        {
            LogPrintf(DEBUG_ALL, "Find Ble [%s],rssi:%d", list->list[i].broadcaseName, list->list[i].rssi);
            tmos_memcpy(bleTry.mac, list->list[i].addr, 6);
            bleTry.addrType = list->list[i].addrType;
            bleChangeFsm(BLE_CONN);
            return;
        }
    }
    LogMessage(DEBUG_ALL, "no find my ble");
    bleChangeFsm(BLE_SCAN);
}

/**************************************************
@bref		连接完成调
@param
@return
@note
**************************************************/

void bleConnCallBack(void)
{
    LogMessage(DEBUG_ALL, "connect success");
    bleChangeFsm(BLE_READY);
    dynamicParam.bleLinkCnt = 0;
    dynamicParamSaveAll();
    tmos_set_event(appCentralTaskId, APP_WRITEDATA_EVENT);
}

/**************************************************
@bref		连接完成调
@param
@return
@note
**************************************************/

void bleTryInit(void)
{
	tmos_memset(&bleTry, 0, sizeof(bleScanTry_s));
}

/**************************************************
@bref		模式启动
@param
@return
@note
**************************************************/

static void modeStart(void)
{
	if (sysparam.pwrOnoff == 0)
	{
		LogPrintf(DEBUG_ALL, "System power key was off");
		modeTryToDone();
		return;
	}
    uint16_t year;
    uint8_t month, date, hour, minute, second;
    portGetRtcDateTime(&year, &month, &date, &hour, &minute, &second);
    ledStatusUpdate(SYSTEM_LED_RUN, 1);
    sysinfo.runStartTick = sysinfo.sysTick;
    sysinfo.gpsuploadonepositiontime = 180;
    updateRTCtimeRequest();
    delMsgReqSet();
    portFsclkChange(0);
    portLdrGpioCfg(1);
    if (sysinfo.mode4First == 0)
    {
		sysinfo.mode4First = 1;
//		lbsRequestSet(DEV_EXTEND_OF_MY);
		wifiRequestSet(DEV_EXTEND_OF_MY);
    	gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
    	netRequestSet();
    }
    sysinfo.nonetTick = 0;
    switch (sysparam.MODE)
    {
        case MODE1:
            portGsensorCtl(0);
            dynamicParam.startUpCnt++;
            dynamicParamSaveAll();
            portSetNextAlarmTime();
            break;
        case MODE2:
            portGsensorCtl(1);
            if (sysparam.accctlgnss == 0)
            {
                gpsRequestSet(GPS_REQUEST_GPSKEEPOPEN_CTL);
            }
            break;
        case MODE3:
            portGsensorCtl(0);
            dynamicParam.startUpCnt++;
            dynamicParamSaveAll();
            break;
        case MODE21:
            portGsensorCtl(1);
            portSetNextAlarmTime();
            break;
        case MODE23:
            portGsensorCtl(1);
            break;
        /*离线模式*/
        case MODE4:
			portGsensorCtl(0);
			modulePowerOn();
		    netResetCsqSearch();
		    portSetNextAlarmTime();
		    portSetNextMode4AlarmTime();
		    changeModeFsm(MODE_RUNING);
        	return;
        default:
            sysparam.MODE = MODE2;
            paramSaveAll();
            break;
    }
    LogPrintf(DEBUG_ALL, "modeStart==>%02d/%02d/%02d %02d:%02d:%02d", year, month, date, hour, minute, second);
    LogPrintf(DEBUG_ALL, "Mode:%d, startup:%d", sysparam.MODE, dynamicParam.startUpCnt);
    //lbsRequestSet(DEV_EXTEND_OF_MY);
    wifiRequestSet(DEV_EXTEND_OF_MY);
    gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
    
    modulePowerOn();
    netResetCsqSearch();
    changeModeFsm(MODE_RUNING);
}

static void sysRunTimeCnt(void)
{
    static uint8_t runTick = 0;
    if (++runTick >= 180)
    {
        runTick = 0;
        dynamicParam.runTime++;
        portSaveStep();
    }
}

static void moduleErrDet(void)
{
	static uint8_t tick = 0;
	if (getModulePwrState() == 0 && getTerminalAccState())
	{
		tick++;
		if (tick >= 60)
		{
		    modulePowerOn();
		}
	}
	else
	{
		tick = 0;
	}
	LogPrintf(DEBUG_ALL, "moduleErrDet==>tick = %d", tick);
}

/**************************************************
@bref		模式运行
@param
@return
@note
**************************************************/

static void modeRun(void)
{
    static uint8_t runtick = 0;
    switch (sysparam.MODE)
    {
        case MODE1:
        case MODE3:
            //该模式下工作3分半钟
            if ((sysinfo.sysTick - sysinfo.runStartTick) >= 210)
            {
                gpsRequestClear(GPS_REQUEST_ALL);
                changeModeFsm(MODE_STOP);
            }
            modeShutDownQuickly();
            break;
        case MODE2:
            //该模式下每隔3分钟记录时长
            sysRunTimeCnt();
            gpsUploadPointToServer();
            break;
        case MODE21:
        case MODE23:
            //该模式下无gps请求时，自动关机
            sysRunTimeCnt();
            modeShutDownQuickly();
            gpsUploadPointToServer();
            moduleErrDet();
            break;
        case MODE4:
			sysRunTimeCnt();
			mode4CloseSocketQuickly();
        	break;
        default:
            LogMessage(DEBUG_ALL, "mode change unknow");
            sysparam.MODE = MODE2;
            break;
    }
}

/**************************************************
@bref		模式接收
@param
@return
@note
**************************************************/

static void modeStop(void)
{
    if (sysparam.MODE == MODE1 || sysparam.MODE == MODE3)
    {
        portGsensorCtl(0);
    }
    ledStatusUpdate(SYSTEM_LED_RUN, 0);
    modulePowerOff();
    changeModeFsm(MODE_DONE);
}


/**************************************************
@bref		等待唤醒模式
@param
@return
@note
**************************************************/

static void modeDone(void)
{
	static uint8_t motionTick = 0;
	//进入到这个模式就把sysinfo.canRunFlag置零，以免别的唤醒源让GPS未经电压检测就起来工作
	sysinfo.canRunFlag = 0;
	static uint8_t tick = 0;
	//保证关机时不会有gps请求
	if (sysparam.pwrOnoff == 0)
	{
		sysinfo.gpsRequest = 0;
	}
	//保证在mode done状态下 模组处于关机
	if (getModulePwrState() == 1)
	{
		if (++tick >= 60)
		    modulePowerOff();
	}
	else
	{
		tick = 0;
	}
	bleTryInit();
    if (sysinfo.gpsRequest)
    {
        motionTick = 0;
        volCheckRequestSet();
        ledStatusUpdate(SYSTEM_LED_RUN, 1);
        changeModeFsm(MODE_START);
        LogMessage(DEBUG_ALL, "modeDone==>Change to mode start");
    }
    else if (sysparam.MODE == MODE1 || sysparam.MODE == MODE3 || sysparam.MODE == MODE4)
    {
    	motionTick = 0;
        if (sysinfo.sleep && getModulePwrState() == 0 && isKeyPressInProgress() == 0)
        {
            tmos_set_event(sysinfo.taskId, APP_TASK_STOP_EVENT);
        }
    }
    else if (sysparam.MODE == MODE23 || sysparam.MODE == MODE21 || sysparam.MODE == MODE2)
    {
		/*检测gsensor是否有中断进来*/
		//LogPrintf(DEBUG_ALL, "motioncnt:%d, motionTick:%d ", motionCheckOut(sysparam.gsdettime), motionTick);
		if (motionCheckOut(sysparam.gsdettime) <= 0)
		{
			if (sysinfo.sleep && getModulePwrState() == 0 && isKeyPressInProgress() == 0)
			{
				tmos_set_event(sysinfo.taskId, APP_TASK_STOP_EVENT);
				motionTick = 0;
			}
		}
    }
}

/**************************************************
@bref		当前是否为运行模式
@param
@return
	1		是
	0		否
@note
**************************************************/

uint8_t isModeRun(void)
{
    if (sysinfo.runFsm == MODE_RUNING || sysinfo.runFsm == MODE_START)
        return 1;
    return 0;
}
/**************************************************
@bref		当前是否为done模式
@param
@return
	1		是
	0		否
@note
**************************************************/

uint8_t isModeDone(void)
{
    if (sysinfo.runFsm == MODE_DONE || sysinfo.runFsm == MODE_STOP)
        return 1;
    return 0;
}


/**************************************************
@bref		系统到时自动唤醒
@param
@return
@note
**************************************************/

static void sysAutoReq(void)
{
#define NO_NET_CHECK_TIME		30
    uint16_t year;
    uint8_t month, date, hour, minute, second;
    static uint16_t noNetTick = 0;
    if (sysparam.pwrOnoff == 0)
    {
		sysinfo.mode4NoNetTick = 0;
		sysinfo.sysMinutes = 0;
		LogPrintf(DEBUG_ALL, "sysAutoReq==>system power off");
		return;
    }

    if (sysparam.MODE == MODE1 || sysparam.MODE == MODE21)
    {
        portGetRtcDateTime(&year, &month, &date, &hour, &minute, &second);
        if (date == sysinfo.alarmDate && hour == sysinfo.alarmHour && minute == sysinfo.alarmMinute)
        {
            LogPrintf(DEBUG_ALL, "sysAutoReq==>%02d/%02d/%02d %02d:%02d:%02d", year, month, date, hour, minute, second);
            gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
            if (sysinfo.kernalRun == 0)
            {
            	volCheckRequestSet();
                tmos_set_event(sysinfo.taskId, APP_TASK_RUN_EVENT);
            }
        }
    }
    else if (sysparam.MODE == MODE4)
    {
    	portGetRtcDateTime(&year, &month, &date, &hour, &minute, &second);
        if (hour == sysinfo.mode4alarmHour && minute == sysinfo.mode4alarmMinute)
        {
            LogPrintf(DEBUG_ALL, "sysAutoReq==>MODE4:%02d/%02d/%02d %02d:%02d:%02d", year, month, date, hour, minute, second);
            gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
            netRequestSet();
            if (sysinfo.kernalRun == 0)
            {
            	volCheckRequestSet();
                tmos_set_event(sysinfo.taskId, APP_TASK_RUN_EVENT);
            }
        }
		if (isModeDone())
		{
			sysinfo.mode4NoNetTick++;
			LogPrintf(DEBUG_ALL, "mode4NoNetTick:%d", sysinfo.mode4NoNetTick);
			if (sysinfo.mode4NoNetTick >= NO_NET_CHECK_TIME)
			{
				sysinfo.mode4NoNetTick = 0;
                LogMessage(DEBUG_ALL, "mode 4 restoration network");
                if (sysinfo.kernalRun == 0)
                {
                	volCheckRequestSet();
                    tmos_set_event(sysinfo.taskId, APP_TASK_RUN_EVENT);
                    
                }
                changeModeFsm(MODE_START);
			}
		}
		else
		{
			sysinfo.mode4NoNetTick = 0;
		}
    }
    else
    {
        //模式2没网络逻辑
    	if (isModeDone() && sysparam.MODE == MODE2)
        {
			noNetTick++;
			LogPrintf(DEBUG_ALL, "mode2NoNetTick:%d ", noNetTick);
			if (noNetTick >= NO_NET_CHECK_TIME)
			{
				noNetTick = 0;
				LogMessage(DEBUG_ALL, "mode 2 restoration network");
                if (sysinfo.kernalRun == 0)
                {
                	volCheckRequestSet();
                    tmos_set_event(sysinfo.taskId, APP_TASK_RUN_EVENT);
                }
	            changeModeFsm(MODE_START);
			}
        }
        else
        {
			noNetTick = 0;
        }
        if (sysparam.gapMinutes != 0)
        {
            sysinfo.sysMinutes++;
            if (sysinfo.sysMinutes % sysparam.gapMinutes == 0)
            {
            	sysinfo.sysMinutes = 0;
				gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
                LogMessage(DEBUG_ALL, "upload period");
                if (sysinfo.kernalRun == 0)
                {
                	volCheckRequestSet();
                    tmos_set_event(sysinfo.taskId, APP_TASK_RUN_EVENT);
                }
            }
        }
    }
}

/**************************************************
@bref		电池低电关机检测
@param
@return
@note	0：正在检测				1：检测完成
**************************************************/

uint8_t SysBatDetection(void)
{
	static uint8_t waitTick = 0;
	/*开机检测电压*/
	if (sysinfo.volCheckReq == 0)
    {
		/* 由于MCU读取的外电电压也是模组的外电电压，因此每次打开kernal的时候，开启模组外电，否则ADC读不到电压 */
		moduleSupplyOn();
        if (sysinfo.canRunFlag)
        {
			waitTick = 0;
			volCheckRequestClear();
			sysinfo.pwrDone = 0;
			LogMessage(DEBUG_ALL, "电池电压正常，正常开机");
        }
        else
        {
			if (waitTick++ >= 6)
			{
				waitTick = 0;
				modeTryToDone();
				volCheckRequestClear();
				sysinfo.pwrDone = 0;
				LogMessage(DEBUG_ALL, "电池电压偏低，关机");
			}
        }
        LogPrintf(DEBUG_ALL, "SysBatDetection==>%.2f", sysinfo.outsidevoltage);
        return 0;
    }
    waitTick = 0;
	/*工作时检测电压*/
	/* 由于不给模组供电无法读取adc */
	/*不能工作*/
	if (sysinfo.canRunFlag == 0)
	{
		/*如果正在工作*/
		if (sysinfo.runFsm == MODE_RUNING)
		{
			modeTryToStop();
			if (sysparam.MODE == MODE2 || sysparam.MODE == MODE21 || sysparam.MODE == MODE23)
			{
				if (sysinfo.gsensorOnoff == 1)
				{
					portGsensorCtl(0);
				}
			}
		}
		else if (sysinfo.runFsm == MODE_START)
		{
			modeTryToDone();
		}
	}
	/*可以工作*/
	else
	{
		if (sysparam.MODE == MODE2 || sysparam.MODE == MODE21 || sysparam.MODE == MODE23)
		{
			if (sysinfo.gsensorOnoff == 0 && sysparam.pwrOnoff)
			{
				portGsensorCtl(1);
			}
		}
		
	}
	return 1;
}

/**************************************************
@bref		电池检查请求设置
@param
@return
@note
**************************************************/

void volCheckRequestSet(void)
{
	sysinfo.volCheckReq = 0;
	sysinfo.canRunFlag = 0;
	LogMessage(DEBUG_ALL, "volCheckRequestSet==>OK");
}

/**************************************************
@bref		电池检查请求清除
@param
@return
@note
**************************************************/

void volCheckRequestClear(void)
{
	sysinfo.volCheckReq = 1;
	LogMessage(DEBUG_ALL, "volCheckRequestClear==>OK");
}

/**************************************************
@bref		模式运行任务
@param
@return
@note
**************************************************/

static void sysModeRunTask(void)
{
	if (SysBatDetection() != 1)
	{
		return;
	}
    switch (sysinfo.runFsm)
    {
        case MODE_START:
            modeStart();
            break;
        case MODE_RUNING:
            modeRun();
            break;
        case MODE_STOP:
            modeStop();
            break;
        case MODE_DONE:
            modeDone();
            break;
    }
}

/**************************************************
@bref		基站上送请求
@param
@return
@note
**************************************************/

void lbsRequestSet(uint8_t ext)
{
    sysinfo.lbsRequest = 1;
    sysinfo.lbsExtendEvt |= ext;
}

/**************************************************
@bref		清除基站上送请求
@param
@return
@note
**************************************************/

void lbsRequestClear(void)
{
	sysinfo.lbsRequest = 0;
    sysinfo.lbsExtendEvt = 0;
}

static void sendLbs(void)
{
    if (sysinfo.lbsExtendEvt & DEV_EXTEND_OF_MY)
    {
        protocolSend(NORMAL_LINK, PROTOCOL_19, NULL);
    }
    if (sysinfo.lbsExtendEvt & DEV_EXTEND_OF_BLE)
    {
        protocolSend(BLE_LINK, PROTOCOL_19, NULL);
    }
    sysinfo.lbsExtendEvt = 0;
}
/**************************************************
@bref		基站上送任务
@param
@return
@note
**************************************************/

static void lbsRequestTask(void)
{
    if (sysinfo.lbsRequest == 0)
    {
        return;
    }
    if (primaryServerIsReady() == 0)
        return;
    sysinfo.lbsRequest = 0;
    if (sysparam.protocol == ZT_PROTOCOL_TYPE)
    {
        moduleGetLbs();
        startTimer(20, sendLbs, 0);
    }
}

/**************************************************
@bref		wifi超时处理
@param
@return
@note
**************************************************/

void wifiTimeout(void)
{
	LogMessage(DEBUG_ALL, "wifiTimeout");
	lbsRequestSet(DEV_EXTEND_OF_MY);
	wifiRequestClear();
	wifiTimeOutId = -1;
}

/**************************************************
@bref		wifi应答成功
@param
@return
@note
**************************************************/

void wifiRspSuccess(void)
{
	if (wifiTimeOutId != -1)
	{
		stopTimer(wifiTimeOutId);
		wifiTimeOutId = -1;
		LogMessage(DEBUG_ALL, "wifiRspSuccess");
		sysinfo.lockTick = 125;
	}
}

/**************************************************
@bref		设置WIFI上送请求
@param
@return
@note
**************************************************/

void wifiRequestSet(uint8_t ext)
{
    sysinfo.wifiRequest = 1;
    sysinfo.wifiExtendEvt |= ext;
}

/**************************************************
@bref		清除WIFI上送请求
@param
@return
@note
**************************************************/

void wifiRequestClear(void)
{
	sysinfo.wifiRequest = 0;
	sysinfo.wifiExtendEvt = 0;
	wifiRspSuccess();
}


/**************************************************
@bref		WIFI上送任务
@param
@return
@note
**************************************************/

static void wifiRequestTask(void)
{
	static uint8_t tick = 0;
    if (sysinfo.wifiRequest == 0)
    {
    	tick = 0;
        return;
    }
    if (primaryServerIsReady() == 0)
        return;
    //有AGPS先等AGPS数据读取完完再发WIFI请求
    if (sysinfo.agpsRequest)
    {
    	tick++;
    	if (tick >= 15)
    	{
    		tick = 0;
			goto __WIFI;
    	}
    	return;
    }
__WIFI:
	tick = 0;
    sysinfo.wifiRequest = 0;
    if (sysparam.protocol == ZT_PROTOCOL_TYPE)
    {
		startTimer(30, moduleGetWifiScan, 0);        
        wifiTimeOutId = startTimer(300, wifiTimeout, 0);
    }
}

/**************************************************
@bref		唤醒设备
@param
@return
@note
**************************************************/
void wakeUpByInt(uint8_t      type, uint8_t sec)
{
    switch (type)
    {
        case 0:
            sysinfo.ringWakeUpTick = sec;
            break;
        case 1:
        	if (sec > sysinfo.cmdTick)
            	sysinfo.cmdTick = sec;
            break;
        case 2:
        	sysinfo.irqTick = sec;
        	break;
    }

    portSleepDn();
}

/**************************************************
@bref		查询是否需要休眠
@param
@return
@note
**************************************************/

static uint8_t getWakeUpState(void)
{
    //打印串口信息时，不休眠
    if (sysinfo.logLevel == DEBUG_FACTORY)
    {
        return 1;
    }
    //未联网，不休眠
    if (primaryServerIsReady() == 0 && isModeRun() && sysparam.MODE != MODE4)
    {
        return 2;
    }
    //开gps时，不休眠
    if (sysinfo.gpsRequest != 0)
    {
        return 3;
    }
    if (sysinfo.ringWakeUpTick != 0)
    {
        return 4;
    }
    if (sysinfo.cmdTick != 0)
    {
        return 5;
    }
    if (sysinfo.irqTick != 0)
    {
		return 6;
    }
    if (sysparam.MODE == MODE4 && isModeRun() && isModuleOfflineStatus() == 0)
    {
    	return 7;
    }
    //非0 时强制不休眠
    return 0;
}

/**************************************************
@bref		自动休眠
@param
@return
@note
**************************************************/

void autoSleepTask(void)
{
    static uint8_t flag = 0;
    if (sysinfo.ringWakeUpTick != 0)
    {
        sysinfo.ringWakeUpTick--;
    }
    if (sysinfo.cmdTick != 0)
    {
        sysinfo.cmdTick--;
    }
    if (sysinfo.irqTick != 0)
    {
		sysinfo.irqTick--;
    }
    uint8_t a = getWakeUpState();
    //LogPrintf(DEBUG_ALL, "getWakeUpState:%d", a);
    if (a)
    {
        portSleepDn();
        if (flag != 0)
        {
            flag = 0;
            portFsclkChange(0);
            LogMessage(DEBUG_ALL, "disable sleep");
            tmos_start_reload_task(sysinfo.taskId, APP_TASK_POLLUART_EVENT, MS1_TO_SYSTEM_TIME(50));
            sysinfo.sleep = 0;
            //portDebugUartCfg(1);
        }
    }
    else
    {
        portSleepEn();
        if (flag != 1)
        {
            flag = 1;
            portFsclkChange(1);
            LogMessage(DEBUG_ALL, "enable sleep");
            tmos_stop_task(sysinfo.taskId, APP_TASK_POLLUART_EVENT);
            sysinfo.sleep = 1;
            //portDebugUartCfg(0);
        }
    }
}

/**************************************************
@bref		每天重启
@param
@note
**************************************************/

static void rebootEveryDay(void)
{
    sysinfo.sysTick++;
    if (sysparam.pwrOnoff)
    {
    	if (sysinfo.ledTick > 0)
			sysinfo.ledTick--;
    }
    //    if (sysinfo.sysTick < 86400)
    //        return ;
    //    if (sysinfo.gpsRequest != 0)
    //        return ;
    //    portSysReset();
}

static void tiltDetectionTask(void)
{
	static uint8_t tick = 0;
    if (sysparam.tiltalm == 0)
    {
        return;
    }
    if (sysinfo.doMotionFlag == 0)
    {
    	tick = 0;
        return;
    }
    /*可执行倾斜*/
	if (sysinfo.tiltNormalCnt >= 2)
	{
//		if (ROLL_DET)
//		{
//			tick++;
//			if (tick > sysparam.tiltThreshold)
//			{
//				alarmRequestSet(ALARM_TILT_REQUEST);
//		        LogMessage(DEBUG_ALL, "Tilt alarm");
//		    	jt808UpdateAlarm(JT808_TILT_ALARM, 1);
//				lbsRequestSet(DEV_EXTEND_OF_MY);
//				wifiRequestSet(DEV_EXTEND_OF_MY);
//				gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
//				tick = 0;
//				sysinfo.doMotionFlag = 0;
//				sysinfo.tiltNormalCnt = 0;
//			}
//		}
//		else
//		{
//			tick= 0;
//			sysinfo.doMotionFlag = 0;
//			sysinfo.tiltNormalCnt = 0;
//		}
		
	}
	/*不可执行倾斜*/
	else
	{
		tick = 0;
		LogMessage(DEBUG_ALL, "Tilt alarm cancel");
		sysinfo.doMotionFlag = 0;
		sysinfo.tiltNormalCnt = 0;
	}

}


/**************************************************
@bref		计算设备正常状态时间
@param
@return
@note	
如果设备处于暗状态或者处于平放状态，每一分钟计数+1
**************************************************/

void calculateNormalTime(void)
{
	if (LDR_READ)
	{
		/*暗*/
		if (sysinfo.ldrDarkCnt < 10)
		{
			sysinfo.ldrDarkCnt++;
		}
	}
}

/**************************************************
@bref		感光任务
@param
@return
@note
**************************************************/
static void lightDetectionTask(void)
{
	if (sysparam.ldrEn == 0)
		return;
	//LogPrintf(DEBUG_ALL, "ldr:%d", LDR_READ);
	if (sysinfo.ldrIrqFlag)
	{
		//亮
		if (sysinfo.ldrDarkCnt >= 2)
		{
			LogMessage(DEBUG_ALL, "Light alarm");
			alarmRequestSet(ALARM_LIGHT_REQUEST);
			gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
			jt808UpdateAlarm(JT808_LIGHT_ALARM, 1);
			//lbsRequestSet(DEV_EXTEND_OF_MY);
			wifiRequestSet(DEV_EXTEND_OF_MY);		
		}
		else
		{
			LogMessage(DEBUG_ALL, "Light alarm cancel");
		}
		sysinfo.ldrIrqFlag = 0;
		sysinfo.ldrDarkCnt = 0;
	}
	else
	{
		//暗
		
	}
}

/**************************************************
@bref       gsensor检查任务
@param
@note
**************************************************/
static void gsensorRepair(void)
{
    portGsensorCtl(1);
    LogMessage(DEBUG_ALL, "repair gsensor");
}

static void gsCheckTask(void)
{
    static uint8_t tick = 0;
    static uint8_t errorcount = 0;
    if (sysinfo.gsensorOnoff == 0)
    {
        tick = 0;
        return;
    }

    tick++;
    if (tick % 60 == 0)
    {
        tick = 0;
        if (readInterruptConfig() != 0)
        {
            LogMessage(DEBUG_ALL, "gsensor error");
            portGsensorCtl(0);
            startTimer(20, gsensorRepair, 0);

        }
        else
        {
            errorcount = 0;
        }
    }
}

/**************************************************
@bref		system shutdown callback
@param
@return
@note
**************************************************/

void systemShutDownCB(void)
{
	tmos_start_task(sysinfo.taskId, APP_TASK_SYSTEM_SHUTDOWN_EVENT, MS1_TO_SYSTEM_TIME(3000));
}

/**************************************************
@bref		表示正在按下按键
@param
@return
@note		目的是为了不让kernel关闭
**************************************************/

uint8_t isKeyPressInProgress(void)
{
	return sysinfo.keypress;
}

/**************************************************
@bref		按键检测
@param
@return
@note
 once防止持续长按按着不松
**************************************************/
#define KEY_ON		1
#define KEY_OFF		0
#define KEY_RESULT_PRESS	1
#define KEY_RESULT_CLICK	2
#define KEY_LONG_PRESS_TIME		25
void keyScan(void)
{
	/* 判断长按按键 */
    static uint32_t cnt = 0;
    /* 判断连击按键 */
    static uint8_t laststatus = 0;	//作用是按下松开后 如果在500ms之内如果还按下去就会把laststatus置1
    static int8_t clicktime;		//点击之间的间隔
    static uint8_t clickcnt = 1;	//点击次数
    /* 防止按着不松手 */
	static uint8_t once = 0;		//0：检测到松开过按键	1:已经判断为一种按键方式了 如果不松开继续按下去是不会判断新的按键方式的  用于防止一直按着键不放
	/* 按键类型判断结果 */
	uint8_t result = 0;
    if (KEY_READ == KEY_ON)//发现按键被按下或者被按下过
    {
    	sysinfo.keypress = 1;
    	//正在处理开关机逻辑，不再计算按键按下的时间
    	//目的是为了防止执行关机的过程中，如果再次按下，当设备执行完关机又立马回去执行开机
    	if (sysinfo.pwrDone)
    		return;
    	
    	//只要按下，灯就工作30秒
		if (sysparam.ledctrl && sysparam.pwrOnoff)
			sysinfo.ledTick = 30;
        cnt++;
        //判断为长按
        if (cnt >= KEY_LONG_PRESS_TIME && once == 0)
        {
            once = 1;
            result = KEY_RESULT_PRESS;
            LogMessage(DEBUG_ALL, "Key Press");
        }

        if (laststatus && once == 0)
        {
            laststatus = 0;
            clickcnt++;
        }
        //连按三下
        if (clickcnt == 3 && once == 0)
        {
            once = 1;
            clickcnt = 0;
            LogMessage(DEBUG_ALL, "Key Triclick");
        }
        //连按n次之间的间隔
        if (cnt >= 10)
        {
            clickcnt = 0;
        }
        clicktime = 5;
    }
    //松开按键
    if (KEY_READ == KEY_OFF)
    {
    	sysinfo.keypress = 0;
    	if (cnt < KEY_LONG_PRESS_TIME && cnt > 0)
    	{
			result = KEY_RESULT_CLICK;
			LogMessage(DEBUG_ALL, "Key Click");
    	}
        clicktime--;
        cnt = 0;
        once = 0;
        if (clicktime >= 0 )
        {
            laststatus = 1;
        }
        else
        {
            laststatus = 0;
            clickcnt = 1;
        }
        if (clicktime <= -100)//防止减过头了
        {
            clicktime = -1;
        }
    }

    /* 执行结果 */
    if (result == KEY_RESULT_PRESS)
    {
		if (sysparam.pwrOnoff && sysinfo.pwrDone == 0)
		{
			LogPrintf(DEBUG_ALL, "System power off");
			sysinfo.gpsRequest = 0;
		    sysinfo.alarmRequest = 0;
		    sysinfo.wifiRequest = 0;
		    sysinfo.lbsRequest = 0;
		    sysinfo.wifiExtendEvt = 0;
		    sysinfo.lbsExtendEvt = 0;
		    portLdrGpioCfg(0);
			portGsensorCtl(0);
			alarmRequestSet(ALARM_SHUTDOWN_REQUEST);
			ledStatusUpdate(SYSTEM_LED_RUN, 0);
			sysinfo.pwrDone = 1;
			systemShutDownCB();
			paramSaveAll();
		}
		else if (sysparam.pwrOnoff == 0 && sysinfo.pwrDone == 0)
		{
			sysparam.pwrOnoff = 1;
			gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
			LogPrintf(DEBUG_ALL, "System power on");
			sysinfo.pwrDone = 1;
			sysinfo.ledTick = 180;
			ledStatusUpdate(SYSTEM_LED_RUN, 1);
			paramSaveAll();
		}
		else
		{
			LogPrintf(DEBUG_ALL, "System in progress");
		}
		
    }
//    else if (result == KEY_RESULT_CLICK)
//    {
//		if (sysparam.ledctrl && sysparam.pwrOnoff)
//			sysinfo.ledTick = 30;
//    }
    //LogPrintf(DEBUG_ALL, "soskeycnt=%d", cnt);
}

void powerKeyTask(void)
{
	static uint8 tick = 0;
	if (KEY_READ == KEY_ON)
	{
		tick++;
		LogPrintf(DEBUG_ALL, "Ready to power %s", sysparam.pwrOnoff ? "Off" : "On");
		if (tick >= 2)
		{
			if (sysparam.pwrOnoff)
				sysparam.pwrOnoff = 0;
			else
				sysparam.pwrOnoff = 1;
			paramSaveAll();
		}
		return;
	}
	tick = 0;
}

/**************************************************
@bref		123上报任务
@param
@return
@note
**************************************************/
static void mode123UploadTask(void)
{
	gpsinfo_s *gpsinfo;
	static uint32_t uploadTick = 0, nofixTick = 0;
	if (gpsRequestGet(GPS_REQUEST_123_CTL) == 0)
	{
		uploadTick = 0;
		sysinfo.mode123RunTick = 0;
		nofixTick = 0;
		return;
	}
	if (sysinfo.mode123Min == 0)
	{
		uploadTick = 0;
		sysinfo.mode123RunTick = 0;
		nofixTick = 0;
		return;
	}
	/* 运行时间结束 */
	if (sysinfo.mode123RunTick >= sysinfo.mode123Min * 60)
	{
		uploadTick = 0;
		sysinfo.mode123RunTick = 0;
		nofixTick = 0;
		gpsRequestClear(GPS_REQUEST_123_CTL);
		return;
	}
	if (sysinfo.mode123GpsFre < GPS_UPLOAD_GAP_MAX)
		calculateTheGPSCornerPoint();
	LogPrintf(DEBUG_ALL, "123runtick:%d nofixTick:%d uploadTick:%d | mode123Min:%d mode123GpsFre:%d", 
				sysinfo.mode123RunTick, nofixTick, uploadTick, sysinfo.mode123Min * 60, sysinfo.mode123GpsFre);
	/* 只要一进来就开始计时 */
	sysinfo.mode123RunTick++;
	nofixTick++;
	gpsinfo = getCurrentGPSInfo();
	if (gpsinfo->fixstatus == 0)
	{
		//不定位 ，取消这次sports指令
		if (nofixTick >= sysinfo.mode123Min * 60)
		{
			sysinfo.mode123RunTick = sysinfo.mode123Min * 60;
			wifiRequestSet(DEV_EXTEND_OF_MY);
		}
		return;
	}
	nofixTick = 0;
	uploadTick++;
	
	if (uploadTick >= sysinfo.mode123GpsFre)
	{
		protocolSend(NORMAL_LINK, PROTOCOL_12, getCurrentGPSInfo());
        jt808SendToServer(TERMINAL_POSITION, getCurrentGPSInfo());
        initLastPoint(getCurrentGPSInfo());
        uploadTick = 0;
	}
}

/**************************************************
@bref		1秒任务
@param
@return
@note
**************************************************/

void taskRunInSecond(void)
{
    rebootEveryDay();
    gpsRequestTask();	//gps状态设置/清除
    motionCheckTask();	//gsensor状态设置/清除
	if (sysparam.pwrOnoff)
	{
	    netConnectTask();
	    gsCheckTask();
	    voltageCheckTask();
	    alarmRequestTask();
	    gpsUplodOnePointTask();
	    lbsRequestTask();
	    wifiRequestTask();
	    mode123UploadTask();
	    serverManageTask();
    }
    autoSleepTask();
    sysModeRunTask();
}


/**************************************************
@bref		串口调试接收
@param
@return
@note
**************************************************/
void doDebugRecvPoll(uint8_t *msg, uint16_t len)
{
    static uint8_t gpsRestore[DEBUG_BUFF_SIZE + 1];
    static uint16_t size = 0;
    uint16_t i, begin;
    if (len + size > DEBUG_BUFF_SIZE)
    {
        size = 0;
    }
    memcpy(gpsRestore + size, msg, len);
    size += len;
    begin = 0;
    for (i = 0; i < size; i++)
    {
        if (gpsRestore[i] == '\n')
        {
            atCmdParserFunction(gpsRestore + begin, i - begin + 1);
            begin = i + 1;
        }
    }
    if (begin != 0)
    {
        memmove(gpsRestore, gpsRestore + begin, size - begin);
        size -= begin;
    }
}


/*调试专用任务*/
void debugtask(void)
{
    static uint8_t fsm;
    static uint8_t tick;
    switch(fsm)
    {
    case 0:
        modulePowerOn();
        //portGsensorCtl(1);
        fsm = 1;
        break;

    case 1:
        tick++;
        if (tick>=5)
        {
            tick = 0;
        	fsm = 3;
        	modulePowerOff();
//        	moduleSleepCtl(1);
        }
//        else
//        {
//			netConnectTask();
//			serverManageTask();
//        }   
        break;
    case 2:
    	LogMessage(DEBUG_ALL, "SHUT DOWN");
        tmos_stop_task(sysinfo.taskId, APP_TASK_POLLUART_EVENT);
        tmos_set_event(sysinfo.taskId, APP_TASK_STOP_EVENT);
		portDebugUartCfg(0);
        portSleepEn();
        fsm = 3;
        break;
    case 3:
        tick++;
        if (tick>=5)
        {
            modulePowerOff();
            tick = 0;
            fsm = 4;
            tmos_stop_task(sysinfo.taskId, APP_TASK_POLLUART_EVENT);
            tmos_set_event(sysinfo.taskId, APP_TASK_STOP_EVENT);
            portSleepEn();
        }
        break;
    }
}

/**************************************************
@bref		系统启动时配置
@param
@return
@note
**************************************************/

void myTaskPreInit(void)
{
    tmos_memset(&sysinfo, 0, sizeof(sysinfo));
    paramInit();
    //sysinfo.logLevel = 9;
    SetSysClock(CLK_SOURCE_PLL_60MHz);
    portGpioSetDefCfg();
    portModuleGpioCfg(1);
    portGpsGpioCfg(1);
    portLedGpioCfg(1);
    portAdcCfg(1);
	portLdrGpioCfg(0);
    portWdtCfg();
    portKeyCfg();
    portDebugUartCfg(1);
    bleTryInit();
    socketListInit();
    
	sysinfo.ledTick = 180;//开机灯工作3分钟
	
    createSystemTask(ledTask, 1);
    createSystemTask(outputNode, 2);
	createSystemTask(keyScan, 1);
    sysinfo.sysTaskId = createSystemTask(taskRunInSecond, 10);

    LogMessage(DEBUG_ALL, ">>>>>>>>>>>>>>>>>>>>>");
    LogPrintf(DEBUG_ALL, "SYS_GetLastResetSta:%x", SYS_GetLastResetSta());
	if (sysparam.pwrOnoff)
	{
		volCheckRequestSet();
		ledStatusUpdate(SYSTEM_LED_RUN, 1);
	}
	else
	{
		volCheckRequestClear();
	}
}

/**************************************************
@bref		tmos 任务回调
@param
@return
@note
**************************************************/

static tmosEvents myTaskEventProcess(tmosTaskID taskID, tmosEvents events)
{
	uint8_t ret;
	uint16_t year;
	uint8_t  month = 0, date = 0, hour = 0, minute = 0, second = 0;
    if (events & SYS_EVENT_MSG)
    {
        uint8 *pMsg;
        if ((pMsg = tmos_msg_receive(sysinfo.taskId)) != NULL)
        {
            tmos_msg_deallocate(pMsg);
        }
        return (events ^ SYS_EVENT_MSG);
    }

    if (events & APP_TASK_KERNAL_EVENT)
    {
        kernalRun();
        moduleRequestTask();
        portWdtFeed();
        return events ^ APP_TASK_KERNAL_EVENT;
    }

    if (events & APP_TASK_POLLUART_EVENT)
    {
        pollUartData();
        portWdtFeed();
        return events ^ APP_TASK_POLLUART_EVENT;
    }

	if (events & APP_TASK_SYSTEM_SHUTDOWN_EVENT)
	{
		sysparam.pwrOnoff = 0;
		LogPrintf(DEBUG_ALL, "System power off done");
		
		if (sysinfo.runFsm <= MODE_RUNING)
		{
			modeTryToStop();
		}
		else
		{
			modeTryToDone();
		}
		portLdrGpioCfg(0);
		portGsensorCtl(0);
		sysinfo.pwrDone = 0;
		
		//关机的时候清空一些必要的参数
		sysinfo.mode123RunTick = 0;
		sysinfo.mode123Min = 0;
		sysinfo.mode123GpsFre = 0;
		if (sysinfo.logLevel == DEBUG_FACTORY)
			sysinfo.logLevel = 0;
		sysinfo.nmeaOutPutCtl = 0;
		paramSaveAll();
		return events ^ APP_TASK_SYSTEM_SHUTDOWN_EVENT;
	}
	
    if (events & APP_TASK_RUN_EVENT)
    {
        LogMessage(DEBUG_ALL, "Task kernal start");
    
        sysinfo.kernalRun = 1;
        /*重新配置IO*/
		portModuleGpioCfg(1);
		portGpsGpioCfg(1);
		portLedGpioCfg(1);
		portAdcCfg(1);
		portWdtCfg();
		
        tmos_start_reload_task(sysinfo.taskId, APP_TASK_KERNAL_EVENT, MS1_TO_SYSTEM_TIME(100));
        return events ^ APP_TASK_RUN_EVENT;
    }
    
    if (events & APP_TASK_STOP_EVENT)
    {
        LogMessage(DEBUG_ALL, "Task kernal stop");
        sysinfo.kernalRun = 0;
        motionClear();
        /*关闭所有IO*/
		portAdcCfg(0);
		portModuleGpioCfg(0);
		portGpsGpioCfg(0);
		portLedGpioCfg(0);
		if (sysinfo.pwrDone)
		{
			sysinfo.pwrDone = 0;
		}
		portWdtCancel();
       	tmos_stop_task(sysinfo.taskId, APP_TASK_KERNAL_EVENT);
        return events ^ APP_TASK_STOP_EVENT;
    }
    
    if (events & APP_TASK_ONEMINUTE_EVENT)
    {
    	sysinfo.oneMinTick++;
    	sysAutoReq();
    	calculateNormalTime();
        LogMessage(DEBUG_ALL, "***************************Task one minutes**********************");
        LogPrintf(DEBUG_ALL,  "*Mode: %d, rungap: %d, System run: %d min, darktime: %d, oneminute: %d*", sysparam.MODE, sysparam.gapMinutes, sysinfo.sysMinutes, sysinfo.ldrDarkCnt, sysinfo.oneMinTick);
        LogMessage(DEBUG_ALL, "*****************************************************************");
        portGetRtcDateTime(&year, &month, &date, &hour, &minute, &second);
        if (hour == 0 && minute == 0)
        {
            LogPrintf(DEBUG_BLE, "time of zero");
            portClearStep();
        }
        return events ^ APP_TASK_ONEMINUTE_EVENT;
    }

    return 0;
}

/**************************************************
@bref		任务初始化
@param
@return
@note
**************************************************/

void myTaskInit(void)
{
    sysinfo.taskId = TMOS_ProcessEventRegister(myTaskEventProcess);
    tmos_set_event(sysinfo.taskId, APP_TASK_RUN_EVENT);
    tmos_start_reload_task(sysinfo.taskId, APP_TASK_POLLUART_EVENT, MS1_TO_SYSTEM_TIME(50));
    tmos_start_reload_task(sysinfo.taskId, APP_TASK_ONEMINUTE_EVENT, MS1_TO_SYSTEM_TIME(60000));

}

