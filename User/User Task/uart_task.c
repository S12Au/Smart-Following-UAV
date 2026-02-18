#include "FreeRTOS.h"
#include "queue.h"
#include "GY86.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "semphr.h"
#include "i2c.h"
#include "getPPM_task.h"
#include "FlightControl.h"
#include "autoconf.h"
#include <string.h>
#include <stdlib.h>

static int tokenEquals(const char* a, const char* b)
{
	if (a == 0 || b == 0)
	{
		return 0;
	}

	while (*a && *b)
	{
		char ca = *a;
		char cb = *b;

		if (ca >= 'A' && ca <= 'Z')
		{
			ca = (char)(ca - 'A' + 'a');
		}
		if (cb >= 'A' && cb <= 'Z')
		{
			cb = (char)(cb - 'A' + 'a');
		}

		if (ca != cb)
		{
			return 0;
		}

		a++;
		b++;
	}

	return (*a == '\0' && *b == '\0');
}

static int parsePidId(const char* s, FlightPidId_t* out)
{
	if (tokenEquals(s, "ra") || tokenEquals(s, "roll_angle"))
	{
		*out = FLIGHT_PID_ROLL_ANGLE;
		return 1;
	}
	if (tokenEquals(s, "pa") || tokenEquals(s, "pitch_angle"))
	{
		*out = FLIGHT_PID_PITCH_ANGLE;
		return 1;
	}
	if (tokenEquals(s, "rr") || tokenEquals(s, "roll_rate"))
	{
		*out = FLIGHT_PID_ROLL_RATE;
		return 1;
	}
	if (tokenEquals(s, "pr") || tokenEquals(s, "pitch_rate"))
	{
		*out = FLIGHT_PID_PITCH_RATE;
		return 1;
	}
	if (tokenEquals(s, "yr") || tokenEquals(s, "yaw_rate"))
	{
		*out = FLIGHT_PID_YAW_RATE;
		return 1;
	}
	if (tokenEquals(s, "alt") || tokenEquals(s, "altitude"))
	{
		*out = FLIGHT_PID_ALTITUDE;
		return 1;
	}

	return 0;
}

static int parseGainType(const char* s, FlightGainType_t* out)
{
	if (tokenEquals(s, "kp"))
	{
		*out = FLIGHT_GAIN_KP;
		return 1;
	}
	if (tokenEquals(s, "ki"))
	{
		*out = FLIGHT_GAIN_KI;
		return 1;
	}
	if (tokenEquals(s, "kd"))
	{
		*out = FLIGHT_GAIN_KD;
		return 1;
	}

	return 0;
}

static const char* pidName(FlightPidId_t id)
{
	switch (id)
	{
		case FLIGHT_PID_ROLL_ANGLE: return "roll_angle";
		case FLIGHT_PID_PITCH_ANGLE: return "pitch_angle";
		case FLIGHT_PID_ROLL_RATE: return "roll_rate";
		case FLIGHT_PID_PITCH_RATE: return "pitch_rate";
		case FLIGHT_PID_YAW_RATE: return "yaw_rate";
		case FLIGHT_PID_ALTITUDE: return "altitude";
		default: return "unknown";
	}
}

static void printPidAll(void)
{
	FlightPidId_t ids[] = {
		FLIGHT_PID_ROLL_ANGLE,
		FLIGHT_PID_PITCH_ANGLE,
		FLIGHT_PID_ROLL_RATE,
		FLIGHT_PID_PITCH_RATE,
		FLIGHT_PID_YAW_RATE,
		FLIGHT_PID_ALTITUDE
	};
	uint8_t i;

	printf("PID,BEGIN\r\n");
	for (i = 0; i < (uint8_t)(sizeof(ids) / sizeof(ids[0])); i++)
	{
		float kp = 0.0f;
		float ki = 0.0f;
		float kd = 0.0f;
		FlightControl_GetPidGain(ids[i], FLIGHT_GAIN_KP, &kp);
		FlightControl_GetPidGain(ids[i], FLIGHT_GAIN_KI, &ki);
		FlightControl_GetPidGain(ids[i], FLIGHT_GAIN_KD, &kd);
		printf("PID,%s,KP=%.5f,KI=%.5f,KD=%.5f\r\n", pidName(ids[i]), kp, ki, kd);
	}
	printf("PID,END\r\n");
}

static void handleUartCommand(char* line)
{
	char* cmd;
	char* arg1;
	char* arg2;
	char* arg3;
	char* endPtr;

	cmd = strtok(line, " \t");
	if (cmd == 0)
	{
		return;
	}

	if (!tokenEquals(cmd, "pid"))
	{
		return;
	}

	arg1 = strtok(0, " \t");
	if (arg1 == 0)
	{
		printf("PID,ERR,usage: pid show | pid set <ra|pa|rr|pr|yr|alt> <kp|ki|kd> <value>\r\n");
		return;
	}

	if (tokenEquals(arg1, "show"))
	{
		printPidAll();
		return;
	}

	if (tokenEquals(arg1, "set"))
	{
		FlightPidId_t pidId;
		FlightGainType_t gainType;
		float value;

		arg2 = strtok(0, " \t");
		arg3 = strtok(0, " \t");
		char* arg4 = strtok(0, " \t");

		if (arg2 == 0 || arg3 == 0 || arg4 == 0)
		{
			printf("PID,ERR,usage: pid set <ra|pa|rr|pr|yr|alt> <kp|ki|kd> <value>\r\n");
			return;
		}

		if (!parsePidId(arg2, &pidId))
		{
			printf("PID,ERR,unknown pid id\r\n");
			return;
		}

		if (!parseGainType(arg3, &gainType))
		{
			printf("PID,ERR,unknown gain type\r\n");
			return;
		}

		value = strtof(arg4, &endPtr);
		if (endPtr == arg4 || *endPtr != '\0')
		{
			printf("PID,ERR,invalid value\r\n");
			return;
		}

		if (!FlightControl_SetPidGain(pidId, gainType, value))
		{
			printf("PID,ERR,set failed\r\n");
			return;
		}

		{
			float kp = 0.0f;
			float ki = 0.0f;
			float kd = 0.0f;
			FlightControl_GetPidGain(pidId, FLIGHT_GAIN_KP, &kp);
			FlightControl_GetPidGain(pidId, FLIGHT_GAIN_KI, &ki);
			FlightControl_GetPidGain(pidId, FLIGHT_GAIN_KD, &kd);
			printf("PID,OK,%s,KP=%.5f,KI=%.5f,KD=%.5f\r\n", pidName(pidId), kp, ki, kd);
		}

		return;
	}

	printf("PID,ERR,unknown subcmd\r\n");
}

static void processUartRx(void)
{
	static char rxLine[96];
	static uint16_t rxLen = 0;
	uint8_t ch;
	uint8_t i;

	for (i = 0; i < 32; i++)
	{
		if (HAL_UART_Receive(&huart1, &ch, 1, 0) != HAL_OK)
		{
			break;
		}

		if (ch == '\r' || ch == '\n')
		{
			if (rxLen > 0)
			{
				rxLine[rxLen] = '\0';
				handleUartCommand(rxLine);
				rxLen = 0;
			}
			continue;
		}

		if (rxLen < (sizeof(rxLine) - 1))
		{
			rxLine[rxLen++] = (char)ch;
		}
		else
		{
			rxLen = 0;
			printf("PID,ERR,line too long\r\n");
		}
	}
}

void Uart_Send_Task()
{
	FlightDebugData_t dbg;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xPeriod = pdMS_TO_TICKS(1000 / CONFIG_UART_DEBUG_RATE_HZ);

	printf("FC,st,lk,cal,thr,r,p,y,rSp,pSp,ySp,rOut,pOut,yOut,m1,m2,m3,m4\r\n");
	while(1)
	{
		processUartRx();

		FlightControl_GetDebugSnapshot(&dbg);
		printf("FC,%d,%u,%u,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%u,%u,%u\r\n",
				(int)dbg.state,
				dbg.linkAlive,
				dbg.sensorCalibrated,
				dbg.throttle,
				dbg.roll,
				dbg.pitch,
				dbg.yaw,
				dbg.rollSp,
				dbg.pitchSp,
				dbg.yawRateSp,
				dbg.rollOut,
				dbg.pitchOut,
				dbg.yawOut,
				dbg.m1,
				dbg.m2,
				dbg.m3,
				dbg.m4);

		vTaskDelayUntil(&xLastWakeTime, xPeriod);
	
	}
}