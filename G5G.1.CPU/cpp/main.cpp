#include "hardware.h"
//#include "options.h"
//#include "hw_emac.h"
#include <CRC\CRC16.h>
#include <ComPort\ComPort.h>
#include <CRC\CRC16_CCIT.h>
#include <list.h>
#include <PointerCRC.h>
#include <SEGGER_RTT\SEGGER_RTT.h>
#include "hw_com.h"
//#include "G_TRM.h"
#include "TaskList.h"
#include "FLASH\nand_ecc.h"
#include "MQCODER\mqcoder.h"
#include <string.h>

//#include "G_TRM.H"


enum { VERSION = 0x101 };

//#pragma O3
//#pragma Otime

#ifndef _DEBUG
	static const bool __debug = false;
#else
	static const bool __debug = true;
#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct MainVars // NonVolatileVars  
{
	u32 timeStamp;

	u16 numDevice;
	u16 genFreq;				//	Частота генератора(Гц), 
	u16 winCount;				//	Количество временных окон(шт), 
	u16 winTime;				//	Длительность временного окна(мкс),
	u16 bLevel;					//	Уровень дискриминации МЗ(у.е), 
	u16 mLevel;					//	Уровень дискриминации БЗ(у.е),
	u16 disableFireNoVibration;	//	Отключение регистрации на стоянке(0 - нет, 1 - да)
	u16 levelNoVibration;		// Уровень вибрации режима отключения регистрации на стойнке(у.е)(ushort)
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static MainVars mv;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u32 fps;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct ManMsg 
{
	enum { VER = 1 };

	struct Header
	{
		byte ver;
		byte magic;
		u16 dataLen;
		u16 dataCRC;
		u16 crc;
	} 
	hdr;

	u16 data[16];
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static ManMsg manRcvData;
static u16 manTrmDtaa[4096];
static u16 manPckData[128 + WINDOW_SIZE*4 + 16];
static u16 manUnpData[128 + WINDOW_SIZE*4 + 16];
static u16 manTrmBaud = 0;
static u16 tlsTrmBaud = 0;

static const u16 manReqWord = 0x0900;
static const u16 manReqMask = 0xFF00;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//u16 txbuf[128 + 512 + 16];

//static u16 reqFireVoltage = 0;
//static u16 curFireVoltage = 300;


static u16 verDevice = VERSION;
static bool numDevValid = false;

i16 temp = 0;

u32 m_ts[WINDOW_SIZE];
u32 b_ts[WINDOW_SIZE];

static byte svCount = 0;

//static Rsp72 *curRsp72 = 0;

u16 fireAmp = 0;
u16 fireFreq = 3000;

i16 loc = 0;
i16 loc_min = 0x7FFF;
i16 loc_max = 0x8000;
u32 loc_gk = 0;
i16 loc_tension = 0;
u32 loc_period = 0;
u16 loc_req_count = 0;
u16 framErrorMask = 0;

i16 ax = 0, ay = 0, az = 0, at = 0;
u16 vibration;

enum { FRAM_ERROR_LOADVARS = 0, FRAM_ERROR_ECC, FRAM_ERROR_PARECC, FRAM_CORR_ECC, FRAM_NOACKR, FRAM_NOACKW, FRAM_LOAD_OK };

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SaveMainParams()
{
	svCount = 1;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u16 RequestMan_00(u16 *data, u16 len, u16 *out)
{
	if (len == 0 || len > 1 || out == 0) return 0;

	__packed u16 *start = out;

	*(out++)	= (manReqWord & manReqMask) | 0;
	*(out++)	= mv.numDevice;
	*(out++)	= verDevice;

	return out - start;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u16 RequestMan_10(u16 *data, u16 len, u16 *out)
{
	if (len == 0 || len > 1 || out == 0) return 0;

	__packed u16 *start = out;

	*(out++)	= (manReqWord & manReqMask) | 0x10;		//	1. Ответное слово (принятая команда)
	*(out++)	= mv.genFreq;							//	2. Частота генератора(Гц), 				
	*(out++)	= mv.winCount;							//	3. Количество временных окон(шт), 					
	*(out++)	= mv.winTime;							//	4. Длительность временного окна(мкс), 						
	*(out++)	= mv.mLevel;							//	5. Уровень дискриминации МЗ(у.е), 					
	*(out++)	= mv.bLevel;							//	6. Уровень дискриминации БЗ(у.е),	
	*(out++)	= mv.disableFireNoVibration;			//	7. Отключение регистрации на стоянке(0 - нет, 1 - да)	
	*(out++)	= mv.levelNoVibration;					//	8. Уровень вибрации режима отключения регистрации на стойнке(у.е)(ushort)	
	
	return out - start;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u16 RequestMan_20(u16 *data, u16 len, u16 *out)
{
	static u32 pt = 0;

	if (len == 0 || len > 1 || out == 0) return 0;

	u32 t = GetCYCCNT();
	u32 dt = t - pt;
	pt = t;
	
	dt /= MCK_MHz;

	if ((data[0] & 1) == 0 || (mv.disableFireNoVibration != 0 && vibration < mv.levelNoVibration ))
	{
		DisableGen();
	}
	else
	{
		EnableGen();
	};

	__packed u16 *start = out;

	out[0] = data[0];					//	1. Ответное слово (принятая команда)
	
	out++;

	u16 wc;
												
	*(out++)	= GetFireCount();				//	2. Количество вспышек(ushort)
	*(out++)	= GetGenWorkTime();				//	3. Наработка генератора(мин)(ushort)					
	*(out++)	= temp;							//	4. Температура в приборе(0.1 гр)(short)								
	*(out++)	= wc = mv.winCount;				//	5. Количество временных окон(шт)			
	*(out++)	= mv.winTime;					//	6. Длительность временного окна(мкс)
	*(out++)	= dt;							//	7,8. Период накопления БЗ,МЗ (мкс)(uint32)
	*(out++)	= dt>>16;						//	7,8. Период накопления БЗ,МЗ (мкс)(uint32)
	*(out++)	= framErrorMask;				//	9. Статус ошибог FRAM (у.е.)
	*(out++)	= Get_FBPOW1();					//	10. Напряжение питания (0.1В)
	*(out++)	= Get_FBPOW2();					//	11. Напряжение генератора (0.1В)
	*(out++)	= ax;							//	12. Ax (у.е)(short)
	*(out++)	= ay;							//	13. Ay (у.е)(short)
	*(out++)	= az;							//	14. Az (у.е)(short)
	*(out++)	= at;							//	15. At (0.1 гр)(short)
	*(out++)	= vibration;					//	16. Вибрация (у.е)(ushort)

	framErrorMask = 0;

	for (u16 i = 0; i < wc; i++)
	{
		u32 tm = m_ts[i]; m_ts[i] = 0;
		u32 tb = b_ts[i]; b_ts[i] = 0;

		out[0]		= MIN(tm, 0xFFFF);			//	17..x. Спектр МЗ(ushort)
		out[wc]	= MIN(tb, 0xFFFF);				//	x..y. Спектр БЗ(ushort)

		out++;
	};

	out += wc;

	return out - start;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u16 RequestMan_80(u16 *data, u16 len, u16 *out)
{
	if (len < 3 || len > 3 || out == 0) return 0;

	switch (data[1])
	{
		case 1:

			mv.numDevice = data[2];

			break;

		case 2:

			manTrmBaud = data[2] - 1;	//SetTrmBoudRate(data[2]-1);

			break;
	};

	out[0] = (manReqWord & manReqMask) | 0x80;

	return 1;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u16 RequestMan_90(u16 *data, u16 len, u16 *out)
{
	if (len < 2 || len > 2 || out == 0) return 0;

	switch(data[1])
	{
		case 0x01:	SetGenFreq(							mv.genFreq	= LIM(data[2], 4, 50)			);	break;	//	0x1 - Частота генератора(4..50 Гц), 
		case 0x02:	SetWindowCount(						mv.winCount	= LIM(data[2], 2, WINDOW_SIZE)	);	break;	//	0x2 - Количество временных окон(2..1024 шт), 
		case 0x03:	SetWindowTime(						mv.winTime	= LIM(data[2], 2, 512)			);	break;	//	0x3 - Длительность временного окна(2..2048 мкс), 
		case 0x04:	AD5312_Set(AD5312_CHANNEL_LEVEL_M,	mv.mLevel	= MIN(data[2], 0x3FF)			);	break;	//	0x4 - Уровень дискриминации МЗ(у.е), 
		case 0x05:	AD5312_Set(AD5312_CHANNEL_LEVEL_B,	mv.bLevel	= MIN(data[2], 0x3FF)			);	break;	//	0x5 - Уровень дискриминации БЗ(у.е),
		case 0x06:	mv.disableFireNoVibration						= MIN(data[2], 1)				;	break;	//	0x6 - Отключение регистрации на стоянке(0 - нет, 1 - да)	
		case 0x07:	mv.levelNoVibration								= data[2]						;	break;	//	0x7 - Уровень вибрации режима отключения регистрации на стойнке(у.е)(ushort)

		default:

			return false;
	};

	out[0] = (manReqWord & manReqMask) | 0x90;

	return 1;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u16 RequestMan_A0(u16 *data, u16 len, u16 *out)
{
	if (len < 2 || len > 3 || out == 0) return 0;

	switch(data[1])
	{
		case 0x01:	ResetGenWorkTime();	break;	

		default:

			return false;
	};

	out[0] = (manReqWord & manReqMask) | 0xA0;

	return 1;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u16 RequestMan_F0(u16 *data, u16 len, u16 *out)
{
	if (len > 1 || out == 0) return false;

	SaveMainParams();

	out[0] = (manReqWord & manReqMask) | 0xF0;

	return 1;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u16 RequestMan(u16 *data, u16 len, u16 *out)
{
	u16 r = 0;

	u16 t = data[0];

	if ((t & manReqMask) != manReqWord || len < 1)
	{
		return 0;
	};

	t = (t>>4) & 0xF;

	switch (t)
	{
		case 0x0: 	r = RequestMan_00(data, len, out); break;
		case 0x1: 	r = RequestMan_10(data, len, out); break;
		case 0x2: 	r = RequestMan_20(data, len, out); break;
		case 0x8: 	r = RequestMan_80(data, len, out); break;
		case 0x9:	r = RequestMan_90(data, len, out); break;
		case 0xA:	r = RequestMan_A0(data, len, out); break;
		case 0xF:	r = RequestMan_F0(data, len, out); break;
	};

	return r;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateMan()
{
	static byte i = 0;
	static ComPort::WriteBuffer wb;
	static ComPort::ReadBuffer rb;
	//static byte buf[1024];

	switch(i)
	{
		case 0:

			rb.data = &manRcvData;
			rb.maxLen = sizeof(manRcvData);
			comdsp.Read(&rb, ~0, US2COM(100));
			i++;

			break;

		case 1:

			if (!comdsp.Update())
			{
				ManMsg *out = (ManMsg*)manTrmDtaa;

				

				if (rb.recieved && rb.len >= sizeof(manRcvData.hdr)
					&& GetCRC16(&manRcvData.hdr, sizeof(manRcvData.hdr)) == 0
					&& GetCRC16(manRcvData.data, manRcvData.hdr.dataLen) == manRcvData.hdr.dataCRC)
				{
					u16 len = RequestMan(manRcvData.data, manRcvData.hdr.dataLen>>1, out->data);

					if (len != 0)
					{
						out->hdr.ver		= out->VER;
						out->hdr.magic		= 0x55;
						out->hdr.dataLen	= len*2;
						out->hdr.dataCRC	= GetCRC16(out->data, out->hdr.dataLen);
						out->hdr.crc		= GetCRC16(&out->hdr, sizeof(out->hdr)-2);

						wb.data = out;
						wb.len = sizeof(out->hdr) + out->hdr.dataLen;
					
						comdsp.Write(&wb);

						i++;

						break;
					};
				};

				i = 0;
			};

			break;
		
		case 2:

			if (!comdsp.Update())
			{
				i = 0;
			};

			break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void UpdateCom()
//{
//	static byte i = 0;
//	static CTM32 ctm;
//	static ComPort::WriteBuffer wb;
//	static ComPort::ReadBuffer rb;
//	static u16 buf[16];
//	
//	switch(i)
//	{
//		case 0:
//
//			buf[0] = 0x220;
//
//			wb.data = buf;
//			wb.len = 2;
//
//			comdsp.Write(&wb);
//
//			i++;
//
//			break;
//
//		case 1:
//
//			if (!comdsp.Update())
//			{
//				rb.data = buf;
//				rb.maxLen = sizeof(buf);
//				comdsp.Read(&rb, MS2COM(10), US2COM(500));
//
//				i++;
//			};
//
//			break;
//
//		case 2:
//
//			if (!comdsp.Update())
//			{
//				if (rb.recieved && rb.len >= 16 && buf[0] == 0x220)
//				{
//					loc				= buf[1];
//					loc_min			= MIN((i16)buf[2], loc_min);
//					loc_max			= MAX((i16)buf[2], loc_max);
//					loc_gk			+= buf[4];
//					loc_tension		= buf[5];
//					loc_period		+= buf[6]|(buf[7]<<16);
//					loc_req_count	+= 1;
//				};
//
//				i++;
//			};
//
//			break;
//
//		case 3:
//
//			if (ctm.Check(MS2CTM(100)))
//			{
//				i = 0;
//			};
//
//			break;
//	};
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static DSCSPI dscAccel;

//static i16 ax = 0, ay = 0, az = 0, at = 0;


static u8 txAccel[25] = { 0 };
static u8 rxAccel[50];

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool AccelReadReg(byte reg, u16 count)
{
	dscAccel.adr = (reg<<1)|1;
	dscAccel.alen = 1;
	//dscAccel.baud = 8000000;
	dscAccel.csnum = 0;
	dscAccel.wdata = 0;
	dscAccel.wlen = 0;
	dscAccel.rdata = rxAccel;
	dscAccel.rlen = count;

	return spiadxl.AddRequest(&dscAccel);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool AccelWriteReg(byte reg, u16 count)
{
	dscAccel.adr = (reg<<1)|0;
	dscAccel.alen = 1;
	dscAccel.csnum = 0;
	dscAccel.wdata = txAccel;
	dscAccel.wlen = count;
	dscAccel.rdata = 0;
	dscAccel.rlen = 0;

	return spiadxl.AddRequest(&dscAccel);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateAccel()
{
	static byte i = 0; 
	static i32 fx = 0, fy = 0, fz = 0, fv = 0, ft = 0;

	static TM32 tm;

	spiadxl.Update();

	switch (i)
	{
	case 0:

		txAccel[0] = 0x52;
		AccelWriteReg(0x2F, 1); // Reset

		i++;

		break;

	case 1:

		if (dscAccel.ready)
		{
			tm.Reset();

			i++;
		};

		break;

	case 2:

		if (tm.Check(35))
		{
			AccelReadReg(0x1E, 18);

			i++;
		};

		break;

	case 3:

		if (dscAccel.ready)
		{
			txAccel[0] = 0;
			AccelWriteReg(0x28, 1); // FILTER SETTINGS REGISTER

			i++;
		};

		break;

	case 4:

		if (dscAccel.ready)
		{
			txAccel[0] = 0;
			AccelWriteReg(0x2D, 1); // CTRL Set PORST to zero

			i++;
		};

		break;

	case 5:

		if (dscAccel.ready)
		{
			AccelReadReg(0x2D, 1);

			i++;
		};

		break;

	case 6:

		if (dscAccel.ready)
		{
			if (rxAccel[0] != 0)
			{
				txAccel[0] = 0;
				AccelWriteReg(0x2D, 1); // CTRL Set PORST to zero
				i--; 
			}
			else
			{
				txAccel[0] = 0;
				AccelWriteReg(0x2E, 1); // Self Test

				tm.Reset();
				i++;
			};
		};

		break;

	case 7:

		if (dscAccel.ready)
		{
			i++;
		};

		break;

	case 8:

		if (tm.Check(10))
		{
			AccelReadReg(6, 11); // X_MSB 

			i++;
		};

		break;

	case 9:

		if (dscAccel.ready)
		{
			i32 t = (rxAccel[0] << 8)  | rxAccel[1];
			i32 x = (rxAccel[2] << 24) | (rxAccel[3] << 16) | (rxAccel[4]  <<8);
			i32 y = (rxAccel[5] << 24) | (rxAccel[6] << 16) | (rxAccel[7]  <<8);
			i32 z = (rxAccel[8] << 24) | (rxAccel[9] << 16) | (rxAccel[10] <<8);

			fx += (x - fx) / 16;
			fy += (y - fy) / 16;
			fz += (z - fz) / 16;
			ft += (t - ft) / 4;

			ay = -(fz / 65536); 
			ax = -(fy / 65536); 
			az =  (fx / 65536);

			//at = 2500 + ((1852 - t) * 2000 + 91) / 181;
			at = 250 + ((1852 - ft) * 1132 + 512) / 1024;

			i32 vx = ABS(x - fx) / 64;
			i32 vy = ABS(y - fy) / 64;
			i32 vz = ABS(z - fz) / 64;

			fv += ((i32)(vx+vy+vz)-fv)/256;

			t = fv/1024;

			vibration = LIM(t, 0, 0xFFFF);

			i--;
		};

		break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateTemp()
{
	static byte i = 0;

	static DSCI2C dsctemp;//, dsc2;

//	static byte reg = 0;
	static u16 rbuf = 0;
	static byte buf[10];

	static TM32 tm;

	switch (i)
	{
		case 0:

			if (tm.Check(100))
			{
				if (!__debug) { HW::ResetWDT(); };

				buf[0] = 0;

				dsctemp.adr = 0x49;
				dsctemp.wdata = buf;
				dsctemp.wlen = 1;
				dsctemp.rdata = &rbuf;
				dsctemp.rlen = 2;
				dsctemp.wdata2 = 0;
				dsctemp.wlen2 = 0;

				if (I2C_AddRequest(&dsctemp))
				{
					i++;
				};
			};

			break;

		case 1:

			if (dsctemp.ready)
			{
				if (dsctemp.ack && dsctemp.readedLen == dsctemp.rlen)
				{
					i32 t = (i16)ReverseWord(rbuf);

					temp = (t * 10 + 64) / 128;
				};

				i = 0;
			};

			break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static u16 prevDstFV = 0;
//static u16 prevCurFV = 0;
//static i16 dCurFV = 0;
//static i16 dDstFV = 0;
/*
static void UpdateHV()
{
	static byte i = 0;
	static DSCI2C dsc;
	static byte wbuf[4];
	static byte rbuf[4];
	static TM32 tm;
	//static CTM32 ctm;
//	static i32 filtFV = 0;
	//static i32 filtMV = 0;
	static u16 correction = 0x200;
	static u16 dstFV = 0;

	//static u16 count = 0;

	//if (!ctm.Check(US2CCLK(50))) return;

	switch (i)
	{
		case 0:

			if (tm.Check(10))
			{
				wbuf[0] = 2;
				wbuf[1] = 0;
				wbuf[2] = 0;

				dsc.adr = 0x48;
				dsc.wdata = wbuf;
				dsc.wlen = 3;
				dsc.rdata = 0;
				dsc.rlen = 0;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				I2C_AddRequest(&dsc);

				i++;
			};

			break;

		case 1:

			if (dsc.ready)
			{
				wbuf[0] = 3;	
				wbuf[1] = 1;	
				wbuf[2] = 0;	

				dsc.adr = 0x48;
				dsc.wdata = wbuf;
				dsc.wlen = 3;
				dsc.rdata = 0;
				dsc.rlen = 0;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				I2C_AddRequest(&dsc);

				i++;
			};

			break;

		case 2:

			if (dsc.ready)
			{
				wbuf[0] = 4;	
				wbuf[1] = 1;	
				wbuf[2] = 1;	

				dsc.adr = 0x48;
				dsc.wdata = wbuf;
				dsc.wlen = 3;
				dsc.rdata = 0;
				dsc.rlen = 0;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				I2C_AddRequest(&dsc);

				i++;
			};

			break;

		case 3:

			if (dsc.ready)
			{
				wbuf[0] = 5;	
				wbuf[1] = 0;	
				wbuf[2] = 0;	

				dsc.adr = 0x48;
				dsc.wdata = wbuf;
				dsc.wlen = 3;
				dsc.rdata = 0;
				dsc.rlen = 0;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				I2C_AddRequest(&dsc);

				i++;
			};

			break;

		case 4:

			if (dsc.ready)
			{
				//curFireVoltage = GetCurFireVoltage();

				u16 t = reqFireVoltage;

				if (t > curFireVoltage)
				{
					if (correction < 0x3FF)
					{
						correction += 1;
					};
				}
				else if (t < curFireVoltage)
				{
					if (correction > 0)
					{
						correction -= 1;
					};
				};

				if (reqFireVoltage > dstFV)
				{
					dstFV += 2;
				}
				else if (reqFireVoltage < dstFV)
				{
					dstFV = reqFireVoltage;
				};

				//dstFV += (i16)reqFireVoltage - (dstFV+16)/32;

				t = dstFV;//(dstFV+16)/32;

				u32 k = (0x1E00 + correction) >> 3;

				t = (k*t+128) >> 10;

				if (t > 955) t = 955;

				t = ~(((u32)t * (65535*16384/955)) / 16384); 

				//if (DacHvInverted()) t = ~t;

				wbuf[0] = 8;	
				wbuf[1] = t>>8;
				wbuf[2] = t;

				dsc.adr = 0x48;
				dsc.wdata = wbuf;
				dsc.wlen = 3;
				dsc.rdata = rbuf;
				dsc.rlen = 0;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				I2C_AddRequest(&dsc);

				//count++;

				//if (count >= 100)
				//{
				//	count = 0;

				//	dCurFV = (i16)curFireVoltage - (i16)prevCurFV;
				//	dDstFV = (i16)dstFV - (i16)prevDstFV;

				//	prevDstFV = dstFV;
				//	prevCurFV = curFireVoltage;
				//};

				i++;
			};

			break;

		case 5:

			if (dsc.ready)
			{
				i = 0;
			};

			break;


	};
}
*/
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitMainVars()
{
	mv.numDevice				= 11111;
	mv.genFreq					= 10;	
	mv.winCount					= 64;	
	mv.winTime					= 32;	
	mv.bLevel					= 430;		
	mv.mLevel					= 430;		
	mv.disableFireNoVibration	= 0;		
	mv.levelNoVibration			= 100;		
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void LoadVars()
{
	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_CYAN "Load Vars ... ");

	static DSCI2C dsc;
	static u16 romAdr = 0;
	
	byte buf[sizeof(mv)*2+16];

	bool c2 = false;

	bool loadVarsOk = false;
	
	framErrorMask = 0;

	romAdr = ReverseWord(FRAM_I2C_MAINVARS_ADR);

	dsc.wdata = &romAdr;
	dsc.wlen = sizeof(romAdr);
	dsc.wdata2 = 0;
	dsc.wlen2 = 0;
	dsc.rdata = buf;
	dsc.rlen = sizeof(buf);
	dsc.adr = 0x50;

	if (I2C_AddRequest(&dsc))
	{
		while (!dsc.ready) { I2C_Update(); };
	};

	if (!dsc.ack) framErrorMask |= (1<<FRAM_NOACKR);

	PointerCRC p(buf);

	u32 err = 0, corrErr = 0, parErr = 0;

	for (byte i = 0; i < 2; i++)
	{
		Nand_ECC_Corr(p.b, sizeof(mv)+2, 256, p.b+sizeof(mv)+2, &err, &corrErr, &parErr);

		p.CRC.w = 0xFFFF;
		p.ReadArrayB(&mv, sizeof(mv));
		p.ReadW();
		p.b += 3;

		if (p.CRC.w == 0) { c2 = true; break; };
	};

	if (c2)			framErrorMask |= 1<<FRAM_LOAD_OK;
	if (!c2)		framErrorMask |= 1<<FRAM_ERROR_LOADVARS;
	if (err)		framErrorMask |= 1<<FRAM_ERROR_ECC;
	if (corrErr)	framErrorMask |= 1<<FRAM_CORR_ECC;
	if (parErr)		framErrorMask |= 1<<FRAM_ERROR_PARECC;

	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_WHITE "FRAM I2C - "); SEGGER_RTT_WriteString(0, (c2) ? (RTT_CTRL_TEXT_BRIGHT_GREEN "OK\n") : (RTT_CTRL_TEXT_BRIGHT_RED "ERROR\n"));

	loadVarsOk = c2;

	if (!loadVarsOk)
	{
		InitMainVars();

		svCount = 2;
	};

	numDevValid = true;

	SetGenFreq(							mv.genFreq	);
	SetWindowCount(						mv.winCount	);
	SetWindowTime(						mv.winTime	);
	AD5312_Set(AD5312_CHANNEL_LEVEL_B,	mv.bLevel	);
	AD5312_Set(AD5312_CHANNEL_LEVEL_M,	mv.mLevel	);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SaveVars()
{
	static DSCI2C dsc;
	static u16 romAdr = 0;
	static byte buf[sizeof(mv) * 2 + 16];

	static byte i = 0;
	static TM32 tm;

	PointerCRC p(buf);

	switch (i)
	{
		case 0:

			if (svCount > 0)
			{
				svCount--;
				i++;
			};

			break;

		case 1:

			mv.timeStamp = GetMilliseconds();

			for (byte j = 0; j < 2; j++)
			{
				byte* start = p.b;
				p.CRC.w = 0xFFFF;
				p.WriteArrayB(&mv, sizeof(mv));
				p.WriteW(p.CRC.w);
				
				Nand_ECC_Calc(start, p.b-start, 256, p.b); p.b += 3; 
			};

			romAdr = ReverseWord(FRAM_I2C_MAINVARS_ADR);

			dsc.wdata = &romAdr;
			dsc.wlen = sizeof(romAdr);
			dsc.wdata2 = buf;
			dsc.wlen2 = p.b-buf;
			dsc.rdata = 0;
			dsc.rlen = 0;
			dsc.adr = 0x50;

			tm.Reset();

			I2C_AddRequest(&dsc);

			i++;

			break;

		case 2:

			if (dsc.ready || tm.Check(100))
			{
				if (!dsc.ack) framErrorMask |= (1<<FRAM_NOACKW);

				i = 0;
			};

			break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void TestFRAM()
{
	static DSCI2C dscfram;
	static u16 romAdr = 0;

	static byte state = 0;
	static TM32 tm;

	static MainVars vv;

	static byte buf[sizeof(vv)*2+16];

	static byte count = 0;

	static u32 timeStamp = 0;

	bool loadVarsOk = false;

	switch(state)
	{
		case 0:

			if (tm.Check(100))
			{
				memset(buf, 0, sizeof(buf));

				u32 tt = timeStamp = GetMilliseconds();

				vv.timeStamp = tt;

				vv.numDevice	= tt += 101;
				vv.genFreq		= tt += 101;	
				vv.winCount		= tt += 101;	
				vv.winTime		= tt += 101;	
				vv.bLevel		= tt += 101;		
				vv.mLevel		= tt += 101;		

				PointerCRC p(buf);

				for (byte j = 0; j < 2; j++)
				{
					byte* start = p.b;
					p.CRC.w = 0xFFFF;
					p.WriteArrayB(&vv, sizeof(vv));
					p.WriteW(p.CRC.w);

					Nand_ECC_Calc(start, p.b-start, 256, p.b); p.b += 3; 
				};

				romAdr = ReverseWord(0x1000);

				dscfram.wdata = &romAdr;
				dscfram.wlen = sizeof(romAdr);
				dscfram.wdata2 = buf;
				dscfram.wlen2 = p.b-buf;
				dscfram.rdata = 0;
				dscfram.rlen = 0;
				dscfram.adr = 0x50;

				tm.Reset();

				I2C_AddRequest(&dscfram);

				state++;
			};

			break;

		case 1:

			if (dscfram.ready)
			{
				if (!dscfram.ack) framErrorMask |= 1<<FRAM_NOACKW;

				tm.Reset();

				state++;
			};

			break;

		case 2:

			if (tm.Check(10))
			{
				romAdr = ReverseWord(0x1000);

				dscfram.wdata = &romAdr;
				dscfram.wlen = sizeof(romAdr);
				dscfram.wdata2 = 0;
				dscfram.wlen2 = 0;
				dscfram.rdata = buf;
				dscfram.rlen = sizeof(buf);
				dscfram.adr = 0x50;

				memset(buf, 0, sizeof(buf));
				memset(&vv, 0, sizeof(vv));

				if (I2C_AddRequest(&dscfram))
				{
					state++;
				};
			};

			break;

		case 3:

			if (dscfram.ready)
			{
				if (!dscfram.ack || dscfram.readedLen != dscfram.rlen) framErrorMask |= (1<<FRAM_NOACKR);

				PointerCRC p(buf);

				u32 err = 0, corrErr = 0, parErr = 0;

				bool c2 = false;

				for (byte i = 0; i < 2; i++)
				{
					Nand_ECC_Corr(p.b, sizeof(vv)+2, 256, p.b+sizeof(vv)+2, &err, &corrErr, &parErr);

					p.CRC.w = 0xFFFF;
					p.ReadArrayB(&vv, sizeof(vv));
					p.ReadW();
					p.b += 3;

					if (p.CRC.w == 0) { c2 = true; break; };
				};

				if (vv.timeStamp != timeStamp) c2 = false;

				if (c2)			framErrorMask |= 1<<FRAM_LOAD_OK;//, count += 1, framErrorMask = (framErrorMask & 0xFF)|(count<<8);
				if (!c2)		framErrorMask |= 1<<FRAM_ERROR_LOADVARS;
				if (err)		framErrorMask |= 1<<FRAM_ERROR_ECC;
				if (corrErr)	framErrorMask |= 1<<FRAM_CORR_ECC;
				if (parErr)		framErrorMask |= 1<<FRAM_ERROR_PARECC;

				state = 0;
			};
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateWindow()
{
	static byte i = 0;

	static WINDSC *d = 0;

	if (d == 0)
	{
		d = GetReadyWinDsc();

		if (d == 0) return;
	};

	u16 bsum = d->b_data[0];
	u16 msum = d->m_data[0];

	b_ts[0] += bsum;
	m_ts[0] += msum;

	u16 t; u32 s;

	for (u32 n = 1; n < d->winCount; n++)
	{
		t = d->b_data[n]; b_ts[n] += (u16)(t - bsum); bsum = t;
		t = d->m_data[n]; m_ts[n] += (u16)(t - msum); msum = t;
	};

	FreeWinDsc(d); d = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void UpdateParams()
//{
//	static byte i = 0;
//
//	#define CALL(p) case (__LINE__-S): p; break;
//
//	enum C { S = (__LINE__+3) };
//	switch(i++)
//	{
//		CALL( UpdateTemp()				);
//		CALL( SaveVars();				);
//		CALL( UpdateHV();				);
//	};
//
//	i = (i > (__LINE__-S-3)) ? 0 : i;
//
//	#undef CALL
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void UpdateMisc()
//{
//	static byte i = 0;
//
//	#define CALL(p) case (__LINE__-S): p; break;
//
//	enum C { S = (__LINE__+3) };
//	switch(i++)
//	{
//		CALL( UpdateCom(); 		);
//		CALL( UpdateHardware();	);
//		CALL( UpdateParams();	);
//	};
//
//	i = (i > (__LINE__-S-3)) ? 0 : i;
//
//	#undef CALL
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static TaskList taskList;

static void InitTaskList()
{
	static Task tsk[] =
	{
		Task(UpdateTemp,		US2CTM(100)		),
		Task(SaveVars,			US2CTM(100)		),
		Task(UpdateHardware,	US2CTM(1)		),
		Task(UpdateMan,			US2CTM(10)		),
//		Task(TestFRAM,			US2CTM(100)		),
		Task(UpdateAccel,		US2CTM(10)		),
		Task(UpdateWindow,		US2CTM(1000)	)
	};

	for (u16 i = 0; i < ArraySize(tsk); i++) taskList.Add(tsk+i);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main()
{
	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_WHITE "main() start ...\n");

	TM32 tm;

	//__breakpoint(0);

	InitHardware();

	LoadVars();

	InitTaskList();

	comdsp.Connect(ComPort::ASYNC, 2000000, 0, 2);

	spiadxl.Connect(ADXL_BAUDRATE);

	u32 fc = 0;
	u16 n = 0;

	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_WHITE "Main Loop start ...\n");

	while (1)
	{
		Pin_MainLoop_Set();

		taskList.Update(); //UpdateMisc();

		Pin_MainLoop_Clr();

		fc++;

		if (tm.Check(1000))
		{
			fps = fc; fc = 0; 

			//PrepareFire((n++)&3, 3000, 1000, 2, 5000); CM4::NVIC->STIR = EIC_0_IRQ+PWM_EXTINT;
			//PrepareFire((n++)&3, 3000, 1000, 2, 5000); HW::EVSYS->SWEVT = 1;
		};
	}; // while (1)
}
