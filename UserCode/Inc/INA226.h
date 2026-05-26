#ifndef INA226_H
#define INA226_H


#include "main.h"
#include "i2c.h"
#include "stdint.h"


#define INA226_I2C hi2c1
#define INA226_ADDR 0x80

#define MAX_CURRRENT 12.0f /* 6A */
#define R_SHUNT 0.005f /* 0.012 ohm */
#define CURRENT_CALIB_B_GENERAL (uint16_t)(0.00512f / (MAX_CURRRENT / 32768.0f) / R_SHUNT)

#define INA226_VBUS_LSB 0.00125f /* 0.00125V/bit */
#define INA226_VSHUNT_LSB 0.0000025f
#define INA226_CURRENT_LSB (MAX_CURRRENT / 32768.0f)
#define INA226_POWER_LSB (INA226_CURRENT_LSB * 25.0f)

#define INA226_SOL 1
#if INA226_SOL 
   #define INA226_OC 1.0f /* 5A */
#endif

#define INA226_SUL 0
#if INA226_SUL
   #define INA226_UC 1.0f /* 1A */
#endif

#define INA226_BOL 0
#if INA226_BOL 
   #define INA226_OV 5.0f /* 5V */
#endif

#define INA226_BUL 0
#if INA226_BUL
   #define INA226_UV 5.0f /* 5V */
#endif

#define INA226_POL 0
#if INA226_POL 
   #define INA226_OP 1.0f /* 1W */
#endif


typedef enum 
{
	INA226_ADDR_CONFIG,
	INA226_ADDR_SHUNT_VOLT,
	INA226_ADDR_BUS_VOLT,
	INA226_ADDR_POWER,
	INA226_ADDR_CURRENT,
	INA226_ADDR_CALIB,
	INA226_ADDR_MASK,
	INA226_ADDR_ALERT,
	INA226_ADDR_MAN_ID = 0xFE,
	INA226_ADDR_DIE_ID = 0xFF,
}INA226_AddrType;

typedef enum 
{
	INA226_AVERAGES_1,
   INA226_AVERAGES_4,
   INA226_AVERAGES_16,
   INA226_AVERAGES_64,
   INA226_AVERAGES_128,
   INA226_AVERAGES_256,
   INA226_AVERAGES_512,
   INA226_AVERAGES_1024,
}INA226_AVGType;

typedef enum 
{
	INA226_BUS_CT_140U,
	INA226_BUS_CT_204U,
	INA226_BUS_CT_332U,
	INA226_BUS_CT_588U,
	INA226_BUS_CT_1M1,
	INA226_BUS_CT_2M116,
	INA226_BUS_CT_4M156,
	INA226_BUS_CT_8M244,
}INA226_VBUSCType;

typedef enum 
{
   INA226_SHUNT_CT_140U,
   INA226_SHUNT_CT_204U,
   INA226_SHUNT_CT_332U,
   INA226_SHUNT_CT_588U,
   INA226_SHUNT_CT_1M1,
   INA226_SHUNT_CT_2M116,
   INA226_SHUNT_CT_4M156,
   INA226_SHUNT_CT_8M244,
}INA226_VSHCType;

typedef enum 
{
   INA226_MODE_SHUTDOWN,
   INA226_MODE_SHUNT_TG,
   INA226_MODE_VBUS_TG,
   INA226_MODE_SHUNTBUS_TG,
   INA226_MODE_SHUTDOWN1,
   INA226_MODE_SHUNT_CONTINUE,
   INA226_MODE_VBUS_CONTINUE,
   INA226_MODE_SHUNTBUS_CONTINUE,
}INA226_MODEType;

/*Setting this bit high configures the Alert pin to be asserted if the XXXXXX measurement following a conversion
   exceeds the value programmed in the Alert Limit Register.*/
typedef enum 
{
   INA226_MASK_SOL,    /*Shunt Voltage Over-Voltage*/
   INA226_MASK_SUL,    /*Shunt Voltage Under-Voltage*/
   INA226_MASK_BOL,    /*Bus Voltage Over-Voltage*/
   INA226_MASK_BUL,    /*Bus Voltage Under-Voltage*/
   INA226_MASK_POL,    /*Power Over-Limit*/
   INA226_MASK_CNVR,   /*Conversion Ready*/
   INA226_MASK_AFF,    /*Alert Function Flag*/
   INA226_MASK_CVRF,   /*Conversion Ready Flag*/
   INA226_MASK_OVF,    /*Math Overflow Flag*/
   INA226_MASK_APOL,   /*Alert Polarity bit; sets the Alert pin polarity.1 = Inverted (active-high open collector) 0 = Normal (active-low open collector) (default)*/
   INA226_MASK_LEN,    /*Alert Latch Enable; configures the latching feature of the Alert pin and Alert Flag bits.*/
}INA226_MASKType;

typedef struct
{
   float vbus;
   float current;
   float shuntcurrent;
   float power;
}INA226_InfoType;

extern INA226_InfoType INA226_Info;

uint8_t INA226_Init();
void INA226_ReadID();
void INA226_ReadInfo();
#endif
