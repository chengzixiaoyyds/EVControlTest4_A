#include "INA226.h"

INA226_InfoType INA226_Info;

static int16_t INA226_RD_Reg(INA226_AddrType addr)
{
   uint8_t temp[2];
   HAL_I2C_Mem_Read(&INA226_I2C, INA226_ADDR, addr, 1, temp, 2, 0xFFFF);
   return temp[1] + temp[0] * 256;
}

static void INA226_WR_Reg(INA226_AddrType addr, uint16_t value)
{
   uint8_t temp[2];
   temp[0] = value >> 8;
   temp[1] = value & 0xFF;
   HAL_I2C_Mem_Write(&INA226_I2C, INA226_ADDR, addr, 1,  temp, 2, 0x100);
}

uint8_t INA226_Init()
{
   uint8_t ret = 0;

   if (INA226_RD_Reg(INA226_ADDR_DIE_ID) != 0x2260)
   {
      ret = 1;
   }
   else
   {

   }

   uint16_t config = INA226_MODE_SHUNTBUS_CONTINUE | (INA226_SHUNT_CT_2M116 << 3) \
   | (INA226_BUS_CT_204U << 6) | (INA226_AVERAGES_64 << 9);

   INA226_WR_Reg(INA226_ADDR_CONFIG, config);
   INA226_WR_Reg(INA226_ADDR_CALIB, CURRENT_CALIB_B_GENERAL);

#if INA226_SOL
   /* 1A OC */
   INA226_WR_Reg(INA226_ADDR_MASK, 0x8000);
   INA226_WR_Reg(INA226_ADDR_ALERT, INA226_OC * R_SHUNT / INA226_VSHUNT_LSB);
#elif INA226_SUL
   /* 1A UC */
   INA226_WR_Reg(INA226_ADDR_MASK, 0x4000);
   INA226_WR_Reg(INA226_ADDR_ALERT, INA226_UC * R_SHUNT / INA226_VSHUNT_LSB);
#elif INA226_BOL
   /* 1A OC */
   INA226_WR_Reg(INA226_ADDR_MASK, 0x2000);
   INA226_WR_Reg(INA226_ADDR_ALERT, INA226_OV / INA226_VBUS_LSB);
#elif INA226_BUL
   /* 1A UC */
   INA226_WR_Reg(INA226_ADDR_MASK, 0x1000);
   INA226_WR_Reg(INA226_ADDR_ALERT, INA226_UV / INA226_VBUS_LSB);
#elif INA226_POL
   /* 1A UC */
   INA226_WR_Reg(INA226_ADDR_MASK, 0x0800);
   INA226_WR_Reg(INA226_ADDR_ALERT, INA226_OP / INA226_POWER_LSB);
#endif
   return ret;
}

void INA226_ReadInfo()
{
   INA226_Info.shuntcurrent = (float)(INA226_RD_Reg(INA226_ADDR_SHUNT_VOLT) * INA226_VSHUNT_LSB) / R_SHUNT;
   INA226_Info.vbus = INA226_RD_Reg(INA226_ADDR_BUS_VOLT) * INA226_VBUS_LSB;
   INA226_Info.current = INA226_RD_Reg(INA226_ADDR_CURRENT) * INA226_CURRENT_LSB;
   INA226_Info.power = INA226_RD_Reg(INA226_ADDR_POWER) * INA226_POWER_LSB;
}