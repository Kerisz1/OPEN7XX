#include "GT911.h"
#include "CT_I2C.h"
//#include "stm32746g_LCD.h"

GT911_Dev Dev_Now,Dev_Backup;

static void GT911_Reset_Sequence()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GT911_RST_CLK();
	GT911_INT_CLK();
	
  GPIO_InitStruct.Pin = GT911_RST_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GT911_RST_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GT911_INT_PIN;
  HAL_GPIO_Init(GT911_INT_PORT, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(GT911_RST_PORT,GT911_RST_PIN,GPIO_PIN_RESET);    
	HAL_Delay(100);
	HAL_GPIO_WritePin(GT911_INT_PORT,GT911_INT_PIN,GPIO_PIN_RESET);    
	HAL_Delay(100);
	HAL_GPIO_WritePin(GT911_RST_PORT,GT911_RST_PIN,GPIO_PIN_SET);    
	HAL_Delay(200);
	
	GPIO_InitStruct.Pin = GT911_INT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GT911_INT_PORT, &GPIO_InitStruct);
	
	HAL_Delay(100);
}

static uint8_t GT911_WR_Reg(uint16_t reg,uint8_t *buf,uint8_t len)
{
	uint8_t i;
	uint8_t ret=0;
	CT_I2C_Start();	
 	CT_I2C_Send_Byte(CT_CMD_WR);   
	CT_I2C_Wait_Ack();
	CT_I2C_Send_Byte(reg>>8);   	
	CT_I2C_Wait_Ack(); 	 										  		   
	CT_I2C_Send_Byte(reg&0XFF);   	
	CT_I2C_Wait_Ack();  
	for(i=0;i<len;i++)
	{	   
    CT_I2C_Send_Byte(buf[i]);  
		ret=CT_I2C_Wait_Ack();
		if(ret)break;  
	}
  CT_I2C_Stop();					
	return ret; 
}

static void GT911_RD_Reg(uint16_t reg,uint8_t *buf,uint8_t len)
{
	uint8_t i;
 	CT_I2C_Start();	
 	CT_I2C_Send_Byte(CT_CMD_WR);  
	CT_I2C_Wait_Ack();
 	CT_I2C_Send_Byte(reg>>8);   
	CT_I2C_Wait_Ack(); 	 										  		   
 	CT_I2C_Send_Byte(reg&0XFF);   	
	CT_I2C_Wait_Ack();  
 	CT_I2C_Stop();  
	
 	CT_I2C_Start();  	 	   
	CT_I2C_Send_Byte(CT_CMD_RD);     
	CT_I2C_Wait_Ack();	   
	for(i=0;i<len;i++)
	{	   
		buf[i]=CT_I2C_Read_Byte(i==(len-1)?0:1); 
	} 
  CT_I2C_Stop();   
}

static uint8_t GT911_ReadStatue(void)
{
	uint8_t buf[4];
	GT911_RD_Reg(GT911_PRODUCT_ID_REG, (uint8_t *)&buf[0], 3);
	GT911_RD_Reg(GT911_CONFIG_REG, (uint8_t *)&buf[3], 1);
//	printf("TouchPad_ID:%c,%c,%c\r\nTouchPad_Config_Version:%2x\r\n",buf[0],buf[1],buf[2],buf[3]);
	return buf[3];
}

static uint16_t GT911_ReadFirmwareVersion(void)
{
	uint8_t buf[2];

	GT911_RD_Reg(GT911_FIRMWARE_VERSION_REG, buf, 2);

//	printf("FirmwareVersion:%2x\r\n",(((uint16_t)buf[1] << 8) + buf[0]));
	return ((uint16_t)buf[1] << 8) + buf[0];
}

void GT911_Scan(GT911_Dev *tp)
{
	uint8_t buf[41];
  uint8_t Clearbuf = 0;
	uint8_t i;
//	uint32_t PointColor[]={LCD_COLOR_BLUE,LCD_COLOR_GREEN,LCD_COLOR_RED,LCD_COLOR_MAGENTA,LCD_COLOR_YELLOW};
	

		GT911_RD_Reg(GT911_READ_XY_REG, buf, 1);		

		if ((buf[0]&0x80) == 0x00)
		{
			GT911_WR_Reg(GT911_READ_XY_REG, (uint8_t *)&Clearbuf, 1);
			//printf("%x\r\n",buf[0]);
			HAL_Delay(10);
		}
		else
		{
			//printf("bufstat:%x\r\n",buf[0]);
			tp->TouchpointFlag = buf[0];
			tp->TouchCount = buf[0]&0x0f;
			if (tp->TouchCount > 5)
			{
				GT911_WR_Reg(GT911_READ_XY_REG, (uint8_t *)&Clearbuf, 1);
				return ;
			}		
			GT911_RD_Reg(GT911_READ_XY_REG+1, &buf[1], tp->TouchCount*8);
			GT911_WR_Reg(GT911_READ_XY_REG, (uint8_t *)&Clearbuf, 1);
			
			tp->Touchkeytrackid[0] = buf[1];
			tp->X[0] = ((uint16_t)buf[3] << 8) + buf[2];
			tp->Y[0] = ((uint16_t)buf[5] << 8) + buf[4];
			tp->S[0] = ((uint16_t)buf[7] << 8) + buf[6];

			tp->Touchkeytrackid[1] = buf[9];
			tp->X[1] = ((uint16_t)buf[11] << 8) + buf[10];
			tp->Y[1] = ((uint16_t)buf[13] << 8) + buf[12];
			tp->S[1] = ((uint16_t)buf[15] << 8) + buf[14];

			tp->Touchkeytrackid[2] = buf[17];
			tp->X[2] = ((uint16_t)buf[19] << 8) + buf[18];
			tp->Y[2] = ((uint16_t)buf[21] << 8) + buf[20];
			tp->S[2] = ((uint16_t)buf[23] << 8) + buf[22];

			tp->Touchkeytrackid[3] = buf[25];
			tp->X[3] = ((uint16_t)buf[27] << 8) + buf[26];
			tp->Y[3] = ((uint16_t)buf[29] << 8) + buf[28];
			tp->S[3] = ((uint16_t)buf[31] << 8) + buf[30];

			tp->Touchkeytrackid[4] = buf[33];
			tp->X[4] = ((uint16_t)buf[35] << 8) + buf[34];
			tp->Y[4] = ((uint16_t)buf[37] << 8) + buf[36];
			tp->S[4] = ((uint16_t)buf[39] << 8) + buf[38];


		}	
	return ;
}

//void GT911_TEST(void)
//{
//	printf("GT911_TEST\r\n");
//	GT911_Reset_Sequence();
//
//	CT_I2C_Init();
//
//	GT911_ReadStatue();
//
//	GT911_ReadFirmwareVersion();
//
//	while(1)
//	{
//		GT911_Scan();
//	}
//}
