#include "stmflash.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32H7开发板
//STM32内部FLASH读写 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2017/8/15
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

/*************************************************************************/
/*************************本代码仅仅实现了BANK1****************************/
/*************************************************************************/


//读取指定地址的字(32位数据) 
//faddr:读地址 
//返回值:对应数据.
u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(__IO uint32_t *)faddr; 
}

//获取某个地址所在的flash扇区,仅用于BANK1！！
//addr:flash地址
//返回值:0~11,即addr所在的扇区
uint16_t STMFLASH_GetFlashSector(u32 addr)
{
	if(addr<ADDR_FLASH_SECTOR_1_BANK1)return FLASH_SECTOR_0;
	else if(addr<ADDR_FLASH_SECTOR_2_BANK1)return FLASH_SECTOR_1;
	else if(addr<ADDR_FLASH_SECTOR_3_BANK1)return FLASH_SECTOR_2;
	else if(addr<ADDR_FLASH_SECTOR_4_BANK1)return FLASH_SECTOR_3;
	else if(addr<ADDR_FLASH_SECTOR_5_BANK1)return FLASH_SECTOR_4;
	else if(addr<ADDR_FLASH_SECTOR_6_BANK1)return FLASH_SECTOR_5;
	else if(addr<ADDR_FLASH_SECTOR_7_BANK1)return FLASH_SECTOR_6;
	return FLASH_SECTOR_7;	
}

//从指定地址开始写入指定长度的数据
//特别注意:因为STM32H7的扇区实在太大,没办法本地保存扇区数据,所以本函数
//         写地址如果非0XFF,那么会先擦除整个扇区且不保存扇区数据.所以
//         写非0XFF的地址,将导致整个扇区数据丢失.建议写之前确保扇区里
//         没有重要数据,最好是整个扇区先擦除了,然后慢慢往后写. 
//该函数对OTP区域也有效!可以用来写OTP区!
//OTP区域地址范围:0X1FF0F000~0X1FF0F41F
//WriteAddr:起始地址(此地址必须为4的倍数!!)
//pBuffer:数据指针
//NumToWrite:字(32位)数(就是要写入的32位数据的个数.) 
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
//    FLASH_EraseInitTypeDef FlashEraseInit;
    HAL_StatusTypeDef FlashStatus=HAL_OK;
    u32 SectorError=0;
	u32 addrx=0;
	u32 endaddr=0;	
    if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//非法地址
    
 	HAL_FLASH_Unlock();             //解锁	
	addrx=WriteAddr;				//写入的起始地址
	endaddr=WriteAddr+NumToWrite*4;	//写入的结束地址  控制写入循环的次数  实质上  NumToWrite*4 = NumToWrite/8 * 32--> 8的意思是 256/32 = 8
									//一次写入的是256bit  pBuffer一次应该移位256bit  --》 u32 时  32*8 = 256
    
//    if(addrx<0X1FF00000)
//    {
//        while(addrx<endaddr)		//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
//		{
//			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//有非0XFFFFFFFF的地方,要擦除这个扇区
//			{   
//				FlashEraseInit.Banks=FLASH_BANK_1;						//操作BANK1
//                FlashEraseInit.TypeErase=FLASH_TYPEERASE_SECTORS;       //擦除类型，扇区擦除 
//                FlashEraseInit.Sector=STMFLASH_GetFlashSector(addrx);   //要擦除的扇区
//                FlashEraseInit.NbSectors=1;                             //一次只擦除一个扇区
//                FlashEraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;      //电压范围，VCC=2.7~3.6V之间!!
//                if(HAL_FLASHEx_Erase(&FlashEraseInit,&SectorError)!=HAL_OK) 
//                {
//                    break;//发生错误了	
//                }
//                SCB_CleanInvalidateDCache();                            //清除无效的D-Cache
//			}else addrx+=4;
//            FLASH_WaitForLastOperation(FLASH_WAITETIME,FLASH_BANK_1);    //等待上次操作完成
//        }
//    }
    FlashStatus=FLASH_WaitForLastOperation(FLASH_WAITETIME,FLASH_BANK_1);       //等待上次操作完成
	if(FlashStatus==HAL_OK)
	{
		while(WriteAddr<endaddr)//写数据
		{
            if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,WriteAddr,(uint64_t)pBuffer)!=HAL_OK)//写入数据
			{ 
				break;	//写入异常
			}
			WriteAddr+=32;	//每个FLASH地址  存储 0xFF 8bit 数据  32*8=256
			pBuffer+=8;	// 8*32 = 256  数据类型为u32  即指针向后移8  移过256bit
		} 
	}
	HAL_FLASH_Lock();           //上锁
} 
//原子团队代码  问题：0.焼写程序选择部分擦除  程序在第二次在线仿真时  在FLASH有数据的情况下写入 会进入HardFault()
//		1.pBuffer+=2； 存在问题  指针应该移位多少  与pBuffer类型有关  (u32) *pBuffer时  +=8   (u64) *pBuffer时  +=4 ，+=2必然会导致数据错位
//		2.代码注释  求endaddr 与 WriteAddr+=32; 处  不容易看明白

//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToRead:字(32位)数
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//读取4个字节.
		ReadAddr+=4;//偏移4个字节.	
	}
}
