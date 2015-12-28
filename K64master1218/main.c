/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * [File Name]     main.c
 * [Platform]      FRDM-K64F
 * [Project]       myProject123
 * [Version]       1.00
 * [Author]        B55335
 * [Date]          10/19/2015
 * [Language]      'C'
 * [History]       1.00 - Original Release
 *
 */

//-----------------------------------------------------------------------
// Standard C/C++ Includes
//-----------------------------------------------------------------------

#include <stdio.h>


//-----------------------------------------------------------------------
// KSDK Includes
//-----------------------------------------------------------------------
#include "main.h"
#include "fsl_mpu_hal.h"
#include "fsl_os_abstraction.h"
//-----------------------------------------------------------------------
// Application Includes
//-----------------------------------------------------------------------

#include "diskio.h"
#include "ff.h"
//-----------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------
bool sdhc_detect(void);
uint8_t control_frame(uint8_t buffer[],uint8_t command);
uint8_t erase_frame(uint8_t buffer[],uint32_t address);
uint8_t write_frame(uint8_t buffer[],uint32_t address,uint32_t num);
uint8_t read_frame(uint8_t buffer[],uint32_t address,uint32_t num);
void ParameterDeinit(void);

extern void UART_DRV_IRQHandler(uint32_t instance);
//-----------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------

//-----------------------------------------------------------------------
// Typedefs
//-----------------------------------------------------------------------

//-----------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------
uint8_t send_buffer[160];
uint8_t rev_buffer[160];
uint8_t buff[128];
uint8_t sendflag;
uint32_t wrblk, erblk, flashsize, relocated_vectors;
uint8_t frame_length = 0;
//-----------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------
#define DATA_Transfer_Instance	                1
#define DATA_Transfer_BaudRate                  9600
#define CHECK_OK     			        0
#define CHECK_FAIL   			        1
#define Station_Number                          0x01
//-----------------------------------------------------------------------
// Main Function
//-----------------------------------------------------------------------
static uart_state_t uartstate;
int main(void)
{
    FRESULT fr;		                        /* FatFs return code */
    DRESULT ds;		                        /* Disk functions return code */
    FATFS FatFs;	                        /* FatFs system object */
    FIL fil;
    uint32_t OFFSET = 0;
    uint32_t State = 3;	                        /* use for State change*/
    uint32_t size = 0;
    uint32_t sizetmp = 0;

    uint32_t data_count = 0;
    uint32_t dest_address = 0;
    uint32_t last_frameLENTH = 0;
    
    uint32_t temp = 0;
    uint8_t u,N,temp1;
    uint32_t i,j;
    /* Initialize Operating System Abstraction layer */
    OSA_Init();
    // Configure board specific pin muxing
    hardware_init();
    // Initialize UART terminal
    dbg_uart_init();
    
    send_buffer[0] = '$';
    send_buffer[1] = Station_Number;
/*--------------------------UART1 init------------------------------*/
    
    //Initialize UART1 for data transfer
    //use PTC3 PTC4 as UART1's pinout
    PORT_HAL_SetMuxMode(PORTC,3u,kPortMuxAlt3);
    PORT_HAL_SetMuxMode(PORTC,4u,kPortMuxAlt3);

    uart_user_config_t uartconfig;
    uartconfig.baudRate = DATA_Transfer_BaudRate;
    uartconfig.bitCountPerChar = kUart8BitsPerChar;
    uartconfig.parityMode = kUartParityDisabled;
    uartconfig.stopBitCount = kUartOneStopBit;
    u = UART_DRV_Init(DATA_Transfer_Instance, &uartstate, &uartconfig);
    if(u == 0x0u)
    {
    	printf("UART has been initialized successfully!\r\n");
    }
    else
    {
    	printf("Failed to initialize UART!\r\n");
    	printf("Error num is %d\r\n",u);
    	for(;;){}
    }
/*--------------------------GPIO init-------------------------------*/


    PORT_HAL_SetDigitalFilterClock(PORTA,kPortLPOClock);	//when use digitalfilter, it should be configured
    PORT_HAL_SetDigitalFilterWidth(PORTA,0x1u);
    PORT_HAL_SetDigitalFilterClock(PORTC,kPortLPOClock);
    PORT_HAL_SetDigitalFilterWidth(PORTC,0x1u);
    /* Initialize SD Card detect input pin */
    GPIO_DRV_InputPinInit(&sdhcCdPin[0]);
    /* Initialize program pin (SW3) */
    GPIO_DRV_InputPinInit(&switchPins[1]);
    /* Initialize program and verify pin (SW2) */
    GPIO_DRV_InputPinInit(&switchPins[0]);
    
    GPIO_DRV_OutputPinInit(&ledPins[0]);
    GPIO_DRV_OutputPinInit(&ledPins[1]);
    GPIO_DRV_OutputPinInit(&ledPins[2]);

    GPIO_DRV_ClearPinOutput(Blue);
    GPIO_DRV_SetPinOutput(Red);
    GPIO_DRV_SetPinOutput(Green);


/*----------------------PIT0 init-------------------------------*/

    pit_user_config_t pit_user_config;
    pit_user_config.isInterruptEnabled = true;
    pit_user_config.periodUs = 500000;
    PIT_DRV_Init(0, false);
    PIT_DRV_InitChannel(0,0,&pit_user_config);
    //PIT_DRV_StartTimer(0,0);
/*--------------------------fatfs init-------------------------------*/
    if(!sdhc_detect())
    {
        printf("Please insert SD Card\r\n");
        /* Wait for SD Card insertion */
        while (!sdhc_detect());
    }
    printf("SD Card inserted\r\n");
    printf("Initializing SD Card...\r\n");

    MPU_HAL_Disable(MPU);
    /* Initialize SDHC driver and SD Card */
    ds = disk_initialize(SD);
    if(ds)
    {
            printf("Failed to initialize SD disk\r\n");
            for(;;){}
    }
    /* Select current logical device driver (0 = USB, 1 = SD) */
    fr = f_chdrive(SD);

    printf("Mounting file system to SD Card volume...\r\n");
    /* Mount file system to the SDCARD volume */
    fr = f_mount(SD, &FatFs);
    if(fr)
    {
            printf("Error mounting file system\r\n");
            for(;;){}
    }

    printf("Press SW3 to start program.\r\n");
    printf("Press SW2 to start verify.\r\n");
    
    
    
    for (;;)                                         // Forever loop
    {

    	switch(State){
              case 3:
                
                  State = (GPIO_DRV_ReadPinInput(kGpioSW2))*2;
                  State += GPIO_DRV_ReadPinInput(kGpioSW3);
                  PIT_DRV_StopTimer(0,0);
                  if(N != 0)
                  {
                      GPIO_DRV_ClearPinOutput(Red);
                      GPIO_DRV_SetPinOutput(Blue);
                      GPIO_DRV_SetPinOutput(Green);                      
                  }
                  break;
                  
              case 2:
                
                  printf("SW3 has been pressed.\r\n");
                  printf("Now begin to program.\r\n");
                  PIT_DRV_StartTimer(0,0);
                  
                  GPIO_DRV_SetPinOutput(Red);
                  GPIO_DRV_SetPinOutput(Blue);
                  GPIO_DRV_SetPinOutput(Green); 
                  
                  //send 'B' command to reboot target
                  frame_length = control_frame(send_buffer,'B');
                  UART_DRV_SendDataBlocking(DATA_Transfer_Instance, send_buffer, frame_length, 1000);

                  if(UART_DRV_ReceiveDataBlocking(DATA_Transfer_Instance, rev_buffer, 6, 1000) == 0)
                  {
                      printf("Enter bootmode successful!!\r\n");
                  }
                  //send 'I' command to get target information
                  frame_length = control_frame(send_buffer,'I');
                  UART_DRV_SendDataBlocking(DATA_Transfer_Instance, send_buffer, frame_length, 1000);
                  
                  //get target MCU info the MCU name should not contain 'U' , may case error.
                  do
                  {
                      //UART_DRV_ReceiveDataBlocking(DATA_Transfer_Instance, &temp1, 1,1000);
                      if(UART_DRV_ReceiveDataBlocking(DATA_Transfer_Instance, &temp1, 1,1000) != 0)
                      {
                          N++;
                      }
                      else
                      {
                          rev_buffer[i++] = temp1;
                      }
                      if(N >= 5)
                      {
                          break;
                      }
                  }
                  while(rev_buffer[i-1] != 0x55);                 //use do{} while() because the length of flame is not fixed.  
                  if( N != 0)
                  {
                      N = 0;
                      State = 3;
                      printf("Receive data timeout.\r\n");
                      PIT_DRV_StopTimer(0,0);
                      GPIO_DRV_ClearPinOutput(Red);
                      GPIO_DRV_SetPinOutput(Blue);
                      break;
                  }                
                  //get MCU info
                  i = 0;
                  while(rev_buffer[i] != '#')
                  {
                      i++;
                  }
                  i++;
                  //get bootloader version
                  while(rev_buffer[i] != '#')
                  {
                      i++;
                  }
                  i++;
                  //get write block size
                  temp = (uint32_t)rev_buffer[i++] << 8;
                  temp += (uint32_t)rev_buffer[i++];
                  wrblk = temp;                                
                  //get erase block size
                  temp = (uint32_t)rev_buffer[i++] << 8;
                  temp += (uint32_t)rev_buffer[i++];
                  erblk = temp;                                 
                  //get flash size
                  temp = (uint32_t)rev_buffer[i++] << 16;
                  temp += (uint32_t)rev_buffer[i++] << 8;
                  temp += (uint32_t)rev_buffer[i++];
                  flashsize = temp;                              
                  //get dontcare_addr start
                  //get dontcare_addr end  
                  //These 6 bytes are not  used now
                  i += 6;
                  //get relocated_vectors
                  temp = (uint32_t)rev_buffer[i++] << 16;
                  temp += (uint32_t)rev_buffer[i++] << 8;
                  temp += (uint32_t)rev_buffer[i++];
                  relocated_vectors = temp;
                  if(relocated_vectors <= 0x400)
                  {
                      printf("Relocated vector address should be great than 0x400.\r\n");
                      State = 3;
                      break;                                    
                  }
                  printf("relocated_vectors = 0x%x\r\n",relocated_vectors);
                  printf("wrblk = 0x%x\r\n",wrblk);
                  printf("erblk = 0x%x\r\n",erblk);
                  
                  printf("start to erase flash...\r\n");
                  // send 'E' command to erase flash
                  dest_address = relocated_vectors;
                  N = 0;
                  while(dest_address < flashsize)
                  {
                      frame_length = erase_frame(send_buffer,dest_address);
                      UART_DRV_SendDataBlocking(DATA_Transfer_Instance, send_buffer, frame_length, 1000);
                      
                      if(UART_DRV_ReceiveDataBlocking(DATA_Transfer_Instance, rev_buffer, 6, 1000) != 0)
                      {
                          N++;
                      }                    
                      dest_address += erblk;
                  }
                  if(N != 0)
                  {
                      printf("Erase MCU failed!! N = %d\r\n", N);
                      State = 3;
                      ParameterDeinit();
                      break;
                  }
                  else
                  {
                      printf("Erase MCU successful!!\r\n");
                  }
                  printf("start to download flash...\r\n");
                  //open .bin file on SD card, start to send data to target
                  fr = f_open(&fil,"123.bin",FA_OPEN_EXISTING|FA_WRITE|FA_READ);
                  if(fr)
                  {
                      printf("Open file error!\r\n");
                      printf("fr = %d\r\n", fr);
                      State = 3;
                      break;
                  }

                  size = f_size(&fil);                          //read file size
                  printf("bin size is %lu\r\n",size);
                  
                  data_count = size / wrblk;                    //Calculate the count of flame
                  last_frameLENTH = size % wrblk;               //The last flame may not be the integer multiple of wrblk
                  dest_address = relocated_vectors;
                  f_lseek(&fil,0);                              //Point the pointer to the starting address of the file
                  
                  //Read wrblk bytes and send to target each time
                  N = 0;
                  for(i = 1; i <= data_count; i++)
                  {
                      f_read(&fil,buff,wrblk,&sizetmp);
                      for(j=0; j<wrblk; j++)
                      {
                          send_buffer[9+j] = buff[j];
                      }
                      frame_length = write_frame(send_buffer,dest_address,wrblk);
                      
                      UART_DRV_SendDataBlocking(DATA_Transfer_Instance, send_buffer, frame_length, 1000);

                      if(UART_DRV_ReceiveDataBlocking(DATA_Transfer_Instance, rev_buffer, 6, 1000) != 0)
                      {
                          N++;
                      }
                      f_lseek(&fil,(i*wrblk));          // point to next wrblk
                      dest_address += wrblk;
                  }
                  //read the last wrblk and send to target
                  f_read(&fil,buff,last_frameLENTH,&sizetmp);
                  for(j=0; j<last_frameLENTH; j++)
                  {
                      send_buffer[9+j] = buff[j];
                  }
                  frame_length = write_frame(send_buffer,dest_address,last_frameLENTH);
                  UART_DRV_SendDataBlocking(DATA_Transfer_Instance, send_buffer, frame_length, 1000);
                  if(UART_DRV_ReceiveDataBlocking(DATA_Transfer_Instance, rev_buffer, 6, 1000) != 0)
                  {
                      N++;
                  }
                  f_close(&fil);
                  
                  if(N != 0)
                  {
                      printf("Write flash failed!! N = %d\r\n", N);
                      State = 3;
                      ParameterDeinit();
                      break;
                  }
                  else
                  {
                      printf("Write flash successful!!\r\n");
                  }
                  //close .bin file and send 'G' command at last
                  frame_length = control_frame(send_buffer,'G');
                  UART_DRV_SendDataBlocking(DATA_Transfer_Instance, send_buffer, frame_length, 1000);
                  
                  // before exit this state, we should deinit the parameter
                  State = 3;
                  ParameterDeinit();
                  PIT_DRV_StopTimer(0,0);
                  GPIO_DRV_ClearPinOutput(Green);
                  GPIO_DRV_SetPinOutput(Blue);
                  printf("Press SW3 to start program.\r\n");
                  printf("Press SW2 to start verify.\r\n");
                  break;
              case 1:
                
                  printf("SW2 has been pressed.\r\n");
                  printf("Now begin to verify.\r\n");
                  PIT_DRV_StartTimer(0,0);
                  GPIO_DRV_SetPinOutput(Red);
                  GPIO_DRV_SetPinOutput(Blue);
                  GPIO_DRV_SetPinOutput(Green); 
                  //send 'B' command to reboot target
                  frame_length = control_frame(send_buffer,'B');
                  UART_DRV_SendDataBlocking(DATA_Transfer_Instance, send_buffer, frame_length, 1000);

                  if(UART_DRV_ReceiveDataBlocking(DATA_Transfer_Instance, rev_buffer, 6, 1000) == 0)
                  {
                      printf("Enter bootmode successful!!\r\n");
                  }
                  //send 'I' command to get target information
                  frame_length = control_frame(send_buffer,'I');
                  UART_DRV_SendDataBlocking(DATA_Transfer_Instance, send_buffer, frame_length, 1000);
                  
                  //get target MCU info the MCU name should not contain 'U' , may case error.
                  do
                  {
                      //UART_DRV_ReceiveDataBlocking(DATA_Transfer_Instance, &temp1, 1,1000);
                      if(UART_DRV_ReceiveDataBlocking(DATA_Transfer_Instance, &temp1, 1,1000) != 0)
                      {
                          N++;
                      }
                      else
                      {
                          rev_buffer[i++] = temp1;
                      }
                      if(N >= 5)
                      {
                          break;
                      }
                  }
                  while(rev_buffer[i-1] != 0x55);                 //use do{} while() because the length of flame is not fixed.  
                  if( N != 0)
                  {
                      N = 0;
                      State = 3;
                      printf("Receive data timeout.\r\n");
                      PIT_DRV_StopTimer(0,0);
                      GPIO_DRV_ClearPinOutput(Red);
                      GPIO_DRV_SetPinOutput(Blue);
                      break;
                  }
                  //get MCU info
                  i = 0;
                  while(rev_buffer[i] != '#')
                  {
                      i++;
                  }
                  i++;
                  //get bootloader version
                  while(rev_buffer[i] != '#')
                  {
                      i++;
                  }
                  i++;
                  //get write block size
                  temp = (uint32_t)rev_buffer[i++] << 8;
                  temp += (uint32_t)rev_buffer[i++];
                  wrblk = temp;                                
                  //get erase block size
                  temp = (uint32_t)rev_buffer[i++] << 8;
                  temp += (uint32_t)rev_buffer[i++];
                  erblk = temp;                                 
                  //get flash size
                  temp = (uint32_t)rev_buffer[i++] << 16;
                  temp += (uint32_t)rev_buffer[i++] << 8;
                  temp += (uint32_t)rev_buffer[i++];
                  flashsize = temp;                              
                  //get dontcare_addr start
                  //get dontcare_addr end  
                  //These 6 bytes are not  used now
                  i += 6;
                  //get relocated_vectors
                  temp = (uint32_t)rev_buffer[i++] << 16;
                  temp += (uint32_t)rev_buffer[i++] << 8;
                  temp += (uint32_t)rev_buffer[i++];
                  relocated_vectors = temp;                  
                  if(relocated_vectors <= 0x400)
                  {
                      printf("Relocated vector address should be great than 0x400.\r\n");
                      State = 3;
                      break;                                    
                  }
                  printf("relocated_vectors = 0x%x\r\n",relocated_vectors);
                  printf("wrblk = 0x%x\r\n",wrblk);
                  printf("erblk = 0x%x\r\n",erblk); 
                  
                  printf("Start to verify...\r\n");                 
                  //open file  begin to verify
                  fr = f_open(&fil,"123.bin",FA_OPEN_EXISTING|FA_WRITE|FA_READ);
                  if(fr)
                  {
                      printf("Open file error!\r\n");
                      printf("fr = %d\r\n", fr);
                      State = 3;
                      break;
                  }

                  size = f_size(&fil);                          //read file size
                  printf("bin size is %lu\r\n",size);
                  
                  data_count = size / wrblk;                    //Calculate the count of flame
                  last_frameLENTH = size % wrblk;               //The last flame may not be the integer multiple of wrblk
                  dest_address = relocated_vectors;
                  f_lseek(&fil,0);                              //Point the pointer to the starting address of the file
                  
                  N = 0;                  
                  for(i = 1; i <= data_count; i++)
                  {                                          
                      frame_length = read_frame(send_buffer,dest_address,wrblk);
                      UART_DRV_SendDataBlocking(DATA_Transfer_Instance, send_buffer, frame_length, 1000);

                      if(UART_DRV_ReceiveDataBlocking(DATA_Transfer_Instance, rev_buffer, (wrblk+6), 1000) != 0)
                      {
                          N++;
                          break;
                      }
                      
                      f_read(&fil,buff,wrblk,&sizetmp);
                      for(j=0; j<wrblk; j++)
                      {
                          if(rev_buffer[j+4] != buff[j])
                          {
                              N++;
                              break;
                          }
                      }                      
                      f_lseek(&fil,(i*wrblk));          // point to next wrblk
                      dest_address += wrblk;
                      if(N != 0)
                      {
                          break;
                      }
                  }
                  
                  // check N whether mismatching exist. 
                  if(N != 0)
                  {
                      printf("Verify flash failed!!\r\n");
                      State = 3;
                      ParameterDeinit();
                      //frame_length = control_frame(send_buffer,'G');
                      //UART_DRV_SendDataBlocking(DATA_Transfer_Instance, send_buffer, frame_length, 1000);
                      break;                  
                  }
                  
                  // Verify last bytes that not integral multiple of wrblk
                  frame_length = read_frame(send_buffer,dest_address,last_frameLENTH);
                  UART_DRV_SendDataBlocking(DATA_Transfer_Instance, send_buffer, frame_length, 1000);
                  
                  if(UART_DRV_ReceiveDataBlocking(DATA_Transfer_Instance, rev_buffer, (last_frameLENTH+6), 1000) != 0)
                  {
                      printf("Verify flash failed!!\r\n");
                      State = 3;
                      ParameterDeinit();
                      //frame_length = control_frame(send_buffer,'G');
                      //UART_DRV_SendDataBlocking(DATA_Transfer_Instance, send_buffer, frame_length, 1000);                      
                      break;    
                  }
                  f_read(&fil,buff,last_frameLENTH,&sizetmp);
                  for(j=0; j<last_frameLENTH; j++)
                  {
                      if(rev_buffer[j+4] != buff[j])
                      {
                          N++;
                          break;                          
                      }
                  }
                  // check N whether mismatching exist.  
                  if(N != 0)
                  {
                      printf("Verify flash failed!!\r\n");
                      State = 3;
                      //frame_length = control_frame(send_buffer,'G');
                      //UART_DRV_SendDataBlocking(DATA_Transfer_Instance, send_buffer, frame_length, 1000);
                      ParameterDeinit();
                      break;                  
                  }
                  printf("Verify flash successful!!\r\n");
                  
                  //send 'G' command to reboot target and run APP
                  frame_length = control_frame(send_buffer,'G');
                  UART_DRV_SendDataBlocking(DATA_Transfer_Instance, send_buffer, frame_length, 1000);
                  ParameterDeinit();
                  PIT_DRV_StopTimer(0,0);
                  GPIO_DRV_ClearPinOutput(Green);
                  GPIO_DRV_SetPinOutput(Blue);
                  printf("Press SW3 to start program.\r\n");
                  printf("Press SW2 to start verify.\r\n");                  
                  State = 3;
                  break;
              default:
                  printf("State is invalid.\r\n");
                  State = 3;
                  break;
    	}
    }
}

/*FUNCTION*----------------------------------------------------------------
* Function Name : sdhc_detect
* Comments : Detect if the SD Card is present or not with a GPIO pin
* Returns: [0] - Card not available
* [1] - Card is present
*END*--------------------------------------------------------------------*/
bool sdhc_detect(void)
{
	uint32_t value = 0;
	if (sdhcCdPin[0].config.pullSelect == kPortPullUp) /* pull up */
	{
            value = GPIO_DRV_ReadPinInput(sdhcCdPin[0].pinName);
            return (!value);
	}
	else /* pull down */
	{
            return (GPIO_DRV_ReadPinInput(sdhcCdPin[0].pinName));
	}
}

/*FUNCTION*----------------------------------------------------------------
* Function Name : control_frame
* Comments : Generate 'B' 'I' 'G' instruction
* Returns: length of the frame
*END*--------------------------------------------------------------------*/
uint8_t control_frame(uint8_t buffer[],uint8_t command)
{
    uint8_t length;
    
    buffer[3] = 1;
    buffer[4] = command;
    buffer[buffer[3]+4] = 0xAA;
    buffer[buffer[3]+5] = 0x55;
    length = 4 + buffer[3] + 2;
    return length;
}

/*FUNCTION*----------------------------------------------------------------
* Function Name : erase_frame
* Comments : Generate 'E' instruction
* Returns: length of the frame
*END*--------------------------------------------------------------------*/
uint8_t erase_frame(uint8_t buffer[],uint32_t address)
{
    uint8_t length;
    uint32_t add;
    add = address;
    
    if(add >= 0x1000000)
    {
        add = 0xFFFFFF;
    }
    buffer[3] = 4;
    buffer[4] = 'E';
    buffer[5] = (uint8_t)(add/65536);
    add = add%65536;
    buffer[6] = (uint8_t)(add/256);
    add = add%256;
    buffer[7] = (uint8_t)add;
    buffer[buffer[3]+4] = 0xAA;
    buffer[buffer[3]+5] = 0x55;
    length = 4 + buffer[3] + 2;
    return length;
}

/*FUNCTION*----------------------------------------------------------------
* Function Name : write_frame
* Comments : Generate 'W' instruction
* Returns: length of the frame
*END*--------------------------------------------------------------------*/
uint8_t write_frame(uint8_t buffer[],uint32_t address,uint32_t num)
{
    uint8_t length;
    uint32_t add,num1;
    add = address;
    num1 = num;
    
    if(add >= 0x1000000)
    {
        add = 0xFFFFFF;
    }
    buffer[3] = num1+5;	//Specific Data length
    buffer[4] = 'W';	//write command
    buffer[5] = (uint8_t)(add/65536);	// calculate address
    add = add%65536;
    buffer[6] = (uint8_t)(add/256);
    add = add%256;
    buffer[7] = (uint8_t)add;
    buffer[8] = num1;	//data length
    buffer[buffer[3]+4] = 0xAA;	//frame end
    buffer[buffer[3]+5] = 0x55;
    length = 4 + buffer[3] + 2;
    return length;
}
/*FUNCTION*----------------------------------------------------------------
* Function Name : read_frame
* Comments : Generate 'R' instruction
* Returns: length of the frame
*END*--------------------------------------------------------------------*/

uint8_t read_frame(uint8_t buffer[],uint32_t address,uint32_t num)
{
    uint8_t length;
    uint32_t add,num1;
    add = address;
    num1 = num;

    if(add >= 0x1000000)
    {
        add = 0xFFFFFF;
    }    
    
    buffer[3] = 5;	//Specific Data length
    buffer[4] = 'R';
    buffer[5] = (uint8_t)(add/65536);	// calculate address
    add = add%65536;
    buffer[6] = (uint8_t)(add/256);
    add = add%256;
    buffer[7] = (uint8_t)add;   
    buffer[8] = num1;	//data length
    buffer[buffer[3]+4] = 0xAA;	//frame end
    buffer[buffer[3]+5] = 0x55;
    length = 4 + buffer[3] + 2;
    return length;
}

void ParameterDeinit(void)
{
    wrblk = 0;
    erblk = 0;
    flashsize = 0;
    relocated_vectors = 0;
    memset(buff,0,128);
    memset(send_buffer,0,160);
    send_buffer[0] = '$';
    send_buffer[1] = Station_Number;
    memset(rev_buffer,0,32);
}


//interrupt handler

void SDHC_IRQHandler(void)
{
    SDHC_DRV_DoIrq(0);
}

void UART1_RX_TX_IRQHandler(void)
{
    UART_DRV_IRQHandler(1);
}

void PIT0_IRQHandler(void)
{
	sendflag = 1;

        GPIO_DRV_TogglePinOutput(Blue);
        
	PIT_DRV_ClearIntFlag(0,0);
}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////



