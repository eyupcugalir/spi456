  /*
   * Copyright (c) 2016, Texas Instruments Incorporated
   * All rights reserved.
   *
   * Redistribution and use in source and binary forms, with or without
   * modification, are permitted provided that the following conditions
   * are met:
   *
   * *  Redistributions of source code must retain the above copyright
   *    notice, this list of conditions and the following disclaimer.
   *
   * *  Redistributions in binary form must reproduce the above copyright
   *    notice, this list of conditions and the following disclaimer in the
   *    documentation and/or other materials provided with the distribution.
   *
   * *  Neither the name of Texas Instruments Incorporated nor the names of
   *    its contributors may be used to endorse or promote products derived
   *    from this software without specific prior written permission.
   *
   * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
   * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
   * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
   * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
   * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
   * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
   * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
   * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
   */

  //
   //  ======== uartecho.c ========
    //

  // XDCtools Header files //
  #include <xdc/std.h>
  #include <xdc/runtime/System.h>
  #include <xdc/cfg/global.h>

  // BIOS Header files //
  #include <ti/sysbios/BIOS.h>
  #include <ti/sysbios/knl/Task.h>
  #include <ti/sysbios/knl/Clock.h>


  // TI-RTOS Header files //
  #include <ti/drivers/PIN.h>
  #include <ti/drivers/UART.h>
  #include <ti/drivers/I2C.h>
  #include <ti/drivers/SPI.h>

  // Example/Board Header files
  #include "Board.h"


  #include <stdlib.h>
  #include <stdbool.h>
  #include <stdint.h>

  #define TASKSTACKSIZE     768

  Task_Struct task0Struct;
  Char task0Stack[TASKSTACKSIZE];

  // Global memory storage for a PIN_Config table
  static PIN_State ledPinState;
  static SPI_Handle spiHandle;
  static SPI_Params spiParams;


  PIN_Config ledPinTable[] = {
      Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
      Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
      Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_GENERALPURPOSE1   | PIN_GPIO_OUTPUT_EN 	| PIN_GPIO_HIGH	 | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_CP2103UARTRESET   | PIN_GPIO_OUTPUT_EN	| PIN_GPIO_HIGH  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	Board_SPI_FLASH_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,




      PIN_TERMINATE
  };


  PIN_Handle ledPinHandle;



#define EFL_ADDR_RECOVERY   			  0x20000
#define EFL_SIZE_RECOVERY				  0x20000
#define EFL_PAGE_SIZE               	  0x1000
#define EFL_FLASH_SIZE                    0x80000
#define EXT_FLASH_PAGE_SIZE               4096

  int bspSpiWrite(const uint8_t *buf, size_t len)
  {

	  static uint8_t wrbuf[256];
	  static uint8_t rdbuf[256];
    SPI_Transaction masterTransaction;

    masterTransaction.count  = sizeof(wrbuf);
    masterTransaction.txBuf  = wrbuf;
    masterTransaction.arg    = NULL;
    masterTransaction.rxBuf  = rdbuf;

    return SPI_transfer(spiHandle, &masterTransaction) ? 0 : -1;
  }




  int bspSpiRead(uint8_t *buf, size_t len)
  {

	static uint8_t wrbuf[256];
	static uint8_t rdbuf[256];
    SPI_Transaction masterTransaction;

    masterTransaction.count  = sizeof(wrbuf);
    masterTransaction.txBuf  = wrbuf;
    masterTransaction.arg    = NULL;
    masterTransaction.rxBuf  = rdbuf;

    return SPI_transfer(spiHandle, &masterTransaction) ? 0 : -1;
  }




  void bspSpiOpen(void)
  {
    if (spiHandle == NULL)
    {
      /*  Configure SPI as master, 1 mHz bit rate*/
      SPI_Params_init(&spiParams);
      spiParams.bitRate = 1000000;
      spiParams.mode         = SPI_MASTER;
      spiParams.frameFormat  = SPI_POL1_PHA1;
      spiParams.transferMode = SPI_MODE_CALLBACK;

      /* Attempt to open SPI. */
      spiHandle = SPI_open(Board_SPI0, &spiParams);

      if (spiHandle == NULL)
      {
        //Task_exit();
	    	System_printf("spi is not open \n");
	  	  System_flush();

      }
      else
      {
	    	System_printf("spi is open \n");
	  	  System_flush();

	  	   if (spiHandle != NULL)
	  	    {
	  	      // Close the RTOS driver
	  	      SPI_close(spiHandle);

	  	     if( spiHandle == NULL){
	  	  	System_printf("spi is closed \n");
	  		  System_flush();
	  	     }
	  	    }
	  	    else
	  	    {
	  	    	System_printf("spi is not  closed \n");
	  	    	  System_flush();
	  	    }


      }
    }


  }




  void bspSpiClose(void)
  {

    if (spiHandle != NULL)
    {
      // Close the RTOS driver
      SPI_close(spiHandle);
      spiHandle = NULL;
  	System_printf("spi is closed \n");
	  System_flush();

    }
    else
    {
    	System_printf("spi is not  closed \n");
    	  System_flush();
    }

  }

//spi flush hariç bsp_spi.c dosyası alindi düzenlendi.


  //  Implementation for WinBond W25X20CL Flash



  // Instruction codes

  #define BLS_CODE_PROGRAM          0x02
  #define BLS_CODE_READ             0x03
  #define BLS_CODE_READ_STATUS      0x05
  #define BLS_CODE_WRITE_ENABLE     0x06
  #define BLS_CODE_SECTOR_ERASE     0x20
  #define BLS_CODE_MDID             0x90

  #define BLS_CODE_DP               0xB9
  #define BLS_CODE_RDP              0xAB


  #define BLS_CODE_ERASE_4K         0x20
  #define BLS_CODE_ERASE_32K        0x52
  #define BLS_CODE_ERASE_64K        0xD8
  #define BLS_CODE_ERASE_ALL        0xC7


  #define BLS_STATUS_SRWD_BM        0x80
  #define BLS_STATUS_BP_BM          0x0C
  #define BLS_STATUS_WEL_BM         0x02
  #define BLS_STATUS_WIP_BM         0x01

  #define BLS_STATUS_BIT_BUSY       0x01


  #define BLS_PROGRAM_PAGE_SIZE     256
  #define BLS_ERASE_SECTOR_SIZE     4096
  #define EXT_FLASH_MAN_ID          0xEF
  #define EXT_FLASH_DEV_ID          0x12



  static void extFlashSelect(void)
  {
  	PIN_setOutputValue(ledPinHandle,Board_SPI_FLASH_CS,Board_FLASH_CS_ON);
  }

  static void extFlashDeselect(void)
  {
	PIN_setOutputValue(ledPinHandle,Board_SPI_FLASH_CS,Board_FLASH_CS_OFF);
  }

  static bool extFlashPowerDown(void)
  {
    uint8_t cmd;
    bool success;

    cmd = BLS_CODE_DP;
    extFlashSelect();
    success = bspSpiWrite(&cmd,sizeof(cmd)) == 0;
    extFlashDeselect();

    return success;
  }


  static int extFlashWaitReady(void)
  {
    const uint8_t wbuf[1] = { BLS_CODE_READ_STATUS };
    int ret;


    {
      uint8_t buf;

      extFlashSelect();
      bspSpiWrite(wbuf, sizeof(wbuf));
      ret = bspSpiRead(&buf,sizeof(buf));
      extFlashDeselect();

      if (ret)
      {
        /* Error */
        return -2;
      }
      if (!(buf & BLS_STATUS_BIT_BUSY))
      {
	    	System_printf("not error \n");


      }
    }

    return 0;
  }


  static bool extFlashPowerStandby(void)
  {
    uint8_t cmd;
    bool success;

    cmd = BLS_CODE_RDP;
    extFlashSelect();
    success = bspSpiWrite(&cmd,sizeof(cmd)) == 0;
    extFlashDeselect();

    if (success)
    {
      success = extFlashWaitReady() == 0;
    }

    return success;
  }



  static bool extFlashVerifyPart(void)
  {
    const uint8_t wbuf[] = { BLS_CODE_MDID, 0xFF, 0xFF, 0x00 };
    uint8_t rbuf[2];

    extFlashSelect();

    int ret = bspSpiWrite(wbuf, sizeof(wbuf));
    if (ret)
    {
      extFlashDeselect();
      return false;
    }

    ret = bspSpiRead(rbuf, sizeof(rbuf));
    extFlashDeselect();

    if (ret || rbuf[0] != EXT_FLASH_MAN_ID || rbuf[1] != EXT_FLASH_DEV_ID)
    {
      return false;
    }
    return true;
  }




  static int extFlashWaitPowerDown(void)
  {
    uint8_t i;

    i = 0;
    while (i<10)
    {
      if (!extFlashVerifyPart())
      {
        return 0;
      }
      i++;
    }

    return -1;
  }


  static int extFlashWriteEnable(void)
  {
    const uint8_t wbuf[] = { BLS_CODE_WRITE_ENABLE };

    extFlashSelect();
    int ret = bspSpiWrite(wbuf,sizeof(wbuf));
    extFlashDeselect();

    if (ret)
    {
      return -3;
    }
    return 0;
  }



  bool extFlashOpen(void)
  {

    /* Make sure SPI is available */
    bspSpiOpen();

    /* Put the part is standby mode */
   // extFlashPowerStandby();

    return extFlashOpen();
  }


  void extFlashClose(void)
  {
      // Put the part in low power mode
     // extFlashPowerDown();
     // extFlashWaitPowerDown();

      /* Make sure SPI lines have a defined state */
      bspSpiClose();

  }


  bool extFlashRead(size_t offset, size_t length, uint8_t *buf)
  {
    uint8_t wbuf[4];

    /* Wait till previous erase/program operation completes */
    int ret ;

    /* SPI is driven with very low frequency (1MHz < 33MHz fR spec)
     * in this temporary implementation.
     * and hence it is not necessary to use fast read. */
    wbuf[0] = BLS_CODE_READ;
    wbuf[1] = (offset >> 16) & 0xff;
    wbuf[2] = (offset >> 8) & 0xff;
    wbuf[3] = offset & 0xff;

    extFlashSelect();



    ret = bspSpiRead(buf, length);

    extFlashDeselect();

    return ret ;
  }


  bool extFlashWrite(size_t offset, size_t length, const uint8_t *buf)
  {
    uint8_t wbuf[4];

    while (length > 0)
    {
      /* Wait till previous erase/program operation completes
      int ret = extFlashWaitReady();
      if (ret)
      {
        return false;
      }

      int ret;
      ret = extFlashWriteEnable();
      if (ret)
      {
        return false;
      }
 */
      size_t ilen; /* interim length per instruction */

      ilen = BLS_PROGRAM_PAGE_SIZE - (offset % BLS_PROGRAM_PAGE_SIZE);
      if (length < ilen)
      {
        ilen = length;
      }

      wbuf[0] = BLS_CODE_PROGRAM;
      wbuf[1] = (offset >> 16) & 0xff;
      wbuf[2] = (offset >> 8) & 0xff;
      wbuf[3] = offset & 0xff;

      offset += ilen;
      length -= ilen;

      /* Up to 100ns CS hold time (which is not clear
       * whether it's application only in between reads)
       * is not imposed here since above instructions
       * should be enough to delay
       * as much. */
      extFlashSelect();

      bspSpiWrite(wbuf, sizeof(wbuf));


     // bspSpiWrite(buf,ilen);

     // buf += ilen;
      extFlashDeselect();
    }

    return true;
  }


  bool extFlashErase(size_t offset, size_t length)
  {
    /* Note that Block erase might be more efficient when the floor map
     * is well planned for OTA but to simplify for the temporary implementation,
     * sector erase is used blindly. */
    uint8_t wbuf[4];
    size_t i, numsectors;

    wbuf[0] = BLS_CODE_SECTOR_ERASE;

    {
      size_t endoffset = offset + length - 1;
      offset = (offset / BLS_ERASE_SECTOR_SIZE) * BLS_ERASE_SECTOR_SIZE;
      numsectors = (endoffset - offset + BLS_ERASE_SECTOR_SIZE - 1) /
        BLS_ERASE_SECTOR_SIZE;
    }

    for (i = 0; i < numsectors; i++)
    {
      /* Wait till previous erase/program operation completes */
      int ret = extFlashWaitReady();
      if (ret)
      {
        return false;
      }

      ret = extFlashWriteEnable();
      if (ret)
      {
        return false;
      }

      wbuf[1] = (offset >> 16) & 0xff;
      wbuf[2] = (offset >> 8) & 0xff;
      wbuf[3] = offset & 0xff;

      extFlashSelect();

      if (bspSpiWrite(wbuf, sizeof(wbuf)))
      {
        /* failure */
        extFlashDeselect();
        return false;
      }
      extFlashDeselect();

      offset += BLS_ERASE_SECTOR_SIZE;
    }

    return true;
  }


  bool extFlashTest(void)
  {
    bool ret;

    ret = extFlashOpen();
    if (ret)
    {
      extFlashClose();
    }

    return ret;
  }




  Void taskFxn(UArg arg0, UArg arg1)
  {
    	PIN_Handle ledPinHandle;

    	ledPinHandle = PIN_open(&ledPinState, ledPinTable);

    	PIN_setOutputValue(ledPinHandle, Board_LED0, 1);

    	//cs pin low
    	PIN_setOutputValue(ledPinHandle,Board_SPI_FLASH_CS,Board_FLASH_CS_ON);
        if (spiHandle == NULL)
        {
          /*  Configure SPI as master, 1 mHz bit rate*/
          SPI_Params_init(&spiParams);
          spiParams.bitRate = 1000000;
          spiParams.mode         = SPI_MASTER;
          spiParams.frameFormat  = SPI_POL1_PHA1;
          spiParams.transferMode = SPI_MODE_CALLBACK;

          /* Attempt to open SPI. */
          spiHandle = SPI_open(Board_SPI0, &spiParams);

          if (spiHandle == NULL)
          {
            //Task_exit();
    	    	System_printf("spi is not open \n");
    	  	  System_flush();

          }
          else
          {
    	    	System_printf("spi is open \n");
    	  	  System_flush();

    	  	  //write enable
    	  	//extFlashWriteEnable();
    	  	const uint8_t wbuf[] = { BLS_CODE_WRITE_ENABLE };
    	  	extFlashWrite(EFL_ADDR_RECOVERY, sizeof(wbuf), wbuf);
    	  	 uint32_t address;
    	  //  for (address= 0; address<EFL_FLASH_SIZE; address+=EFL_ADDR_RECOVERY)
			//	 {
			//	   extFlashErase(address,EFL_ADDR_RECOVERY);
			//	 }
    	    static uint8_t buf[256];
    	    bool success;
    	    success = extFlashWrite(EFL_ADDR_RECOVERY, sizeof(buf), buf);
    	    if (success)
    	    {
    	    	extFlashRead(EFL_ADDR_RECOVERY, sizeof(buf), buf);
    	    	 System_printf("read buf =  %d  ",buf [25] );
    	    					  System_flush();

    	    }
    	    else
    	    {
    	    	 System_printf("yazamadin\n");
							  System_flush();

				// Close the RTOS driver
				  SPI_close(spiHandle);
				  System_printf("spiHandle =  %d  \n",spiHandle );
				  System_flush();
				  spiHandle = NULL;

				  if( spiHandle == NULL){
				System_printf("spi is closed \n");
				  System_flush();
				 }

				else
				{
					System_printf("spi is not  closed \n");
					  System_flush();
				}

    	    }


          }
        }


/*
    	    SPI_Params_init(&spiParams);
    	    spiParams.bitRate = 1000000;

    	    spiParams.mode         = SPI_MASTER;
    	    spiParams.transferMode = SPI_MODE_CALLBACK;


    	    spiHandle = SPI_open(Board_SPI0, &spiParams);

    	    if (spiHandle == NULL)
    	       {
    	    	System_printf("spi is not open \n");
    	    	  System_flush();
 	       }
    	    else{
    	    	System_printf("spi is open \n");
    	    	  System_flush();
*/
    	    	 // bool success;

    	    	   //success = extFlashOpen();

    	    	   //if (success)
    	    	 //  {

    	    	   //  uint32_t address;

    	    	     // Erase external flash
    	    	  //   for (address= 0; address<EFL_FLASH_SIZE; address+=EFL_PAGE_SIZE)
    	    	  //   {
    	    	   //    extFlashErase(address,EFL_PAGE_SIZE);
    	    	   //  }
/*
    	    	     // Install factory image
    	    	     for (address=0; address<EFL_SIZE_RECOVERY && success; address+=EFL_PAGE_SIZE)
    	    	     {
    	    	       success = extFlashErase(EFL_ADDR_RECOVERY+address, EFL_PAGE_SIZE);
    	    	       if (success)
    	    	       {
    	    	         size_t offset;
    	    	         static uint8_t buf[256]; // RAM storage needed due to SPI/DMA limitation

    	    	         for (offset=0; offset<EFL_PAGE_SIZE; offset+=sizeof(buf))
    	    	         {
    	    	           const uint8_t *pIntFlash;

    	    	           // Copy from internal to external flash
    	    	           pIntFlash = (const uint8_t*)address + offset;
    	    	           memcpy(buf,pIntFlash,sizeof(buf));
    	    	           success = extFlashWrite(EFL_ADDR_RECOVERY+address+offset, sizeof(buf), buf);

    	    	           // Verify first few bytes
    	    	           if (success)
    	    	           {
    	    	             extFlashRead(EFL_ADDR_RECOVERY+address+offset, sizeof(buf), buf);
    	    	             success = buf[2] == pIntFlash[2] && buf[3] == pIntFlash[3];
    	    	    	    	System_printf("read buffer %u \n",buf[2]);
    	    	    	    	  System_flush();

    	    	    	    	System_printf(" buffer %u \n",buf[3]);
    	    	    	    	  System_flush();

    	    	           }
    	    	         }
    	    	       }
    	    	     }
*/
    	    		  // bspSpiClose();
    	    	  // }




    }

   //  ======== main ========
   //
  int main(void)
  {
      PIN_Handle ledPinHandle;
      Task_Params taskParams;

      // Call board init functions
      Board_initGeneral();
      Board_initSPI();


      // Construct BIOS objects
      Task_Params_init(&taskParams);
      taskParams.stackSize = TASKSTACKSIZE;
      taskParams.stack = &task0Stack;
      Task_construct(&task0Struct, (Task_FuncPtr)taskFxn, &taskParams, NULL);


      // Open LED pins
      ledPinHandle = PIN_open(&ledPinState, ledPinTable);

      if(!ledPinHandle) {
    	  System_printf("Error initializing board LED pins\n");
    	  System_flush();
      }
      else{
    	  System_printf("Perfect! initializing board LED pins\n");
    	  System_flush();
      }

     // PIN_setOutputValue(ledPinHandle, Board_LED0, 1);
      PIN_setOutputValue(ledPinHandle, Board_LED1, 1);
     // PIN_setOutputValue(ledPinHandle, Board_LED2, 1);
      PIN_setOutputValue(ledPinHandle, Board_CP2103UARTRESET, 1);
     // PIN_setOutputValue(ledPinHandle, Board_GENERALPURPOSE1, 1);


      // Start BIOS
      BIOS_start();

      return (0);
  }
