/******************************************************************************

 @file  bsp_spi.c

 @brief Common API for SPI access. Driverlib implementation.

 Group: WCS, BTS
 Target Device: cc13xx_cc26xx

 ******************************************************************************
 
 Copyright (c) 2014-2022, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/

#include <ti/devices/DeviceFamily.h>
#include <ti/devices/cc23x0/inc/hw_clkctl.h>
#include <ti/devices/cc23x0/inc/hw_pmctl.h>
#include <ti/devices/cc23x0/inc/hw_pmud.h>
#include DeviceFamily_constructPath(driverlib/gpio.h)

/* Temporarily commented out, while LPRFXXWARE-753 is resolved
#include DeviceFamily_constructPath(driverlib/spi.h)
*/

/* Remove following line once LPRFXXWARE-753 is resolved*/
#include "spi.h"

#include "bsp.h"
#include "bsp_spi.h"

/*
 * Note that since this module is an experimental implementation,
 * board specific settings are directly hard coded here.
 */
#define BLS_SPI_BASE      SPI0_BASE
#define BLS_CPU_FREQ      48000000ul

#define IOC_ADDR(index) (IOC_BASE + IOC_O_IOC0 + (sizeof(uint32_t) * index))
#define GPIOCC23XX_CFG_PIN_IS_INPUT_INTERNAL    0x2
#define GPIO_CFG_OUTPUT_DEFAULT_HIGH_INTERNAL   0x1
#define GPIO_CFG_OUT_HIGH                       GPIO_CFG_OUTPUT_DEFAULT_HIGH_INTERNAL
#define GPIO_PIN_TO_MASK(pin)                   (1 << (pin))
#define GPIO_MUX_GPIO_INTERNAL                  IOC_IOC3_PORTCFG_BASE
#define GPIO_MUX_GPIO                           GPIO_MUX_GPIO_INTERNAL
#define GPIO_MUX_PORTCFG_PFUNC_1                0x00000001
#define GPIO_MUX_PORTCFG_PFUNC_2                0x00000002
#define GPIO_MUX_PORTCFG_PFUNC_4                0x00000004
#define GPIO_CFG_INPUT_INTERNAL_NO_PULL         0x20040002

/* See bsp_spi.h file for description */
int bspSpiWrite(const uint8_t *buf, size_t len)
{
  while (len > 0)
  {
    uint32_t ul;

    SPIDataPut(BLS_SPI_BASE, *buf);
    SPIDataGet(BLS_SPI_BASE, &ul);
    len--;
    buf++;
  }

  return (0);
}

/* See bsp_spi.h file for description */
int bspSpiRead(uint8_t *buf, size_t len)
{
  while (len > 0)
  {
    uint32_t ul;

    if (!SPIDataPutNonBlocking(BLS_SPI_BASE, 0))
    {
      /* Error */
      return (-1);
    }

    SPIDataGet(BLS_SPI_BASE, &ul);
    *buf = (uint8_t) ul;
    len--;
    buf++;
  }

  return (0);
}


/* See bsp_spi.h file for description */
void bspSpiFlush(void)
{
  uint32_t ul;

  while (SPIDataGetNonBlocking(BLS_SPI_BASE, &ul));
}

/* See bsp_spi.h file for description */
void bspGpioWrite(uint_least8_t index, unsigned int value)
{
    HWREGB(GPIO_BASE + GPIO_O_DOUT3_0 + index) = (value & 0x1);
}

static void bspSetPinMaskNonatomic(uint32_t index, uint32_t registerBaseAddress)
{
    uint32_t mask                          = GPIO_PIN_TO_MASK(index);
    HWREG(GPIO_BASE + registerBaseAddress) = mask;
}

static void bspGpioSetMux(uint_least8_t index, uint32_t mux)
{
    uint32_t iocfgRegAddr   = IOC_ADDR(index);
    uint32_t previousConfig = HWREG(iocfgRegAddr);

    if ((previousConfig & 0x07) != mux)
    {
        HWREGB(iocfgRegAddr) = (uint8_t)(mux);
    }
}

/* See bsp_spi.h file for description */
void bspGpioSetConfig(uint32_t index, uint32_t pinConfig)
{
    uint32_t iocfgRegAddr = IOC_ADDR(index);

    /* The pin will be an output after configuring */
    if (!(pinConfig & GPIOCC23XX_CFG_PIN_IS_INPUT_INTERNAL))
    {
        // Set the new default value and enable output
        bspGpioWrite(index, pinConfig & GPIO_CFG_OUT_HIGH ? 1 : 0);
        bspSetPinMaskNonatomic(index, GPIO_O_DOESET31_0);
    }
    else
    {
        bspSetPinMaskNonatomic(index, GPIO_O_DOECLR31_0);
    }

    /* Mask off the mux bits containing non-IOC configuration values and apply */
    uint32_t tmpConfig = pinConfig & 0xFFFFFFF8;
    HWREG(iocfgRegAddr)      = tmpConfig | GPIO_MUX_GPIO;
}

/* See bsp_spi.h file for description */
void bspSpiOpen(uint32_t bitRate, uint32_t clkPin)
{
  HWREG(PMUD_BASE + PMUD_O_PREG0) |= PMUD_PREG0_UDIGLDO_EN_EN;
  HWREG(PMCTL_BASE + PMCTL_O_VDDRCTL) |= PMCTL_VDDRCTL_SELECT_DCDC;

  // Enable GPIO clock
  HWREG(CLKCTL_BASE + CLKCTL_O_CLKENSET0) = CLKCTL_CLKENSET0_GPIO;

  // Enable SPI clock
  HWREG(CLKCTL_BASE + CLKCTL_O_CLKENSET0) = CLKCTL_CLKENSET0_SPI0;

  /* SPI configuration */
  SPIIntDisable(BLS_SPI_BASE, SPI_DMA_DONE_TX | SPI_DMA_DONE_RX | SPI_IDLE |
                              SPI_TXEMPTY | SPI_TX | SPI_RX | SPI_RTOUT |
                              SPI_PER | SPI_RXFIFO_OVF);
  SPIIntClear(BLS_SPI_BASE, SPI_RTOUT | SPI_RXFIFO_OVF);

  SPIConfigSetExpClk(BLS_SPI_BASE,
                     BLS_CPU_FREQ, /* CPU rate */
                     SPI_FRF_MOTO_MODE_0, /* frame format */
                     SPI_MODE_CONTROLLER, /* mode */
                     bitRate, /* bit rate */
                     8); /* data size */

  GPIO_setOutputEnableDio(BSP_SPI_MOSI, GPIO_OUTPUT_ENABLE);
  bspGpioSetMux(BSP_SPI_MOSI, GPIO_MUX_PORTCFG_PFUNC_2);

  bspGpioSetConfig(BSP_SPI_MISO, GPIO_CFG_INPUT_INTERNAL_NO_PULL);
  bspGpioSetMux(BSP_SPI_MISO, GPIO_MUX_PORTCFG_PFUNC_1);

  GPIO_setOutputEnableDio(BSP_SPI_CLK_FLASH, GPIO_OUTPUT_ENABLE);
  bspGpioSetMux(BSP_SPI_CLK_FLASH, GPIO_MUX_PORTCFG_PFUNC_4);

  SPIEnable(BLS_SPI_BASE);

  {
    /* Get read of residual data from SSI port */
    uint32_t buf;

    while (SPIDataGetNonBlocking(BLS_SPI_BASE, &buf));
  }
}

/* See bsp_spi.h file for description */
void bspSpiClose(void)
{

}
