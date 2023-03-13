/*
 * Copyright (c) 2015-2022, Texas Instruments Incorporated
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

#include <string.h>
#include <stdio.h>

#include <xdc/runtime/System.h>
#include <ti/drivers/UART2.h>
#include <ti/sysbios/knl/Clock.h>

#include "ti_drivers_config.h"

#include "at/platform/inc/AtTerm.h"
#include "at/AtProcess.h"

static UART2_Handle uartHandle;

int32_t AtTerm_init(void)
{
    UART2_Params uartParams;

    // Create a UART with data processing off
    UART2_Params_init(&uartParams);
    uartParams.readMode         = UART2_Mode_BLOCKING;
    uartParams.writeMode        = UART2_Mode_NONBLOCKING;
    uartParams.readReturnMode   = UART2_ReadReturnMode_FULL;
    uartParams.baudRate         = 115200;

    uartHandle = UART2_open(CONFIG_UART_0, &uartParams);

    if (uartHandle == NULL)
    {
        System_abort("Error opening the UART");
    }
    return 0;
}

int32_t AtTerm_getChar(char* ch)
{
    char c = '\r';
    uint32_t bytes = 0;

    (void)UART2_read(uartHandle, &c, 1, (size_t *)&bytes);

    *ch = c;

    return bytes;
}

void AtTerm_putChar(char ch)
{
    UART2_write(uartHandle, &ch, 1, NULL);
}

void AtTerm_sendStringUi8Value(char *string, uint8_t value, uint8_t format)
{
    char strVal[128] = {0};

    if (format == 10)
    {
        sprintf(strVal, "%s%3d", (char*) string, value);
    }
    else
    {
        sprintf(strVal, "%s%2x", (char*) string, value);
    }

    UART2_write(uartHandle, strVal, strlen(strVal), NULL);
}

void AtTerm_sendStringI8Value(char *string, int8_t value, uint8_t format)
{
    char strVal[128] = {0};

    if (format == 10)
    {
        sprintf(strVal, "%s%3d", (char*) string, value);
    }
    else
    {
        sprintf(strVal, "%s%2x", (char*) string, value);
    }

    UART2_write(uartHandle, strVal, strlen(strVal), NULL);
}

void AtTerm_sendStringUi16Value(char *string, uint16_t value, uint8_t format)
{
    char strVal[128] = {0};

    if (format == 10)
    {
        sprintf(strVal, "%s%5d", (char*) string, value);
    }
    else
    {
        sprintf(strVal, "%s%4x", (char*) string, value);
    }

    UART2_write(uartHandle, strVal, strlen(strVal), NULL);
}

void AtTerm_sendStringI16Value(char *string, int16_t value, uint8_t format)
{
    char strVal[128] = {0};

    if (format == 10)
    {
        sprintf(strVal, "%s%5d", (char*) string, value);
    }
    else
    {
        sprintf(strVal, "%s%4x", (char*) string, value);
    }

    UART2_write(uartHandle, strVal, strlen(strVal), NULL);
}

void AtTerm_sendStringUi32Value(char *string, uint32_t value, uint8_t format)
{
    char strVal[128] = {0};

    if (format == 10)
    {
        sprintf(strVal, "%s%10u", (char*) string, (unsigned int)value);
    }
    else
    {
        sprintf(strVal, "%s%8x", (char*) string, (unsigned int)value);
    }

    UART2_write(uartHandle, strVal, strlen(strVal), NULL);
}

void AtTerm_sendStringI32Value(char *string, int32_t value, uint8_t format)
{
    char strVal[128] = {0};

    if (format == 10)
    {
        sprintf(strVal, "%s%10d", (char*) string, (int)value);
    }
    else
    {
        sprintf(strVal, "%s%8x", (char*) string, (unsigned int)value);
    }

    UART2_write(uartHandle, strVal, strlen(strVal), NULL);
}

void AtTerm_sendString(char *string)
{
    uint32_t len = strlen(string);
    UART2_write(uartHandle, string, len, NULL);
}

void AtTerm_clearTerm(void)
{
    //char clear[] = {0x0C};
    char c = '\f';
    UART2_write(uartHandle, &c, 1, NULL);
}

void AtTerm_getIdAndParam(char *paramStr, uint8_t *radioId, uintptr_t fxnParam, size_t fxnParamLen)
{
    uint32_t _fxnParam;
    uint8_t _radioId = 1;

    char *token;
    char delimiter[] = " ";
    token = strtok(paramStr, delimiter);

    if (NULL != token)
    {
        // a single numeric argument? save it to fxnParam, assume the default radio.
        _fxnParam = atoi(token);

        token = strtok(NULL, delimiter);
        if (NULL != token)
        {
            // 2 numeric arguments separated by a space? the first token
            // must have been the radioId and the next, fxnParam.
            _radioId = _fxnParam;
            _fxnParam = atoi(token);
        }

        if(NULL != (void *)fxnParam){
            switch(fxnParamLen){
            case (sizeof(uint8_t)): *(uint8_t *)fxnParam = (uint8_t)_fxnParam;
            break;
            case (sizeof(uint16_t)): *(uint16_t *)fxnParam = (uint16_t)_fxnParam;
            break;
            default: *(uint32_t *)fxnParam = _fxnParam;
            break;
            }
        }

    }

    if(NULL != radioId)
        *radioId = _radioId;
}
