#include "TJC.h"
#include "Serial.h"
#include "Delay.h"
#include <stdio.h>

#define TJC_END  Serial_SendByte(0xFF); Serial_SendByte(0xFF); Serial_SendByte(0xFF)
#define TJC_TRANSFER_TIMEOUT_MS  100
#define TJC_TRANSFER_RETRIES       2

static uint8_t TJC_WaitTransferFlag(uint8_t wait_end)
{
    uint16_t timeout = TJC_TRANSFER_TIMEOUT_MS;

    while (timeout--) 
    {
        if (wait_end) 
        {
            if (Serial_TakeTJCEnd()) return 1;
        }
        else
        {
            if (Serial_TakeTJCReady()) return 1;
        }
        Delay_ms(1);
    }
    return 0;
}

void TJC_SendText(char *obj, char *content)
{
    Serial_Printf("%s.txt=\"%s\"", obj, content);
    TJC_END;
}

void TJC_SetNum(char *obj, int32_t val)
{
    Serial_Printf("%s.val=%d", obj, (int)val);
    TJC_END;
}

void TJC_SetPage(uint8_t page)
{
    Serial_Printf("page %d", page);
    TJC_END;
}

void TJC_ClearCurve(char *obj)
{
    Serial_Printf("cle %s.id,0", obj);
    TJC_END;
}

// addt透明传输必须等待屏幕就绪，并且只发送len个数据字节。
uint8_t TJC_SendWaveFast(char *obj, uint8_t *buf, uint16_t len)
{
    uint8_t attempt;

    if (len == 0) return 0;

    for (attempt = 0; attempt < TJC_TRANSFER_RETRIES; attempt++) 
    {
        uint16_t i;

        Serial_ResetTJCTransferFlags();
        Serial_Printf("addt %s.id,0,%d", obj, (int)len);
        TJC_END;

        if (!TJC_WaitTransferFlag(0)) 
        {
            Serial_FlushRX();
            Delay_ms(20);
            continue;
        }

        for (i = 0; i < len; i++)
            Serial_SendByte(buf[i]);

        // 透明传输以约定的数据长度结束，不能再追加FF FF FF。
        if (TJC_WaitTransferFlag(1)) return 1;

        Serial_FlushRX();
        Delay_ms(20);
    }

    return 0;
}

uint8_t TJC_GetCmd(void)
{
    if (g_serial_cmd != 0) 
    {
        uint8_t cmd = g_serial_cmd;
        g_serial_cmd = 0;
        return cmd;
    }
    return 0;
}
