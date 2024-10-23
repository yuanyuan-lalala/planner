#pragma once 
#include "verify.h"
void UART_RSDataDecode(u8* buffer, u16 Size);

union float2uchar {
    float f;
    unsigned char ch[4];
};