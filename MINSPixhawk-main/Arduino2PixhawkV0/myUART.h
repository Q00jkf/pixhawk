#ifndef MYUART_H
#define MYUART_H
/** define all of the UART resource used in GP1Z
 * 11/23/2023
*/
/***
SERCOM0: I2C     (PA08, PA09) [sda, scl]
SERCOM1: serial3 (PA17, PA18) [rx, tx]
SERCOM2: serial2 (PA15, PA14) [rx, tx]
SERCOM3: serial4 (PA21, PA20) [rx, tx]
SERCOM4: SPI     (PB10, PB11, PA12, PA13) [ss, miso, mosi, sck]
SERCOM5: serial1 (PB23, PB22) [rx, tx]
  
***/
#include "uartRT.h"
#include "wiring_private.h"

// Arduino Nano 33 IoT只支援Serial和Serial1
// 我們使用軟體串口或重新映射來模擬Serial3和Serial4
// 臨時解決方案：將Serial3和Serial4映射到現有串口

const uint8_t output_header[4] = {0xFE, 0x81, 0xFF, 0x55};



// Arduino Nano 33 IoT 相容性：移除ISR標誌外部聲明
// extern bool ISR_flag_NMEA; // 已移除

void myUART_init(void)
{
    Serial.begin(115200);  // debug (也用作NMEA_OUT_Serial)
    Serial1.begin(230400); // output (PX4) - 也用於NMEA_IN_Serial
    Serial2.begin(115200); // Xsens
    // Arduino Nano 33 IoT 相容性：移除Serial3和Serial4初始化

    // 僅保留必要的引腳配置
    pinPeripheral(24, PIO_SERCOM);  // Serial1 TX
    pinPeripheral(25, PIO_SERCOM);  // Serial1 RX
}

#endif