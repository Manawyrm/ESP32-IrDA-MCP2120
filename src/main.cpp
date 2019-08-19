/*
	ESP32 Emulation for 
	MCP2120 IrDA PHY

	Use irattach /dev/ttyUSB0 -d mcp2120 -s to connect

	Written by Tobias Mädel and Tobias Schramm
	Date: 19.08.2019
 */
#include <Arduino.h>
#include "esp32-hal-uart.c"

#define MCP2120_9600    0x87
#define MCP2120_19200   0x8B
#define MCP2120_38400   0x85
#define MCP2120_57600   0x83
#define MCP2120_115200  0x81

#define MCP2120_COMMIT  0x11

volatile uint8_t commandMode = 0;
volatile uint32_t newBaudRate = 9600;

void IRAM_ATTR irModeISR()
{
	commandMode = !digitalRead(0); 
}

void uartFlushTx(uart_t* uart)
{
    if(uart == NULL)
	{
        return;
    }

    UART_MUTEX_LOCK();
    while(uart->dev->status.txfifo_cnt || uart->dev->status.st_utx_out);
    UART_MUTEX_UNLOCK();
}


void setup()
{
	Serial.begin (9600); // (USB + TX/RX) to check
	Serial2.begin(9600, SERIAL_8N1, 22, 23); // GPIO 17: TXD U2  +  GPIO 16: RXD U2

	WRITE_PERI_REG( 0x3FF6E020 , READ_PERI_REG(0x3FF6E020) | (1<<16) );  //UART_IRDA_EN  "Let there be light"
	attachInterrupt(0, irModeISR, CHANGE);
}

void loop()
{
	// IrDA Transmit
	while(Serial.available())
	{
		if (commandMode)
		{
			uint8_t data = Serial.read();
			Serial.write(data);
			
			switch (data)
			{
				case MCP2120_9600:
					newBaudRate = 9600; 
					break; 
				case MCP2120_19200:
					newBaudRate = 19200; 
					break; 
				case MCP2120_38400:
					newBaudRate = 38400; 
					break; 
				case MCP2120_57600:
					newBaudRate = 57600; 
					break; 
				case MCP2120_115200: 
					newBaudRate = 115200; 
					break; 
				case MCP2120_COMMIT:
					uartFlushTx(Serial._uart);
					uartFlushTx(Serial2._uart);
					Serial.updateBaudRate(newBaudRate);
					Serial2.updateBaudRate(newBaudRate);
					break; 
				default:
					break;
			}

		}
		else
		{
			WRITE_PERI_REG( 0x3FF6E020 , READ_PERI_REG(0x3FF6E020) | (1<<10)); // enable UART_IRDA_TX_EN
			Serial2.write(Serial.read());
		}
	}
	uartFlushTx(Serial2._uart);
	WRITE_PERI_REG( 0x3FF6E020 , READ_PERI_REG(0x3FF6E020) & ~(1<<10)); // disable UART_IRDA_TX_EN


	// IrDA Receive
	while(Serial2.available())
	{
		Serial.write(Serial2.read());
	}
}