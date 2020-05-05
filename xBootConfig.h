#ifndef XBOOTCONFIG_H_INCLUDED
#define XBOOTCONFIG_H_INCLUDED

// Begin Default Configuration Section
// --------------------------------------------------

#ifndef __AVR_XMEGA__
  #define __AVR_XMEGA__
#endif // __AVR_XMEGA__

#ifdef __AVR_XMEGA__
  #include "../xmegaClocks/xmegaClocks.h"
  #define SYSCLK CLK32M
  #define PLL 0
  #define CLOCK_CALIBRATION 0x82
#endif // __AVR_XMEGA__


#ifdef __AVR_XMEGA__
// AVR1008 fixes
// Really only applicable to XMEGA 256a3 rev A and B devices
//#define USE_AVR1008_EEPROM
#endif // __AVR_XMEGA__

// bootloader entrace
#define USE_ENTER_DELAY
//#define USE_ENTER_PIN
#define USE_ENTER_UART
//#define USE_ENTER_I2C
//#define USE_ENTER_FIFO

// bootloader exit
//#define LOCK_SPM_ON_EXIT

// bootloader communication
#define USE_LED
#define USE_UART
#define USE_UART_TxEN_PIN
//#define USE_I2C
//#define USE_I2C_ADDRESS_NEGOTIATION
//#define USE_ATTACH_LED
//#define USE_FIFO

// General Options
//#define USE_INTERRUPTS
#define USE_WATCHDOG

// bootloader features
#define ENABLE_BLOCK_SUPPORT
#define ENABLE_FLASH_BYTE_SUPPORT
#define ENABLE_EEPROM_BYTE_SUPPORT
#define ENABLE_LOCK_BITS
#define ENABLE_FUSE_BITS
#define ENABLE_FLASH_ERASE_WRITE
#define ENABLE_CRC_SUPPORT

// API
//#define ENABLE_API
#define USE_API_VERSION 1
#define ENABLE_API_LOW_LEVEL_FLASH
#define ENABLE_API_SPM_WRAPPER
#define ENABLE_API_FIRMWARE_UPDATE

// Code Protection
//#define ENABLE_CODE_PROTECTION
//#define ENABLE_EEPROM_PROTECTION
//#define ENABLE_BOOTLOADER_PROTECTION

// ENTER_PIN
#define ENTER_PORT_NAME         C
#define ENTER_PIN               0
#define ENTER_PIN_STATE         0
#define ENTER_PIN_PUEN          1

// ENTER_DELAY
#define ENTER_BLINK_COUNT       20
#define ENTER_BLINK_WAIT        240000

// ENTER_UART
#define ENTER_UART_NEED_SYNC

// ENTER_FIFO
//#define ENTER_FIFO_NEED_SYNC

// WATCHDOG
// Select only one
//#define WATCHDOG_TIMEOUT        WDT_PER_8CLK_gc
//#define WATCHDOG_TIMEOUT        WDT_PER_16CLK_gc
//#define WATCHDOG_TIMEOUT        WDT_PER_32CLK_gc
//#define WATCHDOG_TIMEOUT        WDT_PER_64CLK_gc
//#define WATCHDOG_TIMEOUT        WDT_PER_128CLK_gc
//#define WATCHDOG_TIMEOUT        WDT_PER_256CLK_gc
//#define WATCHDOG_TIMEOUT        WDT_PER_512CLK_gc
//#define WATCHDOG_TIMEOUT        WDT_PER_1KCLK_gc
//#define WATCHDOG_TIMEOUT        WDT_PER_2KCLK_gc
//#define WATCHDOG_TIMEOUT        WDT_PER_4KCLK_gc
#define WATCHDOG_TIMEOUT        WDT_PER_8KCLK_gc

// LED
#define LED_PORT_NAME           A
#define LED_PIN                 2
#define LED_INV                 0

// UART
// Select BAUD rate, port name, and UART number
// Port name is only for XMEGA
// For ATMEGA, uart number is usually 0
// UART_U2X will double clock rate for ATMEGA
// Needed for high baud rates
#define UART_BAUD_RATE          115200

#define UART_PORT_NAME          D
#define UART_NUMBER             1
//#define UART_U2X

// Pullup Enable
#define UART_RX_PUEN

// UART RS485 Enable Output
#define UART_EN_PORT_NAME       D
#define UART_TxEN_PIN             4
//#define UART_TxEN_PIN_INV         0
#define UART_RxEN_PIN           5

// FIFO
#define FIFO_DATA_PORT_NAME     C
#define FIFO_CTL_PORT_NAME      D
#define FIFO_RXF_N              3
#define FIFO_TXE_N              2
#define FIFO_RD_N               1
#define FIFO_WR_N               0
#define FIFO_BIT_REVERSE

// I2C
#define I2C_DEVICE_PORT         C

#define I2C_MATCH_ANY           1
#define I2C_ADDRESS             0x10
#define I2C_GC_ENABLE           1

// I2C Address Autonegotiation
// Note: only works on XMega chips for the time being
// There is no easy way to get this to work on regular
// ATMega chips as they have no unique part ID number
#define I2C_AUTONEG_DIS_PROMISC         1
#define I2C_AUTONEG_DIS_GC              0
#define I2C_AUTONEG_PORT_NAME           A
#define I2C_AUTONEG_PIN                 2

// Attach LED
#define ATTACH_LED_PORT_NAME            A
#define ATTACH_LED_PIN                  7
#define ATTACH_LED_INV                  1

// --------------------------------------------------
// End Default Configuration Section

#endif // XBOOTCONFIG_H_INCLUDED
