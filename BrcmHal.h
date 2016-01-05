/*****************************************************************************/
/**
 * \file   BrcmHal.h
 * \brief  Definitions for Broadcom HAL interface.
 */
/*****************************************************************************
 * Copyright 2013-2015 Broadcom Corporation -- http://www.broadcom.com
 * This program is the proprietary software of Broadcom Corporation and/or
 * its licensors, and may only be used, duplicated, modified or distributed
 * pursuant to the terms and conditions of a separate, written license
 * agreement executed between you and Broadcom (an "Authorized License").
 * Except as set forth in an Authorized License, Broadcom grants no license
 * (express or implied), right to use, or waiver of any kind with respect to
 * the Software, and Broadcom expressly reserves all rights in and to the
 * Software and all intellectual property rights therein. IF YOU HAVE NO
 * AUTHORIZED LICENSE, THEN YOU HAVE NO RIGHT TO USE THIS SOFTWARE IN ANY
 * WAY, AND SHOULD IMMEDIATELY NOTIFY BROADCOM AND DISCONTINUE ALL USE OF
 * THE SOFTWARE.
 *****************************************************************************/
#ifndef BRCM_HAL_H
#define BRCM_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
 * Standard Types
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include "BrcmHalMacros.h"

/** \brief  Indicates the current API version.
 *
 *  Indicates the current API version and can be used by customer code to verify
 *  that the standard that was coded to is still the standard in effect for this
 *  API.
 */
#define HAL_API_VERSION 23


/******************************************************************************
 * Common Device Error Status
 */
/** \brief  Broadcom BSP Device Status.
 *
 *  Broadcom BSP Device Status to be returned from BSP functions.
 */
typedef enum BrcmDeviceStatus
{
    BRCM_DEV_STAT_SUCCESS           =  0,       ///< Success without error.
    BRCM_DEV_STAT_INVALID_PARAM     = -1,       ///< Invalid parameters.
    BRCM_DEV_STAT_BUSY              = -2,       ///< Device is busy.
    BRCM_DEV_STAT_NO_RESOURCE       = -3,       ///< Device is out of resources.
    BRCM_DEV_STAT_TIMEOUT           = -4,       ///< Device operation timed out.
    BRCM_DEV_STAT_ABORTED           = -5,       ///< Device operation got aborted.
    BRCM_DEV_STAT_INVALID_OPERATION = -6,       ///< Invalid device's operation.
    BRCM_DEV_STAT_NO_DEVICE         = -7,       ///< No device found.
    BRCM_DEV_STAT_FAILURE           = -8,       ///< Operation failure
    BRCM_DEV_STAT_NO_RESPONSE       = -9,       ///< No response
    BRCM_DEV_STAT_BUS_ERROR         = -10,      ///< Bus error
} BrcmDeviceStatus;


/******************************************************************************
 * ADC
 */
/** \brief  Broadcom BSP ADC Channel.
 *
 *  Broadcom BSP ADC Channel to indicate the channel to access.
 */
typedef enum BrcmAdcChannel
{
    BRCM_ADC_CHANNEL_NONE           = -1,       ///< Internal Use: No channel selected
    BRCM_ADC_CHANNEL_0              = 0,        ///< Channel 0
    BRCM_ADC_CHANNEL_1,                         ///< Channel 1
    BRCM_ADC_CHANNEL_2,                         ///< Channel 2
    BRCM_ADC_CHANNEL_3,                         ///< Channel 3
    BRCM_ADC_CHANNEL_4,                         ///< Channel 4
    BRCM_ADC_CHANNEL_5,                         ///< Channel 5
    BRCM_ADC_CHANNEL_6,                         ///< Channel 6
    BRCM_ADC_CHANNEL_7                          ///< Channel 7
} BrcmAdcChannel;


/** \brief  Initialize the ADC for the requested channel.
 *
 *  Used to select the currently active ADC channel and perform any required
 *  initialization so values can be read from the channel.  A call to
 *  BrcmAdcCloseChannel is not required to change to a new ADC channel.
 *  BrcmAdcInitializeChannel will take care of any closing functionality that
 *  is required for the channel switch.
 *
 *  \param[in]  channel     the selected channel to initialize and perform
 *                          subsequent reads from.
 *
 *  \return     The status of the function on exit.
 */
BrcmDeviceStatus BrcmAdcInitializeChannel(BrcmAdcChannel channel);

/** \brief  Read a single value when the data is valid.
 *
 *  Used to read a single value from the selected ADC channel when it is valid.
 *  This call requires BrcmAdcInitializeChannel to have been called prior to
 *  select the channel to read from.
 *
 *  \param[out] pData       a pointer to a word to store the read ADC value.
 *
 *  \return     The status of the function on exit.
 */
BrcmDeviceStatus BrcmAdcReadChannel(uint16_t* pData);

/** \brief  Close the current ADC channel.
 *
 *  Used to deselect the currently active ADC channel and to disable any power
 *  from the circuits that are no longer required.
 *
 *  \return     The status of the function on exit.
 */
BrcmDeviceStatus BrcmAdcCloseChannel(void);



/******************************************************************************
 * I2Cm
 */
/** \brief  Broadcom BSP I2C Bus ID.
 *
 *  Broadcom BSP I2C Master Bus ID.
 */
typedef enum BrcmI2cBusId
{
    BRCM_I2C_BUS_M1             = 0,            ///< I2C Bus ID for I2C Master 1.
    BRCM_I2C_BUS_M2,                            ///< I2C Bus ID for I2C Master 2.
    BRCM_I2C_BUS_M3                             ///< I2C Bus ID for I2C Master 3.
} BrcmI2cBusId;

/** \brief  Broadcom BSP I2C Address Size.
 *
 *  Broadcom BSP I2C Address Size.
 */
typedef enum BrcmI2cAddrSize
{
    BRCM_I2C_ADDR_7BIT          = 0,            ///< I2C Device Address is 7 bits.
    BRCM_I2C_ADDR_10BIT                         ///< I2C Device Address is 10 bits.
} BrcmI2cAddrSize;

/** \brief  Broadcom BSP I2C Bus Speed.
 *
 *  Broadcom BSP I2C Master Bus Speed.
 */
typedef enum BrcmI2cBusSpeed
{
    BRCM_I2C_SPEED_100KHZ       = 0,            ///< I2C Bus Speed should be 100KHz.
    BRCM_I2C_SPEED_400KHZ,                      ///< I2C Bus Speed should be 400KHz.
    BRCM_I2C_SPEED_1MHZ                         ///< I2C Bus Speed should be 1MHz.
} BrcmI2cBusSpeed;

/** \brief  Broadcom I2C Flags.
 *
 *  Broadcom I2C Flags used to describe the connection type to use when accessing
 *  a specific device.
 */
typedef struct BrcmI2cFlag
{
    BrcmI2cBusId            busId        :3;    ///< I2C Bus ID
    BrcmI2cAddrSize         addrWidth    :1;    ///< I2C Address Width
    BrcmI2cBusSpeed         speed        :3;    ///< I2C Bus Speed
} BrcmI2cFlag;


/** \brief  Transfer data to/from an I2C device.
 *
 *  Used to transfer data between us as the master and a slave I2C device.  The
 *  order of operation is to perform write transfers, if any, followed by read
 *  transfers, if any.  This function will block until the transaction is
 *  completed.
 *
 *  \param[in]  i2cmFlags           flags to describe the connection with a
 *                                  specific I2C slave.
 *  \param[in]  devAddr             I2C slave device address.
 *  \param[in]  writeBytesRequested number of bytes to write to the I2C slave
 *                                  device.  Zero will indicate no write data
 *                                  is to be sent.
 *  \param[in]  pWriteBuffer        buffer address used to send the data to be
 *                                  written to the I2C slave device.
 *  \param[in]  readBytesRequested  number of bytes to read from the I2C slave
 *                                  device.  Zero will indicate no read data
 *                                  is to be retrieved.
 *  \param[out] pReadBuffer         buffer address used to pass the data back to
 *                                  the caller that has been read from the I2C
 *                                  slave device.
 *
 *  \return     The status of the function on exit.
 */
BrcmDeviceStatus BrcmI2cWriteRead(BrcmI2cFlag   i2cmFlags,
                                  uint16_t      devAddr,
                                  uint16_t      writeBytesRequested,
                                  uint8_t*      pWriteBuffer,
                                  uint16_t      readBytesRequested,
                                  uint8_t*      pReadBuffer);



/******************************************************************************
 * I2S
 */
/** \brief  Broadcom BSP I2S Stream Type.
 *
 *  Broadcom BSP I2S Stream Play/Record Type.
 */
typedef enum BrcmI2sStreamType
{
    BRCM_I2S_PLAY_STREAM                = 0,    ///< I2S Play stream.
    BRCM_I2S_RECORD_STREAM              = 1,    ///< I2S Record stream.
    BRCM_I2S_RECORD_PLAY_STREAM         = 2     ///< I2S Record and play stream.
} BrcmI2sStreamType;

/** \brief  Broadcom BSP I2S Mode Type.
 *
 *  Broadcom BSP I2S Master/Mono Mode Type.
 */
typedef enum BrcmI2sMasterMode
{
    BRCM_I2S_SLAVE                      = 0,    ///< I2S Slave mode.
    BRCM_I2S_MASTER                     = 1     ///< I2S Master mode.
} BrcmI2sMasterMode;

/** \brief  Broadcom BSP I2S Sound Format PCM Rate.
 *
 *  Broadcom BSP I2S Sound Format PCM Rate.
 */
typedef enum BrcmI2sSoundFormatRate
{
    BRCM_I2S_SND_FMT_RATE_8000          = 0,    ///< I2S Sound Format 8000 PCM Rate.
    BRCM_I2S_SND_FMT_RATE_16000         = 1,    ///< I2S Sound Format 16000 PCM Rate.
    BRCM_I2S_SND_FMT_RATE_22050         = 2,    ///< I2S Sound Format 22050 PCM Rate.
    BRCM_I2S_SND_FMT_RATE_32000         = 3,    ///< I2S Sound Format 32000 PCM Rate.
    BRCM_I2S_SND_FMT_RATE_44100         = 4,    ///< I2S Sound Format 44100 PCM Rate.
    BRCM_I2S_SND_FMT_RATE_48000         = 5,    ///< I2S Sound Format 48000 PCM Rate.
    BRCM_I2S_SND_FMT_RATE_96000         = 6     ///< I2S Sound Format 96000 PCM Rate.
} BrcmI2sSoundFormatRate;

/** \brief  Broadcom BSP I2S Sound Format Sample Size.
 *
 *  Broadcom BSP I2S Sound Format Sample Size.
 */
typedef enum BrcmI2sSoundFormatSampleSize
{
    BRCM_I2S_SND_FMT_16_BIT_SAMPLE      = 0,    ///< I2S Sound Format 16 bit sample.
    BRCM_I2S_SND_FMT_24_BIT_SAMPLE      = 1,    ///< I2S Sound Format 24 bit sample.
    BRCM_I2S_SND_FMT_32_BIT_SAMPLE      = 2     ///< I2S Sound Format 32 bit sample.
} BrcmI2sSoundFormatSampleSize;

/** \brief  Broadcom BSP I2S Sound Format Justified.
 *
 *  Broadcom BSP I2S Sound Format Justified.
 */
typedef enum BrcmI2sSoundFormatJustified
{
    BRCM_I2S_SND_FMT_LEFT_JUSTIFIED     = 0,    ///< I2S Sound Format Left Justified.
    BRCM_I2S_SND_FMT_RIGHT_JUSTIFIED    = 1     ///< I2S Sound Format Right Justified.
} BrcmI2sSoundFormatJustified;

/** \brief  Broadcom BSP I2S Sound Format Mono/Stereo.
 *
 *  Broadcom BSP I2S Sound Format Mono/Stereo.
 */
typedef enum BrcmI2sSoundFormatMono
{
    BRCM_I2S_SND_FMT_STEREO             = 0,    ///< I2S Sound Format Stereo.
    BRCM_I2S_SND_FMT_MONO               = 1     ///< I2S Sound Format Mono.
} BrcmI2sSoundFormatMono;

/** \brief  Broadcom BSP I2S Open Configuration Structure.
 *
 *  Broadcom BSP I2S Open Configuration Structure.
 */
typedef struct BrcmI2sOpenConfig
{
    BrcmI2sStreamType               streamType;             ///< Stream Type.
    BrcmI2sMasterMode               master;                 ///< Master/Slave.
    BrcmI2sSoundFormatRate          soundFormatRate;        ///< PCM Rate.
    BrcmI2sSoundFormatSampleSize    soundFormatSampleSize;  ///< Bit Sample Size.
    BrcmI2sSoundFormatJustified     soundFormatJustified;   ///< Justified Right/Left.
    BrcmI2sSoundFormatMono          soundFormatMono;        ///< Mono/Stereo.

    uint8_t*                        rx_buffer;              ///< Receive Buffer Pointer
    uint32_t                        rx_buffer_len;          ///< Receive Buffer Byte Length
    uint8_t*                        tx_buffer;              ///< Transmit Buffer Pointer
    uint32_t                        tx_buffer_len;          ///< Transmit Buffer Byte Length
} BrcmI2sOpenConfig;


/** \brief  I2S device open.
 *
 *  Used to open and configure an I2S connection.
 *
 *  \param[in]  pOpenConfig         Pointer to an BrcmI2sOpenConfig structure
 *                                  to configure the stream and open it.
 *
 *  \return     The status of the function on exit.
 */
BrcmDeviceStatus BrcmI2sOpen(BrcmI2sOpenConfig* pOpenConfig);

/** \brief  I2S device read.
 *
 *  Used to read on an I2S connection.
 *
 *  \param[in]  pBuffer             Pointer to a buffer to read into.
 *  \param[in]  len                 Byte length of pBuffer.
 *  \param[in]  timeoutInMs         Timeout in milliseconds.
 *
 *  \return     The status of the function on exit.
 */
BrcmDeviceStatus BrcmI2sRead(uint8_t* pBuffer, int32_t len, uint32_t timeoutInMs);

/** \brief  I2S device write.
 *
 *  Used to write on an I2S connection.
 *
 *  \param[in]  pBuffer             Pointer to a buffer to write from.
 *  \param[in]  len                 Byte length of pBuffer.
 *
 *  \return     The status of the function on exit.
 */
BrcmDeviceStatus BrcmI2sWrite(uint8_t* pBuffer, int32_t len);

/** \brief  I2S device flush RX buffer.
 *
 *  Used to flush the RX buffer on an I2S connection.
 *
 *  \return     The status of the function on exit.
 */
BrcmDeviceStatus BrcmI2sFlushRxBuffer(void);

/** \brief  I2S device stream start.
 *
 *  Used to start a stream on an I2S connection.
 *
 *  \return     The status of the function on exit.
 */
BrcmDeviceStatus BrcmI2sStreamStart(void);

/** \brief  I2S device stream pause.
 *
 *  Used to pause a stream on an I2S connection.
 *
 *  \return     The status of the function on exit.
 */
BrcmDeviceStatus BrcmI2sStreamPause(void);

/** \brief  I2S device stream resume.
 *
 *  Used to resume a stream on an I2S connection.
 *
 *  \return     The status of the function on exit.
 */
BrcmDeviceStatus BrcmI2sStreamResume(void);

/** \brief  I2S device stream stop.
 *
 *  Used to stop a stream on an I2S connection.
 *
 *  \return     The status of the function on exit.
 */
BrcmDeviceStatus BrcmI2sStreamStop(void);

/** \brief  I2S device close.
 *
 *  Used to close an I2S connection.
 *
 *  \return     The status of the function on exit.
 */
BrcmDeviceStatus BrcmI2sClose(void);



/******************************************************************************
 * SPIm
 */
/** \brief  Broadcom BSP SPI Clock Prescale.
 *
 *  Broadcom BSP SPI Clock Prescale.  This is not an exhaustive list.  The
 *  calculation is as follows 26MHz = desiredSpeed * prescale.  The limitations
 *  on prescale include the smallest prescale value is 2 and it has to be an
 *  even number.
 */
typedef enum BrcmSpiClkPrescale
{
    BRCM_SPI_CLK_PRESCALE_13MHZ         = 2,    ///< SPI bus frequency 13MHz (26MHz/prescale).
    BRCM_SPI_CLK_PRESCALE_6_5MHZ        = 4,    ///< SPI bus frequency 6.5MHz (26MHz/prescale).
    BRCM_SPI_CLK_PRESCALE_4_3MHZ        = 6,    ///< SPI bus frequency 4.3MHz (26MHz/prescale).
    BRCM_SPI_CLK_PRESCALE_3_25MHZ       = 8,    ///< SPI bus frequency 3.25MHz (26MHz/prescale).
    BRCM_SPI_CLK_PRESCALE_2_6MHZ        = 10,   ///< SPI bus frequency 2.6MHz (26MHz/prescale).
    BRCM_SPI_CLK_PRESCALE_2_17MHZ       = 12,   ///< SPI bus frequency 2.17MHz (26MHz/prescale).
    BRCM_SPI_CLK_PRESCALE_1_86MHZ       = 14,   ///< SPI bus frequency 1.86MHz (26MHz/prescale).
    BRCM_SPI_CLK_PRESCALE_1_625MHZ      = 16,   ///< SPI bus frequency 1.625MHz (26MHz/prescale).
    BRCM_SPI_CLK_PRESCALE_1_44MHZ       = 18,   ///< SPI bus frequency 1.44MHz (26MHz/prescale).
    BRCM_SPI_CLK_PRESCALE_1_3MHZ        = 20,   ///< SPI bus frequency 1.3MHz (26MHz/prescale).
    BRCM_SPI_CLK_PRESCALE_1_18MHZ       = 22,   ///< SPI bus frequency 1.18MHz (26MHz/prescale).
    BRCM_SPI_CLK_PRESCALE_1_08MHZ       = 24,   ///< SPI bus frequency 1.08MHz (26MHz/prescale).
    BRCM_SPI_CLK_PRESCALE_1MHZ          = 26,   ///< SPI bus frequency 1MHz (26MHz/prescale).
} BrcmSpiClkPrescale;

/** \brief  Broadcom BSP SPI Frame Format.
 *
 *  Broadcom BSP SPI Motorola Frame Format.
 */
typedef enum BrcmSpiFrameFormat
{
    BRCM_SPI_FRAME_FORMAT_0             = 0,    ///< Motorola SPI Frame Format 0 (POL=0, PHA=0).
    BRCM_SPI_FRAME_FORMAT_1,                    ///< Motorola SPI Frame Format 1 (POL=0, PHA=1).
    BRCM_SPI_FRAME_FORMAT_2,                    ///< Motorola SPI Frame Format 2 (POL=1, PHA=0).
    BRCM_SPI_FRAME_FORMAT_3                     ///< Motorola SPI Frame Format 3 (POL=1, PHA=1).
} BrcmSpiFrameFormat;

/** \brief  Broadcom BSP SPI Chip Select GPIO Active State.
 *
 *  Broadcom BSP SPI Chip Select GPIO Active State.
 */
typedef enum BrcmSpiChipSelectActive
{
    BRCM_SPI_CHIP_SELECT_ACTIVE_LOW     = 0,    ///< SPI GPIO Chip Select is Active Low.
    BRCM_SPI_CHIP_SELECT_ACTIVE_HIGH            ///< SPI GPIO Chip Select is Active High.
} BrcmSpiChipSelectActive;

/** \brief  Broadcom BSP SPI Duplex Mode.
 *
 *  Broadcom BSP SPI Duplex Mode.
 */
typedef enum BrcmSpiDuplex
{
    BRCM_SPI_FULL_DUPLEX                = 0,    ///< SPI transfer should be performed in Full Duplex.
    BRCM_SPI_EMULATE_HALF_DUPLEX                ///< SPI transfer should be performed in emulated Half Duplex.
} BrcmSpiDuplex;


/**
 *  \brief  SPI GPIO Chip Select Pin to use if GPIO is not used for the
 *          connection with the current slave.
 */
#define BRCM_SPI_CS_PIN_NO_GPIO_PIN         0xFF


/** \brief  Broadcom SPI Flags.
 *
 *  Broadcom SPI Flags used to describe the connection type to use when accessing
 *  a specific device.
 */
typedef struct BrcmSpiFlag
{
    BrcmSpiClkPrescale      serialClockPrescale     : 8;    ///< Requested bus speed
    BrcmSpiFrameFormat      spiFrameFormat          : 2;    ///< Polarity/Phase for Motorola SPI Frame Format
    BrcmSpiChipSelectActive chipSelectActiveState   : 1;    ///< Electrical active state of chip select
    BrcmSpiDuplex           emulateHalfDuplex       : 1;    ///< Duplex transfer mode
} BrcmSpiFlag;


/** \brief  Transfer data to/from a SPI device.
 *
 *  Used to transfer data between us as the master and a slave SPI device.  The
 *  operation will either be performed in FULL Duplex, which means that a write
 *  data byte is sent for every read data byte.  This is the standard SPI bus
 *  interface.  If the operation is to be performed in Emulated HALF Duplex,
 *  the order of operation is to perform write transfers, if any, followed by read
 *  transfers, if any.
 *
 *  \param[in]  spiFlags            flags to describe the connection with a
 *                                  specific I2C slave. The serialClockPrescale
 *                                  is a value between 2 and 254. It must be even.
 *                                  It is used to specify the clock speed to use
 *                                  for a given device.  The main clock is 26MHz
 *                                  and this value will divide into that. The
 *                                  emulateHalfDuplex is zero to use the interface
 *                                  in SPI Full Duplex mode. In this mode, the
 *                                  readBytesRequested will be ignored.  The
 *                                  writeBuffer and readBuffer have to be the same
 *                                  size. A byte is transferred out of the
 *                                  writeBuffer and the swapped byte is stored in
 *                                  the readBuffer. When emulateHalfDuplex is one
 *                                  the interface will emulate half duplex.  The
 *                                  writeBuffer will contain the actual write
 *                                  request portion and any pads required by the
 *                                  device. The readBuffer will contain only the
 *                                  requested data and not the dummy bytes.
 *  \param[in]  csPin               SPI GPIO Chip Select pin to use.
 *  \param[in]  writeBytesRequested number of write bytes to send for emulated
 *                                  half duplex mode or the number of read and
 *                                  write bytes for full duplex mode.
 *  \param[in]  pWriteBuffer        buffer address used to send the data to be
 *                                  written to the SPI slave device.
 *  \param[in]  readBytesRequested  number of read bytes to receive for emulated
 *                                  half duplex mode and is ignored in full duplex
 *                                  mode.
 *  \param[out] pReadBuffer         buffer address used to pass the data back to
 *                                  the caller that has been read from the SPI
 *                                  slave device.
 *
 *  \return     The status of the function on exit.
 */
BrcmDeviceStatus BrcmSpiWriteRead(
    BrcmSpiFlag             spiFlags,
    uint8_t                 csPin,
    uint16_t                writeBytesRequested,
    uint8_t*                pWriteBuffer,
    uint16_t                readBytesRequested,
    uint8_t*                pReadBuffer);



/*****************************************************************************
 * System time
 * API to Get/Set System Time in microseconds/milliseconds
 */
/** \brief  Get System Time in micro-seconds.
 *
 *  Used to read the System Time and convert this to micro-seconds.
 *
 *  \return     The converted System Time in micro-seconds.
 */
uint64_t BrcmGetSystemTimeUs(void);

/** \brief  Get System Time in milli-seconds.
 *
 *  Used to read the System Time and convert this to milli-seconds.
 *  \return     The converted System Time in milli-seconds.
 */
uint32_t BrcmGetSystemTimeMs(void);



/*****************************************************************************
 * RTC
 * API to set and get the RTC register. That value is maintained as long as power is maintained to the chipset
 */
/** \brief  Set the RTC with this micro-second value.
 *
 *  Used to set the RTC register with this micro-second value.
 *
 *  \param[in]  valueUs     micro-second value to set in the RTC register.
 */
void BrcmSetRtcRegister(uint64_t valueUs);

/** \brief  Get the RTC value and convert this to micro-seconds.
 *
 *  Used to get the RTC register and convert this value to a micro-second value.
 *  If BrcmSetRtcRegister was never set, this will return the same value as
 *  BrcmGetSystemTimeUs
 *
 *  \return     The converted RTC Register value in micro-seconds.
 */
uint64_t BrcmGetRtcRegister(void);



/*****************************************************************************
 * Timer Utilities
 *
 * Deprecated!!! Use the equivalent RTOS function
 */
typedef void (*BrcmOnTimerExpired)(void* pUserData);

#define BrcmTimerCreate     DEPRECATED: Use the RTOS native Timer API
#define BrcmTimerStart      DEPRECATED: Use the RTOS native Timer API
#define BrcmTimerStop       DEPRECATED: Use the RTOS native Timer API
#define BrcmTimerRetrigger  DEPRECATED: Use the RTOS native Timer API
#define BrcmTimerSetPeriod  DEPRECATED: Use the RTOS native Timer API
#define BrcmTimerDelete     DEPRECATED: Use the RTOS native Timer API



/*****************************************************************************
 * Sleep in microseconds/milliseconds
 *
 * Deprecated!!! Use the equivalent RTOS function
 */
#define BrcmSleepUs         DEPRECATED: Use the RTOS native Sleep API
#define BrcmSleepMs         OS_Delay



/*****************************************************************************
 * GPIO
 */
/** \brief  Broadcom BSP GPIO Pin Mux Mode Selector values.
 *
 *  Broadcom BSP GPIO pin Mux Mode Selector values.
 */
typedef enum
{
   BRCM_GPIO_MUX_SEL_0 = 0,                     ///< pin mux selection, mode 0
   BRCM_GPIO_MUX_SEL_1 = 1,                     ///< pin mux selection, mode 1
   BRCM_GPIO_MUX_SEL_2 = 2,                     ///< pin mux selection, mode 2
   BRCM_GPIO_MUX_SEL_3 = 3,                     ///< pin mux selection, mode 3
   BRCM_GPIO_MUX_SEL_4 = 4,                     ///< pin mux selection, mode 4
   BRCM_GPIO_MUX_SEL_5 = 5                      ///< pin mux selection, mode 5
} BrcmGpioMuxSel;

/** \brief  Broadcom BSP GPIO Pin State values.
 *
 *  Broadcom BSP GPIO pin High/Low state values.
 */
typedef enum BrcmGpioPinState
{
    BRCM_GPIO_PIN_LOW  = 0,                     ///< LOW GPIO Pin Value
    BRCM_GPIO_PIN_HIGH = 1,                     ///< HIGH GPIO Pin Value
    BRCM_GPIO_PIN_ERROR = 2                     ///< Errorneous GPIO Pin Value
} BrcmGpioPinState;

/** \brief  Broadcom BSP GPIO Pin Direction.
 *
 *  Broadcom BSP GPIO pin direction values.
 */
typedef enum BrcmGpioDirection
{
   BRCM_GPIO_DIR_INPUT    = 0,                  ///< GPIO Input Direction
   BRCM_GPIO_DIR_OUTPUT   = 1                   ///< GPIO Output Direction
} BrcmGpioDirection;

/** \brief  Broadcom BSP GPIO Pin Pull.
 *
 *  Broadcom BSP GPIO pin pull values.
 */
typedef enum BrcmGpioPull
{
   BRCM_GPIO_PULL_DISABLE       = 0,            ///< GPIO PULL Disabled
   BRCM_GPIO_PULL_ENABLE_UP     = 1,            ///< GPIO PULL-UP
   BRCM_GPIO_PULL_ENABLE_DOWN   = 2             ///< GPIO PULL-DOWN
} BrcmGpioPull;

/** \brief  Broadcom BSP GPIO Interrupt Type.
 *
 *  Broadcom BSP GPIO Interrupt Type values.
 */
typedef enum BrcmGpioInterrupt
{
   BRCM_GPIO_INTERRUPT_OFF                      = 0,  ///< GPIO Interrupt Disabled
   BRCM_GPIO_INTERRUPT_RISING_EDGE              = 1,  ///< GPIO Interrupt Rising Edge
   BRCM_GPIO_INTERRUPT_FALLING_EDGE             = 2,  ///< GPIO Interrupt Falling Edge
   BRCM_GPIO_INTERRUPT_BOTH_EDGES               = 3,  ///< GPIO Interrupt Both Edges
   BRCM_GPIO_INTERRUPT_LEVEL_TRIGGER_HIGH       = 4,  ///< GPIO Interrupt Level High
   BRCM_GPIO_INTERRUPT_LEVEL_TRIGGER_LOW        = 5,  ///< GPIO Interrupt Level Low
   BRCM_GPIO_INTERRUPT_RISING_EDGE_AUTOCLEAR    = 7,  ///< GPIO Interrupt Rising Edge AutoCleared (4773)
   BRCM_GPIO_INTERRUPT_FALLING_EDGE_AUTOCLEAR   = 8,  ///< GPIO Interrupt Falling Edge AutoCleared (4773)
} BrcmGpioInterrupt;

/** \brief Broadcom BSP GPIO Interrupt Callback prototype.
 *
 * Broadcom BSP GPIO Interrupt Callback prototype which passes a GPIO pin and
 * the current state of that pin.
 */
typedef void (*BrcmOnGpioInterrupt)(uint32_t pin, BrcmGpioPinState state);

/** \brief Broadcom BSP GPIO Pin Configuration
 *
 *  Broadcom BSP GPIO Configuration structure used to set the configuration of
 *  a specific GPIO pin.
 */
typedef struct BrcmGpioPinConfig
{
   BrcmGpioDirection    direction;              ///< Direction
   BrcmGpioInterrupt    interrupt;              ///< Interrupt Type
   BrcmGpioPull         pull;                   ///< Pull Type
   BrcmOnGpioInterrupt  callback;               ///< Callback for enabled interrupts
} BrcmGpioPinConfig;


/** \brief  Set the GPIO Pin Configuration
 *
 *  This will set the GPIO Pin Configuration for a specific pin.
 *
 *  \param[in]  pin         GPIO Pin.
 *  \param[in]  config_ptr  GPIO Pin Configuration data.
 *  \return     true if the configuration was set, false if there was a problem.
 */
bool BrcmGpioSetConfig( uint32_t pin, const BrcmGpioPinConfig* config_ptr );

/** \brief  Set the GPIO Pin Mux Mode
 *
 *  This will set the GPIO Pin Mux Mode for a specific pin.
 *
 *  \param[in]  pin         GPIO Pin.
 *  \param[in]  mode        GPIO Pin Mux Mode.
 *  \return     true if the mux mode was set, false if there was a problem.
 */
bool BrcmGpioSetMuxMode( uint32_t pin, BrcmGpioMuxSel mode );

/** \brief  Get the GPIO Pin's Mux Mode
 *
 *  This will get the current Mux Mode for the specified GPIO pin.
 *
 *  \param[in]  pin         GPIO Pin.
 *  \return     the current mux mode of the specified GPIO pin.
 */
BrcmGpioMuxSel BrcmGpioGetMuxMode( uint32_t pin );

/** \brief  Get the GPIO Pin current state
 *
 *  This will get the GPIO Pin current state for a specific pin.
 *
 *  \param[in]  pin         GPIO Pin.
 *
 *  \return     The state of the specified GPIO Pin.
 */
BrcmGpioPinState BrcmGpioGetState( uint32_t pin );

/** \brief  Set the GPIO Pin current state
 *
 *  This will set the GPIO Pin current state for a specific pin.
 *
 *  \param[in]  pin         GPIO Pin.
 *  \param[in]  state       GPIO Pin State value to set.
 *
 *  \return     false if the set failed, otherwise true.
 */
bool BrcmGpioSetState( uint32_t pin, BrcmGpioPinState state );

/** \brief  Clear the GPIO Pin interrupt status
 *
 *  This will clear the GPIO Pin interrupt status for a specific pin.
 *
 *  \param[in]  pin         GPIO Pin.
 *
 *  \return     false if the clear failed, otherwise true.
 */
bool BrcmGpioClearInterruptStatus( uint32_t pin );

/** \brief  Indicate the state of the GPIO Pin interrupt status
 *
 *  This will indicate if the GPIO Pin interrupt is set or not.
 *
 *  \param[in]  pin         GPIO Pin.
 *
 *  \return     true if there is a pending interrupt, otherwise false.
 */
bool BrcmGpioInterruptIsDetected( uint32_t pin );

/** \brief  Enable the GPIO Pin interrupts
 *
 *  This will enable the GPIO Pin interrupts for a specific pin.
 *
 *  \param[in]  pin         GPIO Pin.
 *
 *  \return     false if the enable failed, otherwise true.
 */
bool BrcmGpioEnableInterrupt( uint32_t pin );

/** \brief  Disable the GPIO Pin interrupts
 *
 *  This will disable the GPIO Pin interrupts for a specific pin.
 *
 *  \param[in]  pin         GPIO Pin.
 *
 *  \return     false if the disable failed, otherwise true.
 */
bool BrcmGpioDisableInterrupt( uint32_t pin );



/*****************************************************************************
 * Log Management
 *
 */
/** \brief  Output Log Message
 *
 *  Output a log message to the main transport host port.
 *  NOTE: You really want to call BRCM_HAL_LOG_x calls defined in BrcmHalMacros.h
 *
 *  \param[in]  ucFacPri        Facility/Priority
 *  \param[in]  pFmt            C style format string followed by the inserted
 *                              values, just as in printf
 */
void HalLog(uint8_t ucFacPri, const char* fmt, ...);

/*****************************************************************************
 * Set the logger buffer
 *
 */
/** \brief  Set the logger buffer
 *
 *  The logger will not send any messages to the host unless a buffer is provided
 *  for buffering the log messages.  Recomended size is 10kB, minimum size 2kB.
 *
 *  \param[in]  bufferSize      Size of the buffer
 *  \param[in]  buf             Buffer for buffering log messages.
 *
 */
void BrcmHalLogSetBuffer(uint16_t bufferSize, uint8_t* pBuf);

/*****************************************************************************
 * Set a callback for the User to redirect logs to its prefered output (UART1, write to flash, etc...)
 */
/** \brief  Log callback prototype
 *
 *  This is a protoype to define the callback that will be called when logs are available
 *  Note: This callback is executed from the log caller thread, so special care to not use stack space nor block should be taken.
 */
typedef void (*BrcmOnLog)(void *pUserData, unsigned char *data, unsigned int len);

/** \brief  Set the logger callback
 *
 *  The logger will call this callback when a log is available
 *
 *  \param[in]  cb      Callback for the logger. If NULL (default), logs will go thru the normal transport layer
 *
 */
void BrcmHalLogSetCallback(BrcmOnLog cb, void *pUserData);


/** \brief  Set the priority mask for the logger.
 *
 * The logging priority in BrcmHalMacros.h defines 8 levels of priority for the loggings. The user can use this function
 * to set the mask to enable a priority level. The n-th bit in the priority mask is used to control the priority level n.
 * The 0xff primask is used to enable all 8 levels.
 *
 *  \param[in]  ucPriMask      The priority mask.
 */
void BrcmHalLogSetPriorityMask(uint8_t ucPriMask);


/** \brief  Return the current the priority mask of the logger.
 *
 * This function returns the current setting of the logger's priority mask.
 */
uint8_t BrcmHalLogGetPriorityMask(void);

/** \brief  Set the facility mask for the logger.
 *
 * This function is used to enable the customer logging facility.
 *
 * Example:
 *    BrcmHalLogSetFacilityMask(BRCM_LOG_FACILITY(BRCM_FAC_CUST1)); // Setting facility mask for customer1 log.
 *    BrcmHalLogSetFacilityMask(BRCM_LOG_FACILITY(BRCM_FAC_CUST1) |
 *                              BRCM_LOG_FACILITY(BRCM_FAC_CUST2)); // Setting facility mask for customer1 & customer2 log.
 *
 *  \param[in]  ulFacMask      The facility mask.
 */
void BrcmHalLogSetFacilityMask(uint32_t ulFacMask);


/** \brief  Return the current the facility mask of the logger.
 *
 * This function returns the current setting of the logger's facility mask.
 */
uint32_t BrcmHalLogGetFacilityMask(void);



/*****************************************************************************
 * Power Management
 *
 */
/** \brief  Structure to configure the Broadcom's power management engine.
 *
 *  This structure is used to configure the following parameters.
 *  pmu1P8V                 to configure the PMU power source of 3.3V or 1.8V.
 *  externalLDO             set to 1 to select the external LDO configuration.
 *  sleep_off               to completely turn off the sleep mode.
 */
typedef struct {
    struct {
        uint32_t  reserved            :29;  ///< Unused.
        uint32_t  pmu1P8V             : 1;  ///< 0 = 3.3V power source, 1 = 1.8V power source.
        uint32_t  externalLDO         : 1;  ///< 0 = internal-ldo, 1 = external-ldo.
        uint32_t  sleepOff            : 1;  ///< Turn off the sleep mode. The default sleep mode is on.
    } controlFlags;
    int32_t     sleepDebugGPIO;             ///< GPIO to use for sleep debug signal.
} BrcmPwrMgrConfig;

/** \brief Get the current power management's configuration.
 *
 * \param pConfig pointer to the configuration structure.
 */
void BrcmPwrMgrGetConfig(BrcmPwrMgrConfig *pConfig);

/** \brief Set the power management's configuration.
 *
 *  Set the Power Management configuration options.  This function should be
 *  used to initialize the Power Management settings at startup.  It is
 *  recommended that BrcmPwrMgrGetConfig be used to get the current settings,
 *  change the options that are specific to your settings and then call
 *  BrcmPwrMgrSetConfig.
 *
 * \param pConfig pointer to the configuration structure.
 */
void BrcmPwrMgrSetConfig(BrcmPwrMgrConfig *pConfig);

/** \brief Enable/Disable Power Management operation.
 *
 *  Enables or disables Power Management engine operation. PM operation is
 *  disabled by default. Invoke this function with a parameter value of true to
 *  enable operation of the Power Management engine. Invoke with false to
 *  disable the engine.
 *
 *  \param[in]  enable          true to enable PM operation, false to disable
 */
void BrcmPwrMgrEnable(bool enable);

/** \brief Acquire the power lock.
 *
 * This function is used to prevent the power management to go to any power saving state.
 */
void BrcmAcquirePowerLock(void);

/** \brief Release the power lock.
 *
 * This function releases the previous power lock to allow the power management to resume
 * the power saving operation.
 */
void BrcmReleasePowerLock(void);

/** \brief Enable/Disable the sleep mode.
*
*  Enable and disable Power Management's sleep mode. This function allows the
*  application to quickly turn on or off the sleep mode at run time when it is
*  needed. Note: This function does a fast modification of just the bit
*  "sleepOff" in BrcmPwrMgrConfig. The caller can also use BrcmPwrMgrSetConfig
*  to change "sleepOff" bit instead, but the overhead is higher for having to
*  call BrcmPwrMgrGetConfig to retrieve the current configuration and then do a
*  BrcmPwrMgrSetConfig to set this new value.
*
*  \param[in]  sleepEnable          true to turn on sleep mode, false to turn off sleep mode.
*  \return The previous value of the sleep mode.
*/
bool BrcmPwrMgrEnableSleep(bool sleepEnable);

/** \brief Power up/down a RAM section.
 *
 *  This function allows the user to power down a section of RAM when the section
 *  is not in use to save power. The caller can also use the same function to
 *  power up a section of the power-down RAM when it is needed.
 *
 *  \param[in]  ramStartAddr    the first address of the RAM section to power down/up.
 *  \param[in]  ramEndAddr      the last address of the RAM section to power down/up.
 *  \param[in]  powerDown       true to power down, false to power up.
 *
 *  \return     0 for success, -1 for failure.
 */
int32_t BrcmPowerDownRam(uint32_t ramStartAddr, uint32_t ramEndAddr, bool powerDown);



/*****************************************************************************
 * APIs for sending and receiving data to/from the Host Sensor SW
 */
/** \brief  Sensor Packet from Host callback prototype
 *
 *  This is a protoype to define the callback that occurs when the host sends
 *  a packet of information to the Sensor software.
 *  Note: This callback is executed on a different task, so it should return
 *  quickly, and stack usage should be very limited.
 */
typedef void (*BrcmSensorOnHostPacketCallback)(void *pUserData, uint8_t *pData, size_t len);


/** \brief  Sensor Receive Registration
 *
 *  Register callback to receive packets from the Host. When Host Sensor SW sends
 *  a packet, this callback will be called.
 *
 *  \param[in]  callback        callback to call when data is received from the host.
 *  \param[in]  pUserData       pointer passed to the callback for identification.
 *                              Set this to NULL if it is not used.
 */
void BrcmSensorTransportRegister(BrcmSensorOnHostPacketCallback callback, void *pUserData);

/** \brief  Sensor Send data to the Host
 *
 *  Sends a data packet to the Host Sensor SW.
 *
 *  \param[in]  pData           buffer of data to send to the host
 *  \param[in]  len             number of bytes to send (up to BRCM_SENSOR_TRANSPORT_MAX_BYTES)
 */
#define BRCM_SENSOR_TRANSPORT_MAX_BYTES     1000
void BrcmSensorTransportSend(uint8_t *pData, size_t len);


/**
 ** MCU Request/Response GPIO interface
 **/

/** \brief  Initialization for MCU Request and Request Response
 *
 *  Set the GPIO Pins to be used to perform MCU Request and MCU
 *  Request Response.  These are customizable based on available
 *  GPIOs for a given customer board/platform.
 *
 *  \param[in]  mcuReq          MCU Request GPIO Pin
 *  \param[in]  mcuReqResponse  MCU Request Response GPIO Pin
 */
void BrcmPlatformInit(uint32_t mcuReq, uint32_t mcuReqResponse);

/** \brief  Indicates an uninitialized or invalid GPIO pin number.
 */
#define BRCM_GPIO_INVALID_PIN   (0xFFFFFFFF)

/** \brief  Get MCU Request GPIO Pin
 *
 *  Get the GPIO Pin associated with MCU Request.
 *
 *  \return     MCU Request GPIO Pin if initialized; otherwise,
 *              BRCM_GPIO_INVALID_PIN
 */
uint32_t BrcmGpioGetMcuReq(void);

/** \brief  Get MCU Request Response GPIO Pin
 *
 *  Get the GPIO Pin associated with MCU Request Response.
 *
 *  \return     MCU Request Response GPIO Pin if initialized; otherwise,
 *              BRCM_GPIO_INVALID_PIN
 */
uint32_t BrcmGpioGetMcuReqResponse(void);

/*
 * BrcmGpioSetMcuReq
 * BrcmGpioSetMcuReqResponse
 *
 * These functions are OBSOLETE.  Please use BrcmPlatformInit.
 */



/******************************************************************************
 * UART
 */
/** \brief  Broadcom BSP UART Device ID.
 *
 *  Broadcom BSP UART Device ID.
 */
typedef enum BrcmUartDeviceId
{
    BRCM_DEVICE_UART0                   = 0,    ///< UART0 device ID.
    BRCM_DEVICE_UART1,                          ///< UART1 device ID.
    BRCM_DEVICE_UART2                           ///< UART2 device ID.
} BrcmUartDeviceId;

/** \brief  Broadcom BSP UART Data Bits.
 *
 *  Broadcom BSP Host UART Data Bits.
 */
typedef enum BrcmUartDataBits
{
    BRCM_UART_DATA_5_BITS               = 0,    ///< 5 Data Bits
    BRCM_UART_DATA_6_BITS,                      ///< 6 Data Bits
    BRCM_UART_DATA_7_BITS,                      ///< 7 Data Bits
    BRCM_UART_DATA_8_BITS                       ///< 8 Data Bits
} BrcmUartDataBits;

/** \brief  Broadcom BSP UART Parity Mode.
 *
 *  Broadcom BSP Host UART Parity Mode.
 */
typedef enum BrcmUartParityMode
{
    BRCM_UART_PARITY_NONE               = 0,    ///< No Parity
    BRCM_UART_PARITY_ODD,                       ///< Odd Parity
    BRCM_UART_PARITY_EVEN,                      ///< Even Parity
    BRCM_UART_PARITY_MARK,                      ///< Mark Parity
    BRCM_UART_PARITY_SPACE                      ///< Space Parity
} BrcmUartParityMode;

/** \brief  Broadcom BSP UART Stop Bits.
 *
 *  Broadcom BSP Host UART Stop Bits.
 */
typedef enum BrcmUartStopBits
{
    BRCM_UART_STOP_BITS_1               = 0,    ///< 1 Stop Bit
    BRCM_UART_STOP_BITS_2                       ///< 2 Stop Bits
} BrcmUartStopBits;

/** \brief  Broadcom BSP UART HW Flow Control.
 *
 *  Broadcom BSP UART HW Flow Control.
 */
typedef enum BrcmUartHwFlowControl
{
    BRCM_UART_HW_FLOW_CONTROL_DISABLED  = 0,    ///< Hw Flow Control Disabled
    BRCM_UART_HW_FLOW_CONTROL_ENABLED           ///< Hw Flow Control Enabled
} BrcmUartHwFlowControl;

/** \brief  Broadcom BSP UART In-Band Flow Control.
 *
 *  Broadcom BSP UART In-Band Flow Control.
 */
typedef enum BrcmUartInBandFlowControl
{
    BRCM_UART_IN_BAND_FLOW_CONTROL_DISABLED  = 0,    ///< In-Band Flow Control Disabled
    BRCM_UART_IN_BAND_FLOW_CONTROL_ENABLED           ///< In-Band Flow Control Enabled
} BrcmUartInBandFlowControl;

/** \brief UART Tx Threshold prototype.
 *
 *  Broadcom BSP UART TX Threshold callback prototype.  This function is called
 *  when the transmit FIFO goes below a given threshold.  The threshold is different
 *  depending on if PMA is in use or not.  pUserData will be the address passed in
 *  the Open Configuration structure.  The use of this buffer is up to the caller
 *  for identification or any need that they have in this condition.
 */
typedef void (*BrcmOnUartTxThreshold)(void *pUserData);

/** \brief  UART Open Configuration.
 *
 *  Broadcom BSP Host UART Open Configuration.  This structure is passed to the
 *  open call for this device in order to define the base functionality for this
 *  UART.
 */
typedef struct BrcmUartOpenCfg
{
    uint32_t                baudRate;           ///< Baud Rate
    BrcmUartDataBits        dataBits;           ///< Data Bit Length
    BrcmUartParityMode      parityMode;         ///< Parity Mode
    BrcmUartStopBits        stopBits;           ///< Stop Bit Length
    BrcmUartHwFlowControl   hwFlowControl;      ///< HW Flow Control Enabled

    bool                    usePma;             ///< Use Peripheral DMA
    uint32_t                rxFifoSize;         ///< Receive FIFO Size
    uint8_t*                pRxFifoPtr;         ///< Receive FIFO Address
    uint32_t                txFifoSize;         ///< Transmit FIFO Size
    uint8_t*                pTxFifoPtr;         ///< Transmit FIFO Address

    void*                   pUserData;          ///< pUserData Parameter to be passed to pTxThresholdFunc
    BrcmOnUartTxThreshold   pTxThresholdFunc;   ///< TX Threshold function is called when data in the transmit FIFO gets low

    BrcmUartInBandFlowControl inBandFlowControl;///< In-Band Flow Control Enabled
} BrcmUartOpenCfg;

/** \brief  UART Device Open
 *
 *  Open a UART Device for exclusive use.  Allow for different configuration options.
 *
 *  \param[in]  deviceId        UART Device to open
 *  \param[in]  pOpenConfig     Open Configuration values
 *
 *  \return     BrcmDeviceStatus value
 */
BrcmDeviceStatus BrcmUartOpen(      BrcmUartDeviceId    deviceId,
                                    BrcmUartOpenCfg*    pOpenConfig);

/** \brief  UART Device Close
 *
 *  Close an open UART Device
 *
 *  \param[in]  deviceId        UART Device to open
 *
 *  \return     BrcmDeviceStatus value
 */
BrcmDeviceStatus BrcmUartClose(     BrcmUartDeviceId    deviceId);


/** \brief  Broadcom BSP UART IOCTL Commands.
 *
 *  Broadcom BSP Host UART IOCTL Commands.
 */
typedef enum BrcmUartIoctlCmd
{
    BRCM_UART_IOCTL_SET_SPEED,                  ///< Command for setting the UART bus's speed.
    BRCM_UART_IOCTL_GET_SPEED,                  ///< Command for reading the UART current bus's speed.
    BRCM_UART_IOCTL_FLUSH_RX_BUF,               ///< Flush UART rx buffer.
    BRCM_UART_IOCTL_ENABLE_HW_FLOW_CTRL,        ///< Enable HW Flow Control.
    BRCM_UART_IOCTL_DISABLE_HW_FLOW_CTRL,       ///< Disable HW Flow Control.
    BRCM_UART_IOCTL_ENABLE_SW_FLOW_CTRL,        ///< Enable SW Flow Control.
    BRCM_UART_IOCTL_DISABLE_SW_FLOW_CTRL        ///< Disable SW Flow Control.
} BrcmUartIoctlCmd;

/** \brief  UART Device IOCTL
 *
 *  Send an IO Control command to a UART Device.
 *
 *  \param[in]  deviceId        UART Device to send the IOCTL command to
 *  \param[in]  ioctlCmd        IOCTL Command
 *  \param      pParam          Parameter value, interpreted based on the requested IOCTL Command
 *
 *  \return     BrcmDeviceStatus value
 */
BrcmDeviceStatus BrcmUartIoctl(     BrcmUartDeviceId    deviceId,
                                    BrcmUartIoctlCmd    ioctlCmd,
                                    void*               pParam);

/** \brief  UART Device Read
 *
 *  Read data from an open UART Device.
 *
 *  \param[in]  deviceId        UART Device to read from
 *  \param[out] pBuffer         Buffer to store data read from the UART
 *  \param[in]  length          number of bytes to read
 *  \param[in]  timeoutInMs     number of milliseconds to read data or return with some or none of the requested data
 *
 *  \return     BrcmDeviceStatus value, or byte read count if > 0
 */
int32_t BrcmUartRead(               BrcmUartDeviceId    deviceId,
                                    uint8_t*            pBuffer,
                                    int32_t             length,
                                    uint32_t            timeoutInMs);

/** \brief  UART Device Write
 *
 *  Write data to an open UART Device.
 *
 *  \param[in]  deviceId        UART Device to write to
 *  \param[in]  pBuffer         Buffer to send write data from with the UART as the destination
 *  \param[in]  length          number of bytes to write
 *
 *  \return     BrcmDeviceStatus value, or byte write count if > 0
 */
int32_t BrcmUartWrite(              BrcmUartDeviceId    deviceId,
                                    uint8_t*            pBuffer,
                                    int32_t             length);


/******************************************************************************
 * SSI
 */
/** \brief  Broadcom BSP SSI Device ID.
 *
 *  Broadcom BSP SSI Device ID.  SSI device supports two slave host interfaces: SPI and I2C.
 */
typedef enum BrcmSsiDeviceId
{
    BRCM_DEVICE_SSI_SPI                  = 1,    ///< SSI_SPI device ID.
    BRCM_DEVICE_SSI_I2C                          ///< SSI_I2C device ID.
} BrcmSsiDeviceId;

/** \brief SSI Tx Threshold prototype.
 *
 *  Broadcom BSP SSI TX Threshold callback prototype.  This function is called
 *  when the transmit FIFO goes below a given threshold.
 *  pUserData will be the address passed in the Open Configuration structure.
 *  The use of this buffer is up to the caller for identification or any need that they have in this condition.
 */
typedef void (*BrcmOnSsiTxThreshold)(void *pUserData);

/** \brief  SSI Open Configuration.
 *
 *  Broadcom BSP Host SSI Open Configuration.  This structure is passed to the
 *  open call for this device in order to define the base functionality for this
 *  SSI.
 */
typedef struct BrcmSsiOpenConfig
{
    uint32_t                rxFifoSize;         ///< Receive FIFO Size
    uint8_t*                pRxFifoPtr;         ///< Receive FIFO Address
    uint32_t                txFifoSize;         ///< Transmit FIFO Size
    uint8_t*                pTxFifoPtr;         ///< Transmit FIFO Address

    uint32_t                devAddrId;          ///< I2C Address for BRCM_DEVICE_SSI_I2C

    uint32_t                txThresholdValue;   ///< TX FIFO Threshold value
    void*                   pUserData;          ///< pUserData Parameter to be passed to pTxThresholdFunc
    BrcmOnSsiTxThreshold    pTxThresholdFunc;   ///< TX Threshold function is called when data in the transmit FIFO gets below the txThreshold value

} BrcmSsiOpenConfig;

/** \brief  SSI Device Open
 *
 *  Open a SSI Device for exclusive use.  Allow for different configuration options.
 *
 *  \param[in]  deviceId        SSI Device to open
 *  \param[in]  pOpenConfig     Open Configuration values
 *
 *  \return     BrcmDeviceStatus value
 */
BrcmDeviceStatus BrcmSsiOpen(       BrcmSsiDeviceId     deviceId,
                                    BrcmSsiOpenConfig*  pOpenConfig);

/** \brief  SSI Device Close
 *
 *  Close an open SSI Device
 *
 *  \param[in]  deviceId        SSI Device to open
 *
 *  \return     BrcmDeviceStatus value
 */
BrcmDeviceStatus BrcmSsiClose(      BrcmSsiDeviceId     deviceId);


/** \brief  SSI Device Read
 *
 *  Read data from an open SSI Device.
 *
 *  \param[in]  deviceId        SSI Device to read from
 *  \param[out] pBuffer         Buffer to store data read from the SSI
 *  \param[in]  length          number of bytes to read
 *  \param[in]  timeoutInMs     number of milliseconds to read data or return with some or none of the requested data
 *
 *  \return     BrcmDeviceStatus value, or byte read count if > 0
 */
int32_t BrcmSsiRead(                BrcmSsiDeviceId     deviceId,
                                    uint8_t*            pBuffer,
                                    int32_t             length,
                                    uint32_t            timeoutInMs);

/** \brief  SSI Device Write
 *
 *  Write data to an open SSI Device.
 *
 *  \param[in]  deviceId        SSI Device to write to
 *  \param[in]  pBuffer         Buffer to send write data from with the SSI as the destination
 *  \param[in]  length          number of bytes to write
 *
 *  \return     BrcmDeviceStatus value, or byte write count if > 0
 */
int32_t BrcmSsiWrite(               BrcmSsiDeviceId     deviceId,
                                    uint8_t*            pBuffer,
                                    int32_t             length);

#ifdef __cplusplus
}
#endif

#endif  /* BRCM_HAL_H */
