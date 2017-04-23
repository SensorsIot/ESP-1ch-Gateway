//
// Author: Maarten Westenberg
// Version: 1.0.0
// Date: 2016-03-25
//
// This file contains a number of compile-time settings that can be set on (=1) or off (=0)
// The disadvantage of compile time is minor compared to the memory gain of not having
// too much code compiled and loaded on your ESP8266.
//
// See http://pubsubclient.knolleary.net/api.html for API description of MQTT
//
//
// 2016-06-12 - Charles-Henri Hallard (http://hallard.me and http://github.com/hallard)
//              added support for WeMos Lora Gateway
//              added AP mode (for OTA)
//              added Over The Air (OTA) feature
//              added support for onboard WS2812 RGB Led
//              refactored include source file
//              see https://github.com/hallard/WeMos-Lora
//
// ----------------------------------------------------------------------------------------

#include <Arduino.h>

#define VERSION " ! V. 1.1.3, 110616"

// Enable WeMos Lora Shield Gateway
// see https://github.com/hallard/WeMos-Lora
// Uncomment this line if you're using this shield as gateway
#define WEMOS_LORA_GW

/*******************************************************************************

   Configure these values if necessary!

 *******************************************************************************/

// WiFi definitions
// Setup your Wifi SSID and password
// If your device already connected to your Wifi, then
// let as is (stars), it will connect using
// your previous saved SDK credentials

#ifdef CREDENTIALS   // if sensorsiot credentials.h file is present
#define _SSID    mySSID
#define _PASS    myPASSWORD
#define _EMAIL   myEmail
#else
#define _SSID     "..."
#define _PASS     "..."
#define _EMAIL    "..."
#endif

// Access point Password
#define _AP_PASS  "1Ch@n3l-Gateway"

// MQTT definitions
#define _TTNSERVER "router.eu.thethings.network"
#define _MQTTSERVER "your.server.com"

// TTN related
#define SERVER1 _TTNSERVER    // The Things Network: croft.thethings.girovito.nl "54.72.145.119"
#define PORT1 1700            // The port on which to send data

//#define SERVER2 _MQTTSERVER // 2nd server to send to, e.g. private server
//#define PORT2 "1700"

// Gateway Ident definitions
#define _DESCRIPTION "ESP Gateway"

#define _PLATFORM "ESP8266"
#define _LAT 47.474683
#define _LON 7.767343
#define _ALT 350

// SX1276 - ESP8266 connections
#ifdef WEMOS_LORA_GW
#define DEFAULT_PIN_SS    16          // GPIO16, D0
#define DEFAULT_PIN_DIO0  15          // GPIO15, D8
#define DEFAULT_PIN_RST   NOT_A_PIN   // Unused
#else
#define DEFAULT_PIN_SS    15          // GPIO15, D8
#define DEFAULT_PIN_DIO0  5           // GPIO5,  D1
#define DEFAULT_PIN_RST   NOT_A_PIN   // Unused
#endif

#define STATISTICS 1    // Gather statistics on sensor and Wifi status
#define DEBUG 1         // Initial value of debug var. Can be changed using the admin webserver
// For operational use, set initial DEBUG vaulue 0

// Definitions for the admin webserver
#define A_SERVER   1      // Define local WebServer only if this define is set
#define SERVERPORT 8080   // local webserver port

#define A_MAXBUFSIZE 192  // Must be larger than 128, but small enough to work
#define _BAUDRATE 115200  // Works for debug messages to serial momitor (if attached).

// ntp
#define NTP_TIMESERVER "ch.pool.ntp.org"  // Country and region specific
#define NTP_INTERVAL  3600  // How often doe we want time NTP synchronization
#define NTP_TIMEZONES 2     // How far is our Timezone from UTC (excl daylight saving/summer time)

// ============================================================================
// Set all definitions for Gateway
// ============================================================================

#define REG_FIFO                    0x00
#define REG_FIFO_ADDR_PTR           0x0D
#define REG_FIFO_TX_BASE_AD         0x0E
#define REG_FIFO_RX_BASE_AD         0x0F
#define REG_RX_NB_BYTES             0x13
#define REG_OPMODE                  0x01
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS               0x12
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41
#define REG_MODEM_CONFIG            0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_MODEM_CONFIG3           0x26
#define REG_SYMB_TIMEOUT_LSB        0x1F
#define REG_PKT_SNR_VALUE           0x19
#define REG_PAYLOAD_LENGTH          0x22
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_MAX_PAYLOAD_LENGTH      0x23
#define REG_HOP_PERIOD              0x24
#define REG_SYNC_WORD               0x39
#define REG_VERSION                 0x42

#define SX72_MODE_RX_CONTINUOS      0x85
#define SX72_MODE_TX                0x83
#define SX72_MODE_SLEEP             0x80
#define SX72_MODE_STANDBY           0x81

#define PAYLOAD_LENGTH              0x40

// LOW NOISE AMPLIFIER
#define REG_LNA                     0x0C
#define LNA_MAX_GAIN                0x23
#define LNA_OFF_GAIN                0x00
#define LNA_LOW_GAIN                0x20

// CONF REG
#define REG1                        0x0A
#define REG2                        0x84

#define SX72_MC2_FSK                0x00
#define SX72_MC2_SF7                0x70
#define SX72_MC2_SF8                0x80
#define SX72_MC2_SF9                0x90
#define SX72_MC2_SF10               0xA0
#define SX72_MC2_SF11               0xB0
#define SX72_MC2_SF12               0xC0

#define SX72_MC1_LOW_DATA_RATE_OPTIMIZE  0x01   // mandated for SF11 and SF12

// FRF
#define REG_FRF_MSB       0x06
#define REG_FRF_MID       0x07
#define REG_FRF_LSB       0x08


#define FRF_MSB           0xD9    // 868.1 Mhz
#define FRF_MID           0x06
#define FRF_LSB           0x66
#define BUFLEN 2048               //Max length of buffer

#define PROTOCOL_VERSION  1
#define PKT_PUSH_DATA     0
#define PKT_PUSH_ACK      1
#define PKT_PULL_DATA     2
#define PKT_PULL_RESP     3
#define PKT_PULL_ACK      4

#define TX_BUFF_SIZE  2048
#define STATUS_SIZE   512           // This should(!) be enough based on the static text part.. was 1024

