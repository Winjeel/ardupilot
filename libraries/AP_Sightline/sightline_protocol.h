/*
 * sightline_protocol.h
 *
 *  Created on: 11/07/2017
 *      Author: nexton
 */

#pragma once

#include <AP_Common/AP_Common.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SIGHTLINE_PROTOCOL_H_
#define SIGHTLINE_PROTOCOL_H_


//Discover

//static const char *SL_DiscoverRequest  = "SLDISCOVER";

#define SL_MAGIC_NUMBER (0x51acd00d)

enum {
    SL_MAGIC_1 = 0x51,
    SL_MAGIC_2 = 0xAC,
    SL_MAGIC_DISCOVERY_1 = 0xD0,
    SL_MAGIC_DISCOVERY_2 = 0x0D,
};

#define SL_DISCOVER_VERS_MINOR 4  // Change this if adding to the end of the struct
#define SL_DISCOVER_VERS_MAJOR 0  // Changing this breaks firmware upgrade

// TODO: Use Sightline version of this...
typedef enum {
    SLA_2000_OEM = 0,
    SLA_2100_OEM = 1,
    SLA_1000_OEM = 4,
    SLA_PC_WIN = 5,
    SLA_PC_LINUX = 6,
    SLA_1500_OEM = 7,
    SLA_1501_OEM = 8,
    SLA_UPGRADE_SRV = 10,
    SLA_3000_OEM = 12,

    _dummy = 0x7FFF, // force size
} PACKED SL_BoardType;
static_assert(sizeof(SL_BoardType) == 2, "");

// NOTE:  Don't change this structure without changing the discover version
typedef struct {
    uint8_t magic1;
    uint8_t magic2;
    uint16_t magicDiscovery1;          //!< Magic identifier number = 0xd00d
    uint16_t magicDiscovery2;
    uint32_t length;            //!< Discover message length
    uint16_t versMinor;      //!< Discover protocol minor version.
    uint16_t versMajor;      //!< Discover protocol major version.
    uint16_t services;       //!< Services provided
    SL_BoardType boardType;      //!< Board type, one of SL_BOARD_TYPE (since v0.2).
    char mac[20];            //!< MAC address of sender
    char ipaddr[16];         //!< IP address of sender
    char netmask[16];        //!< netmask assocated with ipaddr
    char name[32];           //!< Human Readable name of sender
    uint16_t videoPort;      //!< Port number where images are sent
    uint16_t comsPort;       //!< Input command port number (new in 2.17, assume 14001 if not present)
} PACKED SL_DiscoverInfo;


// Commands

typedef enum {
    // TODO: Use Sightline version of this...
    SL_MsgId_None = 0x00,
    SL_MsgId_GetVersionNumber = 0x00,
    SL_MsgId_SetMetadataValues = 0x13,
    SL_MsgId_GetParameters = 0x28,
    SL_MsgId_VersionNumber = 0x40,
    SL_MsgId_DoSnapshot = 0x60,
} PACKED SL_MsgId;
static_assert(sizeof(SL_MsgId) == 1, "");

typedef struct {
    uint8_t magic1;
    uint8_t magic2;
    uint8_t length;
    SL_MsgId id;
} PACKED SL_MsgHeader;
static_assert(sizeof(SL_MsgHeader) == 4, "");


typedef struct {
    SL_MsgId id;
} PACKED SL_Cmd_GetParams;


typedef struct {
    uint16_t updateMask;
    uint64_t utcTime_us;

    uint16_t platformHeading;
    int16_t platformPitch;
    int16_t platformRoll;

    int32_t sensorLatitude;
    int32_t sensorLongitude;
    uint16_t sensorAltitude;

    uint16_t sensorHorizontalFieldOfView;
    uint16_t sensorVerticalFieldOfView;

    uint32_t sensorRelativeAzimuth;
    int32_t sensorRelativeElevation;
    uint32_t sensorRelativeRoll;
} PACKED SL_Cmd_SetMetadataValues;
static_assert(sizeof(SL_Cmd_SetMetadataValues) == 42, "");


typedef struct {
    uint8_t day;
    uint8_t month;
    uint16_t year;
    uint8_t second;
    uint8_t minute;
    uint16_t hour;
} PACKED SL_DateTime;

typedef struct {
    uint8_t versionMajor;
    uint8_t versionMinor;
    uint8_t _reserved;
    uint8_t temperature;
    uint8_t hwUID[3];
    uint32_t applicationBits;
    uint8_t hwType;
    uint8_t swRevision;
    uint16_t fpgaVersion;
    uint32_t svnRevision;
    SL_DateTime buildDate;
    uint16_t swBuildNumber;
} PACKED SL_Cmd_VersionNumber;
static_assert(sizeof(SL_Cmd_VersionNumber) == 29, "");


typedef struct {
    uint8_t frameStep;
    uint8_t numSnapshots;
    uint8_t filenameLen; // max 64
    char baseFilename[5];
    uint8_t maskSnapAllCameras;
} PACKED SL_Cmd_DoSnapshot;
static_assert(sizeof(SL_Cmd_DoSnapshot) == 9, "");


#endif /* SIGHTLINE_PROTOCOL_H_ */

#ifdef __cplusplus
}
#endif
