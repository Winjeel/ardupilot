<style>
    body {
        text-align:justify;
        max-width: 25cm;
        margin-left: auto;
        margin-right: auto;
        font-family: Georgia;
        counter-reset: h1counter h2counter  h3counter toc1counter toc2counter toc3counter;
     }

    table {
       border: 1px solid #e0e0e0;
       border-collapse: collapse;
       margin-bottom: 25px;
    }

    th, td {
        border: 1px solid #e0e0e0;
        font-family: Courier, monospace;
        font-size: 90%;
        padding: 2px;
    }

    /*
     * Alternate colors for the table, including the heading row
     */
    th {
    background-color: #e0e0e0   
    }
    tr:nth-child(even){background-color: #e0e0e0}

    h1, h2, h3, h4, h5 { font-family: Arial; }
    h1 { font-size:120%; margin-bottom: 25px; }
    h2 { font-size:110%; margin-bottom: 15px; }
    h3 { font-size:100%; margin-bottom: 10px;}
    h4, li { font-size:100%; }

    caption{ font-family:Arial; font-size:85%;}

    code, pre, .codelike {
        font-family: Courier, monospace;
        font-size: 100%;
        color: darkblue;
    }

    /*
     * Counters for the main headings
     */

    h1:before {
        counter-increment: h1counter;
        content: counter(h1counter) "\00a0 ";
    }
    h1 {
        counter-reset: h2counter;
    }
    
    h2:before {
        counter-increment: h2counter;
        content: counter(h1counter) "." counter(h2counter) "\00a0 ";
    }
    h2 {
        counter-reset: h3counter;
    }
    
    h3:before {
      counter-increment: h3counter;
      content: counter(h1counter) "." counter(h2counter) "." counter(h3counter) "\00a0 ";
    }

    /*
     * The document title, centered
     */
    doctitle {font-family: Arial; font-size:120%; font-weight: bold; margin-bottom:25px; text-align:center; display:block;}
    titlepagetext {text-align:center; display:block;}

    /*
     * The table of contents formatting
     */
    toctitle {font-family: Arial; font-size:120%; font-weight: bold; margin-bottom:25px; display:block;}
    toc1, toc2, toc3 {font-family: Arial; font-size:100%; margin-bottom:2px; display:block;}
    toc1 {text-indent: 0px;}
    toc2 {text-indent: 15px;}
    toc3 {text-indent: 30px;}
    
    toc1:before {
        content: counter(toc1counter) "\00a0 ";
        counter-increment: toc1counter;
    }
    toc1 {
        counter-reset: toc2counter;
    }
    
    toc2:before {
        content: counter(toc1counter) "." counter(toc2counter) "\00a0 ";
        counter-increment: toc2counter;
    }
    toc2 {
        counter-reset: toc3counter;
    }

    toc3:before {
      content: counter(toc1counter) "." counter(toc2counter) "." counter(toc3counter) "\00a0 ";
      counter-increment: toc3counter;
    }

    /* How it looks on a screen, notice the fancy hr blocks and lack of page breaks */
    @media screen {
      body {
        background-color: #f0f0f0;
      }
      .page-break { display: none; }
      hr { 
        height: 25px; 
        border-style: solid; 
        border-color: gray; 
        border-width: 1px 0 0 0; 
        border-radius: 10px; 
      } 
      hr:before { 
        display: block; 
        content: ""; 
        height: 25px; 
        margin-top: -26px; 
        border-style: solid; 
        border-color: gray; 
        border-width: 0 0 1px 0; 
        border-radius: 10px; 
      }
    }

    /* How it looks when printed, hr turned off, in favor of page breaks*/
    @media print {
      hr {display: none;}
      body {background-color: white;}
      .page-break{page-break-before: always;}
    }
</style>



# External definitions for PPDS Motor Pod firmware

This is the external interface for the PPDS Motor Pod firmware, generated with
ProtoGen.

External definitions for PPDS Motor Pod firmware Protocol version is 0.1.3.

## Packet Identifier

The list of packet identifiers.

| Name                                     | Value | Description                                   |
| ---------------------------------------- | :---: | --------------------------------------------- |
| [Software Version](#SOFTWARE_VERSION)    | 0     | The software version of this device.          |
| [Hardware Version](#HARDWARE_VERSION)    | 1     | The hardware version of this device.          |
| [Interface Version](#INTERFACE_VERSION)  | 2     | The version of this device interface.         |
| [Diagnostic Message](#DIAGNOSTIC_MSG)    | 3     | A diagnostic message.                         |
| [Optical Flow Data](#OPTICAL_FLOW_STATE) | 4     | The data measured by the Optical Flow sensor. |
| [ADC Data](#ADC_STATE)                   | 5     | The data measured by the ADC.                 |
[<a name="PacketId"></a>Packet Identifier enumeration]



## Build Type

The list of build types.

| Name              | Value | Description                                                              |
| ----------------- | :---: | ------------------------------------------------------------------------ |
| Development Build | 0     | An experimental build. Not to be flown.                                  |
| Development Build | 1     | An integration build ready for flight ops. Not to be used in production. |
| Development Build | 2     | A verified build suitable for use in production equipment.               |
[<a name="BuildType"></a>Build Type enumeration]



## Diagnostic Level

The diagnostic level of the message.

| Name                    | Value | Description                                                 |
| ----------------------- | :---: | ----------------------------------------------------------- |
| Fatal diagnostic.       | 0     | A message that indicates the system as a whole has failed.  |
| Error diagnostic.       | 1     | A message that indicates an error with part of the system.  |
| Warning diagnostic.     | 2     | A message that indicates a problem with part of the system. |
| Information diagnostic. | 3     | A message that provides information.                        |
| Debug diagnostic.       | 4     | A message used for debugging.                               |
[<a name="DiagnosticLevel"></a>Diagnostic Level enumeration]



## Error Code

The error codes that can be sent in response to a message.

| Name             | Value | Description                           |
| ---------------- | :---: | ------------------------------------- |
| No Error.        | 0     | Success.                              |
| No change.       | 1     | Requested message had no effect.      |
| Not implemented. | 2     | Requested message is not implemented. |
[<a name="ErrorCode"></a>Error Code enumeration]



## <a name="SOFTWARE_VERSION"></a>SoftwareVersion packet

Software version information.

- packet identifier: `SOFTWARE_VERSION` : 0
- minimum data length: 13
- maximum data length: 24


| Bytes   | Name                       | [Enc](#Enc)                           | Repeat | Description |
| ------- | -------------------------- | :-----------------------------------: | :----: | ----------- |
| 0...11  | 1)id                       | Zero-terminated string up to 12 bytes         ||             |
| 12      | 2)major                    | U8                                    | 1      |             |
| 13      | 3)minor                    | U8                                    | 1      |             |
| 14      | 4)patch                    | U8                                    | 1      |             |
| 15      | 5)[build_type](#BuildType) | U8                                    | 1      |             |
| 16...19 | 6)build_time               | U32                                   | 1      |             |
| 20...23 | 7)git_hash                 | U32                                   | 1      |             |
[SoftwareVersion packet bytes]


## <a name="HARDWARE_VERSION"></a>HardwareVersion packet

Hardware revision information.

- packet identifier: `HARDWARE_VERSION` : 1
- minimum data length: 3
- maximum data length: 14


| Bytes  | Name    | [Enc](#Enc)                           | Repeat | Description |
| ------ | ------- | :-----------------------------------: | :----: | ----------- |
| 0...11 | 1)id    | Zero-terminated string up to 12 bytes         ||             |
| 12     | 2)major | U8                                    | 1      |             |
| 13     | 3)minor | U8                                    | 1      |             |
[HardwareVersion packet bytes]


## <a name="INTERFACE_VERSION"></a>InterfaceVersion packet

Interface version information.

- packet identifier: `INTERFACE_VERSION` : 2
- minimum data length: 4
- maximum data length: 15


| Bytes  | Name    | [Enc](#Enc)                           | Repeat | Description |
| ------ | ------- | :-----------------------------------: | :----: | ----------- |
| 0...11 | 1)id    | Zero-terminated string up to 12 bytes         ||             |
| 12     | 2)major | U8                                    | 1      |             |
| 13     | 3)minor | U8                                    | 1      |             |
| 14     | 4)patch | U8                                    | 1      |             |
[InterfaceVersion packet bytes]


## <a name="DIAGNOSTIC_MSG"></a>DiagnosticMessage packet

Diagnostic message.

- packet identifier: `DIAGNOSTIC_MSG` : 3
- minimum data length: 2
- maximum data length: 65


| Bytes  | Name                        | [Enc](#Enc)                           | Repeat | Description |
| ------ | --------------------------- | :-----------------------------------: | :----: | ----------- |
| 0      | 1)[level](#DiagnosticLevel) | U8                                    | 1      |             |
| 1...64 | 2)str                       | Zero-terminated string up to 64 bytes         ||             |
[DiagnosticMessage packet bytes]


## <a name="OPTICAL_FLOW_STATE"></a>OpticalFlowState packet

Optical Flow state.

- packet identifier: `OPTICAL_FLOW_STATE` : 4
- data length: 11


| Bytes  | Name             | [Enc](#Enc) | Repeat | Description              |
| ------ | ---------------- | :---------: | :----: | ------------------------ |
| 0      | 1)sequence       | U8          | 1      |                          |
| 1...4  | 2)timeDelta_us   | U32         | 1      | <br>Units: microseconds. |
| 5      | 3)isMoving       | U8          | 1      |                          |
| 6      | 4)surfaceQuality | U8          | 1      | surface quality.         |
| 7...10 | 5)flowDelta                           |||                          |
| 7...8  | 5.1)x            | I16         | 1      |                          |
| 9...10 | 5.2)y            | I16         | 1      |                          |
[OpticalFlowState packet bytes]


## <a name="ADC_STATE"></a>AdcState packet

ADC state.

- packet identifier: `ADC_STATE` : 5
- data length: 17


| Bytes   | Name           | [Enc](#Enc) | Repeat | Description              |
| ------- | -------------- | :---------: | :----: | ------------------------ |
| 0       | 1)sequence     | U8          | 1      |                          |
| 1...4   | 2)timeDelta_us | U32         | 1      | <br>Units: microseconds. |
| 5...8   | 3)current      | F32         | 1      | <br>Units: amps.         |
| 9...12  | 4)voltage      | F32         | 1      | <br>Units: volts.        |
| 13...16 | 5)temperature  | F32         | 1      | <br>Units: degrees.      |
[AdcState packet bytes]

<div class="page-break"></div>


----------------------------

# About this ICD

This is the interface control document for data *on the wire*, not data in memory. This document was automatically generated by the [ProtoGen software](https://github.com/billvaglienti/ProtoGen), version 2.17.b. ProtoGen also generates C source code for doing most of the work of encoding data from memory to the wire, and vice versa.

## Encodings

Data for this protocol are sent in BIG endian format. Any field larger than one byte is sent with the most signficant byte first, and the least significant byte last.

Data can be encoded as unsigned integers, signed integers (two's complement), bitfields, and floating point.

| <a name="Enc"></a>Encoding | Interpretation                        | Notes                                                                       |
| :--------------------------: | ------------------------------------- | --------------------------------------------------------------------------- |
| UX                           | Unsigned integer X bits long          | X must be: 8, 16, 24, 32, 40, 48, 56, or 64                                 |
| IX                           | Signed integer X bits long            | X must be: 8, 16, 24, 32, 40, 48, 56, or 64                                 |
| BX                           | Unsigned integer bitfield X bits long | X must be greater than 0 and less than 32                                   |
| F16:X                        | 16 bit float with X significand bits  | 1 sign bit : 15-X exponent bits : X significant bits with implied leading 1 |
| F24:X                        | 24 bit float with X significand bits  | 1 sign bit : 23-X exponent bits : X significant bits with implied leading 1 |
| F32                          | 32 bit float (IEEE-754)               | 1 sign bit : 8 exponent bits : 23 significant bits with implied leading 1   |
| F64                          | 64 bit float (IEEE-754)               | 1 sign bit : 11 exponent bits : 52 significant bits with implied leading 1  |

## Size of fields
The encoding tables give the bytes for each field as X...Y; where X is the first byte (counting from 0) and Y is the last byte. For example a 4 byte field at the beginning of a packet will give 0...3. If the field is 1 byte long then only the starting byte is given. Bitfields are more complex, they are displayed as Byte:Bit. A 3-bit bitfield at the beginning of a packet will give 0:7...0:5, indicating that the bitfield uses bits 7, 6, and 5 of byte 0. Note that the most signficant bit of a byte is 7, and the least signficant bit is 0. If the bitfield is 1 bit long then only the starting bit is given.

The byte count in the encoding tables are based on the maximum length of the field(s). If a field is variable length then the actual byte location of the data may be different depending on the value of the variable field. If the field is a variable length character string this will be indicated in the encoding column of the table. If the field is a variable length array this will be indicated in the repeat column of the encoding table. If the field depends on the non-zero value of another field this will be indicated in the description column of the table.

