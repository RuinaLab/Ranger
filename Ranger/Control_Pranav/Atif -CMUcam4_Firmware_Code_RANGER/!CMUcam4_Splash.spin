{{
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CMUcam4 Splash
//
// Author: Kwabena W. Agyeman
// Updated: 6/30/2012
// Designed For: P8X32A
// Version: 1.0
//
// Copyright (c) 2012 Kwabena W. Agyeman
// See end of file for terms of use.
//
// Update History:
//
// v1.0 - Original release - 1/3/2012.
// v1.1 - Fixed commenting - 5/9/2012.
// v1.2 - Enhanced code - 6/30/2012.
//
// This program converts a 160 by 120 monochrome bitmap to a binary splash screen file for the CMUcam4. The program expects
// that the image has already been flipped vertically as BMP files are stored flipped vertically. Use a black background with
// white lines for text and graphics. If an error occurs the program will print that error to the serial terminal and halt.
//
// Nyamekye,
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}}

CON

' //////////////////////Environmental Settings/////////////////////////////////////////////////////////////////////////////////

  _CLKMODE = XTAL1 + PLL16X ' The clock frequency is equal to 96MHz = 6MHz * 16X PLL.
  _XINFREQ = 6_000_000 '

' //////////////////////Pin Numbers////////////////////////////////////////////////////////////////////////////////////////////

  _SD_CD = 15 ' Secure digital card pins.
  _SD_DO = 16 '
  _SD_CLK = 17 '
  _SD_DI = 18 '
  _SD_CS = 19 '
  _SD_WP = -1 '

  _SERIAL_TX_PIN = 30 ' Asynchronous serial bus pins.
  _SERIAL_RX_PIN = 31 '

' //////////////////////Device Constants///////////////////////////////////////////////////////////////////////////////////////

  _BITMAP_X_SIZE = 160 ' Bitmap size constraints. (PRODUCT MUST BE DIVISBLE BY 8 AND 32).
  _BITMAP_Y_SIZE = 120 '

  _BITMAP_SIZE_BYTES = (((_BITMAP_X_SIZE * _BITMAP_Y_SIZE) + 7) / 8) ' Size in bytes and longs.
  _BITMAP_SIZE_LONGS = (((_BITMAP_X_SIZE * _BITMAP_Y_SIZE) + 31) / 32) '

' //////////////////////Operational Values/////////////////////////////////////////////////////////////////////////////////////

  _BAUDRATE = 19_200 ' Serial communication values.
  _NEWLINE = 13 '

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

DAT

' //////////////////////System Strings/////////////////////////////////////////////////////////////////////////////////////////

inputFileName byte "!CMUCAM4.BMP", 0 ' 8.3 Input file name.
outputFileName byte "!CMUCAM4.BIT", 0 ' 8.3 Ouput file name.

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

OBJ

  fat: "SD-MMC_FATEngine.spin" ' File system driver.
  com: "Full-Duplex_COMEngine.spin" ' Serial port driver.

PUB main ' Run on startup.

  fat.FATEngineStart(_SD_DO, _SD_CLK, _SD_DI, _SD_CS, _SD_WP, _SD_CD, -1, -1, -1)
  com.COMEngineStart(_SERIAL_RX_PIN, _SERIAL_TX_PIN, _BAUDRATE) ' Start library drivers.
  waitcnt((clkfreq * 3) + cnt) ' Wait for 3 seconds.
  com.writeString(string("Loading...", _NEWLINE))

  if(result := \script) ' Abort trap the script program and print any error string.
    com.writeString(result)
    com.writeByte(_NEWLINE)

  com.writeString(string("Finished...", _NEWLINE)) ' Shutdown.
  com.COMEngineStop
  fat.FATEngineStop

PRI script | pixelArrayOffset, infoHeaderSize, bitMapBuffer[_BITMAP_SIZE_LONGS]

  fat.mountPartition(0) ' Open input file.
  fat.openFile(@inputFileName, "R")

  if(fat.readShort <> constant("B" + ("M" << 8)))
    abort string("File type invalid")

  if(fat.readLong <> fat.fileSize)
    abort string("File size corrupt")

  fat.fileSeek(fat.fileTell + 4) ' Skip ahead 4 bytes.
  pixelArrayOffset := fat.readLong ' Grab pixel array offset.
  infoHeaderSize := fat.readLong  ' Grab info header size.

  if(infoHeaderSize == 12) ' Handle old BMP file type.

    if(fat.readShort <> _BITMAP_X_SIZE)
      abort string("Unexpected bitmap X size")

    if(fat.readShort <> _BITMAP_Y_SIZE)
      abort string("Unexpected bitmap Y size")

  else

    if((||fat.readlong) <> _BITMAP_X_SIZE)
      abort string("Unexpected bitmap X size")

    if((||fat.readlong) <> _BITMAP_Y_SIZE)
      abort string("Unexpected bitmap Y size")

  if(fat.readShort <> 1)
    abort string("Expected color planes to equal 1")

  if(fat.readShort <> 1)
    abort string("Expected bits per pixel to equal 1")

  fat.fileSeek(pixelArrayOffset) ' Skip ahead. (Past color palette).

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  com.writeString(string("Converting file...", _NEWLINE))
  fat.readData(@bitMapBuffer, _BITMAP_SIZE_BYTES)
  fat.closeFile ' Done with input file.

  result := \fat.deleteEntry(@outputFileName) ' Remake output file.
  fat.openFile(fat.newFile(@outputFileName), "W")

  fat.writeData(@bitMapBuffer, _BITMAP_SIZE_BYTES)
  fat.unmountPartition ' Done with output file.
  abort string("Success!") ' Print string.

{{

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  TERMS OF USE: MIT License
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
// files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
// modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}}