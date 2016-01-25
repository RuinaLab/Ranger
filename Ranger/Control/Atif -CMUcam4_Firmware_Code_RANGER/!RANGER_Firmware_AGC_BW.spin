{{
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  12/25/12 - Atif I. Chaudhry - Cornell Ranger Firmware
//  To use the cmucam4 board on Ranger we are starting with the cmucam4 default firmware and editing it to suit our needs
//  The primary edits are being made to the demo program
//
// CMUcam4 Firmware
//
// Author: Kwabena W. Agyeman
// Updated: 6/30/2012
// Designed For: P8X32A
// Version: 1.02
//
// Copyright (c) 2012 Kwabena W. Agyeman
// See end of file for terms of use.
//
// Update History:
//
// v1.00 - Original release - 2/9/2012.
// v1.01 - Reduced the frame rate to 8 FPS from 10 FPS. While some CMUcam4 boards can grab perfect images at 10 FPS many
//         CMUcam4 boards cannot. 8 FPS was choosen because it produces non-irrational numbers in all our timing calculations.
//         This issue is due to jitter and skew in the PCLK signal from the OV9665 module. This is why some CMUcam4 boards can
//         handle 10 FPS and some cannot handle 10 FPS. Additionally, the servo refresh rate was reduced from 50 Hz to 40 Hz.
//         This was done to keep the ratio between the servo update speed and the frame rate the same. Finally, the mapping
//         translation between RGB and YUV mode has been changed to RGB->VYU from RGB->UYV - 3/23/2012.
//
//         Below are the constant settings needed for different frame rates:
//
//         For 10 FPS: _CAMERA_CLKFREQ = 24_000_000 & _FRAME_RATE = 10 & _CAMERA_CLOCK_VALUE = $82 & _SC_UPDATE_FREQUENCY = 50
//         For 10 FPS: _CAMERA_CLKFREQ = 16_000_000 & _FRAME_RATE = 10 & _CAMERA_CLOCK_VALUE = $81 & _SC_UPDATE_FREQUENCY = 50
//
//         For  8 FPS: _CAMERA_CLKFREQ = 19_200_000 & _FRAME_RATE =  8 & _CAMERA_CLOCK_VALUE = $82 & _SC_UPDATE_FREQUENCY = 40
//         For  8 FPS: _CAMERA_CLKFREQ = 12_800_000 & _FRAME_RATE =  8 & _CAMERA_CLOCK_VALUE = $81 & _SC_UPDATE_FREQUENCY = 40
//
//         NOTE: Please read section 4.9 in the propeller data sheet about cog counters. Change _PLL_MUL4_LONG and
//               _CAMERA_PLL_DIV_4 for values of _CAMERA_CLKFREQ not between 16 MHz to 32 MHz.
//
// v1.02 - The frame rate was increased to 30 FPS by making the entire system synchronous. The propeller chip sends a 48 MHz
//         XCLK signal to the camera. The camera sends back a 24 MHz PCLK signal to the propeller chip. Each and every cog on
//         the propeller chip executes general purpose assembly instructions at 24 MHz. The 30 FPS frame rate was achieved by
//         aligning the propeller chip and the camera clock synchronously. Additionally, the servo refresh rate was increased
//         back to 50 Hz and the default servo proportional and derivative values were changed. Also, different parts of the
//         source code were optimized to make space. The UV LUT was changed and a lot bug fixes occured - 6/30/2012.
//
// Please use the CMUcam4 Splash program to make different splash screens. Splash screens must be named "!CMUCAM4.BIT".
//
// Nyamekye,
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}}

CON

' //////////////////////Environmental Settings/////////////////////////////////////////////////////////////////////////////////

  _CLKMODE = XTAL1 + PLL16X ' The clock frequency is equal to 96MHz = 6MHz * 16X PLL.
  _XINFREQ = 6_000_000 '

  _STACK = 640 ' Reserve some space for the stack and free.
  _FREE = 2 '

' //////////////////////Pin Numbers////////////////////////////////////////////////////////////////////////////////////////////

  _CAM_D2_PIN = 0 ' Camera 8 bit data bus pins. (PIN NUMBERS HERE FOR REFERENCE ONLY - CODE MUST BE CHANGED TO MOVE).
  _CAM_D3_PIN = 1 '
  _CAM_D4_PIN = 2 '
  _CAM_D5_PIN = 3 '
  _CAM_D6_PIN = 4 '
  _CAM_D7_PIN = 5 '
  _CAM_D8_PIN = 6 '
  _CAM_D9_PIN = 7 '

  _CAM_D0_PIN = 8 ' Camera 10 bit data bus pins.
  _CAM_D1_PIN = 9 '

  _CAM_XCLK_PIN = 10 ' Camera signaling pins.
  _CAM_PCLK_PIN = 11 '
  _CAM_VSYNC_PIN = 12 '
  _CAM_HREF_PIN = 13 '

  _CAM_PWDN_PIN = 14 ' Camera state control pins.
  _CAM_RESET_PIN = -1 '

  _SD_CD = 15 ' Secure digital card pins.
  _SD_DO = 16 '
  _SD_CLK = 17 '
  _SD_DI = 18 '
  _SD_CS = 19 '
  _SD_WP = -1 '
            
  _AUX_PUSH_BUTTON_PIN = 20 ' System input and output.
  _AUX_POWER_LED_PIN = 21 '

  _GPIO_TILT = 22 ' Servo control and GPIO pins.
  _GPIO_PAN = 23 '

  _TV_LOW = 24 ' Television control pins. (PIN NUMBERS HERE FOR REFERENCE ONLY - CODE MUST BE CHANGED TO MOVE).
  _TV_MEDIUM = 25 '
  _TV_HIGH = 26 '
  _TV_COLOR = 27 '

  _I2C_CLOCK_PIN = 28 ' I2C bus pins.
  _I2C_DATA_PIN = 29 '

  _SERIAL_TX_PIN = 30 ' Asynchronous serial bus pins.
  _SERIAL_RX_PIN = 31 '

' //////////////////////Device Constants///////////////////////////////////////////////////////////////////////////////////////

  _DEFAULT_MOUNT_PARTITION = 0 ' Partition numbers.
  _DEFAULT_FORMAT_PARTITION = _DEFAULT_MOUNT_PARTITION '

  _EEPROM_WRITE_ADDRESS = $A0 ' These are the I2C addresses to communicate with the EEPROM.
  _EEPROM_READ_ADDRESS = (_EEPROM_WRITE_ADDRESS | $1) '

  _SCCB_CAMERA_WRITE_ADDRESS = $60 ' These are the I2C addresses to communicate with the camera.
  _SCCB_CAMERA_READ_ADDRESS = (_SCCB_CAMERA_WRITE_ADDRESS | $1) '

  _CAMERA_PLL_RESET_ADDRESS = $3E ' Camera pll reset information.
  _CAMERA_PLL_RESET_VALUE = $D0 '

  _CAMERA_SOFTWARE_RESET_ADDRESS = $12 ' Camera software reset information.
  _CAMERA_SOFTWARE_RESET_VALUE = $80 '

  _CLKSEL_MASK = (%111 << 0) ' Propeller chip clock register masks.
  _OSCM_MASK = (%11 << 3) '
  _OSCENA_MASK = (%1 << 5) '
  _PLLENA_MASK = (%1 << 6) '
  _RESET_MASK = (%1 << 7) '

  _PLL_OSC_OSCM_MASK = (_PLLENA_MASK | _OSCENA_MASK | _OSCM_MASK) ' Combined mask for clkset setup.  '= %111000

  _NCO_SE_LONG = (%00100 << 26) ' Numerically controlled oscillator and positive edge detector mode.
  _POSEDGE_D_MOVI = %0_01010_000 '

  _PROPELLER_PORT_A = 0 ' Propeller chip port references.
  _PROPELLER_PORT_B = 1 '

  _HALTED_CLKSET_MODE = 2 ' Propeller chip clkset halt.
  _HALTED_CLKSET_FREQ = 0 '

  _RC_SLOW_MODE = 1 ' Propeller chip RC modes.
  _RC_FAST_MODE = 0 '

  _RC_SLOW_FREQUENCY = 20_000 ' Propeller chip RC frequencies.
  _RC_FAST_FREQUENCY = 12_000_000 '

  _PLL_STABILIZE_TIME = 10_000 ' Resolves into (clkfreq / timeout) seconds of timeout.
  _OSC_STABILIZE_TIME = 100 '

  _EEPROM_WRITE_TIMEOUT = 200 ' Resolves into (clkfreq / timeout) seconds of timeout.
  _CMAERA_PLL_TIMEOUT = 200 '
  _CAMERA_XCLK_TIMEOUT = 10 '
  _CAMERA_SCCB_TIMEOUT = 1_000 '

  _FRAME_RATE = 30 ' Number of frames per second.
  _FRAME_WAIT = 1 '

  _FRAME_TIMEOUT = (_FRAME_RATE / 2) ' Timeout and data propagation speed.
  _FRAME_UPDATED = (_FRAME_WAIT * 2) '

  _CAMERA_H_RES = 640 ' Camera horizontal and vertical resolutions.
  _CAMERA_V_RES = 480 '

  _CAM_H_DIV = 4 ' Camera horizontal and vertical dividers.
  _CAM_V_DIV = 4 '

  _CAMERA_H_WIN = (_CAMERA_H_RES / _CAM_H_DIV) ' Camera horizontal and vertical resolution machine windows.   '_CAMERA_H_WIN = 160
  _CAMERA_V_WIN = (_CAMERA_V_RES / _CAM_V_DIV) '                                                              '_CAMERA_V_WIN = 120

  _CAMERA_MIN_H = 0 ' Camera min and max machine window limits.
  _CAMERA_MAX_H = (_CAMERA_H_WIN - 1) '                '_CAMERA_H_WIN = 159
  _CAMERA_MIN_V = 0 '                                  
  _CAMERA_MAX_V = (_CAMERA_V_WIN - 1) '                '_CAMERA_V_WIN = 119

  _MINIMUM_H_SCALE = 0 ' Horizontal (2 ^ scale) divisor for frame dumps.
  _MAXIMUM_H_SCALE = 3 '

  _MINIMUM_V_SCALE = 0 ' Vertical (2 ^ scale) divisor for frame dumps.
  _MAXIMUM_V_SCALE = 3 '

  _R_V_5 = 5 ' Base RGB565/YUV655 camera color mode.
  _G_Y_6 = 6 '
  _B_U_5 = 5 '

  _R_V_5_DECODED = (|<_R_V_5) ' Decoded RGB565/YUV655 camera color mode.       ' = %00000000 00000000 00000000 00010000
  _G_Y_6_DECODED = (|<_G_Y_6) '                                                ' = %00000000 00000000 00000000 00100000
  _B_U_5_DECODED = (|<_B_U_5) '                                                ' = %00000000 00000000 00000000 00010000

  _R_V_5_MASK = (_R_V_5_DECODED - 1) ' Mask RGB565/YUV655 camera color mode.   ' = %00000000 00000000 00000000 00001111 = 15
  _G_Y_6_MASK = (_G_Y_6_DECODED - 1) '                                         ' = %00000000 00000000 00000000 00011111 = 31
  _B_U_5_MASK = (_B_U_5_DECODED - 1) '                                         ' = %00000000 00000000 00000000 00001111 = 15

  _R_V_5_MIN = 0 ' RGB565/YUV655 mins.
  _G_Y_6_MIN = 0 '
  _B_U_5_MIN = 0 '

  _R_V_5_MAX = _R_V_5_MASK ' RGB565/YUV655 maxes.          ' = %00000000 00000000 00000000 00001111 = 15
  _G_Y_6_MAX = _G_Y_6_MASK '                               ' = %00000000 00000000 00000000 00011111 = 31
  _B_U_5_MAX = _B_U_5_MASK '                               ' = %00000000 00000000 00000000 00001111 = 15

  _R_V_5_COMBINED_SHIFT = ((_R_V_5 * 0) + (_G_Y_6 * 1) + (_B_U_5 * 1)) ' RGB565/YUV655 bit shifts.
  _G_Y_6_COMBINED_SHIFT = ((_R_V_5 * 0) + (_G_Y_6 * 0) + (_B_U_5 * 1)) '
  _B_U_5_COMBINED_SHIFT = ((_R_V_5 * 0) + (_G_Y_6 * 0) + (_B_U_5 * 0)) '

  _R_V_5_COMBINED_MASKS = (_R_V_5_MASK << _R_V_5_COMBINED_SHIFT) ' RGB565/YUV655 bit masks.
  _G_Y_6_COMBINED_MASKS = (_G_Y_6_MASK << _G_Y_6_COMBINED_SHIFT) '
  _B_U_5_COMBINED_MASKS = (_B_U_5_MASK << _B_U_5_COMBINED_SHIFT) '

  _R_V_8 = 8 ' Base RGB888/YUV888 camera color mode.
  _G_Y_8 = 8 '
  _B_U_8 = 8 '

  _R_V_8_DECODED = (|<_R_V_8) ' Decoded RGB888/YUV888 camera color mode.
  _G_Y_8_DECODED = (|<_G_Y_8) '
  _B_U_8_DECODED = (|<_B_U_8) '

  _R_V_8_MASK = (_R_V_8_DECODED - 1) ' Mask RGB888/YUV888 camera color mode.
  _G_Y_8_MASK = (_G_Y_8_DECODED - 1) '
  _B_U_8_MASK = (_B_U_8_DECODED - 1) '

  _R_V_8_MIN = 0 ' RGB888/YUV888 mins.
  _G_Y_8_MIN = 0 '
  _B_U_8_MIN = 0 '

  _R_V_8_MAX = _R_V_8_MASK ' RGB888/YUV888 maxes.
  _G_Y_8_MAX = _G_Y_8_MASK '
  _B_U_8_MAX = _B_U_8_MASK '

  _R_V_8_ROUND = (_R_V_8_MAX - 1) ' RGB888/YUV888 roundings.
  _G_Y_8_ROUND = (_G_Y_8_MAX - 1) '
  _B_U_8_ROUND = (_B_U_8_MAX - 1) '

  #0, _R_V_5_CHANNEL, _G_Y_6_CHANNEL, _B_U_5_CHANNEL ' For RGB/YUV histogram channels.

  _R_V_5_MIN_BIN = 0 ' For RGB/YUV histogram min bins.
  _G_Y_6_MIN_BIN = 0 '
  _B_U_5_MIN_BIN = 0 '

  _R_V_5_MAX_BIN = _R_V_5 ' For RGB/YUV histogram max bins.
  _G_Y_6_MAX_BIN = _G_Y_6 '
  _B_U_5_MAX_BIN = _B_U_5 '

  _NEWLINE = com#Carriage_Return ' Serial communication character values.
  _TAB = com#Horizontal_Tab '
  _BACKSPACE = com#Backspace '
  _NULL = com#Null '

  _PAN_SERVO_NUM = 0 ' Pan and tilt bindings.
  _TILT_SERVO_NUM = 1 '

  _PAN_CTRL_MAXIMUM = 1_000 ' Pan control maximum and minimum values.
  _PAN_CTRL_MINIMUM = 0 '

  _TILT_CTRL_MAXIMUM = 1_000 ' Tilt control maximum and minimum values.
  _TILT_CTRL_MINIMUM = 0 '

' //////////////////////Operational Values/////////////////////////////////////////////////////////////////////////////////////

  _SERIAL_BYTES = 256 ' Serial communication buffer byte size. Includes null terminating character space.
  _SERIAL_LONGS = ((_SERIAL_BYTES + 3) / 4) '

  _PAN_DEFAULT_PULSE = 1_500 ' Default servo pulse length values.   NOTE this is in microseconds....-AIC
  _TILT_DEFAULT_PULSE = 1_500 '

' ///Original Code/Values -AIC
'  _SERVO_MIN = (1_000 - 250) ' Default servo pulse length limits.
'  _SERVO_MAX = (2_000 + 250) '

' ///These are adjusted as the SD-6G transmitter sends signals between 1100 and 1900 -AIC
  _SERVO_MIN = (1_100) ' Default servo pulse length limits.
  _SERVO_MAX = (1_900) '

  _DEFAULT_BAUDRATE = 19_200 ' Default serial communication settings.
  _DEFAULT_EXTRA_STOP_BITS = 0 '

  _MAX_USER_LED_RATE = 10_000_000 ' Default indicator blink rate limits.
  _MIN_USER_LED_RATE = -1 '

  _BUSY_LED_RATE = 10 ' Indicator blink rates.
  _IDLE_LED_RATE = 0 '

' ///Original Code/Values -AIC
'  _DEFAULT_PAN_PCTRL = (400 - 25) ' PD control values for panning in thousandths.
'  _DEFAULT_PAN_DCTRL = (100 + 25) '

' ///Original Code/Values -AIC
'  _DEFAULT_TILT_PCTRL = (400 - 25) ' PD control values for tilting in thousandths.
'  _DEFAULT_TILT_DCTRL = (100 + 25) '

' // Adjusted to mimic response time of the SD-6G, may need to be altered for better
' // deal with noise....  -AIC
  _DEFAULT_PAN_PCTRL = (800) ' PD control values for panning in thousandths.
  _DEFAULT_PAN_DCTRL = (100) ' Adjust this value if target is near center

' // Adjusted to mimic response time of the SD-6G, may need to be altered for better
' // deal with noise....  -AIC
  _DEFAULT_TILT_PCTRL = (800) ' PD control values for tilting in thousandths.
  _DEFAULT_TILT_DCTRL = (100) ' Adjust this value if target is near center

  _HALT_MODE_DECISION_TIME = 5 ' In seconds.
  _DEMO_MODE_DECISION_TIME = 5 '

  _DEMO_MODE_TV_SIGNAL = false ' False for NTSC and true for PAL. ALSO CONTROLS THE BOOTUP TV SIGNAL!

  _DEMO_MODE_PAN_REVERSED = false ' True for reversed and false for not reversed.
  _DEMO_MODE_TILT_REVERSED = false  '

'  _DEMO_MODE_COLOR_SPACE = true ' False for RGB and true for YUV.

' // Since using Black and White mode use RGB space for easier calibration -AIC
  _DEMO_MODE_COLOR_SPACE = false ' False for RGB and true for YUV.

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

DAT ' System strings.

  versionStringWithNewLine byte "CMUcam4 v1.02", _NEWLINE, _NULL ' CMUCAM4 version string with a newline.

  aknowledgeStringWithNewLine byte "ACK", _NEWLINE, _NULL ' Yes return.
  notAknowledgeStringWithNewLine byte "NCK", _NEWLINE, _NULL ' No return.

  cameraTimeoutError byte "ERR: Camera Timeout Error", _NEWLINE, _NULL ' Timeout failure.
  cameraConnectionError byte "ERR: Camera Connection Error", _NEWLINE, _NULL ' Connection failure.

  errorString byte "ERR: ", _NULL ' CMUCAM4 error string token.

OBJ ' System objects.

  fat: "SD-MMC_FATEngine.spin" ' File system driver.
  com: "Full-Duplex_COMEngine.spin" ' Serial port driver.

PUB main | byteBuffer, byteCounter, byteArray[_SERIAL_LONGS]

' // Original code -AID
' ifnot(ina[_AUX_PUSH_BUTTON_PIN]) ' Which mode...  _AUX_PUSH_BUTTON_PIN = 20 ' System input and output.
'   powerLEDOn(_IDLE_LED_RATE)      '_IDLE_LED_RATE = 0

'   waitcnt((clkfreq * _HALT_MODE_DECISION_TIME) + cnt)   '_HALT_MODE_DECISION_TIME = 5 ' In seconds.
'   if(ina[_AUX_PUSH_BUTTON_PIN]) ' Decide mode.
'     powerLEDOff

'     clkset(_HALTED_CLKSET_MODE, _HALTED_CLKSET_FREQ)
'   result := true ' Go to demo mode...

' // Edited to make it easier to start DEMO mode (and recalibrate for testing)
' // Simply
  result := true ' Go to demo mode...

  com.COMEngineStart((_SERIAL_RX_PIN | result), (_SERIAL_TX_PIN | result), _DEFAULT_BAUDRATE) ' Start the library drivers.
  fat.FATEngineStart(_SD_DO, _SD_CLK, _SD_DI, _SD_CS, _SD_WP, _SD_CD, -1, -1, -1)

  com.receiverFlush ' Throw away any garbage.
  com.stopBitTiming(_DEFAULT_EXTRA_STOP_BITS)

  com.writeByte(_NEWLINE) ' Send version on boot.
  com.writeString(@versionStringWithNewLine)

  setupNTSCVsPAL(_DEMO_MODE_TV_SIGNAL)

  if(setupCamera)
    lineGrabberDriverStart
  servoControllerStart
  TVDriverStart

  if(result) ' Demo mode...

    powerLEDOn(_BUSY_LED_RATE)

'// Lets NOT wait! -AIC
'    waitcnt((clkfreq * _DEMO_MODE_DECISION_TIME) + cnt)

    panModeSetup(true, _DEMO_MODE_PAN_REVERSED)
    tiltModeSetup(true, _DEMO_MODE_TILT_REVERSED)

    result := \setupCameraColorSpace(_DEMO_MODE_COLOR_SPACE)
    result := \setupAWB(false)

'//  result := \setupAGC(false)
    result := \setupAGC(true)


'// Setup black and white mode
    result := \setupBlackAndWhite(true)

    powerLEDOn(_IDLE_LED_RATE)
    repeat ' Format saves two bytes.
    until(PBState)

    result := \programParser(string("TW 030 030 030"))
    powerLEDOn(_BUSY_LED_RATE) ' An error occured...

    waitpne(0, 0, _PROPELLER_PORT_A) ' Halt...       '_PROPELLER_PORT_A = 0 ' Propeller chip port references.

  repeat ' Loop awaiting a command.
    powerLEDOn(LEDFrequencySetup)

    byteCounter := 0
    longfill(@byteArray, 0, _SERIAL_LONGS)
    com.writeByte(":") ' Clear the serial buffer.

    repeat ' Read in the command.
      byteBuffer := com.readByte

      if(byteBuffer == _TAB)               ' _TAB = com#Horizontal_Tab    Serial communication character values.
        byteBuffer := " " ' Handle tab character.

      if((byteBuffer == _BACKSPACE) and byteCounter)
        byteArray.byte[--byteCounter] := _NULL ' Handle backspace character.

      if(     (" " =< byteBuffer) and (byteBuffer =< "~") {
        } and (byteCounter < constant(_SERIAL_BYTES - 1)) )
        if(("a" =< byteBuffer) and (byteBuffer =< "z"))
          byteBuffer -= constant("a" - "A") ' Translate lower case to upper case.

        byteArray.byte[byteCounter++] := byteBuffer ' Insert the character.
    while(byteBuffer <> _NEWLINE) ' Quit on newline.

    byteBuffer := \programParser(@byteArray) ' Abort trap setup - parse the command token from the string.
    byteCounter := fat.partitionError

    if(byteCounter) ' A file system error...
      com.writeString(@errorString)

    com.writeString(byteBuffer) ' Print any message.

    if(byteCounter) ' Newline for file system errors...
      com.writeByte(_NEWLINE)

CON _AUTONAME_SIZE = 5 ' Five digits.

DAT AUTONAMEArray byte "00000BM.BMP", _NULL ' File name.

PRI mainAutoName | nameBuffer, nameCounter ' Auto names a file in the current directory.

  fat.listEntries("W") ' Goto the top of the directory.
  repeat while(nameBuffer := fat.listEntries("N"))
    result #>= DECIn(nameBuffer) ' Find the largest file name that starts with a number.

  nameBuffer := DECOut((1 + (result~)) // 65_536)
  nameCounter := strsize(nameBuffer)
  bytemove(@AUTONAMEArray, string("00000"), _AUTONAME_SIZE)
  bytemove((@AUTONAMEArray + (_AUTONAME_SIZE - nameCounter)), nameBuffer, nameCounter)

  fat.openFile(fat.newFile(@AUTONAMEArray), "W")

PRI mainAutoMount ' Auto mounts the partition.

  ifnot(fat.partitionMounted)
    fat.mountPartition(_DEFAULT_MOUNT_PARTITION)

PRI extractArgumentsOne(argumentAddress) ' Gets arguments off the command line and checks them.

  ifnot(strsize(long[argumentAddress] := STRToken(_NULL)))
    abort @notAknowledgeStringWithNewLine

PRI extractArgumentsTwo(argument0Address, argument1Address) ' Gets arguments off the command line and checks them.

  extractArgumentsOne(argument0Address)
  extractArgumentsOne(argument1Address)

PRI extractArgumentsThree(arg1A, arg2A, arg3A) ' Gets arguments off the command line and checks them.

  extractArgumentsOne(arg1A)
  extractArgumentsTwo(arg2A, arg3A)

PRI extractArgumentsFive(arg1A, arg2A, arg3A, arg4A, arg5A) ' Gets arguments off the command line and checks them.

  extractArgumentsTwo(arg1A, arg2A)
  extractArgumentsThree(arg3A, arg4A, arg5A)

DAT ' System settings.

  baudRateSetup long _DEFAULT_BAUDRATE ' Default baud rate.
  stopBitsSetup long _DEFAULT_EXTRA_STOP_BITS ' Default extra stop bits count.

  LEDFrequencySetup long _IDLE_LED_RATE ' Default LED frequency setup.
  LEDModeSetup byte true 'Default LED mode. True for enabled and false for disabled.

  pollModeSetup byte false ' Poll data control.
  lineModeSetup byte false ' Line data control.
  switchingModeSetup byte false ' Switching data control.

PRI programParser(stringPointer)

  stringPointer := STRToken(stringPointer)

  programGV(stringPointer)
  programRS(stringPointer)
  programSD(stringPointer)
  programSL(stringPointer)

  programCB(stringPointer)
  programCC(stringPointer)
  programCR(stringPointer)
  programCW(stringPointer)

  programAG(stringPointer)
  programAW(stringPointer)
  programHM(stringPointer)
  programVF(stringPointer)
  programBW(stringPointer)
  programNG(stringPointer)

  programPI(stringPointer)
  programPO(stringPointer)
  programTI(stringPointer)
  programTO(stringPointer)
  programGI(stringPointer)
  programSO(stringPointer)
  programGS(stringPointer)
  programSS(stringPointer)
  programGB(stringPointer)
  programGD(stringPointer)
  programGP(stringPointer)
  programGR(stringPointer)
  programAP(stringPointer)
  programAT(stringPointer)
  programPP(stringPointer)
  programTP(stringPointer)
  programL0(stringPointer)
  programL1(stringPointer)

  programM0(stringPointer)
  programM1(stringPointer)
  programMF(stringPointer)
  programMS(stringPointer)

  programGT(stringPointer)
  programGW(stringPointer)
  programST(stringPointer)
  programSW(stringPointer)

  programBM(stringPointer)
  programDM(stringPointer)

  programPM(stringPointer)
  programLM(stringPointer)
  programSM(stringPointer)
  programTM(stringPointer)
  programCT(stringPointer)
  programHT(stringPointer)
  programIF(stringPointer)
  programNF(stringPointer)
  programTC(stringPointer)
  programTW(stringPointer)
  programGH(stringPointer)
  programGM(stringPointer)

  programCA(stringPointer)
  programCD(stringPointer)
  programDI(stringPointer)
  programDS(stringPointer)
  programFM(stringPointer)
  programLS(stringPointer)
  programMK(stringPointer)
  programMV(stringPointer)
  programPL(stringPointer)
  programPR(stringPointer)
  programRM(stringPointer)
  programUM(stringPointer)

  programDB(stringPointer)
  programDF(stringPointer)
  programSB(stringPointer)
  programSF(stringPointer)

  ifnot(strsize(stringPointer)) ' Empty command string.
    abort @aknowledgeStringWithNewLine

  abort @notAknowledgeStringWithNewLine ' Invalid command string.

PRI programGV(stringPointer) ' Handle get version command.

  if(strcomp(stringPointer, string("GV")))
    com.writeString(@aknowledgeStringWithNewLine)
    abort @versionStringWithNewLine

PRI programRS(stringPointer) | SDError ' Handle reset system command.

  if(strcomp(stringPointer, string("RS")))
    com.writeString(@aknowledgeStringWithNewLine)

    if(fat.partitionMounted) ' Handle unmounting.
      SDError := \fat.unmountPartition
      fat.partitionError

    com.COMEngineStop
    reboot

PRI programSD(stringPointer) | SDError, SCCBError, clkmodeBuffer, clkfreqBuffer ' Handle sleep deeply command.

  if(strcomp(stringPointer, string("SD")))
    com.writeString(@aknowledgeStringWithNewLine)

    if(fat.partitionMounted) ' Handle unmounting.
      SDError := \fat.unmountPartition
      fat.partitionError

    powerLEDOff

    TVDriverStop
    servoControllerStop
    lineGrabberDriverStop
    SCCBError := \SCCBCameraControl(@Camera_Power_Off_Data, _CAMERA_POWER_OFF_NUMBER)

    cameraPowerDown
    cameraClockOff

    fat.FATEngineStop
    com.COMEngineStop

    clkmodeBuffer := clkmode ' Backup clock mode.
    clkfreqBuffer := clkfreq ' Backup clock frequency.

    clkset(_RC_SLOW_MODE, _RC_SLOW_FREQUENCY) ' Switch to very low power mode.
    waitpeq(0, constant(|<_SERIAL_RX_PIN), _PROPELLER_PORT_A) ' Wait for serial low.

    clkset((clkmodeBuffer & _PLL_OSC_OSCM_MASK), _RC_FAST_FREQUENCY) ' Setup PLL and OSC circuits.
    waitcnt((clkfreq / _OSC_STABILIZE_TIME) + cnt) ' Wait for PLL and OSC to stabilize.
    clkset(clkmodeBuffer, clkfreqBuffer) ' Switch clock mode and clock frequency.

    com.COMEngineStart(_SERIAL_RX_PIN, _SERIAL_TX_PIN, baudRateSetup)
    fat.FATEngineStart(_SD_DO, _SD_CLK, _SD_DI, _SD_CS, _SD_WP, _SD_CD, -1, -1, -1)

    com.receiverFlush
    com.stopBitTiming(stopBitsSetup)
    com.writeString(@aknowledgeStringWithNewLine)

    if(setupCamera)
      lineGrabberDriverStart
    servoControllerStart
    TVDriverStart

    abort

PRI programSL(stringPointer) | SCCBError ' Handle sleep lightly command.

  if(strcomp(stringPointer, string("SL")))
    com.writeString(@aknowledgeStringWithNewLine)

    powerLEDOff

    TVDriverStop
    lineGrabberDriverStop
    SCCBError := \SCCBCameraControl(@Camera_Sleep_On_Data, _CAMERA_SLEEP_ON_NUMBER)

    repeat ' Format saves two bytes.
    until(com.readByte == _NEWLINE)
    com.writeString(@aknowledgeStringWithNewLine)

    if(setupCamera)
      lineGrabberDriverStart
    TVDriverStart

    abort

PRI programCB(stringPointer) | commandArgument ' Handle camera brightness command.

  if(strcomp(stringPointer, string("CB")))
    extractArgumentsOne(@commandArgument)

    com.writeString(@aknowledgeStringWithNewLine)
    setupBrightness(DECIn(commandArgument))
    abort

PRI programCC(stringPointer) | commandArgument ' Handle camera contrast command.

  if(strcomp(stringPointer, string("CC")))
    extractArgumentsOne(@commandArgument)

    com.writeString(@aknowledgeStringWithNewLine)
    setupContrast(DECIn(commandArgument))
    abort

PRI programCR(stringPointer) | regAddress ' Handle camera register read command.

  if(strcomp(stringPointer, string("CR")))
    extractArgumentsOne(@regAddress)
    com.writeString(@aknowledgeStringWithNewLine)

    ifnot(SCCBReadRegister(DECIn(regAddress), @regAddress))
      abort @cameraConnectionError

    com.writeString(DECOut(regAddress))
    abort string(_NEWLINE)

PRI programCW(stringPointer) | regAddress, regValue, regMask ' Handle camera register write command.

  if(strcomp(stringPointer, string("CW")))
    extractArgumentsThree(@regAddress, @regValue, @regMask)
    com.writeString(@aknowledgeStringWithNewLine)

    regAddress.byte[_CR_ADDRESS] := DECIn(regAddress)
    regAddress.byte[_CR_VALUE] := DECIn(regValue)
    regAddress.byte[_CR_MASK] := DECIn(regMask)

    SCCBCameraControl(@regAddress, _CW_COMMAND_STRUCT_ELEMENTS)
    abort

PRI programAG(stringPointer) | commandArgument ' Handle auto gain command.

  if(strcomp(stringPointer, string("AG")))
    extractArgumentsOne(@commandArgument)

    com.writeString(@aknowledgeStringWithNewLine)
    setupAGC(DECIn(commandArgument))
    abort

PRI programAW(stringPointer) | commandArgument ' Handle auto white command.

  if(strcomp(stringPointer, string("AW")))
    extractArgumentsOne(@commandArgument)

    com.writeString(@aknowledgeStringWithNewLine)
    setupAWB(DECIn(commandArgument))
    abort

PRI programHM(stringPointer) | commandArgument ' Handle camera horizontal mirror command.

  if(strcomp(stringPointer, string("HM")))
    extractArgumentsOne(@commandArgument)

    com.writeString(@aknowledgeStringWithNewLine)
    setupHorizontalMirror(DECIn(commandArgument))
    abort

PRI programVF(stringPointer) | commandArgument ' Handle camera vertical flip command.

  if(strcomp(stringPointer, string("VF")))
    extractArgumentsOne(@commandArgument)

    com.writeString(@aknowledgeStringWithNewLine)
    setupVerticalFlip(DECIn(commandArgument))
    abort

PRI programBW(stringPointer) | commandArgument ' Handle camera black and white mode command.

  if(strcomp(stringPointer, string("BW")))
    extractArgumentsOne(@commandArgument)

    com.writeString(@aknowledgeStringWithNewLine)
    setupBlackAndWhite(DECIn(commandArgument))
    abort

PRI programNG(stringPointer) | commandArgument ' Handle camera negative mode command.

  if(strcomp(stringPointer, string("NG")))
    extractArgumentsOne(@commandArgument)

    com.writeString(@aknowledgeStringWithNewLine)
    setupNegative(DECIn(commandArgument))
    abort

PRI programPI(stringPointer) ' Handle pan input command. (I0).

  if(strcomp(stringPointer, string("PI")))
    com.writeString(@aknowledgeStringWithNewLine)
    com.writeString(DECOut(ina[_GPIO_PAN]))
    abort string(_NEWLINE)

PRI programPO(stringPointer) | directionSetup, outputSetup ' Handle pan output command. (S0).

  if(strcomp(stringPointer, string("PO")))
    extractArgumentsOne(@directionSetup)
    directionSetup := DECIn(directionSetup)

    if(directionSetup)
      extractArgumentsOne(@outputSetup)
      outa[_GPIO_PAN] := (DECIn(outputSetup) <> false)

    dira[_GPIO_PAN] := (directionSetup <> false)
    abort @aknowledgeStringWithNewLine

PRI programTI(stringPointer) ' Handle tilt input command. (I1).

  if(strcomp(stringPointer, string("TI")))
    com.writeString(@aknowledgeStringWithNewLine)
    com.writeString(DECOut(ina[_GPIO_TILT]))
    abort string(_NEWLINE)

PRI programTO(stringPointer) | directionSetup, outputSetup ' Handle tilt output command. (S1).

  if(strcomp(stringPointer, string("TO")))
    extractArgumentsOne(@directionSetup)
    directionSetup := DECIn(directionSetup)

    if(directionSetup)
      extractArgumentsOne(@outputSetup)
      outa[_GPIO_TILT] := (DECIn(outputSetup) <> false)

    dira[_GPIO_TILT] := (directionSetup <> false)
    abort @aknowledgeStringWithNewLine

PRI programGI(stringPointer) ' Handle get inputs command.

  if(strcomp(stringPointer, string("GI")))
    com.writeString(@aknowledgeStringWithNewLine)

    result := ina
    com.writeString(DECOut((((result>>_GPIO_PAN)&1)<<_PAN_SERVO_NUM)|(((result>>_GPIO_TILT)&1)<<_TILT_SERVO_NUM)))
    abort string(_NEWLINE)

PRI programSO(stringPointer) | directionSetup, outputSetup ' Handle set outputs command.

  if(strcomp(stringPointer, string("SO")))
    extractArgumentsOne(@directionSetup)
    directionSetup := DECIn(directionSetup)

    if(directionSetup)
      extractArgumentsOne(@outputSetup)
      outputSetup := DECIn(outputSetup)

      outa := (   (outa & constant(!((|<_GPIO_PAN) | (|<_GPIO_TILT))))   {
              } | (((outputSetup >> _PAN_SERVO_NUM) & 1) << _GPIO_PAN)   {
              } | (((outputSetup >> _TILT_SERVO_NUM) & 1) << _GPIO_TILT) )

    dira := (   (dira & constant(!((|<_GPIO_PAN) | (|<_GPIO_TILT))))      {
            } | (((directionSetup >> _PAN_SERVO_NUM) & 1) << _GPIO_PAN)   {
            } | (((directionSetup >> _TILT_SERVO_NUM) & 1) << _GPIO_TILT) )

    abort @aknowledgeStringWithNewLine

PRI programGS(stringPointer) | servoNumber ' Handle get servo command.

  if(strcomp(stringPointer, string("GS")))
    extractArgumentsOne(@servoNumber)
    servoNumber := DECIn(servoNumber)

    case servoNumber
      _PAN_SERVO_NUM: servoNumber := getPanPulse
      _TILT_SERVO_NUM: servoNumber := getTiltPulse
      other: abort @notAknowledgeStringWithNewLine

    com.writeString(@aknowledgeStringWithNewLine)
    com.writeString(DECOut(servoNumber))
    abort string(_NEWLINE)

PRI programSS(stringPointer) | servoNumber, servoState, servoPulse ' Handle set servo command.

  if(strcomp(stringPointer, string("SS")))
    extractArgumentsTwo(@servoNumber, @servoState)

    servoNumber := DECIn(servoNumber) ' Grab servo number.
    servoState := DECIn(servoState) ' Grab servo direction.

    if(servoState)

      extractArgumentsOne(@servoPulse)
      servoPulse := DECIn(servoPulse)

      if((servoPulse < _SERVO_MIN) or (_SERVO_MAX < servoPulse))
        abort @notAknowledgeStringWithNewLine

    case servoNumber

      _PAN_SERVO_NUM:
        setupPanDirection(servoState)

        if(servoState) ' Only change if enabling.
          setupPanPulse(servoPulse)

      _TILT_SERVO_NUM:
        setupTiltDirection(servoState)

        if(servoState) ' Only change if enabling.
          setupTiltPulse(servoPulse)

      other: abort @notAknowledgeStringWithNewLine
    abort @aknowledgeStringWithNewLine

PRI programGB(stringPointer) ' Handle get button state command.

  if(strcomp(stringPointer, string("GB")))
    com.writeString(@aknowledgeStringWithNewLine)
    com.writeString(DECOut(||PBState))
    abort string(_NEWLINE)

PRI programGD(stringPointer) ' Handle get duration state command.

  if(strcomp(stringPointer, string("GD")))
    com.writeString(@aknowledgeStringWithNewLine)
    com.writeString(DECOut(PBtime))
    abort string(_NEWLINE)

PRI programGP(stringPointer) ' Handle get press command.

  if(strcomp(stringPointer, string("GP")))
    com.writeString(@aknowledgeStringWithNewLine)
    com.writeString(DECOut(||buttonPressed))
    abort string(_NEWLINE)

PRI programGR(stringPointer) ' Handle get release command.

  if(strcomp(stringPointer, string("GR")))
    com.writeString(@aknowledgeStringWithNewLine)
    com.writeString(DECOut(||buttonReleased))
    abort string(_NEWLINE)

PRI programAP(stringPointer) | autoEnable, autoReversed ' Handle automatic panning command.

  if(strcomp(stringPointer, string("AP")))
    extractArgumentsOne(@autoEnable)
    autoEnable := DECIn(autoEnable)
    autoReversed := false

    if(autoEnable)
      extractArgumentsOne(@autoReversed)
      autoReversed := DECIn(autoReversed)

    panModeSetup(autoEnable, autoReversed)
    abort @aknowledgeStringWithNewLine

PRI programAT(stringPointer) | autoEnable, autoReversed ' Handle automatic tilting command.

  if(strcomp(stringPointer, string("AT")))
    extractArgumentsOne(@autoEnable)
    autoEnable := DECIn(autoEnable)
    autoReversed := false

    if(autoEnable)
      extractArgumentsOne(@autoReversed)
      autoReversed := DECIn(autoReversed)

    tiltModeSetup(autoEnable, autoReversed)
    abort @aknowledgeStringWithNewLine

PRI programPP(stringPointer) | proportional, derivative ' Handle pan parameters command.

  if(strcomp(stringPointer, string("PP")))
    extractArgumentsTwo(@proportional, @derivative)

    proportional := DECIn(proportional)
    derivative := DECIn(derivative)

    if(    (proportional < _PAN_CTRL_MINIMUM) or (_PAN_CTRL_MAXIMUM < proportional) {
      } or (derivative < _PAN_CTRL_MINIMUM) or (_PAN_CTRL_MAXIMUM < derivative)     )
      abort @notAknowledgeStringWithNewLine

    panPControl := proportional
    panDControl := derivative

    abort @aknowledgeStringWithNewLine

PRI programTP(stringPointer) | proportional, derivative ' Handle tilt parameters command.

  if(strcomp(stringPointer, string("TP")))
    extractArgumentsTwo(@proportional, @derivative)

    proportional := DECIn(proportional)
    derivative := DECIn(derivative)

    if(    (proportional < _TILT_CTRL_MINIMUM) or (_TILT_CTRL_MAXIMUM < proportional) {
      } or (derivative < _TILT_CTRL_MINIMUM) or (_TILT_CTRL_MAXIMUM < derivative)     )
      abort @notAknowledgeStringWithNewLine

    tiltPControl := proportional
    tiltDControl := derivative

    abort @aknowledgeStringWithNewLine

PRI programL0(stringPointer) ' Handle led off command.

  if(strcomp(stringPointer, string("L0")))
    powerLEDOff

    LEDModeSetup := false
    abort @aknowledgeStringWithNewLine

PRI programL1(stringPointer) | frequency ' Handle led on command.

  if(strcomp(stringPointer, string("L1")))
    extractArgumentsOne(@frequency)

    frequency := DECIn(frequency)

    if((frequency < _MIN_USER_LED_RATE) or (_MAX_USER_LED_RATE < frequency))
      abort @notAknowledgeStringWithNewLine

    LEDFrequencySetup := frequency

    LEDModeSetup := true
    abort @aknowledgeStringWithNewLine

PRI programM0(stringPointer) ' Handle monitor off command.

  if(strcomp(stringPointer, string("M0")))
    TVDriverStop
    abort @aknowledgeStringWithNewLine

PRI programM1(stringPointer) ' Handle monitor on command.

  if(strcomp(stringPointer, string("M1")))
    TVDriverStart
    abort @aknowledgeStringWithNewLine

PRI programMF(stringPointer) | commandArgument ' Handle monitor freeze command.

  if(strcomp(stringPointer, string("MF")))
    extractArgumentsOne(@commandArgument)
    com.writeString(@aknowledgeStringWithNewLine)
    setupDynamicVsStatic(DECIn(commandArgument))
    abort

PRI programMS(stringPointer) | commandArgument ' Handle monitor signal command.

  if(strcomp(stringPointer, string("MS")))
    extractArgumentsOne(@commandArgument)
    com.writeString(@aknowledgeStringWithNewLine)
    setupNTSCVsPAL(DECIn(commandArgument))
    abort

PRI programGT(stringPointer) ' Handle get tracking parameters command.

  if(strcomp(stringPointer, string("GT")))
    com.writeString(@aknowledgeStringWithNewLine)
    getTrackingThresholds
    abort

PRI programGW(stringPointer) ' Handle get virtual window command.

  if(strcomp(stringPointer, string("GW")))
    com.writeString(@aknowledgeStringWithNewLine)
    getVirtualWindow
    abort

PRI programST(stringPointer) | arg0, arg1, arg2, arg3, arg4, arg5 ' Handle set tracking parameters command.

  if(strcomp(stringPointer, string("ST")))
    arg0 := STRToken(_NULL)

    ifnot(strsize(arg0))
      setTrackingThresholds(_R_V_8_MIN, _R_V_8_MAX, _G_Y_8_MIN, _G_Y_8_MAX, _B_U_8_MIN, _B_U_8_MAX)

    else
      extractArgumentsFive(@arg1, @arg2, @arg3, @arg4, @arg5)
      setTrackingThresholds(DECIn(arg0), DECIn(arg1), DECIn(arg2), DECIn(arg3), DECIn(arg4), DECIn(arg5))

    com.writeString(@aknowledgeStringWithNewLine)

    setupDynamicVsStatic(false)
    abort

PRI programSW(stringPointer) | arg0, arg1, arg2, arg3 ' Handle set virtual window command.

  if(strcomp(stringPointer, string("SW")))
    arg0 := STRToken(_NULL)

    ifnot(strsize(arg0))
      setVirtualWindow(_CAMERA_MIN_H, _CAMERA_MIN_V, _CAMERA_MAX_H, _CAMERA_MAX_V)

    else
      extractArgumentsThree(@arg1, @arg2, @arg3)
      setVirtualWindow(DECIn(arg0), DECIn(arg1), DECIn(arg2), DECIn(arg3))

    com.writeString(@aknowledgeStringWithNewLine)

    setupDynamicVsStatic(false)
    abort

PRI programBM(stringPointer) | commandArgument ' Handle baud mode command.

  if(strcomp(stringPointer, string("BM")))
    extractArgumentsOne(@commandArgument)

    com.writeString(@aknowledgeStringWithNewLine)
    repeat ' Format saves two bytes.
    until(com.transmitterEmpty)

    com.baudRateTiming(baudRateSetup := DECIn(commandArgument))
    repeat ' Format saves two bytes.
    until(com.readByte == _NEWLINE)

    abort @aknowledgeStringWithNewLine

PRI programDM(stringPointer) | commandArgument ' Handle delay mode command.

  if(strcomp(stringPointer, string("DM")))
    extractArgumentsOne(@commandArgument)

    repeat ' Format saves two bytes.
    until(com.transmitterEmpty)
    com.stopBitTiming(stopBitsSetup := DECIn(commandArgument))

    abort @aknowledgeStringWithNewLine

PRI programLM(stringPointer) | commandArgument ' Handle line mode command.

  if(strcomp(stringPointer, string("LM")))
    extractArgumentsOne(@commandArgument)

    lineModeSetup := (DECIn(commandArgument) <> false)
    abort @aknowledgeStringWithNewLine

PRI programPM(stringPointer) | commandArgument ' Handle poll mode command.

  if(strcomp(stringPointer, string("PM")))
    extractArgumentsOne(@commandArgument)

    pollModeSetup := (DECIn(commandArgument) <> false)
    abort @aknowledgeStringWithNewLine

PRI programSM(stringPointer) | commandArgument ' Handle switching mode command.

  if(strcomp(stringPointer, string("SM")))
    extractArgumentsOne(@commandArgument)

    switchingModeSetup := (DECIn(commandArgument) <> false)
    abort @aknowledgeStringWithNewLine

PRI programTM(stringPointer) | commandArgument ' Handle test mode command.

  if(strcomp(stringPointer, string("TM")))
    extractArgumentsOne(@commandArgument)

    com.writeString(@aknowledgeStringWithNewLine)
    setupColorBars(DECIn(commandArgument))
    abort

PRI programCT(stringPointer) | commandArgument ' Handle color tracking command.

  if(strcomp(stringPointer, string("CT")))
    extractArgumentsOne(@commandArgument)

    com.writeString(@aknowledgeStringWithNewLine)
    setupCameraColorSpace(DECIn(commandArgument))
    lineGrabberDriverStart ' Force update.
    abort

PRI programHT(stringPointer) | commandArgument ' Handle histogram tracking command.

  if(strcomp(stringPointer, string("HT")))
    extractArgumentsOne(@commandArgument)

    setupHistogramTracking(DECIn(commandArgument))
    lineGrabberDriverStart ' Force update.
    abort @aknowledgeStringWithNewLine

PRI programIF(stringPointer) | commandArgument ' Handle inverted filter command.

  if(strcomp(stringPointer, string("IF")))
    extractArgumentsOne(@commandArgument)

    setupInvertedFilter(DECIn(commandArgument))
    lineGrabberDriverStart ' Force update.
    abort @aknowledgeStringWithNewLine

PRI programNF(stringPointer) | commandArgument ' Handle noise filter command.

  if(strcomp(stringPointer, string("NF")))
    extractArgumentsOne(@commandArgument)

    setupNoiseFilter(DECIn(commandArgument))
    lineGrabberDriverStart ' Force update.
    abort @aknowledgeStringWithNewLine

PRI programTC(stringPointer) | arg0, arg1, arg2, arg3, arg4, arg5 ' Handle track color command.

  if(strcomp(stringPointer, string("TC")))
    arg0 := STRToken(_NULL)

    if(strsize(arg0))
      extractArgumentsFive(@arg1, @arg2, @arg3, @arg4, @arg5)
      setTrackingThresholds(DECIn(arg0), DECIn(arg1), DECIn(arg2), DECIn(arg3), DECIn(arg4), DECIn(arg5))

    com.writeString(@aknowledgeStringWithNewLine)

    setupDynamicVsStatic(false)
    printNewData(_LN_TRACKING_DATA_SOURCE, 0, 0)
    abort

PRI programTW(stringPointer) | RVT,GYT,BUT, DX,DY,BX,BY, AR, SB[_STRUCT_HS_S_LONGS],HB[_LN_HISTO_DATA_SIZE_LONGS] ' Handle TW.

  if(strcomp(stringPointer, string("TW")))
    extractArgumentsThree(@RVT, @GYT, @BUT)

    RVT := DECIn(RVT)
    GYT := DECIn(GYT)
    BUT := DECIn(BUT)

    if(    (RVT < _R_V_8_MIN) or (_R_V_8_MAX < RVT) {
      } or (GYT < _G_Y_8_MIN) or (_G_Y_8_MAX < GYT) {
      } or (BUT < _B_U_8_MIN) or (_B_U_8_MAX < BUT) )
      abort @notAknowledgeStringWithNewLine

    com.writeString(@aknowledgeStringWithNewLine)

    result := LNWindow ' Backup window settings.
    DX := (||(result.byte[_STRUCT_VW_X2] - result.byte[_STRUCT_VW_X1]))
    DY := (||(result.byte[_STRUCT_VW_Y2] - result.byte[_STRUCT_VW_Y1]))
    BX := ((((++DX) * 3) >> 3) + (result.byte[_STRUCT_VW_X2] <# result.byte[_STRUCT_VW_X1]))
    BY := ((((++DY) * 3) >> 3) + (result.byte[_STRUCT_VW_Y2] <# result.byte[_STRUCT_VW_Y1]))
    setVirtualWindow(BX, BY, (BX + (DX >> 2)), (BY + (DY >> 2))) ' Center sample window.

    if(AR := \waitForNewData(_FRAME_UPDATED))
      LNWindow := result
      abort AR

    getHistogramData(@HB)
    getHistogramStatistics(@SB, @HB) ' Compute histogram statistics.
    setTrackingThresholds( SB.byte[constant(_STRUCT_HS_R_V_BASE + _STRUCT_HS_MEAN)] - RVT, {
                         } SB.byte[constant(_STRUCT_HS_R_V_BASE + _STRUCT_HS_MEAN)] + RVT, {
                         } SB.byte[constant(_STRUCT_HS_G_Y_BASE + _STRUCT_HS_MEAN)] - GYT, {
                         } SB.byte[constant(_STRUCT_HS_G_Y_BASE + _STRUCT_HS_MEAN)] + GYT, {
                         } SB.byte[constant(_STRUCT_HS_B_U_BASE + _STRUCT_HS_MEAN)] - BUT, {
                         } SB.byte[constant(_STRUCT_HS_B_U_BASE + _STRUCT_HS_MEAN)] + BUT  )
    LNWindow := result~ ' Restore the window.

    setupDynamicVsStatic(false)
    printNewData(_LN_TRACKING_DATA_SOURCE, 0, 0)
    abort

PRI programGH(stringPointer) | histogramChannel, histogramBins ' Handle get histogram command.

  if(strcomp(stringPointer, string("GH")))
    extractArgumentsTwo(@histogramChannel, @histogramBins)
    histogramChannel := DECIn(histogramChannel) ' Grab channel number.
    histogramBins := DECIn(histogramBins) ' Grab bin number.

    if( (histogramChannel < _R_V_5_CHANNEL) or {
      } (_B_U_5_CHANNEL < histogramChannel) or {
      } (histogramBins < lookupz(histogramChannel: _R_V_5_MIN_BIN, _G_Y_6_MIN_BIN, _B_U_5_MIN_BIN)) or {
      } (lookupz(histogramChannel: _R_V_5_MAX_BIN, _G_Y_6_MAX_BIN, _B_U_5_MAX_BIN) < histogramBins)    )
      abort @notAknowledgeStringWithNewLine

    com.writeString(@aknowledgeStringWithNewLine)
    printNewData(_LN_HISTOGRAM_DATA_SOURCE, histogramChannel, histogramBins)
    abort

PRI programGM(stringPointer) ' Handle get mean command.

  if(strcomp(stringPointer, string("GM")))
    com.writeString(@aknowledgeStringWithNewLine)
    printNewData(_LN_STATISTICS_DATA_SOURCE, 0, 0)
    abort

PRI programCA(stringPointer) | entryName, entryAttributes ' Handle change attributes command.

  if(strcomp(stringPointer, string("CA")))
    extractArgumentsTwo(@entryName, @entryAttributes)
    com.writeString(@aknowledgeStringWithNewLine)

    mainAutoMount
    fat.changeAttributes(entryName, entryAttributes)
    abort

PRI programCD(stringPointer) | directoryName ' Handle change directory command.

  if(strcomp(stringPointer, string("CD")))
    extractArgumentsOne(@directoryName)
    com.writeString(@aknowledgeStringWithNewLine)

    mainAutoMount
    fat.changeDirectory(directoryName)
    abort

PRI programDI(stringPointer) ' Handle disk information command.

  if(strcomp(stringPointer, string("DI")))
    com.writeString(@aknowledgeStringWithNewLine)
    mainAutoMount

    fat.listEntries("W") ' Reset.
    fat.listEntries("N") ' Ping.

    com.writeString(string(com#Quotation_Marks))
    com.writeString(fat.partitionVolumeLabel)
    com.writeString(string(com#Quotation_Marks, " ", com#Quotation_Marks))
    com.writeString(fat.partitionFileSystemType)
    com.writeString(string(com#Quotation_Marks, " "))

    com.writeString(HEXOut(fat.partitionDiskSignature))
    com.writeByte(" ")
    com.writeString(HEXOut(fat.partitionVolumeIdentification))
    com.writeByte(" ")
    com.writeString(DECOut(fat.partitionDataSectors))
    com.writeByte(" ")
    com.writeString(DECOut(fat.partitionBytesPerSector))
    com.writeByte(" ")
    com.writeString(DECOut(fat.partitionSectorsPerCluster))
    com.writeByte(" ")
    com.writeString(DECOut(fat.partitionCountOfClusters))

    abort string(_NEWLINE)

PRI programDS(stringPointer) ' Handle disk space command.

  if(strcomp(stringPointer, string("DS")))
    com.writeString(string("ACK", _NEWLINE, "MSG: Scanning Partition", _NEWLINE))
    mainAutoMount

    powerLEDOn(_BUSY_LED_RATE) ' Might take a while.
    com.writeString(DECOut(fat.partitionFreeSectorCount("S")))
    com.writeByte(" ")
    com.writeString(DECOut(fat.partitionUsedSectorCount("F")))

    abort string(_NEWLINE)

PRI programFM(stringPointer) ' Handle format disk command.

  if(strcomp(stringPointer, string("FM")))
    com.writeString(string("ACK", _NEWLINE, "MSG: Formatting Partition", _NEWLINE))

    powerLEDOn(_BUSY_LED_RATE) ' Might take a while.
    fat.formatPartition(_DEFAULT_FORMAT_PARTITION)
    abort

PRI programLS(stringPointer) ' Handle list directory command.

  if(strcomp(stringPointer, string("LS")))
    com.writeString(string("ACK", _NEWLINE, "MSG: ---FILENAME----ATTRIB---SIZE----", _NEWLINE))
    mainAutoMount

    fat.listEntries("W")
    repeat while(stringPointer := fat.listEntries("N"))

      com.writeString(string("     ", com#Quotation_Marks))
      com.writeString(stringPointer)
      com.writeString(string(com#Quotation_Marks, " "))

      com.writeByte("_" + (constant("R" - "_") & fat.listIsReadOnly))
      com.writeByte("_" + (constant("H" - "_") & fat.listIsHidden))
      com.writeByte("_" + (constant("S" - "_") & fat.listIsSystem))
      com.writeByte("_")
      com.writeByte("_" + (constant("D" - "_") & fat.listIsDirectory))
      com.writeByte("_" + (constant("A" - "_") & fat.listIsArchive))

      ifnot(fat.listIsDirectory)
        com.writeByte(" ")
        com.writeString(DECOut(fat.listSize))

      com.writeByte(_NEWLINE)
    abort

PRI programMK(stringPointer) | directoryName ' Handle make directory command.

  if(strcomp(stringPointer, string("MK")))
    extractArgumentsOne(@directoryName)
    com.writeString(@aknowledgeStringWithNewLine)

    mainAutoMount
    fat.newDirectory(directoryName)
    abort

PRI programMV(stringPointer) | oldEntryName, newEntryName ' Handle move entry command.

  if(strcomp(stringPointer, string("MV")))
    extractArgumentsTwo(@oldEntryName, @newEntryName)
    com.writeString(@aknowledgeStringWithNewLine)

    mainAutoMount
    fat.moveEntry(oldEntryName, newEntryName)
    abort

PRI programPL(stringPointer) | fileName, logMessage, abortResult ' Handle print line command.

  if(strcomp(stringPointer, string("PL")))
    extractArgumentsTwo(@fileName, @logMessage)
    com.writeString(@aknowledgeStringWithNewLine)
    mainAutoMount

    abortResult := \fat.newFile(fileName) ' Try...
    fat.partitionError ' Clear any errors...

    fat.openFile(fileName, "A")
    fat.writeString(logMessage)
    fat.writeByte(_NEWLINE)
    fat.closeFile
    abort

PRI programPR(stringPointer) | fileName, buffer, pointer ' Handle file print command.

  if(strcomp(stringPointer, string("PR")))
    extractArgumentsOne(@fileName)
    com.writeString(@aknowledgeStringWithNewLine)
    mainAutoMount

    fat.openFile(fileName, "R")
    com.writeString(DECOut(fat.fileSize))
    com.writeByte(_NEWLINE)

    repeat fat.fileSize
      buffer := \fat.readByte

      ifnot(result)
        pointer := buffer

      result or= fat.partitionError
      com.writeByte(buffer & (not(result)))

    if(result)
      com.writeString(@errorString)
      com.writeString(pointer)
      abort string(_NEWLINE)

    fat.closeFile
    abort

PRI programRM(stringPointer) | entryName ' Handle remove entry command.

  if(strcomp(stringPointer, string("RM")))
    extractArgumentsOne(@entryName)
    com.writeString(@aknowledgeStringWithNewLine)

    mainAutoMount
    fat.deleteEntry(entryName)
    abort

PRI programUM(stringPointer) ' Handle unmount disk command.

  if(strcomp(stringPointer, string("UM")))
    com.writeString(@aknowledgeStringWithNewLine)

    fat.unmountPartition
    abort

PRI programDB(stringPointer) | tempPointer ' Handle dump bitmap command.

  if(strcomp(stringPointer, string("DB")))
    com.writeString(@aknowledgeStringWithNewLine)
    mainAutoMount

    waitForNewData(_FRAME_WAIT) ' Ensure data update.

    mainAutoName ' Ready to write data to a file.
    fat.writeData(@BMPHeader80x60xBW_File, _BMP_HEAD_80x60xBW_SIZE)
    ifnot(lineGrabberDriver(_LN_OUTPUT_FAT)) ' Store image to SD card.
      tempPointer := \fat.deleteEntry(@AUTONAMEArray)
      fat.partitionError ' Clear any errors.

    fat.closeFile
    abort

PRI programDF(stringPointer) | tempPointer, downX, downY ' Handle dump frame command.

  if(strcomp(stringPointer, string("DF")))
    extractArgumentsTwo(@downX, @downY)
    downX := (_CAMERA_H_RES / (BMHorizontalDivide := (|<((DECIn(downX) <# _MAXIMUM_H_SCALE) #> _MINIMUM_H_SCALE))))
    downY := (_CAMERA_V_RES / (BMVerticalDivide := (|<((DECIn(downY) <# _MAXIMUM_V_SCALE) #> _MINIMUM_V_SCALE))))

    result := (downY * downX * constant(_BMP_RGB565_BITS_PER_PIXEL / 8))
    bytemove(@BMPHeader480x640xRGB565_File.byte[_BMP_IMAGE_BYTE_SIZE_INDEX], @result, 4)

    bytemove(@BMPHeader480x640xRGB565_File.byte[_BMP_IMAGE_WIDTH_INDEX], @downY, 4)
    bytemove(@BMPHeader480x640xRGB565_File.byte[_BMP_IMAGE_HEIGHT_INDEX], @downX, 4)

    result += _BMP_HEAD_480x640xRGB565_SIZE
    bytemove(@BMPHeader480x640xRGB565_File.byte[_BMP_IMAGE_FILE_SIZE_INDEX], @result, 4)

    com.writeString(@aknowledgeStringWithNewLine)
    mainAutoMount

    powerLEDOn(_BUSY_LED_RATE) ' Might take a while.

    mainAutoName ' Ready to write data to a file.
    fat.writeData(@BMPHeader480x640xRGB565_File, _BMP_HEAD_480x640xRGB565_SIZE)
    ifnot(bitmapGrabberDriver(_BM_OUTPUT_FAT)) ' Store image to SD card.
      tempPointer := \fat.deleteEntry(@AUTONAMEArray)
      fat.partitionError ' Clear any errors.

    fat.closeFile
    abort 0

PRI programSB(stringPointer) ' Handle send bitmap command.

  if(strcomp(stringPointer, string("SB")))
    com.writeString(@aknowledgeStringWithNewLine)
    waitForNewData(_FRAME_WAIT) ' Ensure data update.
    lineGrabberDriver(_LN_OUTPUT_COM) ' Send image.
    abort

PRI programSF(stringPointer) | downX, downY ' Handle send frame command.

  if(strcomp(stringPointer, string("SF")))
    extractArgumentsTwo(@downX, @downY)
    BMHorizontalDivide := (|<((DECIn(downX) <# _MAXIMUM_H_SCALE) #> _MINIMUM_H_SCALE))
    BMVerticalDivide := (|<((DECIn(downY) <# _MAXIMUM_V_SCALE) #> _MINIMUM_V_SCALE))

    com.writeString(@aknowledgeStringWithNewLine)
    powerLEDOn(_BUSY_LED_RATE) ' Might take a while.
    bitmapGrabberDriver(_BM_OUTPUT_COM) ' Send image.
    abort

CON ' Bitmap constant information.

  _BMP_FILE_HEADER_SIZE = 14
  _BMP_INFO_HEADER_SIZE = 40
  _BMP_BIT_MASKS_SIZE = 12
  _BMP_COLOR_TABLE_SIZE = 8

  _BMP_IMAGE_FILE_SIZE_INDEX = 2
  _BMP_IMAGE_WIDTH_INDEX = 18
  _BMP_IMAGE_HEIGHT_INDEX = 22
  _BMP_IMAGE_BYTE_SIZE_INDEX = 34

  _BMP_HEAD_480x640xRGB565_SIZE = (_BMP_FILE_HEADER_SIZE + _BMP_INFO_HEADER_SIZE + _BMP_BIT_MASKS_SIZE)
  _BMP_RGB565_BITS_PER_PIXEL = (_R_V_5 + _G_Y_6 + _B_U_5)

DAT BMPHeader480x640xRGB565_File ' File header.

  byte byte "BM" ' Bitmap signature.
  byte long (_BMP_HEAD_480x640xRGB565_SIZE + ((_CAMERA_V_RES * _CAMERA_H_RES * _BMP_RGB565_BITS_PER_PIXEL) / 8))
  byte word 0 ' Reserved.
  byte word 0 ' Reserved.
  byte long _BMP_HEAD_480x640xRGB565_SIZE ' Offest to pixel array.

  byte long _BMP_INFO_HEADER_SIZE ' DIB header size.
  byte long _CAMERA_V_RES ' Pixel width.
  byte long _CAMERA_H_RES ' Pixel height.
  byte word 1 ' Number of color planes.
  byte word _BMP_RGB565_BITS_PER_PIXEL ' Bits per pixel.
  byte long 3 ' Bit fields.
  byte long ((_CAMERA_V_RES * _CAMERA_H_RES * _BMP_RGB565_BITS_PER_PIXEL) / 8) ' Number of bytes in pixel array.
  byte long 0 ' Horizontal pixels per meter.
  byte long 0 ' Vertical pixels per meter.
  byte long 0 ' Number of colors in the palette.
  byte long 0 ' Number of important colors in the palette.

  byte long _R_V_5_COMBINED_MASKS ' Red bit mask.
  byte long _G_Y_6_COMBINED_MASKS ' Green bit mask.
  byte long _B_U_5_COMBINED_MASKS ' Blue bit mask.

CON ' Bitmap constant information.

  _BMP_HEAD_80x60xBW_SIZE = (_BMP_FILE_HEADER_SIZE + _BMP_INFO_HEADER_SIZE + _BMP_COLOR_TABLE_SIZE)
  _BMP_BW_BITS_PER_PIXEL = 1

DAT BMPHeader80x60xBW_File ' File header.

  byte byte "BM" ' Bitmap signature.
  byte long (_BMP_HEAD_80x60xBW_SIZE + ((_LN_DS_PADDED_BITS_PER_ROW * _LN_DOWN_SAMPLED_V * _BMP_BW_BITS_PER_PIXEL) / 8))
  byte word 0 ' Reserved.
  byte word 0 ' Reserved.
  byte long _BMP_HEAD_80x60xBW_SIZE ' Offest to pixel array.

  byte long _BMP_INFO_HEADER_SIZE ' DIB header size.
  byte long 0+_LN_DOWN_SAMPLED_H ' Pixel width. Horizontally no flipped.
  byte long 0-_LN_DOWN_SAMPLED_V ' Pixel height. Vertically yes flipped.
  byte word 1 ' Number of color planes.
  byte word _BMP_BW_BITS_PER_PIXEL ' Bits per pixel.
  byte long 0 ' Bit fields.
  byte long ((_LN_DS_PADDED_BITS_PER_ROW * _LN_DOWN_SAMPLED_V * _BMP_BW_BITS_PER_PIXEL) / 8) ' Number of bytes in pixel array.
  byte long 0 ' Horizontal pixels per meter.
  byte long 0 ' Vertical pixels per meter.
  byte long 2 ' Number of colors in the palette.
  byte long 2 ' Number of important colors in the palette.

  byte long $00_00_00 ' Black.
  byte long $FF_FF_FF ' White.

PRI powerLEDOn(frequency) ' Turn on the power LED.

  if(LEDModeSetup)
    if(frequency < 0) ' Negative to turn the LED off.
      powerLEDOff

    else
      ifnot(frequency and (frequency =< (clkfreq >> 1)))
        outa[_AUX_POWER_LED_PIN] := 1

      else
        repeat 32 ' Preform ((result << 32) / clkfreq) rounded up.

          result <<= 1
          frequency <<= 1
          if(frequency => clkfreq)
            frequency -= clkfreq
            result += 1

        frqb := ((result~) - (frequency <> 0)) ' Round up.
        ctrb := constant(_NCO_SE_LONG + _AUX_POWER_LED_PIN)

        outa[_AUX_POWER_LED_PIN] := 0
      dira[_AUX_POWER_LED_PIN] := 1

PRI powerLEDOff ' Turn off the power LED.

  if(LEDModeSetup)
    dira[_AUX_POWER_LED_PIN] := 0
    ctrb := 0

PRI cameraPowerUp ' Power up the camera.

  outa[_CAM_PWDN_PIN] := 0
  dira[_CAM_PWDN_PIN] := 0

PRI cameraPowerDown ' Power down the camera.

  outa[_CAM_PWDN_PIN] := 1
  dira[_CAM_PWDN_PIN] := 1

PRI cameraClockOn ' Turn on the camera clock source.

  frqa := constant(1 -> 1) ' Peak clkfreq.
  ctra := constant(_NCO_SE_LONG + _CAM_XCLK_PIN)

  outa[_CAM_XCLK_PIN] := 0
  dira[_CAM_XCLK_PIN] := 1

PRI cameraClockOff ' Turn off the camera clock source.

  dira[_CAM_XCLK_PIN] := 0
  ctra := 0

CON _DEC_OUT_TEMP_STRING_SIZE = 3 ' Signed 10 digit string plus NULL character.

VAR long tokenStringPointer, tempString[_DEC_OUT_TEMP_STRING_SIZE] ' String processing variables.

PRI STRToken(stringPointer) ' Splits a string into tokens.

' Notes:
'
' StringPointer - The address of the string to tokenize. Zero to continue tokenizing.
'
' Returns a pointer to an individual token from the tokenized string.

  if(stringPointer)
    tokenStringPointer := stringPointer

  if(tokenStringPointer)
    tokenStringPointer := result := STRTrim(tokenStringPointer)

    stringPointer := " "

    if(byte[tokenStringPointer] == com#Quotation_Marks)
      result := (STRTrim(++tokenStringPointer))

      stringPointer := com#Quotation_Marks

    repeat while(byte[tokenStringPointer])
      if(byte[tokenStringPointer++] == stringPointer)
        byte[tokenStringPointer][-1] := _NULL
        quit

PRI STRTrim(stringPointer) ' Trims leading white space from a string.

' Notes:
'
' StringPointer - The address of the string to trim.
'
' Returns a pointer to the trimmed string.

  if(stringPointer)
    result := --stringPointer
    repeat ' Format saves two bytes.
    while(byte[++result] == " ")

PRI DECOut(integer) | sign ' Integer to string.

' Notes:
'
' Integer - The integer to convert.
'
' Returns a pointer to the converted integer string.

  longfill(@tempString, 0, _DEC_OUT_TEMP_STRING_SIZE)
  sign := (integer < 0) ' Store sign.

  repeat result from 10 to 1 ' Convert number.
    tempString.byte[result] := ((||(integer // 10)) + "0")
    integer /= 10

  result := @tempString ' Skip past leading zeros.
  repeat ' Format saves two bytes.
  while(byte[++result] == "0")
  result += (not(byte[result]))

  if(sign) ' Insert sign.
    byte[--result] := "-"

PRI DECIn(stringPointer) | sign ' String to integer.

' Notes:
'
' StringPointer - The string to convert.
'
' Returns the converted string's integer value.

  if(stringPointer := STRTrim(stringPointer))
    sign := ((byte[stringPointer] == "-") | 1)
    stringPointer -= ((sign == -1) or (byte[stringPointer] == "+")) ' Skip sign.

    if(byte[stringPointer] == "0")
      if((byte[stringPointer + 1] == "X") or (byte[stringPointer + 1] == "x"))
        return (HEXIn(stringPointer + 2) * sign)

    repeat (strsize(stringPointer) <# 10) ' Expect 10 digits.
      if((byte[stringPointer] < "0") or ("9" < byte[stringPointer]))
        quit ' Break on a non-digit.

      result := ((result * 10) + (byte[stringPointer++] & $F))
    result *= sign

PRI HEXOut(integer) ' Integer to string.

' Notes:
'
' Integer - The integer to convert.
'
' Returns a pointer to the converted integer string.

  bytemove(@tempString, string("0x00000000h"), 12)

  repeat result from constant(7 + 2) to constant(0 + 2)
    tempString.byte[result] := lookupz((integer & $F): "0".."9", "A".."F")
    integer >>= 4

  return @tempString

PRI HEXIn(stringPointer) ' String to integer.

' Notes:
'
' StringPointer - The string to convert.
'
' Returns the converted string's integer value.

  if(stringPointer)

    repeat (strsize(stringPointer) <# 8)
      ifnot(("0" =< byte[stringPointer]) and (byte[stringPointer] =< "9"))
        ifnot(("A" =< byte[stringPointer]) and (byte[stringPointer] =< "F"))
          quit

        result += constant(($A - ("A" & $F)) -> 4)
      result := ((result <- 4) + (byte[stringPointer++] & $F))

CON ' Servo controller driver values.

  _SC_MILLISECOND_TIMING = 1_000 ' Used for timing control.
  _SC_MICROSECOND_TIMING = 1_000_000 '

  _SC_STACK_SIZE = 32 ' In longs.
  _SC_UPDATE_FREQUENCY = 50 ' Update frequency in hertz.

  _SC_COUNTER_VALUE = 1 ' Added per clock cycle.
  _SC_LIMIT_VALUE = 65_535 ' Push button count limit.

DAT ' Default servo control values.

  panPControl word ((_DEFAULT_PAN_PCTRL <# _PAN_CTRL_MAXIMUM) #> _PAN_CTRL_MINIMUM)
  panDControl word ((_DEFAULT_PAN_DCTRL <# _PAN_CTRL_MAXIMUM) #> _PAN_CTRL_MINIMUM)

  tiltPControl word ((_DEFAULT_TILT_PCTRL <# _TILT_CTRL_MAXIMUM) #> _TILT_CTRL_MINIMUM)
  tiltDControl word ((_DEFAULT_TILT_DCTRL <# _TILT_CTRL_MAXIMUM) #> _TILT_CTRL_MINIMUM)

  panPulseLength word ((_PAN_DEFAULT_PULSE <# _SERVO_MAX) #> _SERVO_MIN)
  tiltPulseLength word ((_TILT_DEFAULT_PULSE <# _SERVO_MAX) #> _SERVO_MIN)

  requestPanPulseLength word 0 ' For latching the update.
  requestTiltPulseLength word 0 ' For latching the update.

PRI servoControllerDriver | buffer,counter, ep,op,et,ot,xb,xc, pctrl_l,tctrl_l, trackingDataBuffer[_LN_TRACK_DATA_SIZE_LONGS]

  ctra := constant(_NCO_SE_LONG + _GPIO_PAN)
  ctrb := constant(_NCO_SE_LONG + _GPIO_TILT)
  frqa := _SC_COUNTER_VALUE
  frqb := _SC_COUNTER_VALUE

  buffer := ep := et := op := ot := 0
  counter := xb := 1 ' Ensure the automatic trigger of "counter" and "xb".

  result := cnt
  repeat ' Loop every millisecond.
    waitcnt(result += (clkfreq / _SC_MILLISECOND_TIMING))
    buffer += (buffer + ina[_AUX_PUSH_BUTTON_PIN]) ' Shift in the button state.

    pctrl_l := PCTRL ' Latch update.
    tctrl_l := TCTRL ' Latch update.

    ifnot((phsa | phsb) & constant(1 -> 1)) ' Make sure the outputs are low.
      dira := (SCDir | (constant(|<_GPIO_PAN) & (pctrl_l <> false)) | (constant(|<_GPIO_TILT) & (tctrl_l <> false)))

    ifnot(--counter)
      if(xb <> LNSwitch)
        xb := LNSwitch
        xc := constant((_SC_UPDATE_FREQUENCY + _FRAME_TIMEOUT - 1) / _FRAME_TIMEOUT)

      if(xc => 1)
        xc -= 1

        getTrackData(@trackingDataBuffer) ' If the tracked pixels are not equal to zero and no timeout.
        if(trackingDataBuffer.byte[_STRUCT_LN_T_TRACKED_PIXELS])

          counter := LNWindow

          ep := (((((counter.byte[_STRUCT_VW_X2] - counter.byte[_STRUCT_VW_X1] + 1) >> 1) + counter.byte[_STRUCT_VW_X1]) {
                } - trackingDataBuffer.byte[_STRUCT_LN_T_X_CENTROID]) * pctrl_l) ' Get new pan error.
          et := (((((counter.byte[_STRUCT_VW_Y2] - counter.byte[_STRUCT_VW_Y1] + 1) >> 1) + counter.byte[_STRUCT_VW_Y1]) {
                } - trackingDataBuffer.byte[_STRUCT_LN_T_Y_CENTROID]) * tctrl_l) ' Get new tilt error.

          if(pctrl_l)
            panPulseLength := (((panPulseLength + ((ep * panPControl) / _PAN_CTRL_MAXIMUM) + {
            } (((ep - op) * panDControl) / _PAN_CTRL_MAXIMUM)) <# _SERVO_MAX) #> _SERVO_MIN)

          if(tctrl_l)
            tiltPulseLength := (((tiltPulseLength + ((et * tiltPControl) / _TILT_CTRL_MAXIMUM) + {
            } (((et - ot) * tiltDControl) / _TILT_CTRL_MAXIMUM)) <# _SERVO_MAX) #> _SERVO_MIN)

          op := ep ' Remember old pan error.
          ot := et ' Remember old tilt error.

      if(requestPanPulseLength) ' Latch in value update.
        panPulseLength := requestPanPulseLength~

      if(requestTiltPulseLength) ' Latch in value update.
        tiltPulseLength := requestTiltPulseLength~

      counter := (clkfreq / _SC_MICROSECOND_TIMING)

      phsa := -(panPulseLength * counter) ' Setup counter module to count down.
      phsb := -(tiltPulseLength * counter) ' Setup counter module to count down.

      counter := constant(_SC_MILLISECOND_TIMING / _SC_UPDATE_FREQUENCY)

    PBTime -= (PBTime < _SC_LIMIT_VALUE) ' Count up to max.

    ifnot(buffer or PBState) ' When the button is pressed and was previously released...
      PBState := true
      PBTime := 0
      PBPressed := true

    if((not(!buffer)) and PBState) ' When the button is released and was previously pressed...
      PBState := false
      PBTime := 0
      PBReleased := true

VAR long PBPressed, PBState, PBReleased, PBTime, PCTRL, TCTRL, SCDir, SCCog, SCStack[_SC_STACK_SIZE]

PRI servoControllerStart ' Initializes the servo controller driver. Returns true on success and false on failure.

  servoControllerStop
  if(chipver == 1)

    PBState := not(ina[_AUX_PUSH_BUTTON_PIN])
    PBTime := 0

    SCCog := cognew(servoControllerDriver, @SCStack)
    result or= ++SCCog

PRI servoControllerStop ' Shutsdown the servo controller driver.

  if(SCCog)
    cogstop(-1 + SCCog~)

PRI setupPanDirection(output) ' True to enable pan output and false to disable pan output.

  SCDir := ((SCDir & constant(!(|<_GPIO_PAN))) | (constant(|<_GPIO_PAN) & (output <> false)))

PRI setupPanPulse(length) ' Sets the pan pulse length output. Length is unsigned in microseconds.

  requestPanPulseLength := length ' 1us resolution.
  repeat ' Format saves two bytes.
  while(requestPanPulseLength) ' Wait for update to go live.

PRI setupTiltDirection(output) ' True to enable tilt output and false to disable tilt output.

  SCDir := ((SCDir & constant(!(|<_GPIO_TILT))) | (constant(|<_GPIO_TILT) & (output <> false)))

PRI setupTiltPulse(length) ' Sets the tilt pulse length output. Length is unsigned in microseconds.

  requestTiltPulseLength := length ' 1us resolution.
  repeat ' Format saves two bytes.
  while(requestTiltPulseLength) ' Wait for update to go live.

PRI getPanPulse ' Returns the pan pulse output - returns zero if no output.

  return (panPulseLength & ((PCTRL <> false) or ((SCDir & constant(|<_GPIO_PAN)) <> false)))

PRI getTiltPulse ' Returns the tilt pulse output - returns zero if no output.

  return (tiltPulseLength & ((TCTRL <> false) or ((SCDir & constant(|<_GPIO_TILT)) <> false)))

PRI buttonPressed ' Returns true if the push button was pressed. Returns true once per press.

  if(PBPressed)
    return PBPressed~

PRI buttonReleased ' Returns true if the push button was released. Returns true once per release.

  if(PBReleased)
    return PBReleased~

PRI panModeSetup(onOrOff, reversed) ' Turn automatic pan on or off.

  PCTRL := ((not(not(onOrOff))) & (1 | (not(not(reversed))))) ' Output is -1, 0, and 1.

PRI tiltModeSetup(onOrOff, reversed) ' Turn automatic tilt on or off.

  TCTRL := ((not(not(onOrOff))) & (1 | (not(not(reversed))))) ' Output is -1, 0, and 1.

DAT

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
'                       Bitmap Grabber Driver
' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                        org     0

' //////////////////////Initialization/////////////////////////////////////////////////////////////////////////////////////////

BMGrabberInit           mov     BMColumnCounter,       BMHorizontalRes wz     ' Setup initial values. Not Z.

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
'                       Frame Grab Loop
' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

BMNextColumn            mov     BMRowCounter,          BMVerticalRes          ' Setup counters. Z set at end of loop.
if_nz                   mov     BMBufferPointer,       par                    '

                        waitpeq BMVSYNCMask,           BMVSYNCMask            ' Wait for VSYNC.

BMNextRow               mov     BMBufferCounter,       BMColumnCounter        ' Figure leading pixels to skip.
                        sub     BMBufferCounter,       BMHorizontalRes wz     '

                        test    BMBufferCounter,       #1 wc                  ' Align grab to an even pixel boundary.
if_c                    add     BMBufferCounter,       #1 wz                  '

                        muxc    BMSwap,                #1                     ' Setup for pixel swap.

' //////////////////////Grab Pixel/////////////////////////////////////////////////////////////////////////////////////////////

                        waitpne BMHREFMask,            BMHREFMask             ' Wait till image end. Skip 4 pixels...
                        waitpeq BMHREFMask,            BMHREFMask             '

                        mov     cnt,                   #7                     ' Align instructions. Can be 6, 7, or 8.
if_nz                   add     cnt,                   #8                     '
                        add     cnt,                   cnt                    '
                        waitcnt cnt,                   cnt                    '

BMSkipPixel if_nz       add     BMBufferCounter,       #1 wc                  ' Skip pixels in sync.
if_nz_and_nc            jmp     #BMSkipPixel                                  '

                        mov     BMCaptureBuffer0,      ina                    ' Read pixels in sync.
                        mov     BMCaptureBuffer1,      ina                    '
                        mov     BMCaptureBuffer2,      ina                    '
                        mov     BMCaptureBuffer3,      ina                    '

' //////////////////////Skip Lines/////////////////////////////////////////////////////////////////////////////////////////////

                        mov     BMBufferCounter,       BMVerticalDivide       ' Setup to skip vertical lines.

BMDivideLoop            sub     BMBufferCounter,       #1 wz                  ' Skip vertical lines.
if_nz                   waitpne BMHREFMask,            BMHREFMask             '
if_nz                   waitpeq BMHREFMask,            BMHREFMask             '
if_nz                   jmp     #BMDivideLoop                                 '

' //////////////////////Upload Data////////////////////////////////////////////////////////////////////////////////////////////

                        cmp     BMColorSpace,          #0 wz                  ' Check color space and swap.
BMSwap                  test    BMSwap,                #0 wc                  '

if_nz_and_c             mov     BMCaptureBuffer0,      BMCaptureBuffer2       ' Select which Y component to use.

if_nz                   and     BMCaptureBuffer0,      #_G_Y_8_MASK           ' Scale Y component.
if_nz                   add     BMCaptureBuffer0,      BMYScaleLUTAddress     '
if_nz                   rdbyte  BMCaptureBuffer0,      BMCaptureBuffer0       '

if_nz                   and     BMCaptureBuffer1,      #_B_U_8_MASK           ' Scale U component.
if_nz                   add     BMCaptureBuffer1,      BMUVScaleLUTAddress    '
if_nz                   rdbyte  BMCaptureBuffer1,      BMCaptureBuffer1       '

if_nz                   and     BMCaptureBuffer3,      #_R_V_8_MASK           ' Scale V component.
if_nz                   add     BMCaptureBuffer3,      BMUVScaleLUTAddress    '
if_nz                   rdbyte  BMCaptureBuffer3,      BMCaptureBuffer3       '

if_nz                   shl     BMCaptureBuffer0,      #_G_Y_6_COMBINED_SHIFT ' Shift Y and V into place.
if_nz                   shl     BMCaptureBuffer3,      #_R_V_5_COMBINED_SHIFT '

if_nz                   or      BMCaptureBuffer0,      BMCaptureBuffer1       ' Build VYU word.
if_nz                   or      BMCaptureBuffer0,      BMCaptureBuffer3       '
if_nz                   wrword  BMCaptureBuffer0,      BMBufferPointer        '

if_z_and_nc             wrbyte  BMCaptureBuffer1,      BMBufferPointer        ' Upload second/fourth and first/third byte.
if_z_and_c              wrbyte  BMCaptureBuffer3,      BMBufferPointer        '
                        add     BMBufferPointer,       #1                     '
if_z_and_nc             wrbyte  BMCaptureBuffer0,      BMBufferPointer        '
if_z_and_c              wrbyte  BMCaptureBuffer2,      BMBufferPointer        '
                        add     BMBufferPointer,       #1                     '

' //////////////////////Handle Buffering///////////////////////////////////////////////////////////////////////////////////////

                        sub     BMRowCounter,          BMVerticalDivide       ' Loop with vertical divide consideration.
                        tjnz    BMRowCounter,          #BMNextRow             '

                        rdlong  BMBufferCounter,       BMSwitchAddress wz     ' Wait for control cog to catch up.
BMThrottleLoop          rdlong  BMBufferCounter,       BMThrottleAddress      '
if_z                    tjnz    BMBufferCounter,       #BMThrottleLoop        '
if_nz                   tjz     BMBufferCounter,       #BMThrottleLoop        '

                        sumz    BMBufferCounter,       #1                     ' Flip buffer switch.
                        wrlong  BMBufferCounter,       BMSwitchAddress        '

' //////////////////////Loop///////////////////////////////////////////////////////////////////////////////////////////////////

                        sub     BMColumnCounter,       BMHorizontalDivide     ' Loop with horizontal divide consideration.
                        tjnz    BMColumnCounter,       #BMNextColumn          '

                        cogid   BMBufferCounter                               ' Shutdown.
                        cogstop BMBufferCounter                               '

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
'                       Data
' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

BMColorSpace            long    0 ' RGB565 or YUV655 (VYU565) color space switch. False for RGB. True for YUV.

BMHorizontalRes         long    _CAMERA_H_RES
BMVerticalRes           long    _CAMERA_V_RES

BMHorizontalDivide      long    1 ' Must be a positive factor of _CAMERA_H_RES. MUST NOT BE 0!
BMVerticalDivide        long    1 ' Must be a positive factor of _CAMERA_V_RES. MUST NOT BE 0!

' //////////////////////Pin Masks//////////////////////////////////////////////////////////////////////////////////////////////

BMHREFMask              long    (|<_CAM_HREF_PIN)
BMVSYNCMask             long    (|<_CAM_VSYNC_PIN)

' //////////////////////Addresses//////////////////////////////////////////////////////////////////////////////////////////////

BMSwitchAddress         long    0
BMThrottleAddress       long    0
BMYScaleLUTAddress      long    0
BMUVScaleLUTAddress     long    0

' //////////////////////Run Time Variables/////////////////////////////////////////////////////////////////////////////////////

BMColumnCounter         res     1
BMRowCounter            res     1

BMCaptureBuffer0        res     1
BMCaptureBuffer1        res     1
BMCaptureBuffer2        res     1
BMcaptureBuffer3        res     1

BMBufferCounter         res     1
BMBufferPointer         res     1

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                        fit     496

CON ' Bitmap data buffer setup.

  _BM_BITS_PER_PIXEL = (_R_V_5 + _G_Y_6 + _B_U_5)
  _BM_BYTES_PER_PIXEL = ((_BM_BITS_PER_PIXEL + 7) / 8)

  _BM_BUFFER_SIZE_BYTES = (_CAMERA_V_RES * _BM_BYTES_PER_PIXEL)
  _BM_BUFFER_SIZE_LONGS = ((_BM_BUFFER_SIZE_BYTES + 3) / 4)

  _BM_BUFFER_SIZE_DOUBLE_BYTES = (_BM_BUFFER_SIZE_BYTES * 2) ' Double Buffered size in bytes.
  _BM_BUFFER_SIZE_DOUBLE_LONGS = ((_BM_BUFFER_SIZE_DOUBLE_BYTES + 3) / 4) ' Double Buffered size in longs.

  _BM_OUTPUT_COM = FALSE ' For selecting the communciation port.
  _BM_OUTPUT_FAT = TRUE ' For selecting the filesystem port.

PRI bitmapGrabberDriver(switch) | buffer,counter, BMSwitch,BMThrottle,BMBuffer[_BM_BUFFER_SIZE_DOUBLE_LONGS] ' Double Buffered.

  BMSwitch := BMThrottle := false
  BMSwitchAddress := @BMSwitch
  BMThrottleAddress := @BMThrottle
  BMYScaleLUTAddress := @YScaleLUT
  BMUVScaleLUTAddress := @UVScaleLUT

  counter := (_BM_BUFFER_SIZE_BYTES / BMVerticalDivide)
  buffer := cognew(@BMGrabberInit, @BMBuffer)
  if(!buffer)

    repeat (_CAMERA_H_RES / BMHorizontalDivide)
      repeat com.receivedNumber
        if(com.readByte == _NEWLINE)

          cogstop(buffer)
          return false

      result := cnt
      repeat while(BMSwitch == BMThrottle)
        if((cnt - result) => (clkfreq / _FRAME_TIMEOUT))
          com.writeString(@cameraTimeoutError)

          cogstop(buffer)
          return false

      result := (@BMBuffer + (counter & BMThrottle))
      if(switch) ' Choose data transfer output.
        result := \fat.writeData(result, counter)

      else
        com.writeString(string("DAT: "))
        com.writeData(result, counter)
        com.writeByte(_NEWLINE)

      if(fat.partitionError)
        com.writeString(@errorString)
        com.writeString(result)
        com.writeByte(_NEWLINE)

        cogstop(buffer)
        return false

      not BMThrottle
    return true ' Returns true on success and false on failure.

DAT

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
'                       Down Sample Driver
' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                        org     0

' //////////////////////Initialization/////////////////////////////////////////////////////////////////////////////////////////

DSDriverInit            mov     DSVerticalCounter,   #_LN_DOWN_SAMPLED_V ' Setup horizontal and vertical loop.
DSVerticalLoop          mov     DSHorizontalCounter, #_LN_WORDS_PER_ROW  '

DSHorizontalLoop        rdbyte  DSDataIn,            DSInputAddress      ' Get input half.
                        add     DSInputAddress,      #1                  '
                        rev     DSDataIn,            #(32 - 8)           '

                        rdbyte  DSDataOut,           DSInputAddress      ' Get input half.
                        add     DSInputAddress,      #1                  '
                        rev     DSDataOut,           #(32 - 8)           '

                        shl     DSDataOut,           #8                  ' Combine halves.
                        or      DSDataIn,            DSDataOut           '

                        mov     DSCounter,           #8                  ' Horizontally down sample pixels.
DSLoop                  shr     DSDataIn,            #2 wc               '
                        rcl     DSDataOut,           #1                  '
                        djnz    DSCounter,           #DSLoop             '

                        wrbyte  DSDataOut,           DSOutputAddress     ' Write output.
                        add     DSOutputAddress,     #1                  '
                        djnz    DSHorizontalCounter, #DSHorizontalLoop   '

                        add     DSInputAddress,      #_LN_BYTES_PER_ROW  ' Vertically down sample pixels.
                        djnz    DSVerticalCounter,   #DSVerticalLoop     '

DSNonZero               wrlong  DSNonZero,           par                 ' Write non-zero to the flag address.

                        cogid   DSBuffer                                 ' Shutdown.
                        cogstop DSBuffer                                 '

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
'                       Data
' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

DSInputAddress          long    0
DSOutputAddress         long    0

' //////////////////////Run Time Variables/////////////////////////////////////////////////////////////////////////////////////

DSBuffer                res     1
DSCounter               res     1

DSDataIn                res     1
DSDataOut               res     1

DSHorizontalCounter     res     1
DSVerticalCounter       res     1

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                        fit     496

CON ' Line data buffer setup.

  LNDivideBuffer = $1F5 ' OUTB
  LNDivideCounter = $1F7 ' DIRB

  _LN_HALF_V_DIV = (_CAM_V_DIV * 2)
  _LN_HALF_V_WIN = (_CAMERA_V_WIN / 2)

  _LN_IF_ALWAYS = (%1111 << 18)
  _LN_IF_NEVER = !(%1111 << 18)

  _LN_SLAVE_DISPLACEMENT = 0
  _LN_MASTER_DISPLACEMENT = 3

  _LN_SLAVE_OFFSET = 0
  _LN_MASTER_OFFSET = 1

  _LN_MASK_FLIPPER = 2
  _LN_MASK_SHIFTER = 3

  _LN_MOVS_BIT_SHIFT = 0
  _LN_MOVD_BIT_SHIFT = _LN_MOVS_BIT_SHIFT + 9
  _LN_XXXX_BIT_SHIFT = _LN_MOVD_BIT_SHIFT + 9
  _LN_MOVI_BIT_SHIFT = _LN_XXXX_BIT_SHIFT + 5

  _LN_GREEN_MASK_TOP = ((_G_Y_6_COMBINED_MASKS >> 8) & $FF)
  _LN_GREEN_MASK_BOTTOM = ((_G_Y_6_COMBINED_MASKS >> 0) & $FF)

  _LN_RED_SHIFT = ((_LN_XXXX_BIT_SHIFT - 1) - (_R_V_5 + _R_V_5_COMBINED_SHIFT))
  _LN_GREEN_SHIFT = (_G_Y_6_COMBINED_SHIFT + (_G_Y_6 / 2))

  _LN_WORDS_PER_ROW = ((_CAMERA_H_WIN + 15) / 16)
  _LN_BYTES_PER_ROW = ((_CAMERA_H_WIN + 7) / 8)

  _LN_BUFFER_SIZE_BYTES = ((_CAMERA_H_WIN * _CAMERA_V_WIN) / 8)
  _LN_BUFFER_SIZE_LONGS = ((_LN_BUFFER_SIZE_BYTES + 3) / 4)

  _LN_BUFFER_SIZE_D_BYTES = (_LN_BUFFER_SIZE_BYTES * 2) ' Double Buffered size in bytes.
  _LN_BUFFER_SIZE_D_LONGS = ((_LN_BUFFER_SIZE_D_BYTES + 3) / 4) ' Double Buffered size in longs.

  _LN_OUTPUT_COM = FALSE ' For selecting the communciation port.
  _LN_OUTPUT_FAT = TRUE ' For selecting the filesystem port.

  _LN_DOWN_SAMPLED_H = (_CAMERA_H_WIN / 2)
  _LN_DOWN_SAMPLED_V = (_CAMERA_V_WIN / 2)

  _LN_DS_BYTES_PER_ROW = ((_LN_DOWN_SAMPLED_H + 7) / 8)
  _LN_DS_LONGS_PER_ROW = ((_LN_DOWN_SAMPLED_H + 31) / 32)

  _LN_DS_PADDED_BITS_PER_ROW = (_LN_DS_LONGS_PER_ROW * 32)
  _LN_DS_PADDED_BYTES_PER_ROW = (_LN_DS_LONGS_PER_ROW * 4)

  _LN_DS_SIZE_BYTES = ((_LN_DOWN_SAMPLED_H * _LN_DOWN_SAMPLED_V) / 8)
  _LN_DS_SIZE_LONGS = ((_LN_DS_SIZE_BYTES + 3) / 4)

VAR long LNBuffer[_LN_BUFFER_SIZE_D_LONGS] ' Double Buffered.

PRI lineGrabberDriver(switch) | downSampleStripe[_LN_DS_LONGS_PER_ROW], downSampleBuffer[_LN_DS_SIZE_LONGS]

  longfill(@downSampleStripe, 0, _LN_DS_LONGS_PER_ROW) ' Clear stripe buffer.

  DSOutputAddress := @downSampleBuffer ' Setup down sampling.
  DSInputAddress := (@LNBuffer + (_LN_BUFFER_SIZE_BYTES & (not(LNSwitch))))

  if(!cognew(@DSDriverInit, @result)) ' Launch a cog to down sample more quickly.

    repeat ' Format saves two bytes.
    until(result) ' Wait for driver to finish - the flag will go high.

    if(switch) ' Choose output.
      switch := @downSampleBuffer

      repeat _LN_DOWN_SAMPLED_V ' Write long row aligned data.
        bytemove(@downSampleStripe, switch, _LN_DS_BYTES_PER_ROW)
        switch += _LN_DS_BYTES_PER_ROW

        result := \fat.writeData(@downSampleStripe, _LN_DS_PADDED_BYTES_PER_ROW)

        if(fat.partitionError) ' Handle any error cleanly.
          com.writeString(@errorString)
          com.writeString(result)
          com.writeByte(_NEWLINE)
          return false

    else ' Send the down sampled bitmap to the serial port.
      com.writeData(@downSampleBuffer, _LN_DS_SIZE_BYTES)
      com.writeByte(_NEWLINE)

    return true ' Return true on success and false on failure.

DAT

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
'                       Line Grabber Driver
' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                        org     0

' //////////////////////Initialization/////////////////////////////////////////////////////////////////////////////////////////

LNGrabberInit           movi    ctra,                   #_POSEDGE_D_MOVI             ' Setup positive edge detector.
                        movs    ctra,                   #_CAM_HREF_PIN               '
                        mov     frqa,                   #1                           '

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
'                       Line Grab Loop
' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

LNGrabberLoop           waitcnt LNRendezvousCount,      #_LN_SHORT_SYNC_OFFSET       ' Master and slave now in sync.
                        rdlong  LNTrackingPixelSetup,   LNModeSetupAddress           '
                        rdlong  LNBlueLowerThreshold,   LNThresholdAddress           '
                        rdlong  LNWindowY2Bound,        LNWindowAddress              '

                        rdlong  LNBufferPointer,        LNSwitchAddress wz           ' Setup data buffer pointer.
                        mov     LNBufferPointer,        par                          '
if_nz                   add     LNBufferPointer,        LNDataBufferSize             '

                        mov     LNHistogramRedBase,     LNHistoDataAddress           ' Setup red histogram base.
if_nz                   add     LNHistogramRedBase,     #_LN_HISTO_DATA_SIZE_BYTES   '

                        mov     LNHistogramGreenBase,   LNHistogramRedBase           ' Setup green histogram base.
                        add     LNHistogramGreenBase,   #(_STRUCT_LN_H_G_Y_6 * 2)    '

                        mov     LNHistogramBlueBase,    LNHistogramRedBase           ' Setup blue histogram base.
                        add     LNHistogramBlueBase,    #(_STRUCT_LN_H_B_U_5 * 2)    '

                        mov     LNRedUpperThreshold,    LNBlueLowerThreshold         ' Extract red thresholds.
                        shr     LNRedUpperThreshold,    #(_R_V_5 + _G_Y_6 + _B_U_5)  '
                        and     LNRedUpperThreshold,    LNRedRGB565BitMask           '
                        mov     LNRedLowerThreshold,    LNBlueLowerThreshold         '
                        and     LNRedLowerThreshold,    LNRedRGB565BitMask           '

                        mov     LNGreenUpperThreshold,  LNBlueLowerThreshold         ' Extract green thresholds.
                        shr     LNGreenUpperThreshold,  #(_R_V_5 + _G_Y_6 + _B_U_5)  '
                        and     LNGreenUpperThreshold,  LNGreenRGB565BitMask         '
                        mov     LNGreenLowerThreshold,  LNBlueLowerThreshold         '
                        and     LNGreenLowerThreshold,  LNGreenRGB565BitMask         '

                        mov     LNBlueUpperThreshold,   LNBlueLowerThreshold         ' Extract blue thresholds.
                        shr     LNBlueUpperThreshold,   #(_R_V_5 + _G_Y_6 + _B_U_5)  '
                        and     LNBlueUpperThreshold,   #_B_U_5_COMBINED_MASKS       '
                        and     LNBlueLowerThreshold,   #_B_U_5_COMBINED_MASKS       '

                        mov     LNWindowX1Bound,        LNWindowY2Bound              ' Extract X1 bound.
                        shr     LNWindowX1Bound,        #_STRUCT_SHIFT_VW_X1         '
                        and     LNWindowX1Bound,        #$FF                         '

                        mov     LNWindowY1Bound,        LNWindowY2Bound              ' Extract Y1 bound.
                        shr     LNWindowY1Bound,        #_STRUCT_SHIFT_VW_Y1         '
                        and     LNWindowY1Bound,        #$FF                         '

                        mov     LNWindowX2Bound,        LNWindowY2Bound              ' Extract X2 bound.
                        shr     LNWindowX2Bound,        #_STRUCT_SHIFT_VW_X2         '
                        and     LNWindowX2Bound,        #$FF                         '

                        shr     LNWindowY2Bound,        #_STRUCT_SHIFT_VW_Y2         ' Extract Y2 bound.
                        and     LNWindowY2Bound,        #$FF                         '

                        mov     LNXBoundCounter,        #0                           ' Reset location counters.
LNYBoundOffset          mov     LNYBoundCounter,        #0                           '

                        mov     LNFilteringPixelSetup,  LNTrackingPixelSetup         ' Extract noise filter threshold.
                        shr     LNFilteringPixelSetup,  #_LN_MODE_SETUP_NF           '
                        and     LNFilteringPixelSetup,  #$FF                         '

                        movd    LNTAOClearLoop,         #LNTAO                       ' Clear tracking data array.
                        mov     LNDivideCounter,        #_STRUCT_LN_TA_SIZE          '
LNTAOClearLoop          mov     LNTAO,                  #0                           '
                        add     LNTAOClearLoop,         LNDestinationIncrement       '
                        djnz    LNDivideCounter,        #LNTAOClearLoop              '

                        mov     (LNTAO+_STRUCT_LN_X1),  #_CAMERA_MAX_H               ' Initialize tracking data array.
                        mov     (LNTAO+_STRUCT_LN_Y1),  #_CAMERA_MAX_V               '

' //////////////////////Next Loop//////////////////////////////////////////////////////////////////////////////////////////////

                        waitcnt LNRendezvousCount,      #0                           ' Master and slave rendezvous.
                        waitpeq LNVSYNCMask,            LNVSYNCMask                  '

                        mov     LNRendezvousCount,      cnt                          ' Setup to rendezvous again.
                        add     LNRendezvousCount,      LNRendezvousDelta            '

                        mov     LNRowCounter,           #_LN_HALF_V_WIN              ' Slave lines (0, 8, ..., 632).
LNSlaveMod0 if_never    jmp     #LNNextRow                                           '

                        mov     LNDivideBuffer,         LNHistogramRedBase           ' Setup to clear histogram data buffer.
                        mov     LNDivideCounter,        #_LN_HISTO_DATA_SIZE_WORDS   '

                        mov     phsa,                   #0                           ' Clear histogram data buffer.
LNMasterZeroLoop        wrword  phsa,                   LNDivideBuffer               '
                        add     LNDivideBuffer,         #2                           '
                        djnz    LNDivideCounter,        #LNMasterZeroLoop            '

                        mov     LNDivideBuffer,         #_CAM_V_DIV                  ' Master lines (4, 12, ..., 636).
LNMasterThrottleLoop    cmp     LNDivideBuffer,         phsa wz, wc                  '
if_nz_and_nc            jmp     #LNMasterThrottleLoop                                '

LNNextRow               mov     LNColumnCounter,        #_CAMERA_H_WIN               ' Setup for the pixel storage loop.
                        movd    LNStorePixelModify,     #LNPixelStorage              '

' //////////////////////Store Pixel////////////////////////////////////////////////////////////////////////////////////////////

                        waitpne LNHREFMask,             LNHREFMask                   ' Wait for image end. Skip 4 pixels...
                        mov     phsa,                   #0                           '
                        waitpeq LNHREFMask,             LNHREFMask                   '

                        mov     cnt,                    #(4 + (7) + 8)               ' Align instructions. Can be 6, 7, or 8.
                        add     cnt,                    cnt                          '
                        waitcnt cnt,                    cnt                          '

LNStorePixelLoop        movd    LNDivideBuffer,         ina                          ' Grab first pixel. Y. U.
                        movs    LNDivideBuffer,         ina                          '

LNRedRGB565BitMask      long    _R_V_5_COMBINED_MASKS                                ' Evaluates to a NOP.

                        movi    LNDivideBuffer,         ina                          ' Grab second pixel. Y. V.

LNStorePixelModify      mov     LNPixelStorage,         LNDivideBuffer               ' Skip third pixel.
                        add     LNStorePixelModify,     LNDestinationIncrement       '

LNGreenRGB565BitMask    long    _G_Y_6_COMBINED_MASKS                                ' Evaluates to a NOP.

                        djnz    LNColumnCounter,        #LNStorePixelLoop            ' Skip fourth pixel.

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
'                       Process Data
' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                        mov     LNFilterPixelCount,     #0                           ' Reset filter pixel count.

                        mov     LNColumnCounter,        #(_CAMERA_H_WIN / 8)         ' Setup processing loop.
                        movs    LNFetchPixelLoop,       #LNPixelStorage              '

LNThresholdPixelLoop    mov     LNDivideCounter,        #8                           ' Setup for 8 passes.

' //////////////////////Extract Data///////////////////////////////////////////////////////////////////////////////////////////

LNFetchPixelLoop        mov     LNBluePart,             LNPixelStorage               ' Grab pixel from storage buffer.
                        mov     LNRedPart,              LNBluePart                   '
                        mov     LNGreenPart,            LNBluePart                   '
                        shr     LNGreenPart,            #_LN_MOVD_BIT_SHIFT          '
                        add     LNFetchPixelLoop,       #1                           '

                        test    LNTrackingPixelSetup,   #_LN_MODE_SETUP_CT wc        ' Figure color space mode.
if_c                    jmp     #LNFetchPixelSkip                                    '

                        and     LNGreenPart,            #_LN_GREEN_MASK_TOP          ' Build a half of green...
                        shl     LNGreenPart,            #_LN_GREEN_SHIFT             '

                        mov     LNPixelStorage,         LNBluePart                   ' Build the whole of green...
                        and     LNPixelStorage,         #_LN_GREEN_MASK_BOTTOM       '
                        or      LNGreenPart,            LNPixelStorage               '

                        shr     LNRedPart,              #_LN_RED_SHIFT               ' Build red...
                        and     LNRedPart,              LNRedRGB565BitMask           '

                        and     LNBluePart,             #_B_U_5_COMBINED_MASKS       ' Build blue...

                        jmp     #LNFetchPixelChecks                                  ' Skip ahead.

LNFetchPixelSkip        shr     LNRedPart,              #_LN_MOVI_BIT_SHIFT          ' Move V into position.

                        and     LNGreenPart,            #_G_Y_8_MASK                 ' Build Y and scale Y.
                        add     LNGreenPart,            LNYScaleLUTAddress           '
                        rdbyte  LNGreenPart,            LNGreenPart                  '

                        and     LNBluePart,             #_B_U_8_MASK                 ' Build U and scale U.
                        add     LNBluePart,             LNUVScaleLUTAddress          '
                        rdbyte  LNBluePart,             LNBluePart                   '

                        and     LNRedPart,              #_R_V_8_MASK                 ' Build V and scale V.
                        add     LNRedPart,              LNUVScaleLUTAddress          '
                        rdbyte  LNRedPart,              LNRedPart                    '

                        shl     LNGreenPart,            #_G_Y_6_COMBINED_SHIFT       ' Move Y and V into position.
                        shl     LNRedPart,              #_R_V_5_COMBINED_SHIFT       '

' //////////////////////Check Windowing////////////////////////////////////////////////////////////////////////////////////////

LNFetchPixelChecks      cmpsub  LNWindowX2Bound,        LNXBoundCounter wc, nr       ' Figure if within horizontal bound.
if_c                    cmpsub  LNXBoundCounter,        LNWindowX1Bound wc, nr       '

if_c                    cmpsub  LNWindowY2Bound,        LNYBoundCounter wc, nr       ' Figure if within vertical bound.
if_c                    cmpsub  LNYBoundCounter,        LNWindowY1Bound wc, nr       '

                        muxnc   par,                    #_LN_FLAG_TEST_VW            ' Store if pixel passed windowing.

' //////////////////////Check Thresholding/////////////////////////////////////////////////////////////////////////////////////

                        cmpsub  LNRedUpperThreshold,    LNRedPart wc, nr             ' Figure if red part is in range.
if_c                    cmpsub  LNRedPart,              LNRedLowerThreshold wc, nr   '

if_c                    cmpsub  LNGreenUpperThreshold,  LNGreenPart wc, nr           ' Figure if green part is in range.
if_c                    cmpsub  LNGreenPart,            LNGreenLowerThreshold wc, nr '

if_c                    cmpsub  LNBlueUpperThreshold,   LNBluePart wc, nr            ' Figure if blue part is in range.
if_c                    cmpsub  LNBluePart,             LNBlueLowerThreshold wc, nr  '

                        muxnc   par,                    #_LN_FLAG_TEST_TT            ' Store if pixel passed thresholding.

' //////////////////////Check Filtering////////////////////////////////////////////////////////////////////////////////////////

                        test    LNTrackingPixelSetup,   #_LN_MODE_SETUP_IF wc        ' Apply inverted filter.
if_c                    xor     par,                    #_LN_FLAG_TEST_TT            '

                        cmp     par,                    #1 wc                        ' Apply noise filter.
                        add     LNFilterPixelCount,     #1                           '
if_nc                   mov     LNFilterPixelCount,     #0                           '
                        cmp     LNFilteringPixelSetup,  LNFilterPixelCount wc        '

' //////////////////////Update Calculations////////////////////////////////////////////////////////////////////////////////////

if_c                    add     (LNTAO+_STRUCT_LN_CX),  LNXBoundCounter              ' Track centroid.
if_c                    add     (LNTAO+_STRUCT_LN_CY),  LNYBoundCounter              '

if_c                    max     (LNTAO+_STRUCT_LN_X1),  LNXBoundCounter              ' Track bounding box.
if_c                    max     (LNTAO+_STRUCT_LN_Y1),  LNYBoundCounter              '
if_c                    min     (LNTAO+_STRUCT_LN_X2),  LNXBoundCounter              '
if_c                    min     (LNTAO+_STRUCT_LN_Y2),  LNYBoundCounter              '

if_c                    add     (LNTAO+_STRUCT_LN_TP),  #1                           ' Shift in pixel.
                        rcl     LNDivideBuffer,         #1                           '

' //////////////////////Update Histogram///////////////////////////////////////////////////////////////////////////////////////

                        mov     LNPixelStorage,         LNDivideCounter              ' Sample only a fourth.
                        and     LNPixelStorage,         #3                           '
LNMasterMod             cmp     LNPixelStorage,         #0 wz                        '
if_nz                   jmp     #LNHistogramSkip                                     '

LNHistogramLockLoop     lockset LNMasterSlaveLock wc                                 ' Lock the histogram buffer.
if_c                    jmp     #LNHistogramLockLoop                                 '

                        cmp     LNFilteringPixelSetup,  LNFilterPixelCount wc        ' Reset the C flag.

                        test    LNTrackingPixelSetup,   #_LN_MODE_SETUP_HT wz        ' Accumulate histogram given mode.
if_z                    test    par,                    #_LN_FLAG_TEST_VW wz         '

if_c_or_z               shr     LNRedPart,              #(_R_V_5_COMBINED_SHIFT - 1) ' Accumulate red pixels in histogram.
if_c_or_z               add     LNRedPart,              LNHistogramRedBase           '
if_c_or_z               rdword  cnt,                    LNRedPart                    '
if_c_or_z               add     cnt,                    #1                           '
if_c_or_z               wrword  cnt,                    LNRedPart                    '

if_c_or_z               shr     LNGreenPart,            #(_G_Y_6_COMBINED_SHIFT - 1) ' Accumulate green pixels in histogram.
if_c_or_z               add     LNGreenPart,            LNHistogramGreenBase         '
if_c_or_z               rdword  cnt,                    LNGreenPart                  '
if_c_or_z               add     cnt,                    #1                           '
if_c_or_z               wrword  cnt,                    LNGreenPart                  '

if_c_or_z               shl     LNBluePart,             #(_B_U_5_COMBINED_SHIFT + 1) ' Accumulate blue pixels in histogram.
if_c_or_z               add     LNBluePart,             LNHistogramBlueBase          '
if_c_or_z               rdword  cnt,                    LNBluePart                   '
if_c_or_z               add     cnt,                    #1                           '
if_c_or_z               wrword  cnt,                    LNBluePart                   '

if_c_or_z               add     (LNTAO+_STRUCT_LN_HP),  #1                           ' Unlock the histogram buffer.
                        lockclr LNMasterSlaveLock                                    '

' //////////////////////Loop///////////////////////////////////////////////////////////////////////////////////////////////////

LNHistogramSkip         add     LNXBoundCounter,        #1                           ' Repeat 8 times and increment X and Y.
                        cmpsub  LNXBoundCounter,        #_CAMERA_H_WIN wc            '
if_c                    add     LNYBoundCounter,        #2                           '
                        djnz    LNDivideCounter,        #LNFetchPixelLoop            '

                        wrbyte  LNDivideBuffer,         LNBufferPointer              ' Transfer to main memory.
                        add     LNBufferPointer,        #1                           '

                        djnz    LNColumnCounter,        #LNThresholdPixelLoop        ' Repeat and skip next row.
                        add     LNBufferPointer,        #_LN_BYTES_PER_ROW           '
                        xor     LNMasterMod,            #_LN_MASK_FLIPPER            '

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
'                       Update Data
' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                        djnz    LNRowCounter,           #LNThrottle nr               ' Update calculations at the end.

                        movd    LNMasterUpdateLoop,     #LNTAI                       ' Setup to transfer data.
                        movd    LNSlaveUpdateLoop,      #LNTAO                       '
                        mov     LNDivideBuffer,         LNTrackingArrayAddress       '
                        mov     LNDivideCounter,        #_STRUCT_LN_TA_SIZE          '

LNMasterUpdateLoop      rdlong  LNTAI,                  LNDivideBuffer               ' Read old values.
                        add     LNMasterUpdateLoop,     LNDestinationIncrement       '

LNSlaveUpdateLoop       wrlong  LNTAO,                  LNDivideBuffer               ' Write new values.
                        add     LNSlaveUpdateLoop,      LNDestinationIncrement       '

                        add     LNDivideBuffer,         #4                           ' Loop through result values.
                        djnz    LNDivideCounter,        #LNMasterUpdateLoop          '

LNSlaveMod1 if_never    jmp     #LNSkipThrottle                                      ' Slave finished.

' //////////////////////Combine Results////////////////////////////////////////////////////////////////////////////////////////

                        add     (LNTAO+_STRUCT_LN_CX),  (LNTAI + _STRUCT_LN_CX)      ' Combine results.
                        add     (LNTAO+_STRUCT_LN_CY),  (LNTAI + _STRUCT_LN_CY)      '
                        max     (LNTAO+_STRUCT_LN_X1),  (LNTAI + _STRUCT_LN_X1)      '
                        max     (LNTAO+_STRUCT_LN_Y1),  (LNTAI + _STRUCT_LN_Y1)      '
                        min     (LNTAO+_STRUCT_LN_X2),  (LNTAI + _STRUCT_LN_X2)      '
                        min     (LNTAO+_STRUCT_LN_Y2),  (LNTAI + _STRUCT_LN_Y2)      '
                        add     (LNTAO+_STRUCT_LN_TP),  (LNTAI + _STRUCT_LN_TP)      '
                        add     (LNTAO+_STRUCT_LN_HP),  (LNTAI + _STRUCT_LN_HP)      '

                        add     LNHistogramRedBase,     #(_STRUCT_LN_H_SIZE * 2)     ' Write the histogram size.
                        wrword  (LNTAO+_STRUCT_LN_HP),  LNHistogramRedBase           '

' //////////////////////Preform Calculations///////////////////////////////////////////////////////////////////////////////////

                        rdlong  LNBufferPointer,        LNSwitchAddress wz           ' Choose data buffer to update.
                        mov     LNBufferPointer,        LNTrackDataAddress           '
if_nz                   add     LNBufferPointer,        #_LN_TRACK_DATA_SIZE_BYTES   '

                        mov     LNDivideDividend,       (LNTAO + _STRUCT_LN_CX)      ' Write out X centroid.
                        mov     LNDivideDivsor,         (LNTAO + _STRUCT_LN_TP)      '
                        call    #LNDivide                                            '
                        wrbyte  LNDivideQuotient,       LNBufferPointer              '
                        add     LNBufferPointer,        #1                           '

                        mov     LNDivideDividend,       (LNTAO + _STRUCT_LN_CY)      ' Write out Y centroid.
                        mov     LNDivideDivsor,         (LNTAO + _STRUCT_LN_TP)      '
                        call    #LNDivide                                            '
                        wrbyte  LNDivideQuotient,       LNBufferPointer              '
                        add     LNBufferPointer,        #1                           '

                        cmp     (LNTAO+_STRUCT_LN_TP),  #0 wz                        ' Check to see if tracked pixels is zero.
if_z                    wrword  (LNTAO+_STRUCT_LN_TP),  LNBufferPointer              '

if_nz                   wrbyte  (LNTAO+_STRUCT_LN_X1),  LNBufferPointer              ' Write out X1 bound coordinate.
                        add     LNBufferPointer,        #1                           '

if_nz                   wrbyte  (LNTAO+_STRUCT_LN_Y1),  LNBufferPointer              ' Write out Y1 bound coordinate.
                        add     LNBufferPointer,        #1                           '

                        wrbyte  (LNTAO+_STRUCT_LN_X2),  LNBufferPointer              ' Write out X2 bound coordinate.
                        add     LNBufferPointer,        #1                           '

                        wrbyte  (LNTAO+_STRUCT_LN_Y2),  LNBufferPointer              ' Write out Y2 bound coordinate.
                        add     LNBufferPointer,        #1                           '

                        mov     LNMultiplyMultiplicand, (LNTAO + _STRUCT_LN_TP)      ' Compute pixel count percentage part.
                        mov     LNMultiplyMultiplier,   #255                         '
                        call    #LNMultiply                                          '
                        mov     (LNTAO+_STRUCT_LN_TP),  LNMultiplyProduct            '

                        mov     LNMultiplyMultiplicand, LNWindowX2Bound              ' Compute virtual window area.
                        sub     LNMultiplyMultiplicand, LNWindowX1Bound              '
                        add     LNMultiplyMultiplicand, #1                           '
                        mov     LNMultiplyMultiplier,   LNWindowY2Bound              '
                        sub     LNMultiplyMultiplier,   LNWindowY1Bound              '
                        add     LNMultiplyMultiplier,   #1                           '
                        call    #LNMultiply                                          '

                        mov     LNDivideDividend,       (LNTAO+_STRUCT_LN_TP)        ' Write out tracked pixels.
                        mov     LNDivideDivsor,         LNMultiplyProduct            '
                        call    #LNDivide                                            '
                        cmp     LNDivideDividend,       #0 wz                        '
if_nz                   add     LNDivideQuotient,       #1                           '
                        wrbyte  LNDivideQuotient,       LNBufferPointer              '
                        add     LNBufferPointer,        #1                           '

                        mov     LNMultiplyMultiplicand, (LNTAO + _STRUCT_LN_X2)      ' Compute bounding box area.
                        sub     LNMultiplyMultiplicand, (LNTAO + _STRUCT_LN_X1)      '
                        add     LNMultiplyMultiplicand, #1                           '
                        mov     LNMultiplyMultiplier,   (LNTAO + _STRUCT_LN_Y2)      '
                        sub     LNMultiplyMultiplier,   (LNTAO + _STRUCT_LN_Y1)      '
                        add     LNMultiplyMultiplier,   #1                           '
                        call    #LNMultiply                                          '

                        mov     LNDivideDividend,       (LNTAO+_STRUCT_LN_TP)        ' Write out tracking confidence.
                        mov     LNDivideDivsor,         LNMultiplyProduct            '
                        call    #LNDivide                                            '
                        cmp     LNDivideDividend,       #0 wz                        '
if_nz                   add     LNDivideQuotient,       #1                           '
                        wrbyte  LNDivideQuotient,       LNBufferPointer              '

                        rdlong  LNBufferPointer,        LNSwitchAddress wz           ' Flip buffer switch.
                        sumz    LNBufferPointer,        #1                           '
                        wrlong  LNBufferPointer,        LNSwitchAddress              '
                        jmp     #LNSkipThrottle                                      '

' //////////////////////Skip Lines/////////////////////////////////////////////////////////////////////////////////////////////

LNThrottle              mov     LNDivideBuffer,         #_LN_HALF_V_DIV              ' Make sure HREF's have passed.
LNThrottleLoop          cmp     LNDivideBuffer,         phsa wz, wc                  '
if_nz_and_nc            jmp     #LNThrottleLoop                                      '

' //////////////////////Loop///////////////////////////////////////////////////////////////////////////////////////////////////

LNSkipThrottle          djnz    LNRowCounter,           #LNNextRow                   ' Loop and right shift the sample pattern.
                        add     LNMasterMod,            #_LN_MASK_SHIFTER            '
                        andn    LNMasterMod,            #4                           '
                        jmp     #LNGrabberLoop                                       '

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
'                       Unsigned Divide
' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

LNDivide                mov     LNDivideQuotient,       #0                           ' Setup to divide.
                        mov     LNDivideBuffer,         #0                           '
                        mov     LNDivideCounter,        #32                          '

                        tjz     LNDivideDivsor,         #LNDivide_ret                ' Clear if dividing by zero.

LNDivideLoopPre         shr     LNDivideDivsor,         #1 wc, wz                    ' Align divisor MSB and count size.
                        rcr     LNDivideBuffer,         #1                           '
if_nz                   djnz    LNDivideCounter,        #LNDivideLoopPre             '

LNDivideLoopPost        cmpsub  LNDivideDividend,       LNDivideBuffer wc            ' Preform division.
                        rcl     LNDivideQuotient,       #1                           '
                        shr     LNDivideBuffer,         #1                           '
                        djnz    LNDivideCounter,        #LNDivideLoopPost            '

LNDivide_ret            ret                                                          ' Return. Remainder in dividend on exit.

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
'                       Unsigned Multiply
' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

LNMultiply              mov     LNMultiplyProduct,      #0                           ' Clear product.

LNMultiplyLoop          shr     LNMultiplyMultiplicand, #1 wc                        ' Preform multiplication.
if_c                    add     LNMultiplyProduct,      LNMultiplyMultiplier         '
                        shl     LNMultiplyMultiplier,   #1                           '
                        tjnz    LNMultiplyMultiplicand, #LNMultiplyLoop              '

LNMultiply_ret          ret                                                          ' Return.

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
'                       Data
' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

LNMasterSlaveLock       long    -1
LNRendezvousDelta       long    0
LNRendezvousCount       long    0

' //////////////////////Constants//////////////////////////////////////////////////////////////////////////////////////////////

LNDataBufferSize        long    _LN_BUFFER_SIZE_BYTES
LNDestinationIncrement  long    $200

' //////////////////////Pin Masks//////////////////////////////////////////////////////////////////////////////////////////////

LNHREFMask              long    (|<_CAM_HREF_PIN)
LNVSYNCMask             long    (|<_CAM_VSYNC_PIN)

' //////////////////////Addresses//////////////////////////////////////////////////////////////////////////////////////////////

LNThresholdAddress      long    0
LNWindowAddress         long    0
LNModeSetupAddress      long    0
LNTrackingArrayAddress  long    0
LNTrackDataAddress      long    0
LNHistoDataAddress      long    0
LNSwitchAddress         long    0
LNYScaleLUTAddress      long    0
LNUVScaleLUTAddress     long    0

' //////////////////////Run Time Variables/////////////////////////////////////////////////////////////////////////////////////

LNBufferPointer         res     1
LNColumnCounter         res     1
LNRowCounter            res     1

LNDivideDividend        res     1
LNDivideDivsor          res     1
LNDivideQuotient        res     1
LNMultiplyMultiplicand  res     1
LNMultiplyMultiplier    res     1
LNMultiplyProduct       res     1

LNRedPart               res     1
LNGreenPart             res     1
LNBluePart              res     1

LNHistogramRedBase      res     1
LNHistogramGreenBase    res     1
LNHistogramBlueBase     res     1

LNRedUpperThreshold     res     1
LNRedLowerThreshold     res     1

LNGreenUpperThreshold   res     1
LNGreenLowerThreshold   res     1

LNBlueUpperThreshold    res     1
LNBlueLowerThreshold    res     1

LNWindowX1Bound         res     1
LNWindowX2Bound         res     1

LNWindowY1Bound         res     1
LNWindowY2Bound         res     1

LNXBoundCounter         res     1
LNYBoundCounter         res     1

LNTrackingPixelSetup    res     1
LNFilteringPixelSetup   res     1
LNFilterPixelCount      res     1

LNTAO                   res     _STRUCT_LN_TA_SIZE ' long 0 - [cx, cy, x1, y1, x2, y2, tp, hp] - long 7.
LNTAI
LNPixelStorage          res     _CAMERA_H_WIN ' long 0 - [pixel0, pixel1, ..., pixel159] - long 159.

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                        fit     496

DAT

' //////////////////////YUV Look Up Tables/////////////////////////////////////////////////////////////////////////////////////

YScaleLUT               byte 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00
                        byte 00, 01, 01, 01, 02, 02, 02, 03, 03, 03, 03, 04, 04, 04, 05, 05
                        byte 05, 05, 06, 06, 06, 07, 07, 07, 07, 08, 08, 08, 09, 09, 09, 09
                        byte 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 13, 13, 13, 13, 14, 14
                        byte 14, 15, 15, 15, 15, 16, 16, 16, 17, 17, 17, 17, 18, 18, 18, 19
                        byte 19, 19, 19, 20, 20, 20, 21, 21, 21, 21, 22, 22, 22, 23, 23, 23
                        byte 24, 24, 24, 24, 25, 25, 25, 26, 26, 26, 26, 27, 27, 27, 28, 28
                        byte 28, 28, 29, 29, 29, 30, 30, 30, 30, 31, 31, 31, 32, 32, 32, 32
                        byte 33, 33, 33, 34, 34, 34, 34, 35, 35, 35, 36, 36, 36, 36, 37, 37
                        byte 37, 38, 38, 38, 38, 39, 39, 39, 40, 40, 40, 40, 41, 41, 41, 42
                        byte 42, 42, 42, 43, 43, 43, 44, 44, 44, 45, 45, 45, 45, 46, 46, 46
                        byte 47, 47, 47, 47, 48, 48, 48, 49, 49, 49, 49, 50, 50, 50, 51, 51
                        byte 51, 51, 52, 52, 52, 53, 53, 53, 53, 54, 54, 54, 55, 55, 55, 55
                        byte 56, 56, 56, 57, 57, 57, 57, 58, 58, 58, 59, 59, 59, 59, 60, 60
                        byte 60, 61, 61, 61, 61, 62, 62, 62, 63, 63, 63, 63, 63, 63, 63, 63
                        byte 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63

                        ' repeat x from 0 to 255 ' This calculation rounds up.
                        '   y := ((((((x <# 235) #> 16) - 16) * 63) + (235 - 16 - 1)) / (235 - 16))

UVScaleLUT              byte 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00
                        byte 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00
                        byte 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00
                        byte 00, 00, 00, 00, 00, 01, 01, 01, 01, 01, 02, 02, 02, 02, 02, 03
                        byte 03, 03, 03, 03, 04, 04, 04, 04, 04, 05, 05, 05, 05, 05, 06, 06
                        byte 06, 06, 06, 07, 07, 07, 07, 07, 08, 08, 08, 08, 08, 09, 09, 09
                        byte 09, 09, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 12, 12, 12, 12
                        byte 12, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15
                        byte 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 19
                        byte 19, 19, 19, 19, 20, 20, 20, 20, 20, 21, 21, 21, 21, 21, 22, 22
                        byte 22, 22, 22, 23, 23, 23, 23, 23, 24, 24, 24, 24, 24, 25, 25, 25
                        byte 25, 25, 26, 26, 26, 26, 26, 27, 27, 27, 27, 27, 28, 28, 28, 28
                        byte 28, 29, 29, 29, 29, 29, 30, 30, 30, 30, 30, 31, 31, 31, 31, 31
                        byte 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31
                        byte 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31
                        byte 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31

                        ' repeat x from 0 to 255 ' This calculation rounds down.
                        '   y := (((x <# 240) #> 16) - 128)
                        '
                        '   case y
                        '
                        '     -112 .. -081: y := 0
                        '     -080 .. +079: y := ((y / 5) + 16)
                        '     +080 .. +112: y := 31

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CON #0-1, _LN_TRACKING_DATA_SOURCE, _LN_HISTOGRAM_DATA_SOURCE, _LN_STATISTICS_DATA_SOURCE ' Source constants.

  _STRUCT_LN_TA_SIZE = 8 ' Tracking Array.

  _STRUCT_LN_CX = 0 ' Centroid X Sum.
  _STRUCT_LN_CY = 1 ' Centroid Y Sum.
  _STRUCT_LN_X1 = 2 ' X1 Tracked Pixel.
  _STRUCT_LN_Y1 = 3 ' Y1 Tracked Pixel.
  _STRUCT_LN_X2 = 4 ' X2 Tracked Pixel.
  _STRUCT_LN_Y2 = 5 ' Y2 Tracked Pixel.
  _STRUCT_LN_TP = 6 ' Tracked Pixels.
  _STRUCT_LN_HP = 7 ' Histogram Pixels.

  _LN_SHORT_SYNC_OFFSET = 384 ' 344 cycle count. 40 cyle buffer.
  _LN_LONG_SYNC_OFFSET = _LN_SHORT_SYNC_OFFSET + 2_500 ' 1,940 cycle count. 560 cyle buffer.

  _LN_COGNEW_TIME = 10_000 ' Resolves into (clkfreq / timeout) seconds of timeout which is about 100 microseconds.

VAR long LNMasterCog, LNSlaveCog, LNTrackingArray[_STRUCT_LN_TA_SIZE] ' Cog stuff.

PRI lineGrabberDriverStart ' Initializes the line grabber driver. Returns true on success and false on failure.

  lineGrabberDriverStop

  LNThresholdAddress := @LNThreshold
  LNWindowAddress := @LNWindow
  LNModeSetupAddress := @LNModeSetup
  LNTrackingArrayAddress := @LNTrackingArray
  LNTrackDataAddress := @LNTrackData
  LNHistoDataAddress := @LNHistogramData
  LNSwitchAddress := @LNSwitch
  LNYScaleLUTAddress := @YScaleLUT
  LNUVScaleLUTAddress := @UVScaleLUT

  if((chipver == 1) and (!(LNMasterSlaveLock := locknew)))
    lockclr(LNMasterSlaveLock) ' Setup the lock.

    LNRendezvousDelta := ((clkfreq / _FRAME_RATE) - _LN_LONG_SYNC_OFFSET)
    LNRendezvousCount := ((clkfreq / constant(_LN_COGNEW_TIME / 4)) + cnt)

    ' Initialize Slave.
    LNSlaveMod0 |= _LN_IF_ALWAYS
    LNSlaveMod1 |= _LN_IF_ALWAYS
    LNYBoundOffset.byte := _LN_SLAVE_OFFSET
    LNMasterMod.byte := _LN_SLAVE_DISPLACEMENT
    LNSlaveCog := cognew(@LNGrabberInit, @LNBuffer)

    if(++LNSlaveCog)
      waitcnt((clkfreq / _LN_COGNEW_TIME) + cnt) ' Wait for slave to start.

      ' Initialize Master.
      LNSlaveMod0 &= _LN_IF_NEVER
      LNSlaveMod1 &= _LN_IF_NEVER
      LNYBoundOffset.byte := _LN_MASTER_OFFSET
      LNMasterMod.byte := _LN_MASTER_DISPLACEMENT
      LNMasterCog := cognew(@LNGrabberInit, (@LNBuffer + _LN_BYTES_PER_ROW))

      if(++LNMasterCog)
        return true

  lineGrabberDriverStop

PRI lineGrabberDriverStop ' Shutsdown the line grabber driver.

  if(LNMasterCog)
    cogstop(-1 + LNMasterCog~)

  if(LNSlaveCog)
    cogstop(-1 + LNSlaveCog~)

  if(!LNMasterSlaveLock)
    lockret(LNMasterSlaveLock)

PRI printNewData(source, arg0, arg1) | flag, temp, SB[_STRUCT_HS_S_LONGS],HB[_LN_HISTO_DATA_SIZE_LONGS],DB[_LN_DS_SIZE_LONGS]

  repeat ' Until enter is pressed.
    waitForNewData(_FRAME_WAIT)

    if(lineModeSetup)
      flag := 0

      DSOutputAddress := @DB ' Setup down sampling.
      DSInputAddress := (@LNBuffer + (_LN_BUFFER_SIZE_BYTES & (not(LNSwitch))))
      result := (cognew(@DSDriverInit, @flag) <> -1) ' Launch a cog to down sample in the background - never fails.

    case source ' Choose source.
      _LN_TRACKING_DATA_SOURCE:
        temp := not(printTrackData)
        ifnot(pollModeSetup)
          powerLEDOn(temp)

      _LN_HISTOGRAM_DATA_SOURCE:
        getHistogramData(@HB)
        printHistogramData(arg0, arg1, @HB)

      _LN_STATISTICS_DATA_SOURCE:
        getHistogramData(@HB)
        getHistogramStatistics(@SB, @HB)
        printHistogramStatistics(@SB)

    if(lineModeSetup and result)
      repeat ' Format saves two bytes.
      until(flag) ' Wait for driver to finish - the flag will go high.

      com.writeString(string("F "))
      com.writeData(@DB, _LN_DS_SIZE_BYTES)
      com.writeByte(_NEWLINE)

    if(pollModeSetup)
      return 0

    if(switchingModeSetup)
      case source

        _LN_TRACKING_DATA_SOURCE: source := _LN_STATISTICS_DATA_SOURCE
        _LN_STATISTICS_DATA_SOURCE: source := _LN_TRACKING_DATA_SOURCE

    repeat com.receivedNumber
      if(com.readByte == _NEWLINE)
        return 0

PRI waitForNewData(count) | buffer, counter

  repeat count
    counter := cnt
    buffer := LNSwitch
    repeat while(buffer == LNSwitch)
      if((cnt - counter) => (clkfreq / _FRAME_TIMEOUT))
        abort @cameraTimeoutError

CON ' Constants for mode setup.

  _LN_MODE_SETUP_CT = $1 ' Camera color mode bit position.
  _LN_MODE_SETUP_HT = $2 ' Camera histogram mode bit position.
  _LN_MODE_SETUP_IF = $4 ' Camera inverted filter bit position.

  _LN_MODE_SETUP_NF = 8 ' Camera noise filter byte position.

  _LN_FLAG_TEST_VW = $1 ' Camera virtual window test passed flag.
  _LN_FLAG_TEST_TT = $2 ' Camera tracking threshold test passed flag.

VAR long LNModeSetup, LNSwitch ' Driver mode and driver filtering setup.

PRI setupColorTracking(onOrOff) ' True for YUV and false for RGB.

  LNModeSetup := ((LNModeSetup & constant(!_LN_MODE_SETUP_CT)) | ((not(not(onOrOff))) & _LN_MODE_SETUP_CT))

PRI setupHistogramTracking(onOrOff) ' True for binning only tracked pixels and false for binning all pixels.

  LNModeSetup := ((LNModeSetup & constant(!_LN_MODE_SETUP_HT)) | ((not(not(onOrOff))) & _LN_MODE_SETUP_HT))

PRI setupInvertedFilter(onOrOff) ' True for on and false for off.

  LNModeSetup := ((LNModeSetup & constant(!_LN_MODE_SETUP_IF)) | ((not(not(onOrOff))) & _LN_MODE_SETUP_IF))

PRI setupNoiseFilter(value) ' Setup the noise filter - between 0 and 255.

  LNModeSetup.byte[constant(_LN_MODE_SETUP_NF / 8)] := value

CON ' Data locations for tracking threshold structure.

  _STRUCT_TT_R_V_5_MIN = _R_V_5_COMBINED_SHIFT
  _STRUCT_TT_G_Y_6_MIN = _G_Y_6_COMBINED_SHIFT
  _STRUCT_TT_B_U_5_MIN = _B_U_5_COMBINED_SHIFT

  _STRUCT_TT_R_V_5_MAX = ((_R_V_5 + _G_Y_6 + _B_U_5) + _STRUCT_TT_R_V_5_MIN)
  _STRUCT_TT_G_Y_6_MAX = ((_R_V_5 + _G_Y_6 + _B_U_5) + _STRUCT_TT_G_Y_6_MIN)
  _STRUCT_TT_B_U_5_MAX = ((_R_V_5 + _G_Y_6 + _B_U_5) + _STRUCT_TT_B_U_5_MIN)

DAT ' Data struct.

  LNThreshold long (   (_R_V_5_MAX << _STRUCT_TT_R_V_5_MAX) {
                   } | (_G_Y_6_MAX << _STRUCT_TT_G_Y_6_MAX) {
                   } | (_B_U_5_MAX << _STRUCT_TT_B_U_5_MAX) {
                   } | (_R_V_5_MIN << _STRUCT_TT_R_V_5_MIN) {
                   } | (_G_Y_6_MIN << _STRUCT_TT_G_Y_6_MIN) {
                   } | (_B_U_5_MIN << _STRUCT_TT_B_U_5_MIN) )

PRI setTrackingThresholds(redLow, redHigh, greenLow, greenHigh, blueLow, blueHigh) ' Sets color tracking thresholds.

  ' NOTE: This function rounds the input up...

  LNThreshold := ( (((((((redHigh   <# redLow)    <# _R_V_8_MAX) #> _R_V_8_MIN) * _R_V_5_MAX) {
                 } + _R_V_8_ROUND)  / _R_V_8_MAX) << _STRUCT_TT_R_V_5_MIN)      |             { ' Choose smallest.

                 } (((((((redLow    #> redHigh)   <# _R_V_8_MAX) #> _R_V_8_MIN) * _R_V_5_MAX) {
                 } + _R_V_8_ROUND)  / _R_V_8_MAX) << _STRUCT_TT_R_V_5_MAX)      |             { ' Choose largest.

                 } (((((((greenHigh <# greenLow)  <# _G_Y_8_MAX) #> _G_Y_8_MIN) * _G_Y_6_MAX) {
                 } + _G_Y_8_ROUND)  / _G_Y_8_MAX) << _STRUCT_TT_G_Y_6_MIN)      |             { ' Choose smallest.

                 } (((((((greenLow  #> greenHigh) <# _G_Y_8_MAX) #> _G_Y_8_MIN) * _G_Y_6_MAX) {
                 } + _G_Y_8_ROUND)  / _G_Y_8_MAX) << _STRUCT_TT_G_Y_6_MAX)      |             { ' Choose largest.

                 } (((((((blueHigh  <# blueLow)   <# _B_U_8_MAX) #> _B_U_8_MIN) * _B_U_5_MAX) {
                 } + _B_U_8_ROUND)  / _B_U_8_MAX) << _STRUCT_TT_B_U_5_MIN)      |             { ' Choose smallest.

                 } (((((((blueLow   #> blueHigh)  <# _B_U_8_MAX) #> _B_U_8_MIN) * _B_U_5_MAX) {
                 } + _B_U_8_ROUND)  / _B_U_8_MAX) << _STRUCT_TT_B_U_5_MAX)                     ) ' Choose largest.

PRI getTrackingThresholds ' Gets color tracking thresholds.

  ' NOTE: This function rounds the output down...

  com.writeString(DECOut((((LNThreshold >> _STRUCT_TT_R_V_5_MIN) & _R_V_5_MASK) * _R_V_8_MAX) / _R_V_5_MAX)) ' Red min.
  com.writeByte(" ")
  com.writeString(DECOut((((LNThreshold >> _STRUCT_TT_R_V_5_MAX) & _R_V_5_MASK) * _R_V_8_MAX) / _R_V_5_MAX)) ' Red max.
  com.writeByte(" ")
  com.writeString(DECOut((((LNThreshold >> _STRUCT_TT_G_Y_6_MIN) & _G_Y_6_MASK) * _G_Y_8_MAX) / _G_Y_6_MAX)) ' Green min.
  com.writeByte(" ")
  com.writeString(DECOut((((LNThreshold >> _STRUCT_TT_G_Y_6_MAX) & _G_Y_6_MASK) * _G_Y_8_MAX) / _G_Y_6_MAX)) ' Green max.
  com.writeByte(" ")
  com.writeString(DECOut((((LNThreshold >> _STRUCT_TT_B_U_5_MIN) & _B_U_5_MASK) * _B_U_8_MAX) / _B_U_5_MAX)) ' Blue min.
  com.writeByte(" ")
  com.writeString(DECOut((((LNThreshold >> _STRUCT_TT_B_U_5_MAX) & _B_U_5_MASK) * _B_U_8_MAX) / _B_U_5_MAX)) ' Blue max.
  com.writeByte(_NEWLINE)

CON ' Data locations for virtual window structure.

  _STRUCT_VW_X1 = 0
  _STRUCT_VW_Y1 = 1
  _STRUCT_VW_X2 = 2
  _STRUCT_VW_Y2 = 3

  _STRUCT_SHIFT_VW_X1 = (_STRUCT_VW_X1 * 8)         '_STRUCT_SHIFT_VW_X1 = 0
  _STRUCT_SHIFT_VW_Y1 = (_STRUCT_VW_Y1 * 8)         '_STRUCT_SHIFT_VW_Y1 = 8
  _STRUCT_SHIFT_VW_X2 = (_STRUCT_VW_X2 * 8)         '_STRUCT_SHIFT_VW_X2 = 16
  _STRUCT_SHIFT_VW_Y2 = (_STRUCT_VW_Y2 * 8)         '_STRUCT_SHIFT_VW_Y2 = 24

  _STRUCT_VW_START = _STRUCT_VW_X1
  _STRUCT_VW_END = _STRUCT_VW_Y2

DAT ' Data struct.

  LNWindow long ( (_CAMERA_MAX_V << _STRUCT_SHIFT_VW_Y2) | (_CAMERA_MAX_H << _STRUCT_SHIFT_VW_X2) | {
                } (_CAMERA_MIN_V << _STRUCT_SHIFT_VW_Y1) | (_CAMERA_MIN_H << _STRUCT_SHIFT_VW_X1)   )
                '_CAMERA_MAX_V = 119;_STRUCT_SHIFT_VW_Y2 = 24   _CAMERA_MAX_H = 159;_STRUCT_SHIFT_VW_X2 = 16 
                '_CAMERA_MIN_V = 0;  _STRUCT_SHIFT_VW_Y1 = 8    _CAMERA_MIN_H = 0;  _STRUCT_SHIFT_VW_X1 = 0

PRI setVirtualWindow(X1Bound, Y1Bound, X2Bound, Y2Bound) ' Sets virtual window bounds.

  LNWindow := ( ((((X2Bound <# X1Bound) <# _CAMERA_MAX_H) #> _CAMERA_MIN_H) << _STRUCT_SHIFT_VW_X1) | { ' Choose smallest.

              } ((((Y2Bound <# Y1Bound) <# _CAMERA_MAX_V) #> _CAMERA_MIN_V) << _STRUCT_SHIFT_VW_Y1) | { ' Choose smallest.

              } ((((X1Bound #> X2Bound) <# _CAMERA_MAX_H) #> _CAMERA_MIN_H) << _STRUCT_SHIFT_VW_X2) | { ' Choose largest.

              } ((((Y1Bound #> Y2Bound) <# _CAMERA_MAX_V) #> _CAMERA_MIN_V) << _STRUCT_SHIFT_VW_Y2)   ) ' Choose largest.

PRI getVirtualWindow | index ' Gets virtual window bounds.

  repeat index from _STRUCT_VW_START to _STRUCT_VW_END
    com.writeString(DECOut(LNWindow.byte[index]))

    if(index < _STRUCT_VW_END)
      com.writeByte(" ")

    else
      com.writeByte(_NEWLINE)

CON ' Tracking data buffer setup.

  _LN_TRACK_DATA_SIZE_BYTES = 8
  _LN_TRACK_DATA_SIZE_LONGS = ((_LN_TRACK_DATA_SIZE_BYTES + 3) / 4)

  _LN_TRACK_DATA_SIZE_D_BYTES = (_LN_TRACK_DATA_SIZE_BYTES * 2) ' Double Buffered size in longs.
  _LN_TRACK_DATA_SIZE_D_LONGS = ((_LN_TRACK_DATA_SIZE_D_BYTES + 3) / 4) ' Double Buffered size in longs.

  _STRUCT_LN_T_X_CENTROID = 0
  _STRUCT_LN_T_Y_CENTROID = 1

  _STRUCT_LN_T_X1_BOUND = 2
  _STRUCT_LN_T_Y1_BOUND = 3
  _STRUCT_LN_T_X2_BOUND = 4
  _STRUCT_LN_T_Y2_BOUND = 5

  _STRUCT_LN_T_TRACKED_PIXELS = 6
  _STRUCT_LN_T_TRACK_CONFIDENCE = 7

  _STRUCT_LN_T_START = _STRUCT_LN_T_X_CENTROID
  _STRUCT_LN_T_END = _STRUCT_LN_T_TRACK_CONFIDENCE

VAR long LNTrackData[_LN_TRACK_DATA_SIZE_D_LONGS] ' Double Buffered.

' byte 0 - [XCentroid, YCentroid, X1Bound, Y1Bound, X2Bound, Y2Bound, trackedPixels, trackingConfidence] - byte 7.

PRI printTrackData | index, trackingDataBuffer[_LN_TRACK_DATA_SIZE_LONGS] ' Prints the track data buffer.

  getTrackData(@trackingDataBuffer)

  com.writeString(string("T "))

  repeat index from _STRUCT_LN_T_START to _STRUCT_LN_T_END
    com.writeString(DECOut(trackingDataBuffer.byte[index]))

    if(index < _STRUCT_LN_T_END)
      com.writeByte(" ")

    else
      com.writeByte(_NEWLINE)

  return trackingDataBuffer.byte[_STRUCT_LN_T_TRACKED_PIXELS]

PRI getTrackData(dataBufferAddress) ' Gets the track data buffer.

  bytemove(dataBufferAddress, (@LNTrackData + ((not(LNSwitch)) & _LN_TRACK_DATA_SIZE_BYTES)), _LN_TRACK_DATA_SIZE_BYTES)

CON ' Histogram data buffer setup.

  _LN_HISTO_DATA_SIZE_WORDS = (_R_V_5_DECODED + _G_Y_6_DECODED + _B_U_5_DECODED + 1)
  _LN_HISTO_DATA_SIZE_LONGS = ((_LN_HISTO_DATA_SIZE_WORDS + 1) / 2)
  _LN_HISTO_DATA_SIZE_BYTES = (_LN_HISTO_DATA_SIZE_WORDS * 2)

  _LN_HISTO_DATA_SIZE_D_WORDS = (_LN_HISTO_DATA_SIZE_WORDS * 2) ' Double Buffered size in bytes.
  _LN_HISTO_DATA_SIZE_D_LONGS = ((_LN_HISTO_DATA_SIZE_D_WORDS + 1) / 2) ' Double Buffered size in longs.

  _STRUCT_LN_H_R_V_5 = ((_R_V_5_DECODED * 0) + (_G_Y_6_DECODED * 0) + (_B_U_5_DECODED * 0)) ' Red histogram base.    '= %00000000 00000000 00000000 00110000
  _STRUCT_LN_H_G_Y_6 = ((_R_V_5_DECODED * 1) + (_G_Y_6_DECODED * 0) + (_B_U_5_DECODED * 0)) ' Green histogram base.  '= %00000000 00000000 00000000 00110001
  _STRUCT_LN_H_B_U_5 = ((_R_V_5_DECODED * 1) + (_G_Y_6_DECODED * 1) + (_B_U_5_DECODED * 0)) ' Blue histogram base.   '= %00000000 00000000 00000000 00110010

  _STRUCT_LN_H_SIZE = (_R_V_5_DECODED + _G_Y_6_DECODED + _B_U_5_DECODED) ' Histogram size.     '= 5+6+5 =16

VAR long LNHistogramData[_LN_HISTO_DATA_SIZE_D_LONGS] ' Double Buffered.

' word 0 - [RVBin0, ..., RVBin31, GYBin0, ..., GYBin63, BUBin0, ..., BUBin31, binSize] - word 128.

PRI printHistogramData(colorChannel, binNumber, histogramBufferAddress) | x, y, z, i, j, k ' Prints the histogram data buffer.

  com.writeString(string("H "))

  z := lookupz(colorChannel: _STRUCT_LN_H_R_V_5, _STRUCT_LN_H_G_Y_6, _STRUCT_LN_H_B_U_5) ' Bin base.
  k := ((j := ((i := lookupz(colorChannel: _R_V_5_DECODED, _G_Y_6_DECODED, _B_U_5_DECODED)) >> binNumber)) - 1)

  y := 0
  repeat (1 << binNumber)

    repeat x from 0 to k ' Accumulate some bins.
      result += word[histogramBufferAddress][x + y + z]

    result := ((result * 255) / word[histogramBufferAddress][_STRUCT_LN_H_SIZE]) ' Compute percentage...     
    com.writeString(DECOut(result~))

    if((y += j) < i) ' Print the space between packets.
      com.writeByte(" ")

  com.writeByte(_NEWLINE)

PRI getHistogramData(dataBufferAddress) ' Gets the histogram data buffer.

  wordmove(dataBufferAddress, (@LNHistogramData + ((not(LNSwitch)) & _LN_HISTO_DATA_SIZE_BYTES)), _LN_HISTO_DATA_SIZE_WORDS)

CON ' Statistics data buffer setup.

  _STRUCT_HS_S_BYTES = 12
  _STRUCT_HS_S_WORDS = ((_STRUCT_HS_S_BYTES + 1) / 2)
  _STRUCT_HS_S_LONGS = ((_STRUCT_HS_S_BYTES + 3) / 4)

  _STRUCT_HS_MEAN = 0
  _STRUCT_HS_MEDIAN = 3
  _STRUCT_HS_MODE = 6
  _STRUCT_HS_STDEV = 9

  #0, _STRUCT_HS_R_V_BASE, _STRUCT_HS_G_Y_BASE, _STRUCT_HS_B_U_BASE      '_STRUCT_HS_R_V_BASE = 0, _STRUCT_HS_G_Y_BASE = 1, _STRUCT_HS_B_U_BASE = 2 

  _STRUCT_HS_START = (_STRUCT_HS_R_V_BASE + _STRUCT_HS_MEAN)   ' = 0
  _STRUCT_HS_END = (_STRUCT_HS_B_U_BASE + _STRUCT_HS_STDEV)    ' = 11

' byte 0 - [RVMean, GY..., BU..., RVMedian, GY..., BU..., RVMode, GY..., BU..., RVStDev, GY..., BU...] - byte 11.

PRI printHistogramStatistics(statisticsBufferAddress)

  com.writeString(string("S "))

  repeat result from _STRUCT_HS_START to _STRUCT_HS_END
    com.writeString(DECOut(byte[statisticsBufferAddress][result]))

    if(result < _STRUCT_HS_END)
      com.writeByte(" ")

    else
      com.writeByte(_NEWLINE)

  result := 0 ' Ensure nothing is returned.

PRI getHistogramStatistics(statisticsBufferAddress, histogramBufferAddress) | x, y, z, i, j, k, MO, MW, ML, MH, NL, NH

  result := word[histogramBufferAddress][_STRUCT_LN_H_SIZE]           '_STRUCT_LN_H_SIZE = 16
  NL := (((result - 1) >> 1) + 1) ' ((N - 1) / 2) Lower median position.
  NH := ((result >> 1) + 1) ' (N / 2) Upper median position.

  bytefill(statisticsBufferAddress, 0, _STRUCT_HS_S_BYTES)            '_STRUCT_HS_S_BYTES = 12
  repeat result from _STRUCT_HS_R_V_BASE to _STRUCT_HS_B_U_BASE       '_STRUCT_HS_R_V_BASE =0, _STRUCT_HS_B_U_BASE = 2 
    y := lookupz(result: _STRUCT_LN_H_R_V_5, _STRUCT_LN_H_G_Y_6, _STRUCT_LN_H_B_U_5) ' Bin start.
     '= %00000000 00000000 00000000 00110000    %00000000 00000000 00000000 00110001     %00000000 00000000 00000000 00110010
    z := MH := lookupz(result: _R_V_5_MAX, _G_Y_6_MAX, _B_U_5_MAX) ' Bin size minus one.   ' _R_V_5_MAX = 15, _G_Y_6_MAX = 31, _B_U_5_MAX = 15

    j := k := MO := 0
    repeat x from 0 to z
      i := word[histogramBufferAddress][x + y] ' Observations...
      j += (i * x) ' Weighted observation sum...

      if(i) ' If there were observations...

        if((k < NL) and (NL =< (k + i)))
          ML := x ' Find the lower median value.

        if(NH =< (k + i))
          MH <#= x ' Find the upper median value.

      if(MO < i) ' If there was a higher number of observations...
        MO := i ' Remember the highest number of observations.
        MW := x ' Remember the highest weight.

      k += i ' Observation sum...
    if(k~) ' If there were observations...
      i := lookupz(result: _R_V_8_MAX, _G_Y_8_MAX, _B_U_8_MAX) ' Scale value.

      j /= word[histogramBufferAddress][_STRUCT_LN_H_SIZE] ' Arithmetic mean.
      byte[statisticsBufferAddress][result + _STRUCT_HS_MEAN] := ((j * i) / z) ' Scaled mean.
      byte[statisticsBufferAddress][result + _STRUCT_HS_MEDIAN] := ((((ML + MH) >> 1) * i) / z) ' Average medians and scale.
      byte[statisticsBufferAddress][result + _STRUCT_HS_MODE] := ((MW * i) / z) ' Scale mode.

      repeat x from 0 to z ' Compute squared differenced sum.
        k += (word[histogramBufferAddress][x + y] * ((x - j) * (x - j)))

      k /= word[histogramBufferAddress][_STRUCT_LN_H_SIZE] ' Variance.
      byte[statisticsBufferAddress][result + _STRUCT_HS_STDEV] := (((^^k) * i) / z) ' Scaled standard deviation.

  result := 0 ' Ensure nothing is returned.

DAT

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
'                       TV Driver
' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                        org     0

' //////////////////////Initialization/////////////////////////////////////////////////////////////////////////////////////////

TVDriverInitialization  rdlong  TVBuffer,           #_MEMORY_TV_CLKFREQ_ADDRESS         ' Grab the clock frequency.
                        mov     TVCounter,          #32                                 '

TVDriverInitLoop        shl     NTSCFrequencySetup, #1                                  ' Do (((freqSetup << 32) / clkfreq).
                        cmpsub  NTSCFrequencySetup, TVBuffer wc                         '
                        rcl     TVNTSCAccumulator,  #1                                  '

                        shl     PALFrequencySetup,  #1                                  ' Do (((freqSetup << 32) / clkfreq).
                        cmpsub  PALFrequencySetup,  TVBuffer wc                         '
                        rcl     TVPALAccumulator,   #1                                  '

                        djnz    TVCounter,          #TVDriverInitLoop                   ' Loop.

                        cmp     NTSCFrequencySetup, #0 wz                               ' Round up.
if_nz                   add     TVNTSCAccumulator,  #1                                  '

                        cmp     PALFrequencySetup,  #0 wz                               ' Round up.
if_nz                   add     TVPALAccumulator,   #1                                  '

                        mov     vcfg,               TVVideoSetup                        ' Setup video hardware.

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
'                       Inactive Video
' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TVLoop                  rdbyte  TVTM,               TVModeOutputAddress wz              ' Choose the output format.
                        movd    TVModeSwitchLoop,   #TVTA                               '

if_z                    movs    TVModeSwitchLoop,   #NTSCTimingArray                    ' Choose the settings to use.
if_nz                   movs    TVModeSwitchLoop,   #PALTimingArray                     '

                        mov     TVCounter,          #_STRUCT_TV_TA_SIZE                 ' Setup to transfer.

TVModeSwitchLoop        mov     TVTA,               0                                   ' Transfer new settings.
                        add     TVModeSwitchLoop,   TVDestinationIncrement              '
                        add     TVModeSwitchLoop,   #1                                  '
                        djnz    TVCounter,          #TVModeSwitchLoop                   '

if_z                    mov     frqa,               TVNTSCAccumulator                   ' Switch counter mode.
if_z                    mov     ctra,               NTSCCounterSetup                    '
if_nz                   mov     frqa,               TVPALAccumulator                    '
if_nz                   mov     ctra,               PALCounterSetup                     '

' //////////////////////Blank Lines////////////////////////////////////////////////////////////////////////////////////////////

                        mov     TVCounter,          (TVTA + _STRUCT_TV_LINE_COUNT)      ' Setup to do blank lines.

                        test    TVPhaseCounter,     #1 wc                               ' Remove one line for odd PAL fields.
                        cmp     TVTM,               #0 wz                               '
if_c_and_nz             sub     TVCounter,          #1                                  '

                        mov     TVSubProcessPtr,    #TVTransferData                     ' Setup background transfer.

TVBlankLinesLoop        mov     vscl,               (TVTA + _STRUCT_TV_SYNC_TIP)        ' Preform blank line sync.
                        waitvid TVSyncColors,       #0                                  '

                        mov     vscl,               (TVTA + _STRUCT_TV_SERRATION_LOW)   ' First part of blank line.
                        waitvid TVBlankColors,      #0                                  '

                        jmpret  TVMainProcessPtr,   TVSubProcessPtr                     ' Background transfer.

                        mov     vscl,               (TVTA + _STRUCT_TV_HALF_LINE)       ' Second part of blank line.
                        waitvid TVBlankColors,      #0                                  '

                        jmpret  TVMainProcessPtr,   TVSubProcessPtr                     ' Background transfer.

                        djnz    TVCounter,          #TVBlankLinesLoop                   ' Repeat. Set output.
                        mov     dira,               TVDirectionSetup                    '

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
'                       Active Video
' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                        mov     TVWordBoundX1,      TVBoundX1                           ' Setup compare flags.
                        shr     TVWordBoundX1,      #4                                  '
                        mov     TVWordBoundX2,      TVBoundX2                           '
                        shr     TVWordBoundX2,      #4                                  '
                        mov     TVWordCentroidB,    TVCentroidX                         '
                        shr     TVWordCentroidB,    #4                                  '

                        mov     TVPointer,          par                                 ' Reset pointers.
                        mov     TVCounter,          (TVTA + _STRUCT_TV_VIDEO_LINES)     '
                        mov     TVRowCounter,       #0                                  '
                        mov     TVColumnCounter,    #0                                  '

' //////////////////////Horizontal Sync////////////////////////////////////////////////////////////////////////////////////////

TVVideoLoop             mov     vscl,               (TVTA + _STRUCT_TV_SYNC_TIP)        ' Blank for sync tip.
                        waitvid TVSyncColors,       #0                                  '

' //////////////////////Color Burst////////////////////////////////////////////////////////////////////////////////////////////

TVLiveVideoColors       mov     TVColorBuffer,      0                                   ' Setup new color.

                        mov     TVBuffer,           NTSCBurstColors                     ' Choose mode - NTSC or PAL.
                        tjz     TVTM,               #TVColorBurst                       '

                        test    TVCounter,          #1 wc                               ' Even or odd, field or frame.
                        test    TVPhaseCounter,     #2 wz                               '

                        mov     TVBuffer,           PALBurstColors                      ' Switch color burst phase.
if_c_eq_z               xor     TVBuffer,           TVPhaseRotate                       '
if_c_eq_z               xor     TVColorBuffer,      TVPhaseRotate                       '

TVColorBurst            mov     vscl,               (TVTA + _STRUCT_TV_BREEZE_WAY)      ' Blank for breeze way.
                        waitvid TVBlankColors,      #0                                  '

                        mov     vscl,               (TVTA + _STRUCT_TV_COLOR_BURST)     ' Modulate for color burst.
                        waitvid TVBuffer,           #0                                  '

' //////////////////////Live Video/////////////////////////////////////////////////////////////////////////////////////////////

                        mov     vscl,               (TVTA + _STRUCT_TV_BACK_PORCH)      ' Blank for back porch.
                        waitvid TVBlankColors,      #0                                  '

                        mov     vscl,               (TVTA + _STRUCT_TV_LIVE_VIDEO)      ' Setup to stream out live video.
                        mov     TVBuffer,           #_TV_WORDS_PER_LINE                 '

                        cmp     TVTM,               #0 wz                               ' Setup to blank PAL overscan lines.
                        cmp     TVCounter,          #(_TV_BLANK_PAL_LINES_BELOW+1)wc    '
if_nc                   cmpsub  TVCounter,          #(_TV_BLANK_PAL_LINES_ABOVE+1)wc,nr '

TVBlankLoop if_c_and_nz waitvid TVBlankColors,      #0                                  ' Blank video lines.
if_c_and_nz             djnz    TVBuffer,           #TVBlankLoop                        '
if_c_and_nz             jmp     #TVLiveVideoSkip                                        '

TVLiveVideoLoop         call    #TVBufferData                                           ' Stream out live video.
                        waitvid TVColorBuffer,      TVData                              '
                        djnz    TVBuffer,           #TVLiveVideoLoop                    '

                        test    TVCounter,          #1 wc                               ' Increment pointers every odd frame.
if_nc                   sub     TVPointer,          #_TV_BYTES_PER_LINE                 '
if_nc                   sub     TVRowCounter,       #1                                  '

TVLiveVideoSkip         mov     vscl,               (TVTA + _STRUCT_TV_FRONT_PORCH)     ' Blank for front porch.
                        waitvid TVBlankColors,      #0                                  '

' //////////////////////Loop///////////////////////////////////////////////////////////////////////////////////////////////////

                        djnz    TVCounter,          #TVVideoLoop                        ' Repeat.

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
'                       Inactive Video
' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                        add     TVPhaseCounter,     #1                                  ' Figure if even or odd frame.
                        test    TVPhaseCounter,     #1 wc                               '

' //////////////////////Equalizing/////////////////////////////////////////////////////////////////////////////////////////////

                        mov     TVCounter,          (TVTA + _STRUCT_TV_PULSE_COUNT)     ' Do pre-equalization pulses.

                        cmp     TVTM,               #0 wz                               ' Add in an extra half line.
if_c_eq_z               add     TVCounter,          #1                                  '

TVPreEqualizationLoop   mov     vscl,               (TVTA+_STRUCT_TV_EQUALIZING_PULSE)  ' Preform pre-equalization pulse.
                        waitvid TVSyncColors,       #0                                  '
                        mov     vscl,               (TVTA+_STRUCT_TV_EQUALIZING_HIGH)   '
                        waitvid TVBlankColors,      #0                                  '

                        djnz    TVCounter,          #TVPreEqualizationLoop              ' Repeat.

' //////////////////////Serration//////////////////////////////////////////////////////////////////////////////////////////////

                        mov     TVCounter,          (TVTA + _STRUCT_TV_PULSE_COUNT)     ' Do serration pulses.

TVEqualizationLoop      mov     vscl,               (TVTA + _STRUCT_TV_SERRATION_LOW)   ' Preform serration pulse.
                        waitvid TVSyncColors,       #0                                  '
                        mov     vscl,               (TVTA + _STRUCT_TV_SYNC_TIP)        '
                        waitvid TVBlankColors,      #0                                  '

                        djnz    TVCounter,          #TVEqualizationLoop                 ' Repeat.

' //////////////////////Equalizing/////////////////////////////////////////////////////////////////////////////////////////////

                        mov     TVCounter,          (TVTA + _STRUCT_TV_PULSE_COUNT)     ' Do post-equalization pulses.

TVPostEqualizationLoop  mov     vscl,               (TVTA+_STRUCT_TV_EQUALIZING_PULSE)  ' Preform post-equalization pulse.
                        waitvid TVSyncColors,       #0                                  '
                        mov     vscl,               (TVTA+_STRUCT_TV_EQUALIZING_HIGH)   '
                        waitvid TVBlankColors,      #0                                  '

                        djnz    TVCounter,          #TVPostEqualizationLoop             ' Repeat.

' //////////////////////Half Line//////////////////////////////////////////////////////////////////////////////////////////////

if_c                    mov     vscl,               (TVTA + _STRUCT_TV_HALF_LINE)       ' Spit out half a line for odd frame.
if_c                    waitvid TVBlankColors,      #0                                  '

' //////////////////////Loop///////////////////////////////////////////////////////////////////////////////////////////////////

                        jmp     #TVLoop                                                 ' Loop.

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
'                       Transfer Data
' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TVTransferData          rdlong  TVData,             TVDataSwitchAddress                 ' Check timeout.
                        cmp     TVData,             TVTimeoutBuffer wz                  '
if_nz                   mov     TVTimeoutBuffer,    TVData                              '
if_nz                   mov     TVTimeoutCounter,   #(_NTSC_TIMEOUT <# _PAL_TIMEOUT)    '

                        movs    TVLiveVideoColors,  #TVDisplayColors                    ' Setup display colors.

                        cmpsub  TVTimeoutCounter,   #1 wc                               ' Figure if updating.
if_nc                   mov     TVBufferDataCTRL,   #$FF                                '
if_c                    rdbyte  TVBufferDataCTRL,   TVModeScreenAddress wz              '
if_nz_or_nc             jmp     #TVTransferDataRet                                      '

                        cmp     TVData,             #0 wz                               ' Figure address to copy from.

                        mov     TVPointer,          TVDataBufferAddress                 ' Setup to load from threshold buffer.
if_z                    add     TVPointer,          TVDataBufferSize                    '

                        mov     TVData,             TVTrackingDataAddress               ' Setup to load from tracking buffer.
if_z                    add     TVData,             #_LN_TRACK_DATA_SIZE_BYTES          '

                        rdbyte  TVCentroidX,        TVData                              ' Download centroid.
                        add     TVData,             #1                                  '
                        rdbyte  TVCentroidY,        TVData                              '
                        add     TVData,             #1                                  '

                        rdbyte  TVBoundX1,          TVData                              ' Download bounding box.
                        add     TVData,             #1                                  '
                        rdbyte  TVBoundY1,          TVData                              '
                        add     TVData,             #1                                  '
                        rdbyte  TVBoundX2,          TVData                              '
                        add     TVData,             #1                                  '
                        rdbyte  TVBoundY2,          TVData                              '
                        add     TVData,             #1                                  '

                        rdbyte  TVBufferDataTEXT,   TVData wz                           ' Figure if tracking.
if_z                    movs    TVLiveVideoColors,  #TVMessageColors                    '

                        mov     TVData,             TVBoundX1                           ' X1 = min [X1:X2], X2 = max [X1:X2].
                        max     TVBoundX1,          TVBoundX2                           '
                        min     TVBoundX2,          TVData                              '

                        mov     TVData,             TVBoundY1                           ' Y1 = min [Y1:Y2], Y2 = max [Y1:Y2].
                        max     TVBoundY1,          TVBoundY2                           '
                        min     TVBoundY2,          TVData                              '

                        mov     TVExtraBuffer,      par                                 ' Setup to copy.
                        mov     TVExtraCounter,     TVDataBufferSize                    '
                        shr     TVExtraCounter,     #2                                  '

TVTransferDataLoop      rdlong  TVData,             TVPointer                           ' Transfer display buffer.
                        add     TVPointer,          #4                                  '
                        wrlong  TVData,             TVExtraBuffer                       '
                        add     TVExtraBuffer,      #4                                  '

                        test    TVExtraCounter,     #$1F wz                             ' Return back to main every 32 cycles.
if_z                    jmpret  TVSubProcessPtr,    TVMainProcessPtr                    '

                        djnz    TVExtraCounter,     #TVTransferDataLoop                 ' Loop.

TVTransferDataRet       jmpret  TVSubProcessPtr,    TVMainProcessPtr                    ' Return.
                        jmp     #TVTransferDataRet                                      '

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
'                       Buffer Data
' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TVBufferData            rdbyte  TVExtraBuffer,      TVPointer                           ' Unpack word.
                        add     TVPointer,          #1                                  '
                        rev     TVExtraBuffer,      #(32 - 8)                           '

                        rdbyte  TVData,             TVPointer                           ' Unpack word.
                        add     TVPointer,          #1                                  '
                        rev     TVData,             #(32 - 8)                           '

                        shl     TVData,             #8                                  ' Combine word.
                        or      TVData,             TVExtraBuffer                       '

' //////////////////////Spilt Word Bits = %%1 (1) - %%0 (0)////////////////////////////////////////////////////////////////////

                        mov     TVExtraBuffer,      TVData                              ' Split word.
                        shl     TVExtraBuffer,      #8                                  '
                        or      TVData,             TVExtraBuffer                       '
                        and     TVData,             TVBin0x00FF                         '

                        mov     TVExtraBuffer,      TVData                              ' Split word.
                        shl     TVExtraBuffer,      #4                                  '
                        or      TVData,             TVExtraBuffer                       '
                        and     TVData,             TVBin0x0F0F                         '

                        mov     TVExtraBuffer,      TVData                              ' Split word.
                        shl     TVExtraBuffer,      #2                                  '
                        or      TVData,             TVExtraBuffer                       '
                        and     TVData,             TVBin0x3333                         '

                        mov     TVExtraBuffer,      TVData                              ' Split word.
                        shl     TVExtraBuffer,      #1                                  '
                        or      TVData,             TVExtraBuffer                       '
                        and     TVData,             TVBin0x5555                         '

' //////////////////////Skip Overlay///////////////////////////////////////////////////////////////////////////////////////////

                        mov     TVWordColumnB,      TVColumnCounter                     ' Compute word column.
                        shr     TVWordColumnB,      #4                                  '

                        tjnz    TVBufferDataCTRL,   #TVBufferDataSkip                   ' Change display depending on mode.
                        tjz     TVBufferDataTEXT,   #TVBufferDataMessage                '

' //////////////////////Add Bounding Box Columns = %%3/////////////////////////////////////////////////////////////////////////

                        cmpsub  TVBoundY2,          TVRowCounter wc, nr                 ' Check if between Y1 and Y2 bounds.
if_c                    cmpsub  TVRowCounter,       TVBoundY1 wc, nr                    '

                        cmp     TVWordColumnB,      TVWordBoundX1 wz                    ' Check if X1 bound is in word column.

if_z_and_c              mov     TVExtraCounter,     TVBoundX1                           ' Write in column bounding box.
if_z_and_c              sub     TVExtraCounter,     TVColumnCounter                     '
if_z_and_c              shl     TVExtraCounter,     #1                                  '
if_z_and_c              mov     TVExtraBuffer,      #3                                  '
if_z_and_c              shl     TVExtraBuffer,      TVExtraCounter                      '
if_z_and_c              or      TVData,             TVExtraBuffer                       '

                        cmp     TVWordColumnB,      TVWordBoundX2 wz                    ' Check if X2 bound is in word column.

if_z_and_c              mov     TVExtraCounter,     TVBoundX2                           ' Write in column bounding box.
if_z_and_c              sub     TVExtraCounter,     TVColumnCounter                     '
if_z_and_c              shl     TVExtraCounter,     #1                                  '
if_z_and_c              mov     TVExtraBuffer,      #3                                  '
if_z_and_c              shl     TVExtraBuffer,      TVExtraCounter                      '
if_z_and_c              or      TVData,             TVExtraBuffer                       '

' //////////////////////Add Bounding Box Rows = %%3////////////////////////////////////////////////////////////////////////////

                        cmp     TVWordColumnB,      TVWordBoundX1 wz                    ' Compute pixel fill shift mask.
                        mov     TVExtraCounter,     TVBoundX2                           '
if_z                    sub     TVExtraCounter,     TVBoundX1                           '
if_nz                   sub     TVExtraCounter,     TVColumnCounter                     '
                        max     TVExtraCounter,     #(_TV_BITS_PER_WORD - 1)            '
                        sub     TVExtraCounter,     #(_TV_BITS_PER_WORD - 1)            '
                        abs     TVExtraCounter,     TVExtraCounter                      '
                        shl     TVExtraCounter,     #1                                  '

                        neg     TVExtraBuffer,      #1                                  ' Build pixel mask.
                        shr     TVExtraBuffer,      TVExtraCounter                      '

                        mov     TVExtraCounter,     TVBoundX1                           ' Apply extra mask offset.
                        cmpsub  TVExtraCounter,     TVColumnCounter wc                  '
if_c                    shl     TVExtraCounter,     #1                                  '
if_c                    shl     TVExtraBuffer,      TVExtraCounter                      '

                        cmpsub  TVWordBoundX2,      TVWordColumnB wc, nr                ' Check if column needs filling.
if_c                    cmpsub  TVWordColumnB,      TVWordBoundX1 wc, nr                '

                        cmp     TVBoundY1,          TVRowCounter wz                     ' Check if Y1 bound is word row.
if_z_and_c              or      TVData,             TVExtraBuffer                       '

                        cmp     TVBoundY2,          TVRowCounter wz                     ' Check if Y2 bound is word row.
if_z_and_c              or      TVData,             TVExtraBuffer                       '

' //////////////////////Add Centroid Box = %%2/////////////////////////////////////////////////////////////////////////////////

                        cmp     TVWordColumnB,      TVWordCentroidB wz                  ' Check if X bound is in word column.

if_z                    cmp     TVRowCounter,       TVCentroidY wz                      ' Check if Y bound is word row.

if_z                    mov     TVExtraCounter,     TVCentroidX                         ' Write in pixel box.
if_z                    sub     TVExtraCounter,     TVColumnCounter                     '
if_z                    shl     TVExtraCounter,     #1                                  '
if_z                    mov     TVExtraBuffer,      #3                                  '
if_z                    shl     TVExtraBuffer,      TVExtraCounter                      '
if_z                    andn    TVData,             TVExtraBuffer                       '
if_z                    andn    TVExtraBuffer,      TVBin0x5555                         '
if_z                    or      TVData,             TVExtraBuffer                       '

' //////////////////////Return/////////////////////////////////////////////////////////////////////////////////////////////////

TVBufferDataSkip        add     TVColumnCounter,    #_TV_BITS_PER_WORD                  ' Increment pointer.
                        cmpsub  TVColumnCounter,    #_TV_HORIZONTAL_RES wc              '
if_c                    add     TVRowCounter,       #1                                  '

TVBufferData_ret        ret                                                             ' Return.

' //////////////////////Text Message///////////////////////////////////////////////////////////////////////////////////////////

TVBufferDataMessage     mov     TVData,             TVRowCounter                        ' Get character row.
                        sub     TVData,             #_TV_ROM_BLANK_BELOW                '
                        mov     TVExtraCounter,     TVData                              '
                        shr     TVExtraCounter,     #5                                  '

                        mov     TVExtraBuffer,      TVExtraCounter                      ' Multiply by 10.
                        shl     TVExtraBuffer,      #3                                  '
                        shl     TVExtraCounter,     #1                                  '
                        add     TVExtraCounter,     TVExtraBuffer                       '

                        add     TVExtraCounter,     TVWordColumnB                       ' Read character.
                        add     TVExtraCounter,     TVMessageBufferAddress              '
                        rdbyte  TVExtraCounter,     TVExtraCounter                      '

                        and     TVData,             #$1F                                ' Compute ROM address.
                        shl     TVData,             #2                                  '
                        shr     TVExtraCounter,     #1 wc                               '
                        shl     TVExtraCounter,     #7                                  '
                        add     TVExtraCounter,     TVData                              '
                        add     TVExtraCounter,     TVCharacterSetAddress               '

                        rdlong  TVData,             TVExtraCounter                      ' Grab character pixels.
if_c                    shr     TVData,             #1                                  '
                        and     TVData,             TVBin0x5555                         '

                        cmp     TVRowCounter,       #_TV_ROM_BLANK_BELOW wc             ' Blank for lines 0-11 and 108-119.
if_nc                   cmpsub  TVRowCounter,       #_TV_ROM_BLANK_ABOVE wc, nr         '
if_c                    mov     TVData,             #0                                  '

                        jmp     #TVBufferDataSkip                                       ' Return.

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
'                       Data
' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TVDisplayColors         long    $aC_07_FB_02 ' %%3 = Red, %%2 = White, %%1 = Blue, %%0 = Black.
TVMessageColors         long    $AB_02 ' %%1 = Green, %%0 = Black.

' //////////////////////Constants//////////////////////////////////////////////////////////////////////////////////////////////

TVDestinationIncrement  long    $200

TVBin0x5555             long    $55_55_55_55
TVBin0x3333             long    $33_33_33_33
TVBin0x0F0F             long    $0F_0F_0F_0F
TVBin0x00FF             long    $00_FF_00_FF

TVSyncColors            long    $00_00_00_00 ' -40 IRE.
TVBlankColors           long    $02_02_02_02 ' 0 IRE.

NTSCBurstColors         long    $8A_8A_8A_8A ' 0 IRE with 180 degree color burst.
PALBurstColors          long    $6A_6A_6A_6A ' 0 IRE with 135 degree color burst.

TVPhaseRotate           long    $F0_F0_F0_F0

TVDataBufferSize        long    _LN_BUFFER_SIZE_BYTES
TVCharacterSetAddress   long    _MEMORY_TV_ROMFONT_ADDRESS

' //////////////////////NTSC Settings//////////////////////////////////////////////////////////////////////////////////////////

NTSCTimingArray         long    6 ' NTSCPulseCount - SYNC pulses...

                        long    ((1 << 12) + 135) ' [EQ] = (2.35 us * CF) = 135. NTSCEqualizingPulse.
                        long    ((1 << 12) + 1685) ' ((LT / 2) - EQ) = (29.428 us * CF) = 1685. NTSCEqualizingHigh.
                        long    ((1 << 12) + 1551) ' ((LT / 2) - ST) = (27.078 us * CF) = 1551. NTSCSerrationLow.
                        long    ((1 << 12) + 269) ' [ST] = (4.7 us * CF) = 269. NTSCSyncTip.
                        long    ((1 << 12) + 1820) ' (LT / 2) = (31.778 us * CF) = 1820. NTSCHalfLine.

                        long    13 ' NTSCLineCount - Blank lines...

                        long    ((1 << 12) + 92 + 228) ' [BP] = (1.6 us * CF) = 92. NTSCBackPorch.
                                                       ' [LO] = (LT - (LV + ST + BW + CB + BP + FP + RO)). NTSCLeftOverscan.

                        long    ((1 << 12) + 86 + 228) ' [FP] = (1.5 us * CF) = 86. NTSCFrontPorch.
                                                       ' [RO] = (LT - (LV + ST + BW + CB + BP + FP + LO)). NTSCRightOverscan.

                        long    ((1 << 12) + 34) ' [BW] = (0.6 us * CF) = 34. NTSCBreezeWay.
                        long    ((16 << 12) + ((16 * 9) - 1)) ' [CB] = (2.5 us * CF) = 143. NTSCColorBurst.

                        long    ((16 << 12) + (16 * 16)) ' [LV] = (44.698 us * CF) = 2560. 10X. NTSCLiveVideo.

                        long    _NTSC_LINES ' NTSCVideoLines.

                        '       Line Time [LT] = 63.556 us = 3640.
                        '       Horizontal Sync [HS] = 10.9 us = 624.

' //////////////////////PAL Settings///////////////////////////////////////////////////////////////////////////////////////////

PALTimingArray          long    5 ' PALPulseCount - SYNC pulses...

                        long    ((1 << 12) + 167) ' [EQ] = (2.35 us * CF) = 167. PALEqualizingPulse.
                        long    ((1 << 12) + 2103) ' ((LT / 2) - EQ) = (29.65 us * CF) = 2103. PALEqualizingHigh.
                        long    ((1 << 12) + 1937) ' ((LT / 2) - ST) = (27.3 us * CF) = 1937. PALSerrationLow.
                        long    ((1 << 12) + 333) ' [ST] = (4.7 us * CF) = 333. PALSyncTip.
                        long    ((1 << 12) + 2270) ' (LT / 2) = (32 us * CF) = 2270. PALHalfLine.

                        long    17 ' PALLineCount - Blank lines...

                        long    ((1 << 12) + 188 + 244 + 0) ' [BP] = (2.65 us * CF) = 188. PALBackPorch.
                                                            ' [LO] = (LT - (LV+ST+BW+CB+BP+FP+RO)). PALLeftOverscan.

                        long    ((1 << 12) + 106 + 244 + 1) ' [FP] = (1.5 us * CF) = 106. PALFrontPorch.
                                                            ' [RO] = (LT - (LV+ST+BW+CB+BP+FP+LO)). PALRightOverscan.

                        long    ((1 << 12) + 64) ' [BW] = (0.9 us * CF) = 64. PALBreezeWay.
                        long    ((16 << 12) + ((16 * 10) - 0)) ' [CB] = (2.25 us * CF) = 160. PALColorBurst.

                        long    ((20 << 12) + (20 * 16)) ' [LV] = (45.110 us * CF) = 3200. 10X. PALLiveVideo.

                        long    _PAL_LINES ' PALVideoLines.

                        '       Line Time [LT] = 64 us = 4540.
                        '       Horizontal Sync [HS] = 12 us = 851.

' //////////////////////Configuration Data/////////////////////////////////////////////////////////////////////////////////////

TVNTSCAccumulator       long    0 ' Set to ((NTSCFrequencySetup << 32) / clkfreq) rounded up on startup.
TVPALAccumulator        long    0 ' Set to ((PALFrequencySetup << 32) / clkfreq) rounded up on startup.

TVVideoSetup            long    %0_10_1_0_1_000_00000000000_011_0_00000111 ' Video setup.
TVDirectionSetup        long    $07_00_00_00 ' Direction setup.

NTSCCounterSetup        long    %0_00001_110_000000000_00000_0000_00000 ' Counter Setup. PLL @8X.
NTSCFrequencySetup      long    7_159_090 ' [CF] = 57_272_720 Hz = 3_579_545 Hz * 16 = 7_159_090 Hz * 8.

PALCounterSetup         long    %0_00001_111_000000000_00000_0000_00000 ' Counter Setup. PLL @16X.
PALFrequencySetup       long    4_433_618 ' [CF] = 70_937_888 Hz = 4_433_618 Hz * 16.

TVTimeoutBuffer         long    1 ' Trigger timeout reset switch on startup.
TVPhaseCounter          long    1 ' Trigger zero phase switch on startup.

' //////////////////////Addresses//////////////////////////////////////////////////////////////////////////////////////////////

TVDataSwitchAddress     long    0
TVDataBufferAddress     long    0
TVTrackingDataAddress   long    0
TVModeScreenAddress     long    0
TVModeOutputAddress     long    0
TVMessageBufferAddress  long    0

' //////////////////////Run Time Variables/////////////////////////////////////////////////////////////////////////////////////

TVPointer               res     1
TVData                  res     1
TVBuffer                res     1
TVCounter               res     1

TVColorBuffer           res     1
TVTimeoutCounter        res     1

TVExtraBuffer           res     1
TVExtraCounter          res     1

TVRowCounter            res     1
TVColumnCounter         res     1

TVMainProcessPtr        res     1
TVSubProcessPtr         res     1

TVBoundX1               res     1
TVBoundY1               res     1
TVBoundX2               res     1
TVBoundY2               res     1

TVCentroidX             res     1
TVCentroidY             res     1

TVWordBoundX1           res     1
TVWordBoundX2           res     1
TVWordColumnB           res     1
TVWordCentroidB         res     1

TVBufferDataCTRL        res     1
TVBufferDataTEXT        res     1

TVTM                    res     1 ' Timing Mode. False for NTSC. True for PAL.
TVTA                    res     _STRUCT_TV_TA_SIZE ' Timing Array.

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                        fit     496

DAT

' //////////////////////TV Frame Buffer////////////////////////////////////////////////////////////////////////////////////////

TVLineBuffer            file "!RANGER.BIT" ' = "LNBufferSize" IN BYTES. (LONG ALIGNED - FOLLOWS ASM DAT).

' //////////////////////TV Message Buffer//////////////////////////////////////////////////////////////////////////////////////

TVMessageBuffer         byte $9F, "T", "R", "A", "C", "K", "I", "N", "G", $9E
                        byte $91, " ", "C", "O", "L", "O", "R", "S", " ", $91
                        byte $9D, $90, $90, "L", "O", "S", "T", $90, $90, $9C

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

DAT

' //////////////////////TV Setup Variables/////////////////////////////////////////////////////////////////////////////////////

TVModeScreen            byte true ' Set to true to show the splash screen and false to show the segmented image.
TVModeOutput            byte _DEMO_MODE_TV_SIGNAL ' Set to false in NTSC countries and true in PAL countries.

' /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CON ' TV driver constants.

  _NTSC_LINES = 240 ' NTSC video lines.
  _PAL_LINES = 288 ' PAL video lines.

  _NTSC_FPS = 60 ' NTSC frames per second.
  _PAL_FPS = 50 ' Pal frames per second.

  _NTSC_TIMEOUT = ((_NTSC_FPS + _FRAME_TIMEOUT - 1) / _FRAME_TIMEOUT)
  _PAL_TIMEOUT = ((_PAL_FPS + _FRAME_TIMEOUT - 1) / _FRAME_TIMEOUT)

  _ROM_CHAR_X = 16 ' Built-in ROM font width.
  _ROM_CHAR_Y = 32 ' Built-in ROM font height.

  _TV_BITS_PER_WORD = 16 ' DO NOT CHANGE!
  _TV_BYTES_PER_WORD = 2 ' DO NOT CHANGE!
  _TV_HORIZONTAL_RES = 160 ' DO NOT CHANGE!
  _TV_VERTICAL_RES = 120 ' DO NOT CHANGE!

  _TV_WORDS_PER_LINE = (_TV_HORIZONTAL_RES / _TV_BITS_PER_WORD) ' Should be 10.
  _TV_BYTES_PER_LINE = (_TV_WORDS_PER_LINE * _TV_BYTES_PER_WORD) ' Should be 20.

  _TV_BLANK_PAL_LINES_BELOW = ((_PAL_LINES - (_TV_VERTICAL_RES * 2)) / 2) ' Equal to or below. 1 to n numbering.
  _TV_BLANK_PAL_LINES_ABOVE = (_TV_BLANK_PAL_LINES_BELOW + (_TV_VERTICAL_RES * 2)) ' Equal to or above. 1 to n numbering.

  _TV_ROM_BLANK_BELOW = ((_TV_VERTICAL_RES - (_ROM_CHAR_Y * 3)) / 2) ' Below. 0 to n-1 numbering.
  _TV_ROM_BLANK_ABOVE = (_TV_ROM_BLANK_BELOW + (_ROM_CHAR_Y * 3)) ' Above. 0 to n-1 numbering.

  _MEMORY_TV_CLKFREQ_ADDRESS = $0000
  _MEMORY_TV_ROMFONT_ADDRESS = $8000

  _STRUCT_TV_TA_SIZE = 13

  _STRUCT_TV_PULSE_COUNT = 0
  _STRUCT_TV_EQUALIZING_PULSE = 1
  _STRUCT_TV_EQUALIZING_HIGH = 2
  _STRUCT_TV_SERRATION_LOW = 3
  _STRUCT_TV_SYNC_TIP = 4
  _STRUCT_TV_HALF_LINE = 5
  _STRUCT_TV_LINE_COUNT = 6
  _STRUCT_TV_BACK_PORCH = 7
  _STRUCT_TV_FRONT_PORCH = 8
  _STRUCT_TV_BREEZE_WAY = 9
  _STRUCT_TV_COLOR_BURST = 10
  _STRUCT_TV_LIVE_VIDEO = 11
  _STRUCT_TV_VIDEO_LINES = 12

VAR long TVCog ' TV stuff.

PRI TVDriverStart ' Initializes the TV driver. Returns true on success and false on failure.

  TVDriverStop
  if(chipver == 1)

    TVDataSwitchAddress := @LNSwitch
    TVDataBufferAddress := @LNBuffer
    TVTrackingDataAddress := @LNTrackData

    TVModeScreenAddress := @TVModeScreen
    TVModeOutputAddress := @TVModeOutput
    TVMessageBufferAddress := @TVMessageBuffer

    TVCog := cognew(@TVDriverInitialization, @TVLineBuffer)
    result or= ++TVCog

PRI TVDriverStop ' Shutsdown the TV driver.

  if(TVCog)
    cogstop(-1 + TVCog~)

PRI setupDynamicVsStatic(dynamicOrStatic) ' False = Dynamic Image Displayed, True = Static Image Displayed.

  ifnot(dynamicOrStatic)
    waitForNewData(_FRAME_UPDATED)

  TVModeScreen := (dynamicOrStatic <> false)

PRI setupNTSCVsPAL(NTSCOrPAL) ' False = NTSC Ouput, True = PAL Output.

  TVModeOutput := (NTSCOrPAL <> false)

CON ' Camera register settings constants.

  _CR_LENGTH = 3

  _CR_ADDRESS = 0
  _CR_VALUE = 1
  _CR_MASK = 2

  _CW_COMMAND_STRUCT_ELEMENTS = 1

PRI setupCamera | index ' Initialize camera. Returns true on success and false on failure.

  cameraClockOn
  cameraPowerUp

  waitcnt((clkfreq / _CAMERA_XCLK_TIMEOUT) + cnt)

  repeat 2 ' Reset the camera pll twice for good measure.
    ifnot(SCCBWriteRegister(_CAMERA_PLL_RESET_ADDRESS, _CAMERA_PLL_RESET_VALUE))
      return false

    waitcnt((clkfreq / _CMAERA_PLL_TIMEOUT) + cnt) ' Wait in between each reset of the pll.

  if(SCCBWriteRegister(_CAMERA_SOFTWARE_RESET_ADDRESS, _CAMERA_SOFTWARE_RESET_VALUE)) ' Reset all camera registers.
    waitcnt((clkfreq / _CAMERA_SCCB_TIMEOUT) + cnt)

    repeat index from _CAMERA_INIT_START to constant(_CAMERA_INIT_END * _CAMERA_INIT_STEP) step _CAMERA_INIT_STEP
      ifnot(SCCBWriteRegister(Camera_Init_Data[index + _CR_ADDRESS], Camera_Init_Data[index + _CR_VALUE]))
        return false

    index := \SCCBCameraControl(@Camera_Setup_Data, CAMERA_SETUP_NUMBER) ' Setup the RGB565 camera color mode.

    ifnot(index)
      setupColorTracking(BMColorSpace := false)
      return true

CON _CAMERA_INIT_NUMBER = 129 ' See OmniVision OV9665 implementation guide and data sheet for more details.

  _CAMERA_INIT_START = 0
  _CAMERA_INIT_END = (_CAMERA_INIT_NUMBER - 1)
  _CAMERA_INIT_STEP = (_CR_LENGTH - 1)

DAT Camera_Init_Data ' 129 Registers

' Format - register, data

byte $D5, $FF
byte $D6, $3F
byte $3D, $3C
byte $11, $80
byte $2A, $00
byte $2B, $00
byte $3A, $D9
byte $3B, $00
byte $3C, $58
byte $3E, $D0
byte $71, $00
byte $15, $00
byte $D7, $10
byte $6A, $24
byte $85, $E7
byte $63, $00
byte $12, $40
byte $4D, $09
byte $17, $0C
byte $18, $5C
byte $19, $02
byte $1A, $3F
byte $03, $03
byte $32, $90
byte $2B, $00
byte $5C, $80
byte $36, $B4
byte $65, $10
byte $70, $02
byte $71, $9F
byte $64, $A4
byte $5C, $80
byte $43, $00
byte $5D, $55
byte $5E, $57
byte $5F, $21
byte $24, $3E
byte $25, $38
byte $26, $72
byte $14, $68
byte $0C, $38
byte $4F, $4F
byte $50, $42
byte $5A, $67
byte $7D, $30
byte $7E, $00
byte $82, $03
byte $7F, $00
byte $83, $07
byte $80, $03
byte $81, $04
byte $96, $F0
byte $97, $00
byte $92, $33
byte $94, $5A
byte $93, $3A
byte $95, $48
byte $91, $FC
byte $90, $FF
byte $8E, $4E
byte $8F, $4E
byte $8D, $13
byte $8C, $0C
byte $8B, $0C
byte $86, $9E
byte $87, $11
byte $88, $22
byte $89, $05
byte $8A, $03
byte $9B, $0E
byte $9C, $1C
byte $9D, $34
byte $9E, $5A
byte $9F, $68
byte $A0, $76
byte $A1, $82
byte $A2, $8E
byte $A3, $98
byte $A4, $A0
byte $A5, $B0
byte $A6, $BE
byte $A7, $D2
byte $A8, $E2
byte $A9, $EE
byte $AA, $18
byte $AB, $E7
byte $B0, $43
byte $AC, $04
byte $84, $40
byte $AD, $82
byte $D9, $11
byte $DA, $00
byte $AE, $10
byte $AB, $E7
byte $B9, $50
byte $BA, $3C
byte $BB, $50
byte $BC, $3C
byte $BD, $08
byte $BE, $19
byte $BF, $02
byte $C0, $08
byte $C1, $2A
byte $C2, $34
byte $C3, $2D
byte $C4, $2D
byte $C5, $00
byte $C6, $98
byte $C7, $18
byte $69, $48
byte $74, $C0
byte $7C, $28
byte $65, $11
byte $66, $00
byte $41, $C0
byte $5B, $24
byte $60, $82
byte $05, $07
byte $03, $03
byte $D2, $94
byte $C8, $06
byte $CB, $40
byte $CC, $40
byte $CF, $00
byte $D0, $20
byte $D1, $00
byte $C7, $18
byte $0D, $92
byte $0D, $90

CON CAMERA_SETUP_NUMBER = 3 ' See OmniVision OV9665 implementation guide and data sheet for more details.

DAT Camera_Setup_Data ' 3 Registers

' Format - register, data, mux mask

byte $6A, $14, $FF
byte $D8, $C1, $C7
byte $D7, $01, $13

CON _CAMERA_POWER_OFF_NUMBER = 5 ' See OmniVision OV9665 implementation guide and data sheet for more details.

DAT Camera_Power_Off_Data ' 5 Registers

' Format - register, data, mux mask

byte $3B, $00, $08
byte $13, $00, $07
byte $39, $6A, $FF
byte $D5, $00, $FF
byte $D6, $00, $FF

CON _CAMERA_SLEEP_ON_NUMBER = 6 ' See OmniVision OV9665 implementation guide and data sheet for more details.

DAT Camera_Sleep_On_Data ' 6 Registers

' Format - register, data, mux mask

byte $13, $E0, $FF
byte $3B, $06, $06
byte $3E, $80, $80
byte $D5, $00, $FF
byte $D6, $00, $FF
byte $09, $10, $10

PRI setupCameraColorSpace(RGB565OrYUV422) ' Change camera color space.

  Camera_Color_Space_Data[_CR_VALUE] := (((RGB565OrYUV422 <>= false) & constant(_YUV422 - _RGB565)) + _RGB565)
  SCCBCameraControl(@Camera_Color_Space_Data, _CAMERA_COLOR_SPACE_NUMBER)
  setupColorTracking(BMColorSpace := RGB565OrYUV422)

CON _CAMERA_COLOR_SPACE_NUMBER = 1 ' See OmniVision OV9665 implementation guide and data sheet for more details.

  _YUV422 = $10
  _RGB565 = $01

DAT Camera_Color_Space_Data ' 1 Register

' Format - register, data, mux mask

byte $D7, $00, $13

PRI setupBrightness(brightness) ' Change brightness settings.

  Brightness_Switch_Data[_CR_VALUE] := ((||brightness) & $7F)
  Brightness_Switch_Data[_BRIGHTNESS_NEGATIVE_CR_VALUE] := (_BRIGHTNESS_DEFAULT | ((brightness < 0) & _BRIGHTNESS_NEGATIVE))
  SCCBCameraControl(@Brightness_Switch_Data, _BRIGHTNESS_SWITCH_NUMBER)

CON _BRIGHTNESS_SWITCH_NUMBER = 3 ' See OmniVision OV9665 implementation guide and data sheet for more details.

  _BRIGHTNESS_DEFAULT = $10
  _BRIGHTNESS_NEGATIVE = $8
  _BRIGHTNESS_NEGATIVE_CR_VALUE = (_CR_VALUE + (_CR_LENGTH * 2))

DAT Brightness_Switch_Data ' 3 Registers

' Format - register, data, mux mask

byte $D1, $00, $FF
byte $C8, $04, $04
byte $C7, $00, $18

PRI setupContrast(contrast) ' Change constrast settings.

  Contrast_Switch_Data[_CR_VALUE] := ((contrast // constant(_CONTRAST_MAXIMUM - _CONTRAST_MINIMUM + 1)) + _CONTRAST_DEFAULT)
  SCCBCameraControl(@Contrast_Switch_Data, _CONTRAST_SWITCH_NUMBER)

CON _CONTRAST_SWITCH_NUMBER = 4 ' See OmniVision OV9665 implementation guide and data sheet for more details.

  _CONTRAST_DEFAULT = $20
  _CONTRAST_MAXIMUM = $1F
  _CONTRAST_MINIMUM = $0

DAT Contrast_Switch_Data ' 4 Registers

' Format - register, data, mux mask

byte $D0, $00, $FF
byte $64, $02, $02
byte $C8, $04, $04
byte $C7, $34, $34

PRI setupAGC(onOrOff) ' Change auto gain control.

  AGC_Switch_Data[_CR_VALUE] := (onOrOff <> false)
  SCCBCameraControl(@AGC_Switch_Data, _AGC_SWITCH_NUMBER)

CON _AGC_SWITCH_NUMBER = 1 ' See OmniVision OV9665 implementation guide and data sheet for more details.

DAT AGC_Switch_Data ' 1 Register

' Format - register, data, mux mask

byte $13, $00, $04

PRI setupAWB(onOrOff) ' Change auto white balance control.

  AWB_Switch_Data[_CR_VALUE] := (onOrOff <> false)
  SCCBCameraControl(@AWB_Switch_Data, _AWB_SWITCH_NUMBER)

CON _AWB_SWITCH_NUMBER = 1 ' See OmniVision OV9665 implementation guide and data sheet for more details.

DAT AWB_Switch_Data ' 1 Register

' Format - register, data, mux mask

byte $13, $00, $02

PRI setupNegative(onOrOff) ' Change negative settings.

  NEG_Switch_Data[_CR_VALUE] := (onOrOff <> false)
  SCCBCameraControl(@NEG_Switch_Data, _NEG_SWITCH_NUMBER)

CON _NEG_SWITCH_NUMBER = 2 ' See OmniVision OV9665 implementation guide and data sheet for more details.

DAT NEG_Switch_Data ' 2 Registers

' Format - register, data, mux mask

byte $C8, $00, $40
byte $C7, $10, $10

PRI setupBlackAndWhite(onOrOff) ' Change black and white settings.

  BAW_Switch_Data[_CR_VALUE] := (onOrOff <> false)
  SCCBCameraControl(@BAW_Switch_Data, _BAW_SWITCH_NUMBER)

CON _BAW_SWITCH_NUMBER = 2 ' See OmniVision OV9665 implementation guide and data sheet for more details.

DAT BAW_Switch_Data ' 2 Registers

' Format - register, data, mux mask

byte $C8, $00, $20
byte $C7, $10, $10

PRI setupVerticalFlip(onOrOff) ' Change vertical flip settings.

  Vertical_Flip_Data[_CR_VALUE] := (onOrOff <> false)
  SCCBCameraControl(@Vertical_Flip_Data, _VERTICAL_FLIP_NUMBER)

CON _VERTICAL_FLIP_NUMBER = 1 ' See OmniVision OV9665 implementation guide and data sheet for more details.

DAT Vertical_Flip_Data ' 1 Register

' Format - register, data, mux mask

byte $04, $00, $40

PRI setupHorizontalMirror(onOrOff) ' Change horizontal mirror settings.

  Horizontal_Mirror_Data[_CR_VALUE] := (onOrOff <> false)
  SCCBCameraControl(@Horizontal_Mirror_Data, _HORIZONTAL_MIRROR_NUMBER)

CON _HORIZONTAL_MIRROR_NUMBER = 2 ' See OmniVision OV9665 implementation guide and data sheet for more details.

DAT Horizontal_Mirror_Data ' 2 Registers

' Format - register, data, mux mask

byte $33, $00, $08
byte $04, $80, $80

PRI setupColorBars(onOrOff) ' Change color bar settings.

  Color_Bars_Data[_CR_VALUE] := (onOrOff <> false)
  SCCBCameraControl(@Color_Bars_Data, _COLOR_BARS_NUMBER)

CON _COLOR_BARS_NUMBER = 1 ' See OmniVision OV9665 implementation guide and data sheet for more details.

DAT Color_Bars_Data ' 1 Register

' Format - register, data, mux mask

byte $69, $00, $04

PRI SCCBCameraControl(address, length) | index, temp ' Updates camera controls from data structures.

' Notes:
'
' Address - The address of the data structure.
' Length - The length of the data structure.
'
' Returns false on success and aborts on failure.

  length := ((length - 1) * _CR_LENGTH)

  lineGrabberDriverStop
  repeat index from 0 to length step _CR_LENGTH

    ifnot(result := SCCBReadRegister(byte[address][index + _CR_ADDRESS], @temp))
      quit

    ifnot(result := SCCBWriteRegister( byte[address][index + _CR_ADDRESS], {
                                   } ( (temp & (!byte[address][index + _CR_MASK])) | {
                                   }   (byte[address][index + _CR_VALUE] & byte[address][index + _CR_MASK]) ) ) )
      quit

  waitcnt((clkfreq / _FRAME_RATE) + cnt)
  lineGrabberDriverStart

  ifnot(result~)
    abort @cameraConnectionError

PRI SCCBWriteRegister(register, data) ' Writes a camera register.

' Notes:
'
' Register - The register number to write.
' Data - The value to write.
'
' Returns true on success and false on failure.

  I2CStart
  result := I2CWrite(_SCCB_CAMERA_WRITE_ADDRESS)
  result and= I2CWrite(register)
  result and= I2CWrite(data)
  I2CStop

PRI SCCBReadRegister(register, longDataAddress) ' Reads a camera register.

' Notes:
'
' Register - The register number to read.
' LongDataAddress - The address of the value to store data to.
'
' Returns true on success and false on failure.

  I2CStart
  result := I2CWrite(_SCCB_CAMERA_WRITE_ADDRESS)
  result and= I2CWrite(register)
  I2CStop

  if(result)

    I2CStart
    result := I2CWrite(_SCCB_CAMERA_READ_ADDRESS)
    long[longDataAddress] := I2CRead(false)
    I2CStop

PRI I2CWrite(data) ' Write out data.

' Notes:
'
' Data - The 8 bit packet to transmit.
'
' Returns true if the receiving device ACK'ed and false if not.

  data := ((!data) >< 8)

  repeat 8 ' Write out all 8 data bits. Leave with clock low.

    dira[_I2C_DATA_PIN] := data
    dira[_I2C_CLOCK_PIN] := 0
    dira[_I2C_CLOCK_PIN] := 1
    data >>= 1

  ' Leave with clock low and data low.

  dira[_I2C_DATA_PIN] := 0
  dira[_I2C_CLOCK_PIN] := 0
  result := not(ina[_I2C_DATA_PIN])
  dira[_I2C_CLOCK_PIN] := 1
  dira[_I2C_DATA_PIN] := 1

PRI I2CRead(aknowledge) ' Read in data.

' Notes:
'
' Aknowledge - True to send the transmitting device an ACK and false to not send a NCK.
'
' Returns the received 8 bit packet.

  dira[_I2C_DATA_PIN] := 0

  repeat 8 ' Read in all 8 data bits. Leave with clock low.

    result <<= 1
    dira[_I2C_CLOCK_PIN] := 0
    result |= ina[_I2C_DATA_PIN]
    dira[_I2C_CLOCK_PIN] := 1

  ' Leave with the clock low and the data low.

  dira[_I2C_DATA_PIN] := (not(not(aknowledge)))
  dira[_I2C_CLOCK_PIN] := 0
  dira[_I2C_CLOCK_PIN] := 1
  dira[_I2C_DATA_PIN] := 1

PRI I2CStart ' Cause the I2C start condition.

  outa[_I2C_DATA_PIN] := 0
  dira[_I2C_DATA_PIN] := 1
  outa[_I2C_CLOCK_PIN] := 0
  dira[_I2C_CLOCK_PIN] := 1

PRI I2CStop ' Cause the I2C stop condition.

  dira[_I2C_CLOCK_PIN] := 0
  dira[_I2C_DATA_PIN] := 0

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
