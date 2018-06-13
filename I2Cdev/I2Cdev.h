// I2Cdev library collection - Main I2C device class header file
// Abstracts bit and byte I2C R/W functions into a convenient class
// 2013-06-05 by Jeff Rowberg <jeff@rowberg.net>
//
// Changelog:
//      2015-10-30 - simondlevy : support i2c_t3 for Teensy3.1
//      2013-05-06 - add Francesco Ferrara's Fastwire v0.24 implementation with small modifications
//      2013-05-05 - fix issue with writing bit values to words (Sasquatch/Farzanegan)
//      2012-06-09 - fix major issue with reading > 32 bytes at a time with Arduino Wire
//                 - add compiler warnings when using outdated or IDE or limited I2Cdev implementation
//      2011-11-01 - fix write*Bits mask calculation (thanks sasquatch @ Arduino forums)
//      2011-10-03 - added automatic Arduino version detection for ease of use
//      2011-10-02 - added Gene Knight's NBWire TwoWire class implementation with small modifications
//      2011-08-31 - added support for Arduino 1.0 Wire library (methods are different from 0.x)
//      2011-08-03 - added optional timeout parameter to read* methods to easily change from default
//      2011-08-02 - added support for 16-bit registers
//                 - fixed incorrect Doxygen comments on some methods
//                 - added timeout value for read operations (thanks mem @ Arduino forums)
//      2011-07-30 - changed read/write function structures to return success or byte counts
//                 - made all methods static for multi-device memory savings
//      2011-07-28 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2013 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _I2CDEV_H_
#define _I2CDEV_H_
	
	#ifndef I2CDEV_IMPLEMENTATION
	#define I2CDEV_IMPLEMENTATION       I2CDEV_ARDUINO_WIRE
	//#define I2CDEV_IMPLEMENTATION       I2CDEV_BUILTIN_FASTWIRE
	#endif // I2CDEV_IMPLEMENTATION


    #define BUFFER_LENGTH 32
  
    #include <Wire.h>




// 1000ms default read timeout (modify with "I2Cdev::readTimeout = [ms];")
#define I2CDEV_DEFAULT_READ_TIMEOUT     1000

class I2Cdev {
    public:
    I2Cdev();

    static     char readBit(unsigned char devAddr, unsigned char regAddr, unsigned char bitNum, unsigned char *data, unsigned short timeout=I2Cdev::readTimeout);
    static     char readBitW(unsigned char devAddr, unsigned char regAddr, unsigned char bitNum, unsigned short *data, unsigned short timeout=I2Cdev::readTimeout);
    static     char readBits(unsigned char devAddr, unsigned char regAddr, unsigned char bitStart, unsigned char length, unsigned char *data, unsigned short timeout=I2Cdev::readTimeout);
    static     char readBitsW(unsigned char devAddr, unsigned char regAddr, unsigned char bitStart, unsigned char length, unsigned short *data, unsigned short timeout=I2Cdev::readTimeout);
    static     char readByte(unsigned char devAddr, unsigned char regAddr, unsigned char *data, unsigned short timeout=I2Cdev::readTimeout);
    static     char readWord(unsigned char devAddr, unsigned char regAddr, unsigned short *data, unsigned short timeout=I2Cdev::readTimeout);
    static     char readBytes(unsigned char devAddr, unsigned char regAddr, unsigned char length, unsigned char *data, unsigned short timeout=I2Cdev::readTimeout);
    static     char readWords(unsigned char devAddr, unsigned char regAddr, unsigned char length, unsigned short *data, unsigned short timeout=I2Cdev::readTimeout);

    static    boolean writeBit(unsigned char devAddr, unsigned char regAddr, unsigned char bitNum, unsigned char data);
    static    boolean writeBitW(unsigned char devAddr, unsigned char regAddr, unsigned char bitNum, unsigned short data);
    static    boolean writeBits(unsigned char devAddr, unsigned char regAddr, unsigned char bitStart, unsigned char length, unsigned char data);
    static    boolean writeBitsW(unsigned char devAddr, unsigned char regAddr, unsigned char bitStart, unsigned char length, unsigned short data);
    static     boolean writeByte(unsigned char devAddr, unsigned char regAddr, unsigned char data);
    static     boolean writeWord(unsigned char devAddr, unsigned char regAddr, unsigned short data);
    static     boolean writeBytes(unsigned char devAddr, unsigned char regAddr, unsigned char length, unsigned char *data);
    static     boolean writeWords(unsigned char devAddr, unsigned char regAddr, unsigned char length, unsigned short *data);

    static uint16 readTimeout;
};



#endif /* _I2CDEV_H_ */
