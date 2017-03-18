#ifndef _6AXIS_MOTIONAPPS20_H_
#define _6AXIS_MOTIONAPPS20_H_

#include "I2cPort.h"
#include "helper_3dmath.h"
#include "AllDevices.h"

// MotionApps 2.0 DMP implementation, built using the MPU-6050EVB evaluation board
#define INCLUDE_DMP_MOTIONAPPS20


using namespace cacaosd_i2cport;
// Tom Carpenter's conditional PROGMEM code
// http://forum.arduino.cc/index.php?topic=129407.0
// Teensy 3.0 library conditional PROGMEM code from Paul Stoffregen
#ifndef __PGMSPACE_H_
	#define __PGMSPACE_H_ 1
	#include <inttypes.h>

	#define PROGMEM
	#define PGM_P  const char *
	#define PSTR(str) (str)
	#define F(x) x

	typedef void prog_void;
	typedef char prog_char;
	typedef unsigned char prog_uchar;
	typedef int8_t prog_int8_t;
	typedef uint8_t prog_uint8_t;
	typedef int16_t prog_int16_t;
	typedef uint16_t prog_uint16_t;
	typedef int32_t prog_int32_t;
	typedef uint32_t prog_uint32_t;
	
	#define strcpy_P(dest, src) strcpy((dest), (src))
	#define strcat_P(dest, src) strcat((dest), (src))
	#define strcmp_P(a, b) strcmp((a), (b))
	
	#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
	#define pgm_read_word(addr) (*(const unsigned short *)(addr))
	#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
	#define pgm_read_float(addr) (*(const float *)(addr))
	
	#define pgm_read_byte_near(addr) pgm_read_byte(addr)
	#define pgm_read_word_near(addr) pgm_read_word(addr)
	#define pgm_read_dword_near(addr) pgm_read_dword(addr)
	#define pgm_read_float_near(addr) pgm_read_float(addr)
	#define pgm_read_byte_far(addr) pgm_read_byte(addr)
	#define pgm_read_word_far(addr) pgm_read_word(addr)
	#define pgm_read_dword_far(addr) pgm_read_dword(addr)
	#define pgm_read_float_far(addr) pgm_read_float(addr)
#endif

/* Source is from the InvenSense MotionApps v2 demo code. Original source is
 * unavailable, unless you happen to be amazing as decompiling binary by
 * hand (in which case, please contact me, and I'm totally serious).
 *
 * Also, I'd like to offer many, many thanks to Noah Zerkin for all of the
 * DMP reverse-engineering he did to help make this bit of wizardry
 * possible.
 */

// NOTE! Enabling DEBUG adds about 3.3kB to the flash program size.
// Debug output is now working even on ATMega328P MCUs (e.g. Arduino Uno)
// after moving string constants to flash memory storage using the F()
// compiler macro (Arduino IDE 1.0+ required).

//#define DEBUG
#ifdef DEBUG
    #define DEBUG_PRINT(x) std::cout << x
    #define DEBUG_PRINTF(x, y) std::cout << y << x
    #define DEBUG_PRINTLN(x) std::cout << x << std::endl
    #define DEBUG_PRINTLNF(x, y) std::cout << y << x << std::endl
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTF(x, y)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTLNF(x, y)
#endif

#define DMP_CODE_SIZE       1929    // dmpMemory[]
#define DMP_CONFIG_SIZE     192     // dmpConfig[]
#define DMP_UPDATES_SIZE    47      // dmpUpdates[]

uint8_t MPU6050::dmpInitialize();

bool MPU6050::dmpPacketAvailable();

// uint8_t MPU6050::dmpSetFIFORate(uint8_t fifoRate);
// uint8_t MPU6050::dmpGetFIFORate();
// uint8_t MPU6050::dmpGetSampleStepSizeMS();
// uint8_t MPU6050::dmpGetSampleFrequency();
// int32_t MPU6050::dmpDecodeTemperature(int8_t tempReg);

//uint8_t MPU6050::dmpRegisterFIFORateProcess(inv_obj_func func, int16_t priority);
//uint8_t MPU6050::dmpUnregisterFIFORateProcess(inv_obj_func func);
//uint8_t MPU6050::dmpRunFIFORateProcesses();

// uint8_t MPU6050::dmpSendQuaternion(uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendGyro(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendAccel(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendLinearAccel(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendLinearAccelInWorld(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendControlData(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendExternalSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendPacketNumber(uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendQuantizedAccel(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendEIS(uint_fast16_t elements, uint_fast16_t accuracy);

uint8_t MPU6050::dmpGetAccel(int32_t *data, const uint8_t* packet);
}
uint8_t MPU6050::dmpGetAccel(int16_t *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetAccel(VectorInt16 *v, const uint8_t* packet);
uint8_t MPU6050::dmpGetQuaternion(int32_t *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetQuaternion(int16_t *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetQuaternion(Quaternion *q, const uint8_t* packet);
// uint8_t MPU6050::dmpGet6AxisQuaternion(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetRelativeQuaternion(long *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetGyro(int32_t *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetGyro(int16_t *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetGyro(VectorInt16 *v, const uint8_t* packet);
// uint8_t MPU6050::dmpSetLinearAccelFilterCoefficient(float coef);
// uint8_t MPU6050::dmpGetLinearAccel(long *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);
// uint8_t MPU6050::dmpGetLinearAccelInWorld(long *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);
// uint8_t MPU6050::dmpGetGyroAndAccelSensor(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetGyroSensor(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetControlData(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetTemperature(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetGravity(long *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetGravity(VectorFloat *v, Quaternion *q);
// uint8_t MPU6050::dmpGetUnquantizedAccel(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetQuantizedAccel(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetExternalSensorData(long *data, int size, const uint8_t* packet);
// uint8_t MPU6050::dmpGetEIS(long *data, const uint8_t* packet);

uint8_t MPU6050::dmpGetEuler(float *data, Quaternion *q);
uint8_t MPU6050::dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);

// uint8_t MPU6050::dmpGetAccelFloat(float *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetQuaternionFloat(float *data, const uint8_t* packet);

uint8_t MPU6050::dmpProcessFIFOPacket(const unsigned char *dmpData);
uint8_t MPU6050::dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed);

// uint8_t MPU6050::dmpSetFIFOProcessedCallback(void (*func) (void));

// uint8_t MPU6050::dmpInitFIFOParam();
// uint8_t MPU6050::dmpCloseFIFO();
// uint8_t MPU6050::dmpSetGyroDataSource(uint_fast8_t source);
// uint8_t MPU6050::dmpDecodeQuantizedAccel();
// uint32_t MPU6050::dmpGetGyroSumOfSquare();
// uint32_t MPU6050::dmpGetAccelSumOfSquare();
// void MPU6050::dmpOverrideQuaternion(long *q);
uint16_t MPU6050::dmpGetFIFOPacketSize();

#endif /* _6AXIS_MOTIONAPPS20_H_ */