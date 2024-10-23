#pragma once

typedef unsigned char  		UCHAR8;                  /** defined for unsigned 8-bits integer variable 	    无符号8位整型变量       */
typedef signed   char  		SCHAR8;                  /** defined for signed 8-bits integer variable		    有符号8位整型变量       */
typedef unsigned short 		USHORT16;                /** defined for unsigned 16-bits integer variable 	    无符号16位整型变量      */
typedef signed   short 		SSHORT16;                /** defined for signed 16-bits integer variable 	    有符号16位整型变量      */
typedef unsigned int   		UINT32;                  /** defined for unsigned 32-bits integer variable 	    无符号32位整型变量      */
typedef int   				SINT32;                  /** defined for signed 32-bits integer variable         有符号32位整型变量      */
typedef float          		FP32;                    /** single precision floating point variable (32bits)   单精度浮点数（32位长度） */
typedef double         		DB64;                    /** double precision floating point variable (64bits)   双精度浮点数（64位长度） */
typedef UCHAR8            u8;                      /** defined for unsigned 8-bits integer variable 	        无符号8位整型变量  */
typedef USHORT16          u16;                     /** defined for unsigned 16-bits integer variable 	    无符号16位整型变量 */
typedef UINT32            u32;                     /** defined for unsigned 32-bits integer variable 	    无符号32位整型变量 */
typedef SCHAR8            s8;                      /** defined for unsigned 8-bits integer variable 	        无符号8位整型变量  */
typedef SSHORT16          s16;                     /** defined for unsigned 16-bits integer variable 	    无符号16位整型变量 */
typedef SINT32            s32;                     /** defined for unsigned 32-bits integer variable 	    无符号32位整型变量 */


u32 Verify_CRC8_Check_Sum(u8 *pchMessage, u32 dwLength);
void Append_CRC8_Check_Sum(u8 *pchMessage, u32 dwLength);
u32 Verify_CRC16_Check_Sum(u8 *pchMessage, u32 dwLength);
void Append_CRC16_Check_Sum(u8 * pchMessage,u32 dwLength);

