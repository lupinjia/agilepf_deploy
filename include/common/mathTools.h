/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef MATHTOOLS_H
#define MATHTOOLS_H

#include <array>
#include <stdio.h>
#include <iostream>
#include "math.h"

inline float fmaxf(float x, float y){
    /// Returns maximum of x, y ///
    return (((x)>(y))?(x):(y));
    }

inline float fminf(float x, float y){
    /// Returns minimum of x, y ///
    return (((x)<(y))?(x):(y));
    }

inline float fmaxf3(float x, float y, float z){
    /// Returns maximum of x, y, z ///
    return (x > y ? (x > z ? x : z) : (y > z ? y : z));
    }

inline float fminf3(float x, float y, float z){
    /// Returns minimum of x, y, z ///
    return (x < y ? (x < z ? x : z) : (y < z ? y : z));
    }
    
inline void limit_norm(float *x, float *y, float limit){
    /// Scales the lenght of vector (x, y) to be <= limit ///
    float norm = sqrt(*x * *x + *y * *y);
    if(norm > limit){
        *x = *x * limit/norm;
        *y = *y * limit/norm;
        }
    }

inline int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }
    
    
inline float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }
	
union F32
{
	float v_float;
	unsigned int v_int;
	unsigned char buf[4];
}f32;

inline void float32_to_float16(float *float32,unsigned short int *float16)
{
	unsigned short int temp=0;
	f32.v_float=*float32;
//	*float16 = ((f32.v_int & 0x7fffffff) >> 13) - (0x38000000 >> 13);
//  *float16 |= ((f32.v_int & 0x80000000) >> 16);
	temp=(f32.buf[3]&0x7F)<<1|((f32.buf[2]&0x80)>>7);
	temp-=112;
	*float16=temp<<10|(f32.buf[2]&0x7F)<<3|f32.buf[1]>>5;
  *float16 |= ((f32.v_int & 0x80000000) >> 16);
}

inline void float16_to_float32(unsigned short int *float16,float *float32)
{
//	f32.v_int=*float16;
//	f32.v_int = ((f32.v_int & 0x7fff) << 13) + 0x7f000000;
//  f32.v_int |= ((*float16 & 0x8000) << 16);
//	*float32=f32.v_float;
	unsigned short int temp2=0;
	f32.v_int=0;
	temp2=(((*float16&0x7C00)>>10)+112);
	f32.buf[3]=temp2>>1;
	f32.buf[2]=((temp2&0x01)<<7)|(*float16&0x03FC)>>3;
	f32.buf[1]=(*float16&0x03)<<6;
	f32.v_int |= ((*float16 & 0x8000) << 16);
	*float32=f32.v_float;
}

/**
 * @brief wrap the angle to [-pi, pi]
 * @param angle the input angle
 * @return the wrapped angle
 */
float wrap_to_pi(float angle)
{
    while(angle > M_PI) angle -= 2*M_PI;
    while(angle < -M_PI) angle += 2*M_PI;
    return angle;
}

/**
 * @brief clip the value to the range [low_limit, high_limit]
 * @param value the input value
 * @param low_limit the lower limit
 * @param high_limit the upper limit
 * @return the clipped value
 */
float clip(float value, float low_limit, float high_limit)
{
    if(value <= low_limit) return low_limit;
    if(value >= high_limit) return high_limit;
    return value;
}

float max_abs(std::array<float, 6> arr)
{
    float max_abs = 0.0;
    for(int i=0; i<6; i++){
        if(fabs(arr[i]) > max_abs){
            max_abs = fabs(arr[i]);
        }
    }
    return max_abs;
}

template<typename T1, typename T2>
inline T1 max(const T1 a, const T2 b){
	return (a > b ? a : b);
}

template<typename T1, typename T2>
inline T1 min(const T1 a, const T2 b){
	return (a < b ? a : b);
}

#endif  // MATHTOOLS_H