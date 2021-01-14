/************************************************
 *  Copyright (C) 2020 Tanway Technology Co., Ltd
 *  License:　BSD 3-Clause License
 *
 *  Created on: 15-07-2019
 *  Edited on: 26-11-2019
 *  Author: Elodie Shan
 *  Editor: LF Shen
 *
 *  UDP data utils for Tensor 3D LIDARs
**************************************************/


#ifndef RAWDATAUTILS_H_
#define RAWDATAUTILS_H_


  static inline long int FourHexToInt(unsigned char high, unsigned char highmiddle, unsigned char middle, unsigned char low)
  {
    long int addr = low & 0xFF;
    addr |= ((middle<<8) & 0xFF00);
    addr |= ((highmiddle<<16) & 0xFF0000);
    addr |= ((high<<24) & 0xFF000000);
    return addr;
  }

  static inline int HexToInt(unsigned char x)
  {
    int addr = x & 0xFF;
    return addr;
  }

  static inline int TwoHextoInt(unsigned char high,unsigned char low)
  {
    int addr = low & 0xFF;
    addr |= ((high<<8) & 0XFF00);
    return addr;
  }


#endif
