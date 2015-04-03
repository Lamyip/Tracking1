#ifndef LIT_H
#define LIT_H

#include "stdafx.h"
#include "math.h"
#include "vector"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>


#define MaxDataLength 512

using namespace cv;
Point2f LIT_Angle(Point2f *Position,float xoff,float yoff);

typedef struct S_ArtDmx {
	uchar ID[8];                    // protocol ID = "Art-Net"
	ushort OpCode;                  // == OpOutput
	uchar ProtVerHi;                // 0
	uchar ProtVerLo;                // protocol version, set to ProtocolVersion
	uchar Sequence;			// 0 if not used, else 1-255 incrementing sequence number
	uchar Physical;                 // The physical i/p 0-3 that originated this packet. For Debug / info only
	uchar SubUni;                	// The low 8 bits of the 15 bit universe address.
	uchar Net;                	// The high 7 bits of the 15 bit universe address.
	ushort Length;			// BYTE-SWAPPED MANUALLY. Length of array below
	uchar Data[MaxDataLength];      // Variable length array. First entry is channel 1 level (NOT THE START CODE)
} T_ArtDmx;
//void LIT_Process(Point2f *Position, Point2f *Angle);
#endif // !LIT_H
