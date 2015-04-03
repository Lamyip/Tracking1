#include "stdafx.h"
#include "stdlib.h"
#include <stdio.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <vector>
#include <math.h>
#include <algorithm>
#include "Filter.h"
#include "LIT.h"
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <fstream>

using namespace cv;
using namespace std;
using boost::asio::ip::udp;

boost::array<char, 530> ArtDMX;
bool DatatabInit;
vector<Point2f> VecAngle(12,Point2f(0.0,0.0));

// Main function
void Filter_Process(vector<Point2f> *ImgPts)
{
	vector<float> Xproj; // X coordinate light offset
	vector<float> Yproj; // Y coordinate light offset
	vector<float> Panproj; // Pan coordinate light offset
	vector<float> Tiltproj; // Tilt Coordinate Light offset
	ifstream File("Calibration.txt"); // retrieve Coordinate from calibration file
	if (!File.is_open())
	{
		cout << "Fichier de Calibration illisible ";
		waitKey(1000000);
	}
	while (File.good())
	{
		line_add(File, Xproj , Yproj, Panproj,Tiltproj);
	}
	File.close(); 

	/*****************************************************************************/
	//					Angle computation from x and y coordinate
	/*****************************************************************************/
	
	vector < Point2f > ::iterator it2 = VecAngle.begin();
	for (vector<Point2f>::iterator it = ImgPts->begin(); it != ImgPts->end(); it++)
	{
		*it2 = LIT_Angle(&(*(it)), Xproj[it - ImgPts->begin()], Yproj[it-ImgPts->begin()]);
		it2++;
	}
	cout << "Angle : \n ";
	VectDisplay(&VecAngle);
	//uchar DataTOSend[40];
	DataUDPSend(VecAngle,Panproj,Tiltproj); // send data through UDP Socket
	
}

//****************************************************************//
//		Functions													//
//***************************************************************//

//Read and retrieve offset value from text file
void line_add(ifstream& finput, vector<float>& Xcam, vector<float>& Ycam, vector<float>& Pan, vector<float>& Tilt)
{
	string value;
	getline(finput, value, ',');
	Xcam.push_back(stof(value));
	getline(finput, value, ',');
	Ycam.push_back(stof(value));
	getline(finput, value, ',');
	Pan.push_back(stof(value));
	getline(finput, value, '\n');
	Tilt.push_back(stof(value));
}

// Data RX command through UDP
void DataUDPSend(vector<Point2f> DataTab, vector<float> panoff, vector<float> tiltoff)
{
	boost::asio::io_service io_service;
	udp::endpoint receiver_endpoint(boost::asio::ip::address::from_string("2.0.0.1"), 6454); // Set UDP
	udp::socket socket(io_service); // (1)

	socket.open(udp::v4());

	if (!DatatabInit) // Reinitialization of Artnet Packet
	{
		for (int k = 0; k < 530; k++)
		{
			ArtDMX[k] = 0x00;
		}
		ArtDMX[0] = 'A'; ArtDMX[1] = 'r'; ArtDMX[2] = 't'; ArtDMX[3] = '-';  ArtDMX[4] = 'N'; ArtDMX[5] = 'e'; ArtDMX[6] = 't'; ArtDMX[7] = 0x00;
		ArtDMX[8] = 0x00;//Opcode HighByte
		ArtDMX[9] = 0x50;//Opcode HighByte
		ArtDMX[10] = 0x00;//Prover LowByte
		ArtDMX[11] = 0x0e;//Prover HighByte
		ArtDMX[12] = 0x00;//Sequence
		ArtDMX[13] = 0x00;// Physical
		ArtDMX[14] = 0x00;//SubUni
		ArtDMX[15] = 0x00;//UpUni
		ArtDMX[16] = 0x02;//Length
		ArtDMX[17] = 0x00;//Length2
		DatatabInit = true;
	}


	// 	if (DataTab.size() > 0)
	// 	{
	// 		int k = 0;
	// 		ushort pan = -(short)((((float)DataTab[0].x * 65025 / 530))) + (0x5700);//+ (short) Panproj.at(Light_No-1);
	// 
	// 		ushort tilt = +(short)((((float)DataTab[0].y) * 65025 / 280)) + (0x8800);// +(ushort)Tiltproj.at(Light_No - 1);
	// 		cout << tilt << endl;
	// 		ArtDMX[415 + 17 + k * 14] = (uchar)(pan >> 8);				// Pan
	// 		ArtDMX[415 + 17 + k * 14 + 1] = 0;	// Pan Fine
	// 		ArtDMX[415 + 17 + k * 14 + 2] = (uchar)(tilt >> 8);		// Tilt
	// 		ArtDMX[415 + 17 + k * 14 + 3] = 0;	// Tilt Fine
	// 	}
	//if (DataTab.size() > 0)
	//{
	//	int k = 0;
	//	ushort pan = -(short)((((float)DataTab[0].x * 65025 / 530))) + (0x7900);//+ (short) Panproj.at(Light_No-1);

	//	ushort tilt = +(short)((((float)DataTab[0].y) * 65025 / 280)) + (0x8800);// +(ushort)Tiltproj.at(Light_No - 1);
	//	cout << tilt << endl;
	//	ArtDMX[261 + 17 + k * 14] = (uchar)(pan >> 8);				// Pan
	//	ArtDMX[261 + 17 + k * 14 + 1] = 0;	// Pan Fine
	//	ArtDMX[261 + 17 + k * 14 + 2] = (uchar)(tilt >> 8);		// Tilt
	//	ArtDMX[261 + 17 + k * 14 + 3] = 0;	// Tilt Fine
	//}
	//if (DataTab.size() > 1)
	//{
	//	for (int k = 1; k < DataTab.size()&& k < 12; k++)
	//	{
	//		ushort pan = -(short)((((float)DataTab[k].x * 65025 / 530))) +(((short) (panoff[k])) << 8);
	//		ushort tilt = +(short)((((float)DataTab[k].y) * 65025 / 280)) + (((short)(tiltoff[k])) << 8);
	//		cout << tilt << endl;
	//		ArtDMX[261 + 17 + k * 14] = (uchar)(pan >> 8) ;				// Pan
	//		ArtDMX[261 + 17 + k * 14 + 1] = 0;	// Pan Fine
	//		ArtDMX[261 + 17 + k * 14 + 2] = (uchar)(tilt >> 8) ;		// Tilt
	//		ArtDMX[261 + 17 + k * 14 + 3] = 0;	// Tilt Fine
	//	}
	//}
	//ArtDMX[400] = (uchar)0x01;
	// 	for (int k = 0; k < DataTab.size(); k++)
	// 	{
	// 		ushort pan = (ushort)((((float)(DataTab[k].y + 360)) * 65025 / 530)+panoff[k]);
	// 		ArtDMX[400+17+k*4] = (uchar)(pan >> 8);				// Pan
	// 		ArtDMX[400+17 + k * 4 + 1] = (uchar)pan &(0x00FF);	// Pan Fine
	// 		ushort tilt = (uchar)((((float)DataTab[k].x+ 140 ) * 65025 / 280)+tiltoff[k]);
	// 		cout << hex << pan << " // " << hex << tilt << endl;
	// 		ArtDMX[400 +17 + k * 4 + 2] = (uchar)(tilt >> 8);		// Tilt
	// 		ArtDMX[400+17 + k * 4 + 3] = (uchar)tilt &(0x00FF);	// Tilt Fine
	// 		ArtDMX[500 + 17 + k] = 0xFF;
	// 	}

	// 	if (DataTab.size() > 0)
	// 	{
	// 		ushort pan = +(ushort)((((float)(DataTab[0].x)) * 65025 / 530)) + (0x5600);
	// 		ArtDMX[415 + 17] = (uchar)(pan >> 8);				// Pan
	// 		ArtDMX[416 + 17] = (uchar)pan &(0x00FF);	// Pan Fine
	// 		//ushort tilt = +(0x7000) - (ushort)((((float)DataTab[0].x) * 65025 / 280));
	// 		ushort tilt = -(0x4000) - (ushort)((((float)DataTab[0].y) * 65025 / 280));
	// 		cout << hex << pan << " // " << hex << tilt << endl;
	// 		ArtDMX[417 + 17] = (uchar)(tilt >> 8);		// Tilt
	// 		ArtDMX[418 + 17] = (uchar)tilt &(0x00FF);	// Tilt Fine
	// 	}
	// 	else
	// 	{
	// 		cout << hex << ArtDMX[415 + 17] << ArtDMX[416 + 17] << " // " << hex << ArtDMX[417 + 17] << ArtDMX[418 + 17] << endl;
	// 	}
	//if(DataTab.size()>1 )
	//{
	//	ushort pan = +(short)((((float)(DataTab[1].x)) * 65025 / 530)) + (0x7F00);
	//	ArtDMX[415+17] = (uchar)(pan >> 8);				// Pan
	//	ArtDMX[416 + 17] = (uchar)pan &(0x00FF);	// Pan Fine
	//	//ushort tilt = +(0x7000) - (ushort)((((float)DataTab[0].x) * 65025 / 280));
	//	ushort tilt = (0x7F00) + (short)((((float)DataTab[1].y) * 65025 / 280) );
	//	cout << hex << pan << " // " << hex << tilt << endl;
	//	ArtDMX[417 + 17] = (uchar)(tilt >> 8);		// Tilt
	//	ArtDMX[418 + 17] = (uchar)tilt &(0x00FF);	// Tilt Fine
	//	ArtDMX[421 + 17] = 0xFF;
	//	ArtDMX[439 + 17] = 0xFF;
	//	ArtDMX[440 + 17] = 0xFF;

	//}
	//else
	//{
	//	cout << hex << ArtDMX[415 + 17] << ArtDMX[416 + 17] << " // " << hex << ArtDMX[417 + 17] << ArtDMX[418 + 17] << endl;
	//}
	// 	if (DataTab.size() > 2)
	// 	{
	// 		ushort pan = (ushort)((((float)(DataTab[2].y)) * 65025 / 530) + panoff[1]) + (0x8600);
	// 		ArtDMX[415 + 17] = (uchar)(pan >> 8);				// Pan
	// 		ArtDMX[416 + 17] = (uchar)pan &(0x00FF);	// Pan Fine
	// 		ushort tilt = (uchar)((((float)DataTab[2].x + 140) * 65025 / 280) + tiltoff[1]);
	// 		cout << hex << pan << " // " << hex << tilt << endl;
	// 		ArtDMX[417 + 17] = (uchar)(tilt >> 8);		// Tilt
	// 		ArtDMX[418 + 17] = (uchar)tilt &(0x00FF);	// Tilt Fine
	// 	}
	// 	 		ushort pan = (ushort)((((float)(DataTab[0].y + 360)) * 65025 / 530)+panoff[0]);
	// 	 		ArtDMX[33+17] = (uchar)(pan >> 8);				// Pan
	// 	 		ArtDMX[33+17 + 1] = (uchar)pan &(0x00FF);	// Pan Fine
	// 	 		ushort tilt = (uchar)((((float)DataTab[0].x+ 140 ) * 65025 / 280)+tiltoff[0]);
	// 	 		cout << hex << pan << " // " << hex << tilt << endl;
	// 	 		ArtDMX[33 +17 + 2] = (uchar)(tilt >> 8);		// Tilt
	// 	 		ArtDMX[33+17  + 3] = (uchar)tilt &(0x00FF);	// Tilt Fine
	// 	 		ArtDMX[500 + 17 ] = 0xFF;
	//ArtDMX[33+17 ] = 0xFF;
	//ArtDMX[405] = 0x0D;
	// 	//Compteur++;
	// 	cout << hex << ArtDMX[0] << " " << hex << ArtDMX[1] << " " << hex << ArtDMX[2] << " " << hex << ArtDMX[3] << " " << hex << ArtDMX[4] << " " << hex << ArtDMX[5] << endl;


	// 	for (int l = 0; l < sizeof(DataTab); l++)
	// 	{
	// 		send_buf[l] = (uchar)l;
	// 		ushort pan = (ushort) ((*(DataTab+2*l)+360)*530/65025);
	// 		send_buf[l + 1] = (uchar)pan &(0xFF00) << 4;
	// 		send_buf[l+2] = (uchar) pan &(0x00FF);
	// 		ushort tilt = (ushort) (*(DataTab+2*l+1)*280/65025);
	// 		send_buf[l+3] = (uchar)tilt &(0xFF00) << 4;
	// 		send_buf[l+4] = (uchar) tilt &(0x00FF);
	// 
	// 	}

	//socket.send_to(boost::asio::buffer(ArtDMX), receiver_endpoint); // (3)
	socket.close();

	//}
	//waitKey(20);

}

void DataUDPReceive()
{
	for (;;)
	{
		//MedialonCommand.lock();
		try{
			//cout << " receiving";
		boost::asio::io_service io_service;
		MedialonCommand.activation = true;
		udp::endpoint receiver_endpoint(boost::asio::ip::address::from_string("2.0.0.1"), 7171);
		udp::socket socket(io_service); // (1)
		socket.open(udp::v4());
		boost::array<char, 128> recv_buf; // (4)
		udp::endpoint sender_endpoint;
		size_t len = socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint); // (5)
		socket.close();
		cout << recv_buf[0];
		cout << "hello\n";
		}
		catch(std::exception& e)
		{
		
		}
		//MedialonCommand.unlock();
	}

}

// Obsolete functions
void Filter_Sort(vector<Point2f>* InputVec, vector<Point2f>* OutputVec)
{
	float min = 0.0;
	int MinIndex = 0;
	for (vector<Point2f>::iterator it = InputVec->begin(); it != InputVec->end(); it++)
	{
		MinIndex = it - InputVec->begin();
		static bool InitFlag = true;
		if (it != InputVec->end())
		{
			for (vector<Point2f>::iterator it2 = OutputVec->begin() + (it - InputVec->begin()); it2 != OutputVec->end(); it2++)
			{
				float Value = Filter_Dist(*it, *it2);
				if (InitFlag)
				{
					min = Value;
					MinIndex = (it2 - OutputVec->begin());
					InitFlag = false;
				}
				if (Value < min)
				{
					min = Value;
					MinIndex = (it2 - OutputVec->begin());
				}
			}
			if (MinIndex != it - InputVec->begin())
			{
				Filter_Swap(InputVec, MinIndex, it - InputVec->begin());
			}
		}
	}
	// 	int TabInd[sizeof(*OutputVec)];
	// 	vector<Point2f> Result;
	// 	for (int k = 0; k < sizeof(*OutputVec); k++)
	// 		Result.push_back(Point2f());
	// 	for (vector<Point2f>::iterator it = OutputVec->begin(); it != OutputVec->end(); it++)
	// 	{
	// 		MinIndex = it - OutputVec->begin();
	// 		static bool InitFlag = true;
	// 		for (vector<Point2f>::iterator it2 = InputVec->begin(); it2 != InputVec->end(); it2++)
	// 		{
	// 			float Value = Filter_Dist(*it, *it2);
	// 			if (InitFlag)
	// 			{
	// 				min = Value;
	// 				TabInd[it - OutputVec->begin()] = 0;
	// 				MinIndex = 0;
	// 				InitFlag = false;
	// 			}
	// 			if (Value < min)
	// 			{
	// 				min = Value;
	// 				MinIndex = (it2 - OutputVec->begin());
	// 			}
	// 
	// 		}
	// 		TabInd[it - OutputVec->begin()] = MinIndex;
	// 	}
	// 	for (int k = 0; k < sizeof(*OutputVec);k++)
	// 	{
	// 		Result[]
	// 	}
}

void Filter_Swap(vector<Point2f>* Vec,int a, int b)
{
	Point2f Pointtmp;
	vector<Point2f>::iterator it = Vec->begin();
	Pointtmp = *(it + a);
	*(it + a) = *(it + b);
	*(it + b) = Pointtmp;

}

void Filter_Resize(vector<Point2f>* VecA, vector<Point2f>* VecB)
{
	if (VecA->size() > VecB->size())
	{
		int cond = (VecA->size() - VecB->size());
		for (int k=0; k < cond; k++)
		{
			VecB->push_back(Point2f());
		}
	} 
	else if (VecA->size() == VecB->size())
	{

	}
	else
	{
		int cond = (VecB->size() - VecA->size());
		for (int k=0; k < VecB->size() - VecA->size(); k++)
		{
			VecB->pop_back();
		}
	}
}

float Filter_Dist(Point2f PointA, Point2f PointB)
{
	return sqrt((PointA.x - PointB.x)*(PointA.x - PointB.x) + (PointA.y - PointB.y) *(PointA.y - PointB.y));
}

vector<Point2f> Filter_Sort2(vector<Point2f>* InputVec, vector<Point2f>* OutputVec)
{
	float min = 0.0;
	int MinIndex = 0;
	int TabIndex[50];
	vector<Point2f> ReturnVec;
	for (vector<Point2f>::iterator it = OutputVec->begin(); it != OutputVec->end(); it++)
	{
		MinIndex = it - OutputVec->begin();
		bool InitFlag = true;

		for (vector<Point2f>::iterator it2 = InputVec->begin(); it2 != InputVec->end(); it2++)
		{
			float Value = Filter_Dist(*it, *it2);
			if (InitFlag)
			{
				min = Value;
				InitFlag = false;
				MinIndex = (it2 - InputVec->begin());
			}
			if (Value < min)
			{
				min = Value;
				MinIndex = (it2 - InputVec->begin());
			}
		}
		TabIndex[it - OutputVec->begin()] = MinIndex ;

	}
	
	for (int k = 0; k < OutputVec->size(); k++)
	{
		ReturnVec.push_back(Point2f());
	}
	vector<Point2f>::iterator it2 = InputVec->begin();
	for (int k = 0; k < OutputVec->size(); k++)
	{
		ReturnVec[k] = *(it2+TabIndex[k]);
	}
	return ReturnVec;
}

void VectDisplay(vector<Point2f> *vinput)
{
	cout << "vecteur ->" << endl;
	for each (Point2f k in *vinput)
	{
		cout << "Pan: " << k.x << " Tilt: " << k.y << ' ' << endl;
	}
	cout << endl;
}