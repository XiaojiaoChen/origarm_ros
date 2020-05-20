#ifndef MYDATA_H_
#define MYDATA_H_

#define seg 9
#define act 6
#define SEGMENTNUM 9
#define BELLOWNUM 6

typedef struct QUATERNION_TAG{
 /*unCompressed Quaternion*/
 int16_t imuData[4];
}QUATERNION;

typedef struct SENSORDATA_TAG {
	int16_t pressure;
	uint16_t distance;
	QUATERNION quaternion;
}SENSORDATA;

typedef struct COMMANDDATA_TAG {
	int16_t values[5];
	uint16_t commandType;
}COMMANDDATA;

enum COMMAND_MODE{openingCommandType, pressureCommandType};

typedef struct SPIDATA_T_TAG{
	SENSORDATA data[SEGMENTNUM][BELLOWNUM];
	char infos[10];
}SPIDATA_T;

typedef struct SPIDATA_R_TAG{
	COMMANDDATA data[SEGMENTNUM][BELLOWNUM];
	char infos[10];
}SPIDATA_R;


SPIDATA_T sensorData;
SPIDATA_R commandData;

//ABL->Pressure
float k0 = 1700;
float length0 = 0.055;
float radR = 0.06;
float radr = 0.02;
float crossA = 0.00126;           //M_PI*radr^2
float C1 = 6*k0*radR*0.5/crossA;


#endif // MYDATA_H__