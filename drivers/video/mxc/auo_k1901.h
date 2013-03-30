#ifndef _AUO_K1901_H//[
#define _AUO_K1901_H




typedef struct tagK1901_VERSION {
	unsigned short wTemprature ;
	unsigned short wLotNumber;
	unsigned char bProductID;
	unsigned char bSample_Version;
	unsigned char bTCON_Version;
	unsigned char bP_WF_Version;// product & waveform version .
}K1901_VERSION;

int K1901_read_version(K1901_VERSION *O_ptK1901_version);


#endif //]_AUO_K1901_H

