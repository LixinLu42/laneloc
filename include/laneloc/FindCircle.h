#ifndef _FINDCIRCLE_H_
#define _FINDCIRCLE_H_

#include <stdbool.h>
#include <stdlib.h>
#include <math.h>



#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/highgui/highgui_c.h"


#ifdef __cplusplus
    extern "C"{
#endif


	#define IMG_DN_X     	1280//112 //pixel number each row
	#define IMG_DN_Y    	460//112 //column number each frame
	#define accum_X			182//182 //before is 482
	#define accum_Y			182//182 //before is 482	


	#define IMG_UP_X     	1280//188 //pixel number each row
	#define IMG_UP_Y    	460//120 //column number each frame
	#define MAX_CIRCLE_NUMS 100//array size not bigger than 100
	/***
	#define IMG_DN_X     	112//112 //pixel number each row
	#define IMG_DN_Y    	112//112 //column number each frame
	#define accum_X			182//182 //before is 482
	#define accum_Y			182//182 //before is 482

	#define MAX_CIRCLE_NUMS 100//array size not bigger than 100
	#define DSPSDRAMLocaltion  
	#define DSPL2RAMLocaltion  
	#define DSPSDRAMBank3     

	//

	#define DSPSDRAMLocaltion section("sdram0_data")
	#define DSPL2RAMLocaltion section ("L2_sram")

	#define DSP_SDRAM_BANK0   section("sdram_bank0")
	#define DSPSDRAMLocaltion section("sdram_bank1")
	#define DSPSDRAMBank2     section("sdram_bank2")
	//#define DSPSDRAMBank3     section("sdram_bank3")
	#define DSPL2RAMLocaltion section ("L2_sram_uncached")
	//

	#define IMG_ROWS			(120)//(120)//(480)	//(448) //
	#define IMG_COLS			(188)//(188) /(752)/(448) //
	#define  BLACK				(0)
	#define  WHITE				(255)
	//#define  BAR_RESIZE		(172)

	#define U6_MAX_ROW_EDGE_NUM  16  //max segment count per row
	#define U8_MAX_LABEL_SUM     24  //max search rect count
	#define U8_SEG_LIST_MAX      250 //segment list max,please set to <255

	#define AREA_FILTER_EN0 0 //area fliter enable 
	#define BW_LOOP			4
	#define BW_THRES		(9-BW_LOOP)

	#define COMMON_UP		0
	#define COMMON_RIGHT	1
	#define COMMON_DOWN		2
	#define COMMON_LEFT		3

	#define ABS(x)		((x)>0?(x):-(x))
	#define min(X,Y)	(((X) < (Y)) ? (X) : (Y))
	#define max(X,Y)	(((X) > (Y)) ? (X) : (Y))

	//#define IMG_UP_X     	448 //pixel number each row
	//#define IMG_UP_Y    	448 //column number each frame

	#define IMG_UP_X     	188//188 //pixel number each row
	#define IMG_UP_Y    	120//120 //column number each frame
	***/
	






	static  char edge[IMG_DN_X*IMG_DN_Y];  // binary edge image
	static  unsigned char uEdge[IMG_DN_X*IMG_DN_Y];
	static  char gradient[IMG_DN_X*IMG_DN_Y]; // gradient image
	static  char gx[IMG_DN_X*IMG_DN_Y];
	static  char gy[IMG_DN_X*IMG_DN_Y];  // edge intensity of x and y axis
	//static DSPSDRAMBank3 int normal[IMG_DN_X*IMG_DN_Y ];
	static  unsigned short accumulator_matrix[accum_X*accum_Y];
	//static DSPSDRAMBank3 int accumulator[IMG_DN_X*IMG_DN_Y ];
	static  unsigned short Accumulator_filter[IMG_DN_X*IMG_DN_Y];
	static  bool matrix_flag[IMG_DN_X*IMG_DN_Y];
	static  unsigned int integralImg[IMG_UP_Y][IMG_UP_X];//»ý·ÖÊý¾Ý//752*480

	static  short gx_sht[IMG_UP_Y*IMG_UP_X];
	static  short gy_sht[IMG_UP_Y*IMG_UP_X];








	typedef struct
	{
		unsigned char *buffer;
		int cols;
		int rows;
	}stImage;

	typedef struct
	{
		int cols;//image width
		int rows;//image height
	}stImageSize;

	typedef struct
	{
		int x;
		int y;
	} stPoint;

	typedef struct //ax + by + c = 0
	{
		double a;
		double b;
		double c;
	} stLine;

	typedef struct
	{
		int sx;
		int sy;
		int ex;
		int ey;
	}stSegment;

	typedef struct
	{
		int baseX;
		int baseY;
		int width;
		int height;
	} stRect;

	typedef enum
	{	//clockwise
		DIR_UP = 0x00,
		DIR_RIGHT = 0x01,
		DIR_DOWN = 0x02,
		DIR_LEFT = 0x03
	}enDirection;

	typedef enum
	{	//
		UP_DOWN_LINE_PAIRS = 0x00,
		LEFT_RIGHT_LINE_PAIRS = 0x01
	}enLinePairs;

	typedef enum
	{	//clockwise
		LEFT_UP = 0x00,
		UP_RIGHT = 0x01,
		RIGHT_DOWN = 0x02,
		DOWN_LEFT = 0x03
	}enCornerDirection;

	typedef struct
	{
		int leftNum;
		float leftWeight;
		int rightNum;
		float rightWeight;
	} stSampleInfo;

	typedef enum
	{
		eCtrlMode_Dn_BlackSquare = 0x00, //mode1
		eCtrlMode_Dn_Decode = 0x03, //mode3
		eCtrlMode_Up_BlackSquare = 0x02, //mode2
		eCtrlMode_Up_Decode = 0x01	 //mode4
	}enControlMode;

	typedef enum
	{
		eBarcode_Shift_Rect = 0x10,	//shelf rect 	
		eBarcode_Shift_Decode = 0x11,	//shelf decode
		eBarcode_Ground_Rect = 0x21,	//ground rect
		eBarcode_Ground_Decode = 0x22,	//ground decode
	}enBarcodeType;

	typedef enum
	{
		eConfidenceItem_LineNums = 0x00,	//	
		eConfidenceItem_CornerNums = 0x01,	//

	}enConfItem;

	typedef struct
	{
		unsigned int AbsolutePositionX;
		unsigned int AbsolutePositionY;
		signed int   Angle;
		unsigned short int QRCodeX;
		unsigned short int QRCodeY;
		unsigned int ImageNo;

		enBarcodeType BarcodeType;	//¶þÎ¬ÂëÀàÐÍ:	
		unsigned short int ShiftCode;
		unsigned short int DecodeX;
		unsigned short int DecodeY;
		unsigned short int BarcodeCenterX;
		unsigned short int BarcodeCenterY;
		unsigned short int FrameDelay;	//´ÓÊÕµ½Ö÷¿ØÅÄÕÕÖÐ¶Ïµ½ÓÐÐ§Ö¡¿ªÊ¼ÅÄÕÕµÄÊ±¼äÏûºÄ
		unsigned short int crc;
	}tagPostureInfo;

	typedef enum
	{
		DOWN_DECODE_NONE = 0, //down sensor & no decode
		DOWN_DECODE_ALL = 1, //down sensor & decode
		UP_DECODE_NONE = 2, //up sensor & no decode
		UP_DECODE_ALL = 3, //up sensor & decode
	}enDecodeMode;

	typedef struct SEGMENT
	{
		unsigned short iRow;
		unsigned short u9_left;
		unsigned short u9_right;
		unsigned short iSegIndex;
		unsigned short valid;
	}SEGMENT;

	typedef struct MyRect
	{
		short u9_left;
		short u8_top;
		short u9_right;
		short u8_bottom;
	}MyRect;

	typedef struct
	{
		bool bPos;//valid positoin flag
		struct MyRect  rect; //Mark Rect
		int x;
		int y;
		double evangle; //evaluate angle
		double evconf;  //evaluate confidence
		double angle;   //real angle
		double conf;    //real confidence
		double adangle; //adjust angle
		double adconf;  //adjust confidence
		double caangle; //calc angle
		double caconf;  //calc confidence
		double trangle; //transfer angle
		int absangle; //abs angle
		bool bCode; //valid angle flag
		int code;  //code data
		int corn; //valid corn index
		int line; //patch line index
		bool bLPos; //LPos found
		bool bReAngle; //re calc angle
		int ldist[4];//line distance
	}POSITION_INFO;

	typedef struct
	{
		stPoint point;
		double angle;
		char msg[20];
	}stImgPosture;

	//confidence coefficient
	typedef struct
	{
		int mSampleNums[4];//4 direction sample nums
		int mSampleWeight[4]; // nums/(max nums)
		int mLineNums; //how many lines are fitted
		int mCornerNums; //how many corners are found
		int mParallelCheck[2]; //if 2 parallel check have run 
		unsigned char mErrorCode; //which stage the prog goes to
		unsigned char mConfidenceValue;//output coefficient value; calculate at last
		unsigned char lowconfidencecause;//low confidencecause cause 0 cornernums else 1
	}stConfidence;

	typedef struct
	{
		int current_point_number;
		int max_point_number;
		stPoint* sample_point;
	}stSamplePoint;

	void InitSamplePoint(stPoint* sample_point_memory, stSamplePoint* sample);
	extern void AddSamplePoint(const stPoint* point, stSamplePoint* sample);
	extern stPoint GetSamplePointMember(const stSamplePoint* sample, int index);
	extern int GetSamplePointQuantity(const stSamplePoint* sample);
	extern int CalCorner2SamplesAveralDis(stPoint stCorner, stPoint* stSamplePts, int SampleNums);
	extern void ConfidenceProcess(stConfidence* pConfidence);

	// the circle structure
	struct Circle
	{
		float centre_x;    // the x-coordinate of centre of the circle, the unit is pixel 
		float centre_y;    // the y-coordinate of centre of the circle, the unit is pixel
		float radius;      // the radius of the circle, the unit is pixel
		float confidence;  // the confidence score
	};

	// the circles structure
	struct Circles
	{
		int n;                        // the number of circles
		struct Circle circles[MAX_CIRCLE_NUMS]; // circles
	};

	// a 2D pixel
	struct Pixel
	{
		short r;  // the row index
		short c;  // the column index
	};

	// connected components
	struct CC
	{
		int n;                          // the number of pixels in the connected component
		struct Pixel component[1000];   // a list of pixels belongs to the connected component
	};

	// a list of connected components
	struct CCs
	{
		int n;                         // the number of connected components
		struct CC components[200];     // a list of components, supports at most 100
	};





	// c version of Matlab function imfindcircles
	// Warning: no arguments checking and memory checking. Make sure all arguments are legal and reasonable
	//          and the input image is not too large (resolution is not large).


	struct Circles* imfindcircles_m(stImage* pstImage, int radius_min, int radius_max,float sensitivity);
	int add(int a, int b);

	// Auxilary functions
	// extract sobel edges
	/*void getSobelEdge(int *input, int rows, int cols,
					  int *ret_edge_img, int *ret_gradient, int *ret_gx, int *ret_gy);*/
	void getSobelEdge(stImage* pstImage, char *ret_edge_img, char *ret_gradient, char *ret_gx, char *ret_gy);

	// compute accumulator array
	void computeAccumulator(char *edge_img, char *gradient, char *gx, char *gy, stImageSize* pImgSize,
		int radius_min, int radius_max, unsigned short *ret_accumulator_matrix);

	// compute centres of circles
	struct Circles* computeCentres(unsigned short *accumulator, stImageSize* ImgSize, int radius_max, float sensitivity, int medfilter_size);
	//struct Circles* computeCentres(int *accumulator, int *accumulator_matrix, int radius_max, int medfilter_size);
	//bool computeCentres(int *accumulator, int *accumulator_matrix, int radius_max, int medfilter_size, struct Circles* pCircleIn);
	// estimate radii of circles
	void computeRadii(struct Circles* pCircles, char *gradient,
		int rows, int cols, int radius_min, int radius_max,
		float *ret_radii);

	float otsuThreshold(int* pPxlCnt, float* pPxlRate, int row, int col);
	// utility functions
	// get the median of an array of number;

	void AdaptiveThresholdPartOpt(int* input, int height, int width, int* bin, int block, int ratio);

#ifdef __cplusplus
    }
#endif



#endif
