#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <laneloc/FindCircle.h>


#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/highgui/highgui_c.h"

IplImage* Img_Edge;
IplImage* img_component;



#define FILE_DEBUG	(0)


// [c version of Matlab function imfindcircles]
// This function takes grayscale image as input and return all detected circles (centre, radiu, confidence)
// based on input parametres (radis range, edge threshold and circle sensitiviey). The centre of detected
// circle can be out of image range.
// Warning: no arguments checking and memory checking. Make sure all arguments are legal and reasonable
//          and the input image is not too large (resolution is not large).
// @Param input: the input grayscale image
// @Param rows, cols: the number of rows and columns of input image
// @Param radius_min, radius_max: the range of radii
// @Param is_dark_object: specifies whether the object is darker than the background
// @Param sensitivity: specifies the sensitivity factor in the range 0~1 for finding circles. 
//                     A high sensitivity value leads to detecting more circles, including weak or 
//                     partially obscured ones, at the risk of a higher false detection rate
// @Param edge_threshold: the gradient threshold for determining edge pixels
// @RETURN: circles found in the input image

//add by wang



	float radii[100];//20 maybe overflow;==circle.ns
	const int c_r = 1;//compress rate

	static struct Circles circleslast;
	static struct Circles g_Circles_a;
	static struct Circles circles;
	static struct CCs ccs;
	static struct Circles ret_circles;
	static struct CC cc;


	int add(int a, int b){
		printf("a+b: ", a+b);
		return a+b;
	}

	double PI =3.1415926;

	struct Circles* pCircles = NULL;

	struct Circles* imfindcircles_m(stImage* pstImage, int radius_min, int radius_max, float sensitivity)
	{
			
		printf("no problme!5");

		int n, i, j;
				
		stImageSize iImgSize;
		int rows = pstImage->rows;
		int cols = pstImage->cols;

		printf("rows:%d ",rows);
		printf("cols:%d ",cols);


		unsigned char* input = pstImage->buffer;
		clock_t a, b;
		
		// edge detection to create binary edge 2d array
		memset(edge, 0, (rows*cols));

		memset(gradient, 0, (rows*cols) * sizeof(unsigned char));
		memset(gx, 0, (rows*cols) * sizeof(unsigned char));
		memset(gy, 0, (rows*cols) * sizeof(unsigned char));
		//memset(normal, 0, (rows*cols) * sizeof(int));
		
		a = clock();
		getSobelEdge(pstImage, edge, gradient, gx, gy);//148,755,812//97,196488
		b = clock();

		double c = (double)(b - a)/ CLOCKS_PER_SEC * 1000.0;
		printf("   1.soble time   = %fms\n", c);

		for (i = 0; i < rows; i++)
			for (j = 0; j < cols; j++)
			{
				if (1 == edge[i*cols + j])
					uEdge[i*cols + j] = 0xFF;
				else
					uEdge[i*cols + j] = 0;
			}

		Img_Edge = cvCreateImage(cvSize( 1280, 460), IPL_DEPTH_8U, 1);

		Img_Edge->imageData = gx;
		cvShowImage("gx", Img_Edge);		
		cvSaveImage("/home/llx/geekplus/cut_video_handle/bird/gx.jpg",Img_Edge);


		Img_Edge->imageData = gy;
		cvShowImage("gy", Img_Edge);		
		cvSaveImage("/home/llx/geekplus/cut_video_handle/bird/gy.jpg",Img_Edge);


		Img_Edge->imageData = gradient;
		cvShowImage("gradient", Img_Edge);		
		cvSaveImage("/home/llx/geekplus/cut_video_handle/bird/gradient.jpg",Img_Edge);

		//Img_Edge->imageData = uEdge;
		Img_Edge->imageData = edge;
		cvSaveImage("/home/llx/geekplus/cut_video_handle/bird/edge.jpg",Img_Edge);

		cvShowImage("edge_img", Img_Edge);		


		//cvSmooth(Img_Edge, Img_Edge, CV_MEDIAN, 3, 3, 0, 0);

		memset((void*)uEdge, 0, IMG_DN_X*IMG_DN_Y);




		// do Hough Transform to get accumulator array
		// the accumulator matrix recording CRH voting

		int accum_rows = rows + radius_max + radius_max;
		int accum_cols = cols + radius_max + radius_max;

		iImgSize.cols = cols;
		iImgSize.rows = rows;

		a = clock();

		computeAccumulator(edge, gradient, gx, gy, &iImgSize, radius_min, radius_max, accumulator_matrix);//340,829,806//199,429787



		b = clock();
		c = (double)(b - a)/ CLOCKS_PER_SEC * 1000.0;
		printf("   2.accu time    = %fms\n", c);

		// do connected components and weighted summing to get centroids
		int medfilter_size = 5;
		iImgSize.cols = cols;
		iImgSize.rows = rows;

		a = clock();
		pCircles = computeCentres(&accumulator_matrix[0], &iImgSize, radius_max, sensitivity, medfilter_size);//790,252,216//51,255639
		b = clock();



		c = (double)(b - a)/ CLOCKS_PER_SEC * 1000.0;
		printf("   3.center time  = %fms\n", c);

		int cirn = 0;
		printf("   num:   %d\n", pCircles->n);

		for (i = 0; i < pCircles->n; i++)
		{
			if (pCircles->circles[cirn].confidence > 0)//1000(not change)//1500
			{
				circleslast.circles[cirn].radius = radii[i];
				circleslast.circles[cirn].centre_x = c_r * pCircles->circles[i].centre_x;
				circleslast.circles[cirn].centre_y = c_r * pCircles->circles[i].centre_y;
				circleslast.circles[cirn].confidence = pCircles->circles[i].confidence;
				cirn++;
				printf("conf_centres: %f",circleslast.circles[cirn].confidence);
			}
		}

		circleslast.n = cirn;

		// use annulus accumulator array to estimate radii
		a = clock();
		computeRadii(&circleslast, gradient, c_r * rows, c_r * cols, c_r * radius_min, c_r * radius_max, radii);//70,361,665
		b = clock();
		c = (double)(b - a) / CLOCKS_PER_SEC * 1000.0;
		printf("   4. radius time = %fms\n", c);



		cirn = 0;
		for (i = 0; i < circleslast.n; i++)
		{
			if (circleslast.circles[i].confidence > 0)//1000(not change)
			{
				//printf("%f", radii[i]);
				pCircles->circles[i].radius = radii[i];
				//printf(" %f", circleslast.circles[i].centre_x);
				pCircles->circles[i].centre_x = circleslast.circles[i].centre_x;
				pCircles->circles[i].centre_y = circleslast.circles[i].centre_y;
				pCircles->circles[i].confidence = circleslast.circles[i].confidence;
				cirn++;
				printf("conf_radii: %f", pCircles->circles[i].confidence);

			}

		}
		pCircles->n = cirn;
		//system("pause");
		printf("  return:   %d\n", pCircles->n);

		return (pCircles);

	}




	// extract sobel edges
	// @Param input: the grayscale image
	// @Param rows, cols: the number of rows and columns of input image 
	// @Param edge_threshold: the threshold that specifies a pixel is an edge pixel
	// @RETURN Param ret_edge_img: the binary edge image
	// @RETURN Param ret_gradient: the gradient edge image
	// @RETURN Param ret_gx: the gradient intensity w.r.t x-axis
	// @RETURN Param ret_gy: the gradient intensiry w.r.t y-axis
	int g_edgeCnt = 0;
	#define GRAY_SCALE 256
	int pixelCount[GRAY_SCALE];
	float pixelPro[GRAY_SCALE];

	void getSobelEdge(stImage* pstImage, char *ret_edge_img, char *ret_gradient, char *ret_gx, char *ret_gy)
	{

		printf("no problem! 7");

		float edge_threshold = 0.0;
	
		int rows = pstImage->rows;
		int cols = pstImage->cols;

		printf("rows:%d ",rows);
		printf("cols:%d ",cols);

		
		unsigned char* input = pstImage->buffer;

		int i, j, pixelSum = rows * cols;
		int i_up;
		int i_down;
		int j_left;
		int j_right;
		int pixel_now_value = 0;
		int pixel_idx = 0;
		const int sobel_size = 3;
		const int half_sobel_size = 3 - 1 / 2;
		int gx_tmp = 0;
		int gy_tmp = 0;
		int upcolmulti = 0;
		int dncolmulti = 0;
		int icolmulti = 0;
		float max_float = 0.0;
		float max_float_reciprocal = 0.0;
		int max_int = 0;
		int threshold = 0;
		int gz_tmp = 0;
		//float normalFactor = 1 / (1.414 * 511) * 127;
		int normalFactor = 0;//= 0.175766 * 1024;//(1.414 * 511) * 127;*2^18
		normalFactor = 0.175766 * 1024;//(1.414 * 511) * 127;*2^18
		
		//init paras to 0
		memset(pixelCount, 0, GRAY_SCALE * sizeof(int));
		
		memset(pixelPro, 0, GRAY_SCALE * sizeof(float));

		memset(gx_sht, 0, rows*cols * sizeof(short));
		
		memset(gy_sht, 0, rows*cols * sizeof(short));


		// compute edge (gradient intensity)
		//float max_float = 0.0;
		int gx_max = 0;
		int gy_max = 0;
		int gz_max = 0;
		for (i = 0 + half_sobel_size; i < rows - half_sobel_size; i++)	//43,246,578
			for (j = 0 + half_sobel_size; j < cols - half_sobel_size; j++)
			{
				pixel_idx = i * cols + j;

				i_up = i - 1;
				i_down = i + 1;
				j_left = j - 1;
				j_right = j + 1;

				upcolmulti = i_up * cols;
				dncolmulti = i_down * cols;
				icolmulti = i * cols;
				gx_tmp = input[upcolmulti + j_left] - input[upcolmulti + j_right] +
					input[icolmulti + j_left] * 2 - input[icolmulti + j_right] * 2 +
					input[dncolmulti + j_left] - input[dncolmulti + j_right];
				gy_tmp = input[upcolmulti + j_left] - input[dncolmulti + j_left] +
					input[upcolmulti + j] * 2 - input[dncolmulti + j] * 2 +
					input[upcolmulti + j_right] - input[dncolmulti + j_right];

				//ret_gx[pixel_idx] = gx_tmp;//overflow
				//ret_gy[pixel_idx] = gy_tmp;
				gx_sht[pixel_idx] = gx_tmp;//overflow
				gy_sht[pixel_idx] = gy_tmp;
				if (gx_tmp > gx_max)
					gx_max = gx_tmp;
				if (gy_tmp > gy_max)
					gy_max = gy_tmp;

				//ret_gradient[pixel_idx] = sqrt(gx_tmp * gx_tmp + gy_tmp * gy_tmp);
			}
			
		gz_max = sqrt(gx_max*gx_max + gy_max * gy_max) + 1;


		normalFactor = 1048576 * 255 / gz_max;//(1.414 * 511) * 127;*2^18
		for (i = 0 + half_sobel_size; i < rows - half_sobel_size; i++)	//43,246,578
			for (j = 0 + half_sobel_size; j < cols - half_sobel_size; j++)
			{
				pixel_idx = i * cols + j;

				gx_tmp = gx_sht[pixel_idx];
				gy_tmp = gy_sht[pixel_idx];;

				gx_tmp = gx_tmp * normalFactor;//normalization:1/(1.414*510)*255
				gy_tmp = gy_tmp * normalFactor;

				ret_gx[pixel_idx] = gx_tmp = gx_tmp >> 20;
				ret_gy[pixel_idx] = gy_tmp = gy_tmp >> 20;

				gz_tmp = gx_tmp * gx_tmp + gy_tmp * gy_tmp;
				gz_tmp = sqrt(gz_tmp);
				ret_gradient[pixel_idx] = gz_tmp;

			}
	
		/*int gz_total = 0;
		int gz_cnt = 0;
		for (i = 0; i < rows*cols; i++)
			if (ret_gradient[i])
			{
				gz_total += ret_gradient[i];
				gz_cnt++;
			}*/
	#if 0

			//search the max gradient value//±ß½çÓ¦¸ÃÏÈinit to 0//14,315153//4268551
			/*for (i = 0 ; i < rows*cols; i++)
			{
				if (ret_gradient[i] > max_int)
					max_int = ret_gradient[i];
			}*/

			/*max_float = max_int;
			max_float_reciprocal = 1.0/max_int;
			float fNormalization = 255 * max_float_reciprocal;*/
		for (i = 0; i < rows; i++)//64,516254//64308543
		{
			for (j = 0; j < cols; j++)
			{
				pixelCount[ret_gradient[i*cols + j]]++;
			}
		}

		//¼ÆËãÃ¿¸öÏñËØÔÚÕû·ùÍ¼ÏñÖÐµÄ±ÈÀý  
		for (i = 0; i < 256; i++)
		{
			pixelPro[i] = (float)pixelCount[i] / pixelSum;
		}

		edge_threshold = otsuThreshold(pixelCount, pixelPro, rows, cols);//18,295476

		// threshold edges
		edge_threshold = edge_threshold * 255;

		printf("edge-thres: %.2f \n", edge_threshold);
	#else
			//edge_threshold = gz_total / gz_cnt;
		edge_threshold = 30;
	#endif



		//cvThreshold	
		int iEdgeThres = (int)edge_threshold;

	#if (!DSP_HARDWARE_)
		int k, l, tmp = 0;
		for (k = 0; k < rows; k++)//4,473149
		{
			for (l = 0; l < cols; l++)
			{
				tmp = k * cols + l;
				if (ret_gradient[tmp] > iEdgeThres)
				{
					ret_edge_img[tmp] = 255;
					g_edgeCnt++;
				}
				else
				{
					ret_edge_img[tmp] = 0;
				}
			}
		}
	#else
		//bf609 thresholding
		//cvThreshold( ret_gradient, ret_edge_img, 50.0, 255.0, CV_THRESH_BINARY );
		ADI_CVThresholdWrapper(ret_gradient, ret_edge_img, (rows*cols), cols, rows,
			CV_THRESH_BINARY, IPL_DEPTH_8U, iEdgeThres, 255);
	#endif

	}

	/***
	float otsuThreshold(int* pPxlCnt, float* pPxlRate, int row, int col)
	{
		int width = col;
		int height = row;

		int* pixelCount = pPxlCnt;
		float* pixelPro = pPxlRate;
		int i, j, pixelSum = 448 * 448, threshold = 0;

		//±éÀú»Ò¶È¼¶[0,255]  
		float w0, w1, u0tmp, u1tmp, u0, u1, u,
			deltaTmp, deltaMax = 0;
		for (i = 0; i < 256; i++)
		{
			w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
			for (j = 0; j < 256; j++)
			{
				if (j <= i)   //±³¾°²¿·Ö  
				{
					w0 += pixelPro[j];
					u0tmp += j * pixelPro[j];
				}
				else   //Ç°¾°²¿·Ö  
				{
					w1 += pixelPro[j];
					u1tmp += j * pixelPro[j];
				}
			}
			u0 = u0tmp / w0;
			u1 = u1tmp / w1;
			u = u0tmp + u1tmp;
			deltaTmp =
				w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
			if (deltaTmp > deltaMax)
			{
				deltaMax = deltaTmp;
				threshold = i;
			}
		}

		float fThreshold = (float)((float)(threshold + 1) / 255.0);
		return fThreshold;
	}

	***/
	// compute accumulator array using Circular Hough Transform
	// @Param edge_img: the binary edge image
	// @Param gradient: the gradient of image
	// @Param gx: the gradient intensity w.r.t x-axis
	// @Param gy: the gradient intensity w.r.t y-axis
	// @Param rows, cols: the number of rows and columns of input image
	// @Param radius_min, radius_max: the range of radius
	// @Param is_dark_object: specify whether the circle is darker than background
	// @RETURN Param ret_accumulator_matrix: the accumulator matrix. size is [rows + 2 x radius_max, cols + 2 x radius_max] 
	//                                       It can be larger than the size of image. At most larger than radius_max from each side.


	//int radius_setx_q[9]={13, 13.5, 14, 14.5, 15, 15.5, 16, 16.5, 17};
	//int radius_sety_q[9]={13, 13.5, 14, 14.5, 15, 15.5, 16, 16.5, 17};
	//const static int radius_setx_q0[9] = { -26, -27, -28, -29, -30, -31, -32, -33, -34 };
	//const static int radius_sety_q0[9] = { -26, -27, -28, -29, -30, -31, -32, -33, -34 };
	//int radius_setx_q[9] = { -26, -27, -28, -29, -30, -31, -32, -33, -34 };
	//int radius_sety_q[9] = { -26, -27, -28, -29, -30, -31, -32, -33, -34 };
	//-------------------------not change img-----------------------------//
	//const static int radius_setx_q0[9] = { -252, -253, -254, -255, -256, -257, -258, -259, -260 };
	//const static int radius_sety_q0[9] = { -252, -253, -254, -255, -256, -257, -258, -259, -260 };
	//int radius_setx_q[9] = { -252, -253, -254, -255, -256, -257, -258, -259, -260 };
	//int radius_sety_q[9] = { -252, -253, -254, -255, -256, -257, -258, -259, -260 };
	//-------------------------1/2 change img-----------------------------//
	//const static int radius_setx_q0[9] = { -124, -125, -126, -127, -128, -129, -130, -131, -132 };
	//const static int radius_sety_q0[9] = { -124, -125, -126, -127, -128, -129, -130, -131, -132 };
	//int radius_setx_q[9] = { -124, -125, -126, -127, -128, -129, -130, -131, -132 };
	//int radius_sety_q[9] = { -124, -125, -126, -127, -128, -129, -130, -131, -132 };
	//-------------------------1/4 change img-----------------------------//
	const static int radius_setx_q0[9] = {  -50, -51, -52 ,-53, -54, -55, -56, -57, -58 };
	const static int radius_sety_q0[9] = { -50, -51, -52 ,-53, -54, -55, -56, -57, -58 };
	int radius_setx_q[9] = { -50, -51, -52 ,-53, -54, -55, -56, -57, -58};
	int radius_sety_q[9] = {  -50, -51, -52 ,-53, -54, -55, -56, -57, -58 };
	//-------------------------1/8 change img-----------------------------//
	/*const static int radius_setx_q0[9] = { -24, -25, -26, -27, -28, -29, -30, -31, -32 };
	const static int radius_sety_q0[9] = { -24, -25, -26, -27, -28, -29, -30, -31, -32 };
	int radius_setx_q[9] = { -24, -25, -26, -27, -28, -29, -30, -31, -32 };
	int radius_sety_q[9] = { -24, -25, -26, -27, -28, -29, -30, -31, -32 };*/
	static const float VOTE_WEIGHT_FACTOR = 10000.0;
	void computeAccumulator(char *edge_img, char *gradient, char *gx, char *gy, stImageSize* pImgSize,
		int radius_min, int radius_max, unsigned short*ret_accumulator_matrix)
	{
		// normaliser (circumference) for every radius
		int num_of_radius = (radius_max - radius_min) * 2 + 1;  // the resolution of radius is 0.5 pixels
		float radius_set[9];
		int weight_set[9];
		float r = (float)radius_min;
		int i;
		int cols = pImgSize->cols;
		int rows = pImgSize->rows;

		for (i = 0; i < num_of_radius; i++)
		{
			radius_set[i] = -r;
			//weight_set[i] = (int)((float)10000.0 / (float)(2.0 * PI * r));
			//----notchang----//weight_set[i] = (int)((float)85000.0 / (float)(2.0 * PI * r));
			weight_set[i] = (int)((float)80000.0 / (float)(c_r * 2.0 * PI * r));
			r += 0.5;
		}

		// initialise ret_accumulator_matrix	
		int expanded_width = radius_max + radius_max;
		memset(ret_accumulator_matrix, 0, ((rows + expanded_width)*(cols + expanded_width)) * sizeof(unsigned short));

		int x, y, xc, yc;
		int pxlNow = 0;
		float gradient_reciprocal = 0.0;
		float XGradient = 0.0;
		float YGradient = 0.0;
		int XGradient_q = 0;
		int YGradient_q = 0;
		int gx_tmp = 0;
		int gy_tmp = 0;
		int gg = 0;
		int gx_gg = 0;
		int gy_gg = 0;
		// vote to accumulator matrix pixel by pixel

		for (y = 0; y < rows; y++)
			for (x = 0; x < cols; x++)
			{
				if (edge_img[y*cols + x] == 0)
					continue;

				pxlNow = y * cols + x;

				// for each edge pixel, add to accumulator
	#if 1
	#if 1
				gx_tmp = gx[pxlNow];
				gy_tmp = gy[pxlNow];
				gg = gradient[pxlNow];

				if (0 == gg)
					continue;
				gg = 4096 / gg;//2^12

				gx_gg = gx_tmp * gg;
				gy_gg = gy_tmp * gg;
				//gx_tmp = gx_tmp * 32768;// 2 ^ 15;
				//gy_tmp = gy_tmp * 32768;//2 ^ 15;
				//gradient_reciprocal = 1.0 / (float)gradient[pxlNow];
				//XGradient = (gx[pxlNow] * gradient_reciprocal);
				//YGradient = (gy[pxlNow] * gradient_reciprocal);
				//XGradient_q = XGradient*16384;//2^14
				//YGradient_q = YGradient*16384;//2^14
				//for (i = 0; i < num_of_radius; i++)
				//{
				//	radius_setx_q[i] = radius_setx_q0[i] * gx_tmp / (gg);//2^15
				//	radius_sety_q[i] = radius_setx_q0[i] * gy_tmp / (gg);
				//}

				for (i = 0; i < num_of_radius; i++)
				{
					radius_setx_q[i] = radius_setx_q0[i] * gx_gg;
					radius_sety_q[i] = radius_sety_q0[i] * gy_gg;//kb:before revise, radius_sety_q0 is x
					xc = (x - (radius_setx_q[i] >> 13));//2^15
					yc = (y - (radius_sety_q[i] >> 13));
					// out of parametre space				
					if (xc > 0 && yc > 0 && xc < cols && yc < rows)
						ret_accumulator_matrix[(yc)*(cols)+xc] += weight_set[i];//1/ÖÜ³¤
				}

	#else
				pxlNow = y * cols + x;
				gradient_reciprocal = 1.0 / (float)gradient[pxlNow];
				XGradient = (gx[pxlNow] * gradient_reciprocal);
				YGradient = (gy[pxlNow] * gradient_reciprocal);

				for (i = 0; i < num_of_radius; i++)
				{
					xc = ((x - radius_set[i] * XGradient));
					yc = ((y - radius_set[i] * YGradient));
					// out of parametre space
					if (xc + radius_max > 0 && yc + radius_max > 0 && xc + radius_max < cols + expanded_width && yc + radius_max < rows + expanded_width)
						ret_accumulator_matrix[(yc + radius_max)*(cols + expanded_width) + xc + radius_max] += weight_set[i];
				}
	#endif
	#else

				pxlNow = y * cols + x;
				gradient_reciprocal = 1.0 / (float)gradient[pxlNow];
				XGradient = (gx[pxlNow] * gradient_reciprocal);
				YGradient = (gy[pxlNow] * gradient_reciprocal);

				//0.
				xc = (int)(((float)x - radius_set[0] * XGradient) + 0.5);
				yc = (int)(((float)y - radius_set[0] * YGradient) + 0.5);
				// out of parametre space
				if (xc + radius_max > 0 && yc + radius_max > 0 && xc + radius_max < cols + expanded_width && yc + radius_max < rows + expanded_width)
					ret_accumulator_matrix[(yc + radius_max)*(cols + expanded_width) + xc + radius_max] += weight_set[0];
				//1.
				xc = (int)(((float)x - radius_set[1] * XGradient) + 0.5);
				yc = (int)(((float)y - radius_set[1] * YGradient) + 0.5);
				// out of parametre space
				if (xc + radius_max > 0 && yc + radius_max > 0 && xc + radius_max < cols + expanded_width && yc + radius_max < rows + expanded_width)
					ret_accumulator_matrix[(yc + radius_max)*(cols + expanded_width) + xc + radius_max] += weight_set[1];
				//2.
				xc = (int)(((float)x - radius_set[2] * XGradient) + 0.5);
				yc = (int)(((float)y - radius_set[2] * YGradient) + 0.5);
				// out of parametre space
				if (xc + radius_max > 0 && yc + radius_max > 0 && xc + radius_max < cols + expanded_width && yc + radius_max < rows + expanded_width)
					ret_accumulator_matrix[(yc + radius_max)*(cols + expanded_width) + xc + radius_max] += weight_set[2];
				//3.
				xc = (int)(((float)x - radius_set[3] * XGradient) + 0.5);
				yc = (int)(((float)y - radius_set[3] * YGradient) + 0.5);
				// out of parametre space
				if (xc + radius_max > 0 && yc + radius_max > 0 && xc + radius_max < cols + expanded_width && yc + radius_max < rows + expanded_width)
					ret_accumulator_matrix[(yc + radius_max)*(cols + expanded_width) + xc + radius_max] += weight_set[3];
				//4.
				xc = (int)(((float)x - radius_set[4] * XGradient) + 0.5);
				yc = (int)(((float)y - radius_set[4] * YGradient) + 0.5);
				// out of parametre space
				if (xc + radius_max > 0 && yc + radius_max > 0 && xc + radius_max < cols + expanded_width && yc + radius_max < rows + expanded_width)
					ret_accumulator_matrix[(yc + radius_max)*(cols + expanded_width) + xc + radius_max] += weight_set[4];
				//5.
				xc = (int)(((float)x - radius_set[5] * XGradient) + 0.5);
				yc = (int)(((float)y - radius_set[5] * YGradient) + 0.5);
				// out of parametre space
				if (xc + radius_max > 0 && yc + radius_max > 0 && xc + radius_max < cols + expanded_width && yc + radius_max < rows + expanded_width)
					ret_accumulator_matrix[(yc + radius_max)*(cols + expanded_width) + xc + radius_max] += weight_set[5];
				//6.
				xc = (int)(((float)x - radius_set[6] * XGradient) + 0.5);
				yc = (int)(((float)y - radius_set[6] * YGradient) + 0.5);
				// out of parametre space
				if (xc + radius_max > 0 && yc + radius_max > 0 && xc + radius_max < cols + expanded_width && yc + radius_max < rows + expanded_width)
					ret_accumulator_matrix[(yc + radius_max)*(cols + expanded_width) + xc + radius_max] += weight_set[6];
				//7.
				xc = (int)(((float)x - radius_set[7] * XGradient) + 0.5);
				yc = (int)(((float)y - radius_set[7] * YGradient) + 0.5);
				// out of parametre space
				if (xc + radius_max > 0 && yc + radius_max > 0 && xc + radius_max < cols + expanded_width && yc + radius_max < rows + expanded_width)
					ret_accumulator_matrix[(yc + radius_max)*(cols + expanded_width) + xc + radius_max] += weight_set[7];
				//8.
				xc = (int)(((float)x - radius_set[8] * XGradient) + 0.5);
				yc = (int)(((float)y - radius_set[8] * YGradient) + 0.5);
				// out of parametre space
				if (xc + radius_max > 0 && yc + radius_max > 0 && xc + radius_max < cols + expanded_width && yc + radius_max < rows + expanded_width)
					ret_accumulator_matrix[(yc + radius_max)*(cols + expanded_width) + xc + radius_max] += weight_set[8];
	#endif
			}
	}



	// compute centres of circles using connected component method to get clusters and 
	// weighted sum all points to get centroids
	// @Param accumulator_matrix: the accumulator matrix
	// @Param mat_rows, mat_cols, the number of rows and columns of accumulator_matrix
	// @Param radius_max: the maximum radius of circles
	// @Param sensitivity: the sensitivity score that specifies a maxima is a centre of circle
	// @Param medfilter_size: the size of median filter
	// @RETURN Param ret_circles: circles with centres computed

	static struct Pixel queue[1000];
	static int data_array[25];
	static int order_idx[20];
	struct Circles* computeCentres(unsigned short *pAccumulator, stImageSize* ImgSize, int radius_max, float sensitivity, int medfilter_size)
	{
		int mat_rows = ImgSize->rows;
		int mat_cols = ImgSize->cols;
		int half_medfilter_size = (medfilter_size - 1) / 2;
		int medfilter_area = medfilter_size * medfilter_size;
		int half_midArea = (medfilter_area - 1) / 2;
		int i, j, k, a, b, m;
		int start_i;
		int start_j;
		int end_i;
		int end_j;
		int data_id;
		int bTemp;
		int cc_id = 0;

	#if (FILE_DEBUG)
		FILE *fp1 = fopen("pAccumulator.txt", "w");
		for (i = 0; i < mat_rows; i++)
		{
			for (j = 0; j < mat_cols; j++)
			{
				fprintf(fp1, "%d,", pAccumulator[i*mat_cols + j]);
			}
			fprintf(fp1, "\n");
		}
		fclose(fp1);
	#endif

	#if (!DSP_HARDWARE_)
		memcpy(Accumulator_filter, pAccumulator, (mat_rows*mat_cols) * sizeof(int));
	#else
		int nInputDataLen = 8;
		int nInputBytes = (mat_cols * mat_rows) * nInputDataLen;
		ADI_CVSmoothWrapper((void *)pAccumulator,
			(void *)Accumulator_filter,
			nInputBytes + (((1 + 1) * mat_cols)) * nInputDataLen,
			mat_cols,
			mat_rows,
			CV_MEDIAN,
			IPL_DEPTH_16U,
			ADI_CVSMOOTH_MEDIAN_3x3,
			ADI_CVSMOOTH_PARAM2_DEFAULT,
			ADI_CVSMOOTH_PARAM3_DEFAULT,
			ADI_CVSMOOTH_PARAM4_DEFAULT);
	#endif


	#if 0
		memset(Accumulator_filter, 0, (mat_rows*mat_cols) * sizeof(int));

	#if 0

	#if 0
		//median filter//777,250,918
		for (i = 0; i < mat_rows; i++)
		{
			for (j = 0; j < mat_cols; j++)
			{
				memset(data_array, 0, 25 * sizeof(4));

				start_i = i - half_medfilter_size;
				if (start_i < 0)
					start_i = 0;
				start_j = j - half_medfilter_size;
				if (start_j < 0)
					start_j = 0;
				end_i = i + half_medfilter_size;
				if (end_i >= mat_rows)
					end_i = mat_rows - 1;
				end_j = j + half_medfilter_size;
				if (end_j >= mat_cols)
					end_j = mat_cols - 1;

				data_id = 0;
				for (a = start_i; a <= end_i; a++)
				{
					for (b = start_j; b <= end_j; b++)
					{
						data_array[data_id] = pAccumulator[a*mat_cols + b];
						data_id++;
					}
				}

				//bubble sort
				for (m = 0; m < medfilter_area - 1; m++)
				{
					for (k = 0; k < medfilter_area - m - 1; k++)
					{
						if (data_array[k] > data_array[k + 1])
						{
							//exchange
							bTemp = data_array[k];
							data_array[k] = data_array[k + 1];
							data_array[k + 1] = bTemp;
						}
					}
				}

				// ¼ÆËãÖÐÖµ
				if ((medfilter_area & 1) > 0)
				{
					// Êý×éÓÐÆæÊý¸öÔªËØ£¬·µ»ØÖÐ¼äÒ»¸öÔªËØ
					bTemp = data_array[half_midArea];
				}
				else
				{
					// Êý×éÓÐÅ¼Êý¸öÔªËØ£¬·µ»ØÖÐ¼äÁ½¸öÔªËØÆ½¾ùÖµ
					bTemp = (data_array[medfilter_area / 2] + data_array[medfilter_area / 2 + 1]) / 2;
				}
				Accumulator_filter[i*mat_cols + j] = bTemp;
				//Accumulator_filter[i*mat_cols + j] = pAccumulator[i*mat_cols + j];
			}

		}
	#else
		//mean filter//
	#if 1
		for (i = 0 + half_medfilter_size; i < mat_rows - half_medfilter_size; i++)
		{
			for (j = 0 + half_medfilter_size; j < mat_cols - half_medfilter_size; j++)
			{

				/*start_i = i - half_medfilter_size;
				if (start_i < 0)
					start_i = 0;
				start_j = j - half_medfilter_size;
				if (start_j < 0)
					start_j = 0;
				end_i = i + half_medfilter_size;
				if (end_i >= mat_rows)
					end_i = mat_rows - 1;
				end_j = j + half_medfilter_size;
				if (end_j >= mat_cols)
					end_j = mat_cols - 1;*/
				start_i = i - half_medfilter_size;
				start_j = j - half_medfilter_size;
				end_i = i + half_medfilter_size;
				end_j = j + half_medfilter_size;

				data_id = 0;
				data_array[0] = 0;
				for (a = start_i; a <= end_i; a++)
				{
					for (b = start_j; b <= end_j; b++)
					{
						data_array[0] += pAccumulator[a*mat_cols + b];
						data_id++;
					}
				}
				bTemp = data_array[0] / data_id;

				Accumulator_filter[i*mat_cols + j] = bTemp;
			}

		}
	#else
	//»ý·ÖÍ¼ÊµÏÖ
		AdaptiveThresholdPartOpt(pAccumulator, mat_rows, mat_cols, Accumulator_filter, medfilter_size, 0);
	#endif
	#endif
	#else
		unsigned char* src = NULL;
		unsigned char* dst = NULL;

		memcpy((void*)Accumulator_filter, (void*)pAccumulator, mat_cols * mat_rows * sizeof(int));
		//for (i = 0; i < mat_rows; i++)
		//{
		//	src = (unsigned char*)(&pAccumulator[i*mat_cols]);
		//	//dst = (unsigned char*)&Accumulator_filter[17 * accum_X + i * accum_X + 17];
		//	dst = (unsigned char*)(&Accumulator_filter[i*mat_cols]);
		//	memcpy((void*)dst, (void*)src, mat_cols * sizeof(int));
		//}

	#endif

	#if (FILE_DEBUG)
		FILE *fp2 = fopen("Accumulator_filter.txt", "w");
		for (i = 0; i < mat_rows; i++)
		{
			for (j = 0; j < mat_cols; j++)
			{
				fprintf(fp2, "%d,", Accumulator_filter[i*mat_cols + j]);
			}
			fprintf(fp2, "\n");
		}
		fclose(fp2);
	#endif

	#endif


		// suppresses all maxima and get connected components
		int suppress_threshold = 1500;
		// initialise matrix_sup and matrix_flag
		memset(matrix_flag, false, (mat_rows*mat_cols * sizeof(bool)));


		// suppress and find connected components//12,949,368
		int adjacent_x[8] = { -1, -1, -1,  0, 0,  1, 1, 1 };
		int adjacent_y[8] = { -1,  0,  1, -1, 1, -1, 0, 1 };
		int queue_head = 0;
		int queue_tail = 1;
		int pxl_now_idx = 0;
		int pxl_now_value = 0;
		cc_id = 0;

		for (i = 1; i < mat_rows - 1; i++)
			for (j = 1; j < mat_cols - 1; j++) // no consideration on outline (1 pixel)
			{
				pxl_now_idx = i * mat_cols + j;//pixel index
				pxl_now_value = Accumulator_filter[pxl_now_idx];//pixel value

				// has searched
				if (matrix_flag[pxl_now_idx] == true)
					continue;
				// do not consider those pixels under suppress_threshold
				if (pxl_now_value < suppress_threshold)
				{
					matrix_flag[pxl_now_idx] = true;
					continue;
				}
				// is not maxima
				if (pxl_now_value < Accumulator_filter[(i - 1)*mat_cols + j - 1] ||
					pxl_now_value < Accumulator_filter[(i - 1)*mat_cols + j] ||
					pxl_now_value < Accumulator_filter[(i - 1)*mat_cols + j + 1] ||
					pxl_now_value < Accumulator_filter[pxl_now_idx - 1] ||
					pxl_now_value < Accumulator_filter[pxl_now_idx + 1] ||
					pxl_now_value < Accumulator_filter[(i + 1)*mat_cols + j - 1] ||
					pxl_now_value < Accumulator_filter[(i + 1)*mat_cols + j] ||
					pxl_now_value < Accumulator_filter[(i + 1)*mat_cols + j + 1])
					continue;

				//struct Pixel queue[1000];
				queue_head = 0;
				queue_tail = 1;
				queue[queue_head].r = i;
				queue[queue_head].c = j;
				int component_id = 1;
				cc.component[0] = queue[0];
				matrix_flag[i*mat_rows + j] = true;//col or row?????

				int k, kk, next_r, next_c, cur_x, cur_y, nnext_r, nnext_c;
				bool is_maxima;
				while (queue_head < queue_tail)
				{
					// dequeue
					cur_x = queue[queue_head].r;
					cur_y = queue[queue_head].c;
					cc.component[component_id] = queue[queue_head];
					component_id += 1;
					queue_head += 1;
					// 8 connected pixles
					for (k = 0; k < 8; k++)
					{
						next_r = cur_x + adjacent_x[k];
						next_c = cur_y + adjacent_y[k];
						// out of boundary
						if (next_r < 0 || next_r >= mat_rows || next_c < 0 || next_c >= mat_cols)
							continue;
						// has searhced
						if (matrix_flag[next_r*mat_cols + next_c])
							continue;
						// less than suppress_threshold
						if (Accumulator_filter[next_r*mat_cols + next_c] < suppress_threshold)
							continue;
						// is not maxima
						is_maxima = true;
						for (kk = 0; kk < 8; kk++)
						{
							nnext_r = next_r + adjacent_x[kk];
							nnext_c = next_c + adjacent_x[kk];
							if (nnext_r >= 0 && nnext_r < mat_rows && nnext_c >= 0 && nnext_c < mat_cols)
							{
								if (matrix_flag[nnext_r*mat_cols + nnext_c])
									continue;

								if (Accumulator_filter[nnext_r*mat_cols + nnext_c] > Accumulator_filter[next_r*mat_cols + next_c])
								{
									is_maxima = false;
									break;
								}
							}
						}
						if (!is_maxima)
							continue;

						// enqueue
						queue[queue_tail].r = next_r;
						queue[queue_tail].c = next_c;
						matrix_flag[next_r*mat_cols + next_c] = true;
						queue_tail += 1;
					} // end for k = (0,8)
					cc.n = component_id;
				} // end while queue

				if (cc_id <= MAX_CIRCLE_NUMS)//·ÀÖ¹Êý×éÔ½½ç
				{
					ccs.components[cc_id] = cc;
					cc_id += 1;
				}
				else
				{
					printf("\n array almost overflow \n");
				}
			}

		printf("cc_id: %d", cc_id);
		if (cc_id > MAX_CIRCLE_NUMS)
			cc_id = MAX_CIRCLE_NUMS;
		ccs.n = cc_id;



	#if 0//(!DSP_HARDWARE_)
		//IplImage* img_component = cvCreateImage(cvSize(IMG_DN_X, IMG_DN_Y), IPL_DEPTH_8U, 1);
		cvZero(img_component);
		int ix = 0;
		int iy = 0;
		for (i = 0; i < ccs.n; i++)
			for (j = 0; j < ccs.components[i].n; j++)
			{
				ix = ccs.components[i].component[j].r;
				iy = ccs.components[i].component[j].c;
				img_component->imageData[ix*img_component->widthStep + iy] = 0xFF;
			}

		printf("ccs.n=cc_id= %d \n", cc_id);
	#endif

		// computer circles' centre: centroids of connected components
		//struct Circles g_Circles_a;
		g_Circles_a.n = ccs.n;
		float avg_r = 0.0;
		float avg_c = 0.0;
		float sum_weight = 0.0;
		float maxconfidence = 0.0;
		int point_r, point_c;
		int tmp1 = 0;
		int tmp2 = 0;

		//for (i = 0; i < ccs.n; i++)//ccs component less than 100(struct define)
		int ForIndexMax = 100;
		if (ccs.n < 100)
			ForIndexMax = ccs.n;
		for (i = 0; i < ForIndexMax; i++)
		{
			avg_r = 0.0;
			avg_c = 0.0;
			sum_weight = 0.0;
			maxconfidence = 0.0;
			for (j = 0; j < ccs.components[i].n; j++)
			{

				point_r = ccs.components[i].component[j].r;
				point_c = ccs.components[i].component[j].c;
				tmp1 = point_r * mat_cols + point_c;
				avg_r += point_r * Accumulator_filter[tmp1];
				avg_c += point_c * Accumulator_filter[tmp1];
				sum_weight += Accumulator_filter[tmp1];
				if (maxconfidence < Accumulator_filter[tmp1])
					maxconfidence = Accumulator_filter[tmp1];
			}
			avg_r /= sum_weight;
			avg_c /= sum_weight;
			g_Circles_a.circles[i].centre_x = avg_c;
			g_Circles_a.circles[i].centre_y = avg_r;
			g_Circles_a.circles[i].confidence = maxconfidence - suppress_threshold;
			/*printf("/n Column");
			printf("%f", g_Circles_a.circles[i].centre_x);
			printf("/n ROW");
			printf("%f", avg_r);
			printf("/n MAX CONF");
			printf("%f", maxconfidence);
			system("pause");*/
			//sum_weight/ ccs.components[i].n-0.15;//add by wang
		}

	#if 1
		//¿ÉÒÔ²»¶Ô×îÖÕ½á¹ûÅÅÐò
		// sort the circles to descending order w.r.t confidence score
		// pop sort
		for (i = 0; i < g_Circles_a.n; i++)
			order_idx[i] = i;
		for (i = 0; i < g_Circles_a.n - 1; i++)
		{
			for (j = 1; j < g_Circles_a.n - i; j++)
			{
				if (g_Circles_a.circles[order_idx[j - 1]].confidence < g_Circles_a.circles[order_idx[j]].confidence)
				{
					int temp = order_idx[j - 1];
					order_idx[j - 1] = order_idx[j];
					order_idx[j] = temp;
				}
			}
		}

		ret_circles.n = g_Circles_a.n;

		for (i = 0; i < g_Circles_a.n; i++)
		{
			ret_circles.circles[i].centre_x = g_Circles_a.circles[order_idx[i]].centre_x;
			ret_circles.circles[i].centre_y = g_Circles_a.circles[order_idx[i]].centre_y;
			ret_circles.circles[i].confidence = g_Circles_a.circles[order_idx[i]].confidence;

			//printf("%d", i);
			//system("pause");
			//printf("%d", ret_circles.circles[i].confidence);
			//system("pause");
		}
	#endif
		return (&ret_circles);
		//return (&g_Circles_a);
	}


	// estimate radii of circles using circle histogram
	// @Param circles: circles with centres determined
	// @Param gradient: the gradient image
	// @Param rows, cols: the number of rows and columns of the image
	// @Param radius_min, radius_max: the radii range
	// @RETURN Param: the radii corresponds to circles
	int histogram[9];//num_of_bins
	float weight[9];
	void computeRadii(struct Circles* pCircles, char *gradient,
		int rows, int cols, int radius_min, int radius_max,
		float *ret_radii)
	{
		int i, j, c;
		float r;
		int start_i;
		int end_i;
		int start_j;
		int end_j;
		float centre_r;
		float centre_c;
		float dist;
		int dist_square = 0;
		int bin, max, max_id;
		int radius_min_square = radius_min * radius_min;
		int radius_max_square = radius_max * radius_max;
		int num_of_bins = (radius_max - radius_min) * 2 + 1;

		for (i = 0; i < num_of_bins; i++)
		{
			r = (float)radius_min + (float)i * 0.5;
			weight[i] = 1.0 / (2.0 * PI * r);
		}

		for (c = 0; c < pCircles->n; c++)//9
		{
			for (i = 0; i < num_of_bins; i++)
				histogram[i] = 0;

			start_i = (int)(pCircles->circles[c].centre_y - (float)radius_max + 0.5);
			if (start_i < 0)
				start_i = 0;
			end_i = (int)(pCircles->circles[c].centre_y + (float)radius_max + 0.5);
			if (end_i >= rows)
				end_i = rows - 1;
			start_j = (int)(pCircles->circles[c].centre_x - (float)radius_max + 0.5);
			if (start_j < 0)
				start_j = 0;
			end_j = (int)(pCircles->circles[c].centre_x + (float)radius_max + 0.5);
			if (end_j >= cols)
				end_j = cols - 1;

			centre_r = pCircles->circles[c].centre_y;
			centre_c = pCircles->circles[c].centre_x;
			for (i = start_i; i <= end_i; i++)
			{
				for (j = start_j; j <= end_j; j++)
				{
	#if 0
					dist = sqrt(pow((float)i - centre_r, 2) + pow((float)j - centre_c, 2));
					if (dist >= radius_min && dist <= radius_max)
					{
						bin = (int)(((dist - (float)radius_min) / 0.5) + 0.5);
						if (bin < 0)
							bin = 0;
						if (bin >= num_of_bins)
							bin = num_of_bins - 1;
						histogram[bin] += gradient[i*rows + j];
					}
	#else
					dist_square = (int)(((float)i - centre_r)*((float)i - centre_r) + ((float)j - centre_c)*((float)j - centre_c));
					if (dist_square >= radius_min_square && dist_square <= radius_max_square)
					{
						bin = (int)(((sqrt(dist_square) - (float)radius_min) * 2) + 0.5);
						if (bin < 0)
							bin = 0;
						else if (bin >= num_of_bins)
							bin = num_of_bins - 1;
						histogram[bin] += gradient[i*rows + j];
					}
	#endif

				}
			}
			max = 0;
			max_id = 0;
			for (i = 0; i < num_of_bins; i++)
			{
				histogram[i] *= weight[i];
				if (histogram[i] > max)
				{
					max = histogram[i];
					max_id = i;
				}
			}
			ret_radii[c] = (float)radius_min + max_id * 0.5;
		} // end for

	}


	float getMedianNum(float *bArray, int iFilterLen)
	{
		int     i, j;            // Ñ­»·±äÁ¿
		float bTemp;

		// ÓÃÃ°ÅÝ·¨¶ÔÊý×é½øÐÐÅÅÐò
		for (j = 0; j < iFilterLen - 1; j++)
		{
			for (i = 0; i < iFilterLen - j - 1; i++)
			{
				if (bArray[i] > bArray[i + 1])
				{
					// »¥»»
					bTemp = bArray[i];
					bArray[i] = bArray[i + 1];
					bArray[i + 1] = bTemp;
				}
			}
		}

		// ¼ÆËãÖÐÖµ
		if ((iFilterLen & 1) > 0)
		{
			// Êý×éÓÐÆæÊý¸öÔªËØ£¬·µ»ØÖÐ¼äÒ»¸öÔªËØ
			bTemp = bArray[(iFilterLen - 1) / 2];
		}
		else
		{
			// Êý×éÓÐÅ¼Êý¸öÔªËØ£¬·µ»ØÖÐ¼äÁ½¸öÔªËØÆ½¾ùÖµ
			bTemp = (bArray[iFilterLen / 2] + bArray[iFilterLen / 2 + 1]) / 2;
		}

		return bTemp;
	}


	void AdaptiveThresholdPartOpt(int* input, int height, int width, int* bin, int block, int ratio)
	{
		int S = (block - 1) / 2;
		int i, j;
		long sum = 0;
		int index = 0;
		int x1, y1, x2, y2;

		if (0 == block % 2)//block must be odd
			return;

		// ¼ÆËã»ý·ÖÍ¼Ïñ	
		integralImg[0][0] = input[0];

		i = 0;
		for (j = 1; j < height; j++)
		{
			index = j * width;
			integralImg[j][i] = integralImg[j - 1][i] + input[index];
		}


		j = 0;
		for (i = 1; i < width; i++)
		{
			index = i;
			integralImg[j][i] = integralImg[j][i - 1] + input[index];
		}

		for (j = 1; j < height; j++)
		{
			for (i = 1; i < width; i++)
			{
				index = j * width + i;
				integralImg[j][i] = integralImg[j][i - 1] + integralImg[j - 1][i] - integralImg[j - 1][i - 1] + input[index];

			}
		}

	#if (0)
		FILE *fp3 = fopen("integralImg.txt", "w");
		for (i = 0; i < height; i++)
		{
			for (j = 0; j < width; j++)
			{
				fprintf(fp3, "%d,", integralImg[i][j]);
			}
			fprintf(fp3, "\n");
		}
		fclose(fp3);
	#endif

		for (i = (S + 1); i < (height - S); i++)
		{
			for (j = (S + 1); j < (width - S); j++)
			{
				index = i * width + j;

				// set the SxS region
				x1 = j - S - 1;
				x2 = j + S;
				y1 = i - S - 1;
				y2 = i + S;

				sum = integralImg[y2][x2] - integralImg[y1][x2] - integralImg[y2][x1] + integralImg[y1][x1];

				bin[index] = sum / (block*block);

			}
		}
	}



