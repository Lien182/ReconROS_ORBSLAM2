

#include <iostream>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#define W 30
#define FAST_WINDOW_SIZE 50
#define IMAGE_CACHE_WIDTH   1280
#define IMAGE_CACHE_HEIGHT  150

extern "C" {
#include "reconos_thread.h"
#include "reconos_calls.h"
}

using namespace std;

static const int EDGE_THRESHOLD = 19;
static const int iniThFAST = 20;
static const int minThFAST = 7;

extern "C" THREAD_ENTRY(); // this is required because of the mixture of c and c++


#define read_next_lines {for(int __i = 0; __i < FAST_WINDOW_SIZE; __i++){ \
                                    _offset = (image_ptr + cache_cnt*(image_step))&3; \
                                    memcpy((void*)&image_data[(cache_cnt%IMAGE_CACHE_HEIGHT)*IMAGE_CACHE_WIDTH],(void*)((image_ptr + cache_cnt*(image_step))&(~3)), ((image_width+_offset+3)&(~3)));\
                                    cache_cnt+=1;} }


void mat_copy( uint8_t * data, cv::Mat&_dest, int iniY, int iniX, int cache_cnt,uint32_t image_ptr_offset, uint32_t image_width)
{
    for(int i = 0; i < (FAST_WINDOW_SIZE); i++)
    {
        uint32_t line_index = (iniY+i);
        uint32_t offset = (image_ptr_offset +  line_index*(image_width))&3;

        for(int j = 0; j < (FAST_WINDOW_SIZE); j++)
        {
            _dest.data[j+FAST_WINDOW_SIZE*i] = data[offset + (j+iniX)+ (line_index %IMAGE_CACHE_HEIGHT) *IMAGE_CACHE_WIDTH];
        }
    }
}

THREAD_ENTRY() {


    uint32_t nresults;

    THREAD_INIT();
	uint32_t initdata = (uint32_t)GET_INIT_DATA();
    uint8_t image_data[IMAGE_CACHE_HEIGHT*IMAGE_CACHE_WIDTH];

	struct mbox * request;
	struct mbox * response;

	if(initdata == 0)
	{
		request = resources_fast_request_0;
		response = resources_fast_response_0;
		printf("Hello from FPGA0\n");
	}		
	else
	{
		request = resources_fast_request_1;
		response = resources_fast_response_1;
		printf("Hello from FPGA1\n");
	}
		

	while(1)
	{
        uint32_t cache_cnt = 0;
        uint32_t nWrittenPoints = 0;
        uint32_t image_ptr    = MBOX_GET(request);
        uint32_t image_width  = MBOX_GET(request);
        uint32_t image_height = MBOX_GET(request);
        uint32_t image_step	  = MBOX_GET(request);
        uint32_t feature_dest = MBOX_GET(request);

		//printf("image_ptr=%x, image_width=%d, image_height=%d, feature_dest=%x \n", image_ptr, image_width, image_height, feature_dest);

        uint32_t image_ptr_offset = image_ptr & 3;
	    const int minBorderX = EDGE_THRESHOLD-3;
        const int minBorderY = minBorderX;
        const int maxBorderX = image_width -EDGE_THRESHOLD+3;
        const int maxBorderY = image_height-EDGE_THRESHOLD+3;

        const int width = (maxBorderX-minBorderX);
        const int height = (maxBorderY-minBorderY);

        const int wCell = 50;
        const int hCell = 50;
        const int nCols = width/50;
        const int nRows = height/50;

		vector<cv::KeyPoint>* vToDistributeKeys = (vector<cv::KeyPoint>*)feature_dest;

		uint32_t _offset = 0;

		//cv::Mat *Image = (cv::Mat*)image_ptr;

        read_next_lines;

		for(int i=0; i<nRows; i++)
		{
			read_next_lines;

			const int iniY =minBorderY+i*hCell; //this was float
			int maxY = iniY+hCell+6; //this was float

			if(iniY>=maxBorderY-3)
				continue;
			if(maxY>maxBorderY)
				maxY = maxBorderY;

			for(int j=0; j<nCols; j++)
			{
				const int iniX =minBorderX+j*wCell; //this was float
				int maxX = iniX+wCell+6;//this was float
				if(iniX>=maxBorderX-6)
					continue;
				if(maxX>maxBorderX)
					maxX = maxBorderX;

				cv::Mat tmp = cv::Mat(50, 50, CV_8UC1);
				mat_copy( image_data, tmp, iniY, iniX,cache_cnt, image_ptr_offset, image_width);

				vector<cv::KeyPoint> vKeysCell;
				cv::FAST(tmp, vKeysCell, 14, true);

				//cv::FAST(Image->rowRange(iniY,maxY).colRange(iniX,maxX), vKeysCell, 14, true);
				
				//key point (x<16>,y<16>,size<16>,angle<16>, response<16>, octave<16> )

				if(!vKeysCell.empty())
				{
					for(vector<cv::KeyPoint>::iterator vit=vKeysCell.begin(); vit!=vKeysCell.end();vit++)
					{
						(*vit).pt.x+=j*wCell;
						(*vit).pt.y+=i*hCell;
						vToDistributeKeys->push_back(*vit);
						nWrittenPoints+= 1;
					}
				}
			}
		}
		//*((vector<cv::KeyPoint>*)feature_dest) = vToDistributeKeys;
        MBOX_PUT(response, nWrittenPoints);
	}
}