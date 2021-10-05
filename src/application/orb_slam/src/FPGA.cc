#include "FPGA.h"

#if USE_RECONOS ==1 

extern "C" {
    #include "reconos.h"
    #include "reconos_app.h"
}

#endif


#include <iostream>

#include <stdint.h>
#include <string.h>
#include <vector>

#define FAST_WINDOW_SIZE 50


#define MEM_READ(src, dest, n) memcpy((void*)dest, (void*)src, n)


#define read_next_lines {for(int i = 0; i < FAST_WINDOW_SIZE; i++){ \
                                    offset = ((uint64_t)image_ptr + cache_cnt*(image_width))&3; \
                                    MEM_READ((((uint64_t)image_ptr + cache_cnt*(image_width))&(~3)), image_data + IMAGE_CACHE_WIDTH*(cache_cnt%IMAGE_CACHE_HEIGHT),\
                                    ((image_width+offset+3)&(~3)));} cache_cnt+=50;}

#define W 30
#define IMAGE_CACHE_WIDTH   1400
#define IMAGE_CACHE_HEIGHT  128

static sem_t fpga_sema;
static pthread_mutex_t fpga_mutex_0=PTHREAD_MUTEX_INITIALIZER;

static pthread_mutex_t fpga_mutex_1=PTHREAD_MUTEX_INITIALIZER;

static uint32_t * result_buffer_1;
static uint32_t * result_buffer_2;

extern int bUseHw;


void FPGA::FPGA_Init(void)
{
    sem_init(&fpga_sema,0,2);

    result_buffer_1 = (uint32_t *)malloc(sizeof(uint32_t) * 1024 * 8);
    result_buffer_2 = (uint32_t *)malloc(sizeof(uint32_t) * 1024 * 8);

    if(result_buffer_1 == NULL || result_buffer_2 == NULL)
        printf("FPGA: Error while allocating memory \n");

}

void FPGA::FPGA_FAST( InputArray image, CV_OUT std::vector<KeyPoint>& keypoints, int threshold, bool nonmaxSuppression )
{
    FAST(image, keypoints, threshold, nonmaxSuppression );
}

static const int EDGE_THRESHOLD = 19;
static const int iniThFAST = 20;
static const int minThFAST = 7;


void FPGA::Compute_Keypoints( uint8_t* image_ptr, uint32_t image_width, uint32_t image_height, uint32_t nfeatures, vector<KeyPoint> & keypoints )
{

    vector<KeyPoint> * cvKeypoints;

#if 1
    #warning FPGA.cc: USE FPGA enabled
    uint32_t nres; 

    sem_wait(&fpga_sema);

    if(pthread_mutex_trylock( &fpga_mutex_0 ) == 0)
    {   
        //std::cout << "FPGA0: Send request" << std::endl;
        mbox_put(resources_fast_request_0, (uint32_t)image_ptr);
        mbox_put(resources_fast_request_0, image_width);
        mbox_put(resources_fast_request_0, image_height);
        mbox_put(resources_fast_request_0, (uint32_t)result_buffer_1);

        uint32_t tmp = (uint32_t)image_ptr;

        //std::cout << "image_ptr " <<  tmp << "; image_width"  <<  image_width << "; image_height" <<  image_height << std::endl;

        nres =  mbox_get(resources_fast_response_0);
        //std::cout << "FPGA0: Got " << nres << "results" << std::endl;
        if(bUseHw == 1)
        {
            for(int i =0 ; i < nres; i++)
            {
                keypoints.push_back(KeyPoint((float)(result_buffer_1[i] & 0x0000ffff), (float)((result_buffer_1[i] & 0xffff0000)>>16), 7.f, -1, 0));
            }
        }
        else
        {
            cvKeypoints = (vector<KeyPoint>*)result_buffer_1;
            for(vector<cv::KeyPoint>::iterator vit=cvKeypoints->begin(); vit!=cvKeypoints->end();vit++)
            {
                keypoints.push_back(*vit);
            }

        }


        //std::cout << cnt << " Got res: " << res << std::endl;
        pthread_mutex_unlock( &fpga_mutex_0 );
        sem_post(&fpga_sema);
    }
    else
    {
        pthread_mutex_lock( &fpga_mutex_1 );
        //std::cout << "FPGA1: Send request" << std::endl;
        mbox_put(resources_fast_request_1, (uint32_t)image_ptr);
        mbox_put(resources_fast_request_1, image_width);
        mbox_put(resources_fast_request_1, image_height);
        mbox_put(resources_fast_request_1, (uint32_t)result_buffer_2);

        uint32_t tmp = (uint32_t)image_ptr;

        //std::cout << "image_ptr " <<  tmp << "; image_width"  <<  image_width << "; image_height" <<  image_height << std::endl;

        nres =  mbox_get(resources_fast_response_1);
        //std::cout << "FPGA1: Got " << nres << "results" << std::endl;
        if(bUseHw == 1)
        {
            for(int i =0 ; i < nres; i++)
            {
                keypoints.push_back(KeyPoint((float)(result_buffer_2[i] & 0x0000ffff), (float)((result_buffer_2[i] & 0xffff0000)>>16), 7.f, -1, 0));
            }
        }
        else
        {
            cvKeypoints = (vector<KeyPoint>*)result_buffer_2;
            for(vector<cv::KeyPoint>::iterator vit=cvKeypoints->begin(); vit!=cvKeypoints->end();vit++)
            {
                keypoints.push_back(*vit);
            }

        }

        pthread_mutex_unlock(&fpga_mutex_1);
        sem_post(&fpga_sema);
    }
    




}
#else
    uint8_t image_data[IMAGE_CACHE_WIDTH*IMAGE_CACHE_HEIGHT];

    uint32_t cache_cnt = 0;

    //std::cout << "Image width: " << image_width << "; Image height " << image_height << std::endl;

    const int minBorderX = EDGE_THRESHOLD-3;
    const int minBorderY = minBorderX;
    const int maxBorderX = image_width -EDGE_THRESHOLD+3;
    const int maxBorderY = image_height-EDGE_THRESHOLD+3;

    const int width = (maxBorderX-minBorderX);
    const int height = (maxBorderY-minBorderY);

/*
    const int nCols = width/W;
    const int nRows = height/W;
    const int wCell = (width/nCols);
    const int hCell = (height/nRows);
*/

    const int wCell = 50;
    const int hCell = 50;
    const int nCols = width/50;
    const int nRows = height/50;

    uint64_t offset = 0;

    read_next_lines;

    uint32_t nkeypoints = 0;

    //std::cout << "nCols " << nCols << "; nRows " << nRows << "; wCell " << wCell << ";hCell " << hCell << std::endl;

    for(int i=0; i<nRows; i++)
    {

        read_next_lines;

        const int iniY =minBorderY+i*hCell;
        int maxY = iniY+hCell+6;

        if(iniY>=maxBorderY-3)
            continue;
        if(maxY>maxBorderY)
            maxY = maxBorderY;

        for(int j=0; j<nCols; j++)
        {
            const int iniX =minBorderX+j*wCell;
            int maxX = iniX+wCell+6;
            if(iniX>=maxBorderX-6)
                continue;
            if(maxX>maxBorderX)
                maxX = maxBorderX;


            

            /*
            cv::Mat * img = (cv::Mat *)image; 
            cv::Mat tmp = img->rowRange(iniY,iniY+50).colRange(iniX,iniX+50);
            */

            cv::Mat tmp = Mat(50, 50, CV_8UC1);

            for(int i = 0; i < (50); i++)
            {
                for(int j = 0; j < (50); j++)
                {
                    tmp.data[i*50+j] = image_data[offset+(j+iniY)+ ((iniX+i+cache_cnt) %IMAGE_CACHE_HEIGHT) *IMAGE_CACHE_WIDTH];
                }
            }
            



            vector<cv::KeyPoint> vKeysCell;
            FPGA::FPGA_FAST(tmp, vKeysCell,iniThFAST,true);
            //std::cout << "1. Attempt: Number of keypoints " << vKeysCell.size() << std::endl;

            //if(vKeysCell.empty())
            //{
            //    FPGA::FPGA_FAST(tmp, vKeysCell,minThFAST,true);
            //    std::cout << "2. Attempt: Number of keypoints " << vKeysCell.size() << std::endl;
            //}

            nkeypoints+= vKeysCell.size();
            

            if(!vKeysCell.empty())
            {
                for(vector<cv::KeyPoint>::iterator vit=vKeysCell.begin(); vit!=vKeysCell.end();vit++)
                {
                    (*vit).pt.x+=j*wCell;
                    (*vit).pt.y+=i*hCell;
                    keypoints.push_back((uint32_t)(*vit).pt.x | ((uint32_t)(*vit).pt.y << 16));
                }
            }

        }
    }

    //std::cout << "Size of the cell vector: " << nkeypoints << std::endl;
}
#endif



#if 0

void FPGA::Compute_Keypoints( uint8_t* image_ptr, uint32_t image_width, uint32_t image_height, uint32_t nfeatures, vector<cv::KeyPoint> & Keypoints )
{

#if 0
    uint32_t res; 
    pthread_mutex_lock( &fpga_mutex );

    mbox_put(resources_fast_request, (uint32_t)image_ptr);
	mbox_put(resources_fast_request, image_width);
	mbox_put(resources_fast_request, image_height);

    uint32_t tmp = (uint32_t)image_ptr;

    keypoints.clear();

    //std::cout << "image_ptr " <<  tmp << "; image_width"  <<  image_width << "; image_height" <<  image_height << std::endl;

    int cnt = 0;

    do{

        res = mbox_get(resources_fast_response);        
        cnt++;
        if(res != 0xffffffff)
        {
            keypoints.push_back(res);
        }

        //std::cout << cnt++ << " Got res: " << res << std::endl;

    }while(res != 0xffffffff);
    //std::cout << cnt << " Got res: " << res << std::endl;
    pthread_mutex_unlock( &fpga_mutex );


}
#else
    uint8_t image_data[IMAGE_CACHE_WIDTH*IMAGE_CACHE_HEIGHT];

    uint32_t cache_cnt = 0;

    //std::cout << "Image width: " << image_width << "; Image height " << image_height << std::endl;

    const int minBorderX = EDGE_THRESHOLD-3;
    const int minBorderY = minBorderX;
    const int maxBorderX = image_width -EDGE_THRESHOLD+3;
    const int maxBorderY = image_height-EDGE_THRESHOLD+3;

    const int width = (maxBorderX-minBorderX);
    const int height = (maxBorderY-minBorderY);

/*
    const int nCols = width/W;
    const int nRows = height/W;
    const int wCell = (width/nCols);
    const int hCell = (height/nRows);
*/

    const int wCell = 50;
    const int hCell = 50;
    const int nCols = width/50;
    const int nRows = height/50;

    uint64_t offset = 0;

    read_next_lines;

    uint32_t nkeypoints = 0;

    Keypoints.clear();

    //std::cout << "nCols " << nCols << "; nRows " << nRows << "; wCell " << wCell << ";hCell " << hCell << std::endl;

    for(int i=0; i<nRows; i++)
    {

        read_next_lines;

        const int iniY =minBorderY+i*hCell;
        int maxY = iniY+hCell+6;

        if(iniY>=maxBorderY-3)
            continue;
        if(maxY>maxBorderY)
            maxY = maxBorderY;

        for(int j=0; j<nCols; j++)
        {
            const int iniX =minBorderX+j*wCell;
            int maxX = iniX+wCell+6;
            if(iniX>=maxBorderX-6)
                continue;
            if(maxX>maxBorderX)
                maxX = maxBorderX;


            

            /*
            cv::Mat * img = (cv::Mat *)image; 
            cv::Mat tmp = img->rowRange(iniY,iniY+50).colRange(iniX,iniX+50);
            */

            cv::Mat tmp = Mat(50, 50, CV_8UC1);

            for(int i = 0; i < (50); i++)
            {
                for(int j = 0; j < (50); j++)
                {
                    tmp.data[i*50+j] = image_data[offset+(j+iniY)+ ((iniX+i+cache_cnt) %IMAGE_CACHE_HEIGHT) *IMAGE_CACHE_WIDTH];
                }
            }
            



            vector<cv::KeyPoint> vKeysCell;
            FPGA::FPGA_FAST(tmp, vKeysCell,minThFAST,true);
            //std::cout << "1. Attempt: Number of keypoints " << vKeysCell.size() << std::endl;

            //if(vKeysCell.empty())
            //{
            //    FPGA::FPGA_FAST(tmp, vKeysCell,minThFAST,true);
            //    std::cout << "2. Attempt: Number of keypoints " << vKeysCell.size() << std::endl;
            //}

            nkeypoints+= vKeysCell.size();
            

            if(!vKeysCell.empty())
            {
                for(vector<cv::KeyPoint>::iterator vit=vKeysCell.begin(); vit!=vKeysCell.end();vit++)
                {
                    (*vit).pt.x+=j*wCell;
                    (*vit).pt.y+=i*hCell;
                    Keypoints.push_back(*vit);
                }
            }

        }
    }

    //std::cout << "Size of the cell vector: " << nkeypoints << std::endl;

}

#endif

#endif