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

    if(bUseHw==1)
    {
        result_buffer_1 = (uint32_t *)malloc(sizeof(uint32_t) * 1024 * 8);
        result_buffer_2 = (uint32_t *)malloc(sizeof(uint32_t) * 1024 * 8);

        if(result_buffer_1 == NULL || result_buffer_2 == NULL)
            printf("FPGA: Error while allocating memory \n");

    }




}

void FPGA::FPGA_FAST( InputArray image, CV_OUT std::vector<KeyPoint>& keypoints, int threshold, bool nonmaxSuppression )
{
    FAST(image, keypoints, threshold, nonmaxSuppression );
}

static const int EDGE_THRESHOLD = 19;
static const int iniThFAST = 20;
static const int minThFAST = 7;


void FPGA::Compute_Keypoints( cv::Mat &image, uint32_t nfeatures, vector<KeyPoint> & keypoints )
{

    vector<KeyPoint> cvKeypoints;
    uint32_t nres; 

    sem_wait(&fpga_sema);

    if(pthread_mutex_trylock( &fpga_mutex_0 ) == 0)
    {   

        //if(bUseHw == 1)
        //printf("cv::Mat &image.step = %d, v::Mat &image.cols = %d \n", (int)image.step,(int)image.cols);
        mbox_put(resources_fast_request_0, (uint32_t)image.data);
        //else
        //    mbox_put(resources_fast_request_0, (uint32_t)&image);

        mbox_put(resources_fast_request_0, (uint32_t)image.cols);
        mbox_put(resources_fast_request_0, (uint32_t)image.rows);
        mbox_put(resources_fast_request_0, (uint32_t)image.step);
        mbox_put(resources_fast_request_0, (uint32_t)&cvKeypoints);

        nres =  mbox_get(resources_fast_response_0);

        if(bUseHw == 1)
        {
            for(int i =0 ; i < nres; i++)
            {
                keypoints.push_back(KeyPoint((float)(result_buffer_1[i] & 0x0000ffff), (float)((result_buffer_1[i] & 0xffff0000)>>16), 7.f, -1, 0));
            }
        }
        else
        {
           //printf("FPGA 0: Got %d points \n", cvKeypoints.size());
            for(vector<cv::KeyPoint>::iterator vit=cvKeypoints.begin(); vit!=cvKeypoints.end();vit++)
            {
                keypoints.push_back(*vit);
            }

        }


        pthread_mutex_unlock( &fpga_mutex_0 );
        sem_post(&fpga_sema);
    }
    else
    {
        pthread_mutex_lock( &fpga_mutex_1 );

        //if(bUseHw == 1)
        //printf("cv::Mat &image.step = %d, v::Mat &image.cols = %d \n", (int)image.step,(int)image.cols);
        mbox_put(resources_fast_request_1, (uint32_t)image.data);
        //else
        //    mbox_put(resources_fast_request_1, (uint32_t)&image);

        mbox_put(resources_fast_request_1, (uint32_t)image.cols);
        mbox_put(resources_fast_request_1, (uint32_t)image.rows);
        mbox_put(resources_fast_request_1, (uint32_t)image.step);
        mbox_put(resources_fast_request_1, (uint32_t)&cvKeypoints);

        nres =  mbox_get(resources_fast_response_1);

        if(bUseHw == 1)
        {
            for(int i =0 ; i < nres; i++)
            {
                keypoints.push_back(KeyPoint((float)(result_buffer_2[i] & 0x0000ffff), (float)((result_buffer_2[i] & 0xffff0000)>>16), 7.f, -1, 0));
            }
        }
        else
        {

            //printf("FPGA 1: Got %d points \n", cvKeypoints.size());
            for(vector<cv::KeyPoint>::iterator vit=cvKeypoints.begin(); vit!=cvKeypoints.end();vit++)
            {
                keypoints.push_back(*vit);
            }

        }
        pthread_mutex_unlock(&fpga_mutex_1);
        sem_post(&fpga_sema);
    }
}