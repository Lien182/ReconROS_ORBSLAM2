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

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint32_t score;
}t_keypoint;



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

static t_keypoint * result_buffer_0;
static t_keypoint * result_buffer_1;

extern int bUseHw;


void FPGA::FPGA_Init(void)
{
    sem_init(&fpga_sema,0,2);

    uint32_t memory_size = sizeof(t_keypoint) * 10000;
    result_buffer_0 = (t_keypoint *)malloc(memory_size);
    result_buffer_1 = (t_keypoint *)malloc(memory_size);

    if(result_buffer_0 == NULL || result_buffer_1 == NULL)
        printf("FPGA: Error while allocating memory \n");
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
        mbox_put(resources_fast_request_0, (uint32_t)result_buffer_0);

        nres =  mbox_get(resources_fast_response_0);

        for(int i =0 ; i < nres; i++)
        {
            keypoints.push_back(KeyPoint((float)(result_buffer_0[i].x), (float)(result_buffer_0[i].y), 7.f, -1, (float)(result_buffer_0[i].score)));
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
        mbox_put(resources_fast_request_1, (uint32_t)result_buffer_1);

        nres =  mbox_get(resources_fast_response_1);

        for(int i =0 ; i < nres; i++)
        {
            keypoints.push_back(KeyPoint((float)(result_buffer_1[i].x), (float)(result_buffer_1[i].y), 7.f, -1, (float)(result_buffer_1[i].score)));
        }

        pthread_mutex_unlock(&fpga_mutex_1);
        sem_post(&fpga_sema);
    }
}