#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

using namespace cv;
using namespace std;

#define USE_RECONOS 1
#define USE_FPGA 1
#define FPGA_SIMULATION 0 

namespace FPGA{


    void FPGA_Init();

    void FPGA_FAST( InputArray image, CV_OUT std::vector<KeyPoint>& keypoints, int threshold, bool nonmaxSuppression=true );

    void Compute_Keypoints( uint8_t* image_ptr, uint32_t image_width, uint32_t image_height, uint32_t nfeatures, vector<KeyPoint> & keypoints);


}
