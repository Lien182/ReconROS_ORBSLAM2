#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include "orb_slam/include/System.h"
#include "orb_slam/include/FPGA.h"
#include "orb_slam/include/Converter.h"



#if USE_RECONOS == 1

extern "C" {
    #include "reconos.h"
    #include "reconos_app.h"
}

#endif

#define COMPILEDWITHC11

using namespace std;

int bUseHw = 0;

void LoadImages(const string &strPathToSequence,  vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 6)
    {
        cerr << endl << "Usage: ./orbslam path_to_vocabulary path_to_settings path_to_sequence number_of_pictures <hw/sw>" << endl;
        return 1;
    }

#if USE_RECONOS == 1

    #warning main.cc: USE ReconOS enabled
    //Reconos Stuff

    reconos_init();
	reconos_app_init();
	int clk = reconos_clock_threads_set(100000);
    


#endif
/*
    uint8_t image[1000*1000];


    for(int i = 0; i < 10; i++)
    {
        mbox_put(resources_fast_request, (uint32_t)image);
	    mbox_put(resources_fast_request, 999);
	    mbox_put(resources_fast_request, 999);

        uint32_t res = mbox_get(resources_fast_response);
        std::cout << "Got res: " << res << std::endl;
    }

    return 0;
*/
    uint32_t img_cnt = 0;

    // Retrieve paths to images
    //vector<string> vstrImageLeft;
    //vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vTimestamps);

    int nImages = vTimestamps.size();

    int _nImages = atoi(argv[4]);
    if(_nImages != 0)
    {
        nImages = _nImages;
    }

    cout << "Number of Images:" << nImages << "; Argument: " << _nImages << endl;
    

    if(strcmp(argv[5], "hw") == 0)
    {
        cout << "Use ReconROS HW" << endl;
        bUseHw = 1;
        reconos_thread_create_hwt_fast((void*)0);
        reconos_thread_create_hwt_fast((void*)1);
        FPGA::FPGA_Init();

    }
    else if(strcmp(argv[5], "sw") == 0)
    {
        cout << "Use ReconROS SW" << endl;
        bUseHw = 2;
        reconos_thread_create_swt_fast((void*)0, 0);
        reconos_thread_create_swt_fast((void*)1, 0);
        FPGA::FPGA_Init();
    }
    else
    {
        cout << "Use original" << endl;
        bUseHw = 0;
    }
    


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
#if USE_FPGA == 0
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,false);
#else
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,false);
#endif
    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   

    // Main loop
    cv::Mat imLeft, imRight;
    for(int ni=0; ni<nImages; ni++)
    {

        const int nTimes = vTimestamps.size();


        stringstream ss;
        ss << setfill('0') << setw(6) << img_cnt;
        string strImageLeft  = string(argv[3]) + "/image_0/" + ss.str() + ".png";
        string strImageRight = string(argv[3]) + "/image_1/" + ss.str() + ".png";

        img_cnt++;


        // Read left and right images from file
        imLeft = cv::imread(strImageLeft,CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(strImageRight,CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: " << string(strImageLeft) << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        cv::Mat Tcw = SLAM.TrackStereo(imLeft,imRight,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        //https://github.com/raulmur/ORB_SLAM2/issues/597

        
        //geometry_msg_out->header.stamp = ros::Time::now();
        //geometry_msg_out->header.frame_id ="map";

        try
        {
            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t(); // Rotation information
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3); // translation information
            vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);




            //tf::Transform new_transform;
            //new_transform.setOrigin(tf::Vector3(twc.at<float>(0, 0), twc.at<float>(0, 1), twc.at<float>(0, 2)));

            //tf::Quaternion quaternion(q[0], q[1], q[2], q[3]);
            //new_transform.setRotation(quaternion);

            //tf::poseTFToMsg(new_transform, pose.pose);
            
            resources_geometry_msg_out->pose.position.x = twc.at<float>(0, 0);
            resources_geometry_msg_out->pose.position.y = twc.at<float>(0, 1);
            resources_geometry_msg_out->pose.position.z = twc.at<float>(0, 2);

            resources_geometry_msg_out->pose.orientation.x = q[0];
            resources_geometry_msg_out->pose.orientation.y = q[1];
            resources_geometry_msg_out->pose.orientation.z = q[2];
            resources_geometry_msg_out->pose.orientation.w = q[3];
            
            ros_publisher_publish(resources_pub_out, resources_geometry_msg_out);
        }
        catch(...)
        {
            std::cout << "exception" << std::endl;
        }

        


        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        cout << ni << ", " << ttrack << ";" << endl;

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    return 0;
}

void LoadImages(const string &strPathToSequence,
                 vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }
}
