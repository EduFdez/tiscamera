//////////////////////////////////////////////////////////////////
/*
Tcam Software Trigger
This sample shows, how to trigger two cameras by software and use a callback for image handling.

Prerequisits
It uses the the examples/cpp/common/tcamcamera.cpp and .h files of the *tiscamera* repository as wrapper around the
GStreamer code and property handling. Adapt the CMakeList.txt accordingly.
*/
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>

#include "tcamcamera.h"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace std::chrono;
using namespace gsttcam;

// Create a custom data structure to be passed to the callback function. 
typedef struct
{
    int ImageCounter;
    bool ReceivedAnImage;
    bool busy;
    char imagepath[55]; // Prefix for the image file names.
   	cv::Mat frame; 
} CUSTOMDATA;

char getKeyPress(const char *text)
{
    cout << text << endl;
    char ch;
    cin >> ch;
    return ch;
}

////////////////////////////////////////////////////////////////////
// Simple implementation of "getch()"

void waitforkey(char *text)
{
    printf("%s\n",text);
    char dummyvalue[10];
    scanf("%c",dummyvalue);
}
////////////////////////////////////////////////////////////////////
// List available properties helper function.
void ListProperties(TcamCamera &cam)
{
    // Get a list of all supported properties and print it out
    auto properties = cam.get_camera_property_list();
    std::cout << "Properties:" << std::endl;
    for(auto &prop : properties)
    {
        std::cout << prop->to_string() << std::endl;
    }
}

////////////////////////////////////////////////////////////////////
// Increase the frame count and save the image in CUSTOMDATA
void SaveImage(CUSTOMDATA *pCamData) //, const string & img_name)
{
    char ImageFileName[256];
    sprintf(ImageFileName,"%s%05d.jpg", pCamData->imagepath, pCamData->ImageCounter);
    //sprintf(ImageFileName,"%s%s.jpg", pCamData->imagepath, img_name.c_str());
    cv::imwrite(ImageFileName, pCamData->frame);
    pCamData->ImageCounter++;
    cout << "ImageFileName " << ImageFileName << endl;
}

////////////////////////////////////////////////////////////////////
// Callback called for new images by the internal appsink
GstFlowReturn new_frame_cb(GstAppSink *appsink, gpointer data)
{

    int width, height ;
    const GstStructure *str;

    // Cast gpointer to CUSTOMDATA*
    CUSTOMDATA *pCamData = (CUSTOMDATA*)data;

    //cout << pCamData->ImageCounter << " " << pCamData->imagepath << " new_frame_cb\n";

    if( pCamData->busy) // Return, if will are busy. Will result in frame drops
       return GST_FLOW_OK;

    pCamData->busy = true;

    // The following lines demonstrate, how to acces the image
    // data in the GstSample.
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    GstCaps *caps = gst_sample_get_caps(sample);

    str = gst_caps_get_structure (caps, 0);    

    if( strcmp( gst_structure_get_string (str, "format"),"BGRx") == 0)  
    {
        gst_structure_get_int (str, "width", &width);
        gst_structure_get_int (str, "height", &height);

        GstMapInfo info;
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        gst_buffer_map(buffer, &info, GST_MAP_READ);
        if (info.data != NULL) 
        {
            pCamData->frame.create(height,width,CV_8UC(4));
            memcpy( pCamData->frame.data, info.data, width*height*4);
            pCamData->ReceivedAnImage = true;
        }
        gst_buffer_unmap (buffer, &info);
    }    
    // Calling Unref is important!
    gst_sample_unref(sample);

    // Set our flag of new image to true, so our main thread knows about a new image.
    pCamData->busy = false;
    return GST_FLOW_OK;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    gst_init(&argc, &argv);
    // Create and initialize the custom data for the callback.
    CUSTOMDATA CamData1;  // Customdata for camera 1
    CUSTOMDATA CamData2;  // Customdata for camera 2

    // Create output directory
    string out_dir = static_cast<string>(argv[1]);
    string dir_cam0 = out_dir + "/cam0/"; // Left camera
    string dir_cam1 = out_dir + "/cam1/"; // Right camera
    cout << "out_dir: " << out_dir << "\n";
    string mkdir_command = "mkdir -p ";
    const int dir_err = system((mkdir_command + dir_cam0).c_str());
    if (-1 == dir_err)
    {
        cout << "Error creating directory!" << out_dir << "\n";
        return -1;
    }
    system((mkdir_command + dir_cam1).c_str());

    CamData1.ImageCounter = 0;
    CamData1.ReceivedAnImage = false;
    CamData1.busy = true; // Avoid saving of images when the stream is started
    strcpy(CamData1.imagepath, dir_cam0.c_str()); // Specify the image prefix

    CamData2.ImageCounter = 0;
    CamData2.ReceivedAnImage = false;
    CamData2.busy = true; // Avoid saving of images when the stream is started
    strcpy(CamData2.imagepath, dir_cam1.c_str()); // Specify the image prefix

    // Open cameras by serial number. Use "tcam-ctrl -l" to query a list of cameras
    TcamCamera cam1("28610163");
    TcamCamera cam2("28610165");
    
    // Set video format, resolution and frame rate
    cam1.set_capture_format("BGRx", FrameSize{1920,1200}, FrameRate{30,1}); // FrameSize{640,480}, FrameRate{60,1} // FrameSize{1280,720}, FrameRate{30,1} // FrameSize{1920,1200}, FrameRate{20,1} // "GRAY8", FrameSize{1920,1080}, FrameRate{20,1}
    cam2.set_capture_format("BGRx", FrameSize{1920,1200}, FrameRate{30,1});

    //Uncomment following line, if properties shall be listed
    //ListProperties(cam1);    

    std::shared_ptr<Property> TriggerMode;
    try
    {
        TriggerMode = cam1.get_property("Trigger Mode");
    }
    catch(...)
    {
        printf("Your camera does not support triggering.\n");
        return(1);
    }
      // Query the software trigger property. Attention: The "Software Triggere" name
    // can be different on USB and GigE camera. 
    std::shared_ptr<Property> Softwaretrigger;
    try
    {
        Softwaretrigger = cam1.get_property("Software Trigger");
    }
    catch(...)
    {
        printf("Your camera does not support software triggering.\n");
        return(2);
    }

    // Comment following line, if no live video display is wanted.
    cam1.enable_video_display(gst_element_factory_make("ximagesink", NULL));
    cam2.enable_video_display(gst_element_factory_make("ximagesink", NULL));

    // Register a callback to be called for each new frame
    cam1.set_new_frame_callback(new_frame_cb, &CamData1);
    cam2.set_new_frame_callback(new_frame_cb, &CamData2);

    // Disable trigger mode fist
    // TriggerMode->set(cam,"Off"); // Use this line for GigE cameras
    TriggerMode->set(cam1,0); // Use this line for USB cameras
    TriggerMode->set(cam2,0); // Use this line for USB cameras
    
    // Start the camera
    cam1.start();
    cam2.start();

    // Enable trigger mode
    TriggerMode->set(cam1,1); // Use this line for USB cameras
    TriggerMode->set(cam2,1); // Use this line for USB cameras

    // allow saving of images when the stream is started
    CamData1.busy = false;
    CamData2.busy = false;

    printf("Starting triggers now\n");

    high_resolution_clock::time_point p = high_resolution_clock::now();
    milliseconds prev_ms = duration_cast<milliseconds>(p.time_since_epoch());
    // Main loop for software trigger. Image saving is done in the callback.
    char ch ='a';
    while( ch != 'q' && ch != 'Q' )
    {
        ch = getKeyPress("Press key to get an image. 'q' to exit acquisition \n");
        cout << CamData1.ImageCounter << " trigger " << endl;
        cout << CamData2.ImageCounter << " trigger " << endl;

        CamData1.ReceivedAnImage = false;
        CamData2.ReceivedAnImage = false;

        Softwaretrigger->set(cam1,1);
        Softwaretrigger->set(cam2,1);

        // Wait with timeout until we got images from both cameras.
        int tries = 0;
        while( !( CamData1.ReceivedAnImage && CamData2.ReceivedAnImage) && ++tries < 500)
            usleep(1000); // microseconds

        milliseconds ms = duration_cast<milliseconds>(high_resolution_clock::now().time_since_epoch());
        std::cout << high_resolution_clock::now().time_since_epoch().count() << " Time ms " << (ms.count() - prev_ms.count()) << " tries " << tries << std::endl;
        prev_ms = ms;

        // If there are images received from both cameras, save them.
        if(CamData1.ReceivedAnImage && CamData1.ReceivedAnImage)
        {
            SaveImage(&CamData1);
            SaveImage(&CamData2);
        }
        else
        {
            // Check, from which camera we may did not receive an image.
            // It is for convinience only and could be deleted.
            if(!CamData1.ReceivedAnImage)
                printf("Did not receive an image from camera 1.\n");

            if(!CamData2.ReceivedAnImage)
                printf("Did not receive an image from camera 2.\n");
        }
    }
    
    //waitforkey("Press Enter to end the program");

    CamData1.busy = true; // deny saving of images
    CamData2.busy = true; // deny saving of images

    // Disable the trigger mode, so other programs will see a live video.
    TriggerMode->set(cam1,0); // Use this line for USB cameras
    TriggerMode->set(cam2,0); // Use this line for USB cameras

    Softwaretrigger->set(cam1,1); // Needed for cam1.stop() not waiting for an image.
    Softwaretrigger->set(cam2,1); // Needed for cam2.stop() not waiting for an image.

    cam1.stop();
    cam2.stop();
    return 0;
}
