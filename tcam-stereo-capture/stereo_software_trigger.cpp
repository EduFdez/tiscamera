//////////////////////////////////////////////////////////////////
/*
Tcam Software Trigger
This sample shows, how to trigger two cameras by software and use a callback for image handling.

Prerequisits
It uses the the examples/cpp/common/tcamcamera.cpp and .h files of the *tiscamera* repository as wrapper around the
GStreamer code and property handling.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <chrono>
#include <thread>

#include "tcamcamera.h"
#include <unistd.h>

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
    char imageprefix[55]; // Prefix for the image file names.
   	cv::Mat frame; 
} CUSTOMDATA;

////////////////////////////////////////////////////////////////////
// Increase the frame count and save the image in CUSTOMDATA
void SaveImage(CUSTOMDATA *pCustomData, const string & img_name)
{
    char ImageFileName[256];
    //pCustomData->ImageCounter++;
    //sprintf(ImageFileName,"%s%05d.jpg", pCustomData->imageprefix, pCustomData->ImageCounter);
    sprintf(ImageFileName,"%s%s.jpg", pCustomData->imageprefix, img_name.c_str());
    cv::imwrite(ImageFileName, pCustomData->frame);
    //cout << "ImageFileName " << ImageFileName << endl;
}

////////////////////////////////////////////////////////////////////
// Simple implementation of "getch()"
void waitforkey(const char *text)
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
// Callback called for new images by the internal appsink
GstFlowReturn new_frame_cb(GstAppSink *appsink, gpointer data)
{
    int width, height ;
    const GstStructure *str;

    // Cast gpointer to CUSTOMDATA*
    CUSTOMDATA *pCustomData = (CUSTOMDATA*)data;

    //cout << duration_cast<milliseconds>(high_resolution_clock::now().time_since_epoch()).count() << " im " << pCustomData->ImageCounter << " " << pCustomData->imageprefix << " new_frame_cb\n";

    if( pCustomData->busy) // Return, if will are busy. Will result in frame drops
       return GST_FLOW_OK;

    pCustomData->busy = true;

    // The following lines demonstrate, how to acces the image
    // data in the GstSample.
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    GstCaps *caps = gst_sample_get_caps(sample);

    str = gst_caps_get_structure (caps, 0);    

    if( strcmp( gst_structure_get_string (str, "format"),"BGRx") == 0)  
    {
        gst_structure_get_int (str, "width", &width);
        gst_structure_get_int (str, "height", &height);

        // Get the image data
        GstMapInfo info;
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        gst_buffer_map(buffer, &info, GST_MAP_READ);
        if (info.data != NULL) 
        {
            // Create the cv::Mat
            pCustomData->frame.create(height,width,CV_8UC(4));
            // Copy the image data from GstBuffer intot the cv::Mat
            memcpy( pCustomData->frame.data, info.data, width*height*4);
            // Set the flag for received and handled an image.
            pCustomData->ReceivedAnImage = true;
            pCustomData->ImageCounter++;
        }       
        gst_buffer_unmap (buffer, &info);
    }    
    // Calling Unref is important!
    gst_sample_unref(sample);

    // Set our flag of new image to true, so our main thread knows about a new image.
    pCustomData->busy = false;

    return GST_FLOW_OK;
}

////////////////////////////////////////////////////////////
// Enable trigger mode for the passed camera object
//
int setTriggerMode(TcamCamera &cam, int onoff)
{
    // Query the trigger mode property. Attention: The "Trigger Mode" name
    // can differ on USB and GigE camera. If you do not know the names of these
    // properties, uncomment the above mentions "ListProperties(cam1)" line in order
    // to get a list of available properties.
    std::shared_ptr<Property> TriggerMode;
    try
    {
        TriggerMode = cam.get_property("Trigger Mode");
        // TriggerMode->set(cam,"Off"); // Use this line for GigE cameras
        TriggerMode->set(cam,onoff); // Use this line for USB cameras
        return 1;
    }
    catch(...)
    {
        printf("Your camera does not support triggering.\n");
        return(0);
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    gst_init(&argc, &argv);
    // Create and initialize the custom data for the callback.
    CUSTOMDATA CustomData1;  // Customdata for camera 1
    CUSTOMDATA CustomData2;  // Customdata for camera 2

    // Get input params
    string out_dir = static_cast<string>(argv[1]);
    int fps = atoi(argv[2]);
    int seq_length = 100;
    if(argc > 3)
        seq_length = atoi(argv[3]);

    // Create output directory
    cout << "out_dir: " << out_dir << "\n";
    string dir_cam0 = out_dir + "/cam0/"; // Left camera
    string dir_cam1 = out_dir + "/cam1/"; // Right camera
    string mkdir_command = "mkdir -p ";
    const int dir_err = system((mkdir_command + dir_cam0).c_str());
    if (-1 == dir_err)
    {
        cout << "Error creating directory!" << out_dir << "\n";
        return -1;
    }
    system((mkdir_command + dir_cam1).c_str());

    CustomData1.ImageCounter = 0;
    CustomData1.ReceivedAnImage = false;
    CustomData1.busy = true; // Avoid saving of images when the stream is started
    strcpy(CustomData1.imageprefix, dir_cam0.c_str()); // Specify the image prefix

    CustomData2.ImageCounter = 0;
    CustomData2.ReceivedAnImage = false;
    CustomData2.busy = true; // Avoid saving of images when the stream is started
    strcpy(CustomData2.imageprefix, dir_cam1.c_str()); // Specify the image prefix

    // Open cameras by serial number. Use "tcam-ctrl -l" to query a list of cameras
    TcamCamera cam1("28610163");
    TcamCamera cam2("28610165");

    // Set video format, resolution and frame rate
    cam1.set_capture_format("BGRx", FrameSize{1920,1200}, FrameRate{fps,1});
    cam2.set_capture_format("BGRx", FrameSize{1920,1200}, FrameRate{fps,1});

    //Uncomment following line, if properties shall be listed
    ListProperties(cam1);
    

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
    cam1.set_new_frame_callback(new_frame_cb, &CustomData1);
    cam2.set_new_frame_callback(new_frame_cb, &CustomData2);

    // Disable trigger mode fist
    // TriggerMode->set(cam,"Off"); // Use this line for GigE cameras
    TriggerMode->set(cam1,0); // Use this line for USB cameras
    TriggerMode->set(cam2,0); // Use this line for USB cameras

    // Start the camera
    cam1.start();
    cam2.start();

    CustomData1.busy = false; // allow saving of images when the stream is started
    CustomData1.ImageCounter = 0; // Reset the image counter
    CustomData2.busy = false; // allow saving of images when the stream is started
    CustomData2.ImageCounter = 0; // Reset the image counter

    // Enable trigger mode
    Softwaretrigger->set(cam1,0);
    Softwaretrigger->set(cam2,0);

    waitforkey("Press Enter to stop triggering and image saving.");    
    printf("Starting triggers now\n");

    high_resolution_clock::time_point p = high_resolution_clock::now();
    milliseconds prev_ms = duration_cast<milliseconds>(p.time_since_epoch());
    // Main loop for software trigger. Image saving is done in the callback.
    for( int i = 0; i < seq_length; i++)
    {
        CustomData1.ReceivedAnImage = false;
        CustomData2.ReceivedAnImage = false;

        Softwaretrigger->set(cam1,1);
        Softwaretrigger->set(cam2,1);
        string timestamp = to_string(high_resolution_clock::now().time_since_epoch().count());
        cout << i << " Trigger. Timestamp: " << timestamp << endl;

        // Wait with timeout until we got images from both cameras.
        int tries = 0;
        while( !( CustomData1.ReceivedAnImage && CustomData2.ReceivedAnImage) && ++tries <= 50)
            usleep(1000);  // 1ms

        milliseconds ms = duration_cast<milliseconds>(high_resolution_clock::now().time_since_epoch());
        //std::cout << high_resolution_clock::now().time_since_epoch().count() << " Time ms " << (ms.count() - prev_ms.count()) << std::endl;
        prev_ms = ms;

        // If there are images received from both cameras, save them.
        if(CustomData1.ReceivedAnImage && CustomData1.ReceivedAnImage)
        {
            thread save_img0(SaveImage, &CustomData1, ref(timestamp));
            thread save_img1(SaveImage, &CustomData2, ref(timestamp));
            save_img0.join();
            save_img1.join();
        }
        else
        {
            // Check, from which camera we may did not receive an image.
            // It is for convinience only and could be deleted.
            if(!CustomData1.ReceivedAnImage)
                printf("Did not receive an image from camera 1.\n");
                
            if(!CustomData2.ReceivedAnImage)
                printf("Did not receive an image from camera 2.\n");
        }
    }
    
    waitforkey("Press Enter to end the program");

    CustomData1.busy = true; // deny saving of images 
    CustomData2.busy = true; // deny saving of images 

    // Disable the trigger mode, so other programs will see a live video.
    TriggerMode->set(cam1,0); // Use this line for USB cameras
    TriggerMode->set(cam2,0); // Use this line for USB cameras

    Softwaretrigger->set(cam1,1); // Needed for cam1.stop() not waiting for an image.
    Softwaretrigger->set(cam2,1); // Needed for cam2.stop() not waiting for an image.

    cam1.stop();
    cam2.stop();

    return 0;
}
