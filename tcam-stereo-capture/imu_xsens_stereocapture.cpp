#include <xsens/xsportinfoarray.h>
#include <xsens/xsdatapacket.h>
#include <xsens/xstime.h>
#include <xsens/xssyncsettinglist.h>
#include <xcommunication/legacydatapacket.h>
#include <xcommunication/int_xsdatapacket.h>
#include <xcommunication/enumerateusbdevices.h>

#include "xsens/deviceclass.h"

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

#ifdef __GNUC__
#include "xsens/conio.h" // for non ANSI _kbhit() and _getch()
#else
#include <conio.h>
#endif

using namespace std;
using namespace std::chrono;
using namespace gsttcam;


// Create a custom data structure to be passed to the callback function.
typedef struct
{
    //static std::string timestamp; // Name for the image file names.
    int ImageCounter;
    bool ReceivedAnImage;
    bool busy;
    char imagepath[55]; // Prefix for the image file names.
    char imagename[20]; // Name for the image file names.
    cv::Mat frame;
} CameraData;

//CameraData::timestamp = 0;

////////////////////////////////////////////////////////////////////
// Increase the frame count and save the image in CameraData
void SaveImage(CameraData *pCamData, const string & img_name)
{
    char ImageFileName[256];
    pCamData->ImageCounter++;
    sprintf(ImageFileName,"%s%s.jpg", pCamData->imagepath, img_name.c_str());
    cv::imwrite(ImageFileName, pCamData->frame);
    cout << "ImageFileName " << img_name << endl;
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
// Callback called for new images by the internal appsink
GstFlowReturn new_frame_cb(GstAppSink *appsink, gpointer data)
{
    int width, height ;
    const GstStructure *str;

    // Cast gpointer to CameraData*
    CameraData *pCamData = (CameraData*)data;

    cout << duration_cast<milliseconds>(high_resolution_clock::now().time_since_epoch()).count() << " im "
         << pCamData->ImageCounter << " " << pCamData->imagepath << pCamData->imagename << " new_frame_cb\n";

    if( pCamData->busy ) // Return, if will are busy. Will result in frame drops
       return GST_FLOW_OK;

    pCamData->busy = true;

    // The following lines demonstrate, how to acces the image
    // data in the GstSample.
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    GstCaps *caps = gst_sample_get_caps(sample);

    str = gst_caps_get_structure (caps, 0);

    if( strcmp( gst_structure_get_string (str, "format"),"BGRx") == 0 )
    {
        //cout << " Save image new_frame_cb\n";
        gst_structure_get_int (str, "width", &width);
        gst_structure_get_int (str, "height", &height);

        // Get the image data
        GstMapInfo info;
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        gst_buffer_map(buffer, &info, GST_MAP_READ);
        if (info.data != NULL)
        {
            // Create the cv::Mat
            pCamData->frame.create(height,width,CV_8UC(4));
            // Copy the image data from GstBuffer intot the cv::Mat
            memcpy( pCamData->frame.data, info.data, width*height*4);
            pCamData->ImageCounter++;
            cv::imwrite( string(pCamData->imagepath) + pCamData->imagename + ".jpg", pCamData->frame );
//            thread save_img(SaveImage, pCamData, pCamData->imagename);
//            pCamData->ReceivedAnImage = true;
            //cout << " Save image new_frame_cb : " << pCamData->imagepath << pCamData->imagename << " time " << uint64_t(high_resolution_clock::now().time_since_epoch().count())/1000000 << endl;
        }
        gst_buffer_unmap (buffer, &info);
    }
    // Calling Unref is important!
    gst_sample_unref(sample);

    // Set our flag of new image to true, so our main thread knows about a new image.
    pCamData->busy = false;

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

int main(int argc, char* argv[])
{
    // Get input params
    string out_dir = static_cast<string>(argv[1]);
    int fps = atoi(argv[2]);
    int imu_frequency = 200;
    if(argc > 3)
        imu_frequency = atoi(argv[3]);
    uint16_t imu_freq(imu_frequency);
    cout << "IMU Frequency " << imu_frequency << endl;

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

    /******************************************************************************************/
    /*                                  CONFIGURE THE CAMERAS                                 */
    /******************************************************************************************/
    gst_init(&argc, &argv);
    // Create and initialize the custom data for the callback.
    CameraData CamData1;  // Customdata for camera 1
    CameraData CamData2;  // Customdata for camera 2

    string tstamp = to_string((high_resolution_clock::now().time_since_epoch().count()/uint64_t(1e6))*uint64_t(1e6));

    CamData1.ImageCounter = 0;
//    CamData1.ReceivedAnImage = false;
    CamData1.busy = false; // Avoid saving of images when the stream is started
    strcpy(CamData1.imagepath, dir_cam0.c_str()); // Specify the image prefix
    strcpy(CamData1.imagename,tstamp.c_str()); // Specify the image name

    CamData2.ImageCounter = 0;
//    CamData2.ReceivedAnImage = false;
    CamData2.busy = false; // Avoid saving of images when the stream is started
    strcpy(CamData2.imagepath, dir_cam1.c_str()); // Specify the image prefix
    strcpy(CamData2.imagename,tstamp.c_str()); // Specify the image name

    // Open cameras by serial number. Use "tcam-ctrl -l" to query a list of cameras
    TcamCamera cam1("28610163");
    TcamCamera cam2("28610165");

    // Set video format, resolution and frame rate // {1920,1200} // {1280,720} | FrameRate{20,1}
    cam1.set_capture_format("BGRx", FrameSize{1920,1200}, FrameRate{54,1});
    cam2.set_capture_format("BGRx", FrameSize{1920,1200}, FrameRate{54,1});

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
    // Disable trigger mode first
    // TriggerMode->set(cam,"Off"); // Use this line for GigE cameras
    TriggerMode->set(cam1,0);
    TriggerMode->set(cam2,0);

    std::shared_ptr<Property> prop_gain = cam1.get_property("Gain");
    prop_gain->set(cam1, 300); // 2 outdoor | 300 indoor
    prop_gain->set(cam2, 300);

    std::shared_ptr<Property> prop_brightness = cam1.get_property("Brightness");
    prop_brightness->set(cam1, 400);
    prop_brightness->set(cam2, 400);

    std::shared_ptr<Property> prop_exposure = cam1.get_property("Exposure");
    prop_exposure->set(cam1, 10000);
    prop_exposure->set(cam2, 10000);

//    std::shared_ptr<Property> prop_trigger_delay = cam1.get_property("Trigger Delay (us)"); // Trigger Delay (us)
//    prop_trigger_delay->set(cam1, 1000);
//    prop_trigger_delay->set(cam2, 1000);

    // Comment following line, if no live video display is wanted.
    cam1.enable_video_display(gst_element_factory_make("ximagesink", NULL));
    cam2.enable_video_display(gst_element_factory_make("ximagesink", NULL));

    // Register a callback to be called for each new frame
    cam1.set_new_frame_callback(new_frame_cb, &CamData1);
    cam2.set_new_frame_callback(new_frame_cb, &CamData2);

    // The IMU must be the first device connected to the USB to have the port address: /dev/ttyUSB0
    DeviceClass imu;
    XsPortInfo mtPort("/dev/ttyUSB0", XsBaud::numericToRate(115200));
    string out_imu_file = out_dir + "/imu0.csv";
    //cout << "IMU output file: " << out_imu_file << endl;

	try
	{
        /******************************************************************************************/
        /*                                  CONFIGURE THE IMU                                     */
        /******************************************************************************************/

        // Open the port with the detected imu
        cout << "Opening port..." << endl;
        if (!imu.openPort(mtPort))
            throw runtime_error("Could not open port. Aborting.");

        // Put the imu in configuration mode
        cout << "Putting imu into configuration mode..." << endl;
        if (!imu.gotoConfig()) // Put the imu into configuration mode before configuring the imu
		{
            throw runtime_error("Could not put imu into configuration mode. Aborting.");
		}

        // Request the imu Id to check the imu type
        mtPort.setDeviceId(imu.getDeviceId());

        // Check if we have an MTi / MTx / MTmk4 imu
		if (!mtPort.deviceId().isMt9c() && !mtPort.deviceId().isLegacyMtig() && !mtPort.deviceId().isMtMk4() && !mtPort.deviceId().isFmt_X000())
		{
            throw runtime_error("No MTi / MTx / MTmk4 imu found. Aborting.");
		}
        cout << "Found a imu with id: " << mtPort.deviceId().toString().toStdString() << " @ port: " << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate() << endl;

		try
		{
            // Print information about detected MTi / MTx / MTmk4 imu
            cout << "Device: " << imu.getProductCode().toStdString() << " opened." << endl;

            // Configure the imu. Note the differences between MTix and MTmk4
            cout << "Configuring the imu..." << mtPort.deviceId().isMt9c() << " " << mtPort.deviceId().isLegacyMtig() << " " << mtPort.deviceId().isMtMk4() << " " << mtPort.deviceId().isFmt_X000() << endl;
			if (mtPort.deviceId().isMt9c() || mtPort.deviceId().isLegacyMtig())
			{
//				XsOutputMode outputMode = XOM_Orientation; // output orientation data
//				XsOutputSettings outputSettings = XOS_OrientationMode_Quaternion; // output orientation data as quaternion
                XsOutputMode outputMode = XOM_Calibrated; // Calibrated data
                XsOutputSettings outputSettings = XOS_CalibratedMode_All;

                // set the imu configuration
                if (!imu.setDeviceMode(outputMode, outputSettings))
				{
                    throw runtime_error("Could not configure MT imu. Aborting.");
				}
			}
			else if (mtPort.deviceId().isMtMk4() || mtPort.deviceId().isFmt_X000())
			{
                //XsOutputConfiguration sync(XMID_SetSyncConfiguration, imu_freq);
                XsOutputConfiguration quat(XDI_Quaternion, imu_freq);
                XsOutputConfiguration time(XDI_UtcTime, imu_freq);
                XsOutputConfiguration acc(XDI_Acceleration,imu_freq);
                XsOutputConfiguration vel(XDI_RateOfTurn, imu_freq);
                XsOutputConfiguration sta(XDI_StatusWord, imu_freq);
				XsOutputConfigurationArray configArray;
                //configArray.push_back(sync);
                configArray.push_back(quat);
                configArray.push_back(time);
                configArray.push_back(acc);
                configArray.push_back(vel);
                configArray.push_back(sta);
                if (!imu.setOutputConfiguration(configArray))
				{
                    throw runtime_error("Could not configure MTmk4 imu. Aborting.");
				}
			}
			else
			{
                throw runtime_error("Unknown imu while configuring. Aborting.");
			}

            ofstream out_imu( out_imu_file.c_str() );
            if (!out_imu)
            {
                cerr << "IMU output file could not be opened: " << out_imu_file << endl;
                exit(1);
            }
            else
            {
                out_imu << "### timestamp, " <<"omega_X, " << "omega_Y, " << "omega_Z, " << "alpha_X, " << "alpha_Y, " << "alpha_Z, " << "roll, " << "pitch, " << "yaw ###" << endl;
                out_imu.flush();
            }

//            // Defining SyncOut Settings // XsSyncFunction ??
//            XsSyncSetting syncOutSetting(XSL_Outputs, XSF_IntervalTransitionMeasurement, XSP_Both, imu_freq, 0, 0, 10, 1, 0);
//            XsSyncSettingArray syncSetting;
//            syncSetting.push_back(syncOutSetting);
//            //imu.XMID_SetSyncConfiguration(0x2C);

            // Put the imu in measurement mode
            cout << "Putting imu into measurement mode..." << endl;
            if (!imu.gotoMeasurement())
			{
                throw runtime_error("Could not put imu into measurement mode. Aborting.");
			}

            cout << "\nMain loop (press any key to quit)" << endl;
            cout << string(79, '-') << endl;

            bool b_first_tstamp = true;
            uint64_t timestamp_offset(0);
            XsUtcTime tevent;
			XsByteArray data;
			XsMessageArray msgs;

            // Start the camera
            cam1.start();
            cam2.start();

            std::shared_ptr<Property> prop_gain_auto = cam1.get_property("Gain Auto");
            prop_gain_auto->set(cam1, 0);
            prop_gain_auto->set(cam2, 0);

//            std::shared_ptr<Property> prop_whitebalance_auto = cam1.get_property("whitebalance-auto");
//            prop_whitebalance_auto->set(cam1, 0);
//            prop_whitebalance_auto->set(cam2, 0);

            std::shared_ptr<Property> prop_exposure_auto = cam1.get_property("Exposure Auto");
            prop_exposure_auto->set(cam1, 0);
            prop_exposure_auto->set(cam2, 0);

//            std::shared_ptr<Property> prop_offset_center_auto = cam1.get_property("Offset Auto Center");
//            prop_offset_center_auto->set(cam1, 0);
//            prop_offset_center_auto->set(cam2, 0);

//            std::shared_ptr<Property> prop_exposure_max = cam1.get_property("Exposure Max");
//            prop_exposure_max->set(cam1, 40000);
//            prop_exposure_max->set(cam2, 40000);

//            int exposure;
//            prop_exposure->get(cam1, exposure);
//            std::cout << "Exposure Value cam1: " << exposure << std::endl;

//            waitforkey("Press Enter to stop triggering and image saving.");

            CamData1.busy = false; // allow saving of images when the stream is started
            CamData2.busy = false; // allow saving of images when the stream is started
            CamData1.ImageCounter = 0; // Reset the image counter
            CamData2.ImageCounter = 0; // Reset the image counter

            // Enable trigger mode
            TriggerMode->set(cam1,1);
            TriggerMode->set(cam2,1);

            cout << "ListProperties: \n" << endl;
            ListProperties(cam1);

            int iter = 0;
            int relative_freq = imu_frequency / fps;
			while (!_kbhit())
			{
                imu.readDataToBuffer(data);
                imu.processBufferedData(data, msgs);
                //cout << "timestamp system " << duration_cast<milliseconds>(high_resolution_clock::now().time_since_epoch()).count() << endl;

				for (XsMessageArray::iterator it = msgs.begin(); it != msgs.end(); ++it)
				{
					// Retrieve a packet
					XsDataPacket packet;
                    if ((*it).getMessageId() == XMID_MtData)
                    {
						LegacyDataPacket lpacket(1, false);
						lpacket.setMessage((*it));
						lpacket.setXbusSystem(false);
						lpacket.setDeviceId(mtPort.deviceId(), 0);
                        lpacket.setDataFormat(XOM_Calibrated, XOS_CalibratedMode_All,0);	//lint !e534
						XsDataPacket_assignFromLegacyDataPacket(&packet, &lpacket, 0);
					}
                    else if ((*it).getMessageId() == XMID_MtData2)
                    {
						packet.setMessage((*it));
						packet.setDeviceId(mtPort.deviceId());
					}

                    //uint64_t timestamp = 0;
                    if (packet.containsUtcTime())
                    {
                        //ListProperties(cam1);

                        tevent = packet.utcTime();
                        //cout << "Time: " << uint32_t(tevent.m_hour) << ":" << uint32_t(tevent.m_minute) << ":" << uint32_t(tevent.m_second) << " - " << tevent.m_nano << endl;
                        uint32_t status = packet.status();

                        uint64_t timestamp = uint64_t(((uint64_t((uint64_t(((uint64_t((tevent.m_year - 1925)*365)) + uint64_t((tevent.m_month - 1)*30) + tevent.m_day)*24) + tevent.m_hour)*60) + tevent.m_minute)*60 + tevent.m_second)*1000000000) + uint64_t((tevent.m_nano/1000000)*1000000);
                        //cout << "timestamp " << timestamp << endl;
                        if(b_first_tstamp)
                        {
                            uint64_t chrono_timestamp = (high_resolution_clock::now().time_since_epoch().count()/uint64_t(1e6))*uint64_t(1e6);
                            timestamp_offset = chrono_timestamp - timestamp;
                            //cout << "timestamp chrono " << chrono_timestamp << " diff " << timestamp_offset << " round " << 1e6 << endl;
                            b_first_tstamp = false;
                        }
                        timestamp = timestamp + timestamp_offset;

                        if( (iter++) % relative_freq == 0 )
                        {
                            cout << "timestamp updated " << timestamp << endl;
                            //thread save_img(cv::imwrite, (string(pCamData->imagepath)+to_string(high_resolution_clock::now().time_since_epoch().count())+".jpg"), pCamData->frame);
                            string timestp_name( to_string(timestamp) );
                            strcpy(CamData1.imagename, timestp_name.c_str()); // Specify the image name
                            strcpy(CamData2.imagename, timestp_name.c_str()); // Specify the image name
                        }

                        vector<XsReal> Acceleration( (packet.calibratedAcceleration()).toVector() );
                        vector<XsReal> Velocity( (packet.calibratedGyroscopeData()).toVector() );

//                        // Get the quaternion data
//                        XsQuaternion quaternion = packet.orientationQuaternion();
    //                    cout << "\r"
    //                              << "W:" << setw(5) << fixed << setprecision(2) << quaternion.w()
    //                              << ",X:" << setw(5) << fixed << setprecision(2) << quaternion.x()
    //                              << ",Y:" << setw(5) << fixed << setprecision(2) << quaternion.y()
    //                              << ",Z:" << setw(5) << fixed << setprecision(2) << quaternion.z()
    //					;

                        // Convert packet to euler
                        XsEuler euler = packet.orientationEuler();
    //                    cout << ",Roll:" << setw(7) << fixed << setprecision(2) << euler.roll()
    //                              << ",Pitch:" << setw(7) << fixed << setprecision(2) << euler.pitch()
    //                              << ",Yaw:" << setw(7) << fixed << setprecision(2) << euler.yaw()
    //					;
    //                    cout << flush;

    //                    if (packet.containsSampleTime64())
    //                        cout << "sampleTime64 " << packet.sampleTime64() << endl;
    ////                        uint64_t timestamp = packet.sampleTime64();
    ///
                        out_imu << timestamp << "," << Velocity[0] << "," << Velocity[1] << "," << Velocity[2] << "," << Acceleration[0] << "," << Acceleration[1] << "," << Acceleration[2]
                                //<< "," << quaternion.w() << "," << quaternion.x() << "," << quaternion.y() << "," << quaternion.z()
                                << "," << euler.roll() << "," << euler.pitch() << "," << euler.yaw()
                                << endl;
                        out_imu.flush();
                    }
				}
				msgs.clear();
                XsTime::msleep(0);
			}
			_getch();
            cout << "\n" << string(79, '-') << "\n";
            cout << endl;
		}
        catch (runtime_error const & error)
		{
            cout << error.what() << endl;
		}
		catch (...)
		{
            cout << "An unknown fatal error has occured. Aborting." << endl;
		}

        // Close cameras
        CamData1.busy = true; // deny saving of images
        CamData2.busy = true; // deny saving of images
//        // Disable the trigger mode, so other programs will see a live video.
//        TriggerMode->set(cam1,0); // Use this line for USB cameras
//        TriggerMode->set(cam2,0); // Use this line for USB cameras
        cam1.stop();
        cam2.stop();

        // Close imu port
        cout << "Closing port..." << endl;
        imu.close();
	}
    catch (runtime_error const & error)
	{
        cout << error.what() << endl;
	}
	catch (...)
	{
        cout << "An unknown fatal error has occured. Aborting." << endl;
	}

    cout << "Successful exit." << endl;

    cout << "Press [ENTER] to continue." << endl; cin.get();

	return 0;
}
