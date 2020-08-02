////
////  opencv_artoolkit_main.cpp
////  nftSimple
////
////  Created by Nadkarni, Kanika on 7/20/20.
////
//
////
////  opencv_artoolkit_github.cpp
////
////
////  Created by Nadkarni, Kanika on 7/20/20.
////
//
////-----------------------//#include "ar_toolkit.h"---------------
//#ifdef _WIN32
//#  include <windows.h>
//#endif
//#include <stdio.h>
//#ifdef _WIN32
//#  define snprintf _snprintf
//#endif
//#include <string.h>
//#ifdef __APPLE__
//#  include <GLUT/glut.h>
//#else
//#  include <GL/glut.h>
//#endif
//
//
//#include <AR/ar.h>
//#include <AR/arMulti.h>
//#include <AR/video.h>
//#include <AR/gsub_lite.h>
//#include <AR/arFilterTransMat.h>
//#include <AR2/tracking.h>
//#include <AR/paramGL.h>
//#include <ARUtil/time.h>
//#include <AR2/util.h>
//
//#include "ARMarkerNFT.h"
//#include "trackingSub.h"
//#include <opencv2/core/core_c.h>
//
//// ============================================================================
////    Constants
//// ============================================================================
//
//#define PAGES_MAX               10          // Maximum number of pages expected. You can change this down (to save memory) or up (to accomodate more pages.)
//
//#define VIEW_SCALEFACTOR        1.0            // Units received from ARToolKit tracking will be multiplied by this factor before being used in OpenGL drawing.
//#define VIEW_DISTANCE_MIN        10.0        // Objects closer to the camera than this will not be displayed. OpenGL units.
//#define VIEW_DISTANCE_MAX        10000.0        // Objects further away from the camera than this will not be displayed. OpenGL units.
//
//// ============================================================================
////    Global variables
//// ============================================================================
//
//// Image acquisition.
//static ARUint8        *gARTImage = NULL;
//static long            gCallCountMarkerDetect = 0;
//
//// Markers.
//ARMarkerNFT *markersNFT = NULL;
//int markersNFTCount = 0;
//
//// NFT.
//static THREAD_HANDLE_T     *threadHandle = NULL;
//static AR2HandleT          *ar2Handle = NULL;
//static KpmHandle           *kpmHandle = NULL;
//static int                  surfaceSetCount = 0;
//static AR2SurfaceSetT      *surfaceSet[20];
//
//// Drawing.
//static ARParamLT *gCparamLT = NULL;
//static ARGL_CONTEXT_SETTINGS_REF gArglSettings = NULL;
//static ARdouble cameraLens[16];
//
//// ============================================================================
////    Function prototypes
//// ============================================================================
//
//static int setupCamera(const char *cparam_name, char *vconf, ARParamLT **cparamLT_p)
//{
//    ARParam            cparam;
//    int                xsize, ysize;
//    AR_PIXEL_FORMAT pixFormat;
//
//    // Open the video path.
//    if (arVideoOpen(vconf) < 0) {
//        ARLOGe("setupCamera(): Unable to open connection to camera.\n");
//        return (FALSE);
//    }
//
//    // Find the size of the window.
//    if (arVideoGetSize(&xsize, &ysize) < 0) {
//        ARLOGe("setupCamera(): Unable to determine camera frame size.\n");
//        arVideoClose();
//        return (FALSE);
//    }
//    ARLOGi("Camera image size (x,y) = (%d,%d)\n", xsize, ysize);
//
//    // Get the format in which the camera is returning pixels.
//    pixFormat = arVideoGetPixelFormat();
//    if (pixFormat == AR_PIXEL_FORMAT_INVALID) {
//        ARLOGe("setupCamera(): Camera is using unsupported pixel format.\n");
//        arVideoClose();
//        return (FALSE);
//    }
//
//    // Load the camera parameters, resize for the window and init.
//    if (arParamLoad(cparam_name, 1, &cparam) < 0) {
//        ARLOGe("setupCamera(): Error loading parameter file %s for camera.\n", cparam_name);
//        arVideoClose();
//        return (FALSE);
//    }
//    if (cparam.xsize != xsize || cparam.ysize != ysize) {
//        ARLOGw("*** Camera Parameter resized from %d, %d. ***\n", cparam.xsize, cparam.ysize);
//        arParamChangeSize(&cparam, xsize, ysize, &cparam);
//    }
//#ifdef DEBUG
//    ARLOG("*** Camera Parameter ***\n");
//    arParamDisp(&cparam);
//#endif
//    if ((*cparamLT_p = arParamLTCreate(&cparam, AR_PARAM_LT_DEFAULT_OFFSET)) == NULL) {
//        ARLOGe("setupCamera(): Error: arParamLTCreate.\n");
//        arVideoClose();
//        return (FALSE);
//    }
//
//    return (TRUE);
//}
//
//// Modifies globals: kpmHandle, ar2Handle.
//static int initNFT(ARParamLT *cparamLT, AR_PIXEL_FORMAT pixFormat)
//{
//    ARLOGd("Initialising NFT.\n");
//    //
//    // NFT init.
//    //
//
//    // KPM init.
//    kpmHandle = kpmCreateHandle(cparamLT);
//    if (!kpmHandle) {
//        ARLOGe("Error: kpmCreateHandle.\n");
//        return (FALSE);
//    }
//    //kpmSetProcMode( kpmHandle, KpmProcHalfSize );
//
//    // AR2 init.
//    if( (ar2Handle = ar2CreateHandle(cparamLT, pixFormat, AR2_TRACKING_DEFAULT_THREAD_NUM)) == NULL ) {
//        ARLOGe("Error: ar2CreateHandle.\n");
//        kpmDeleteHandle(&kpmHandle);
//        return (FALSE);
//    }
//    if (threadGetCPU() <= 1) {
//        ARLOGi("Using NFT tracking settings for a single CPU.\n");
//        ar2SetTrackingThresh(ar2Handle, 5.0);
//        ar2SetSimThresh(ar2Handle, 0.50);
//        ar2SetSearchFeatureNum(ar2Handle, 16);
//        ar2SetSearchSize(ar2Handle, 6);
//        ar2SetTemplateSize1(ar2Handle, 6);
//        ar2SetTemplateSize2(ar2Handle, 6);
//    } else {
//        ARLOGi("Using NFT tracking settings for more than one CPU.\n");
//        ar2SetTrackingThresh(ar2Handle, 5.0);
//        ar2SetSimThresh(ar2Handle, 0.50);
//        ar2SetSearchFeatureNum(ar2Handle, 16);
//        ar2SetSearchSize(ar2Handle, 12);
//        ar2SetTemplateSize1(ar2Handle, 6);
//        ar2SetTemplateSize2(ar2Handle, 6);
//    }
//    // NFT dataset loading will happen later.
//    return (TRUE);
//}
//
//
//// Modifies globals: threadHandle, surfaceSet[], surfaceSetCount
//static int unloadNFTData(void)
//{
//    int i, j;
//
//    if (threadHandle) {
//        ARLOGi("Stopping NFT2 tracking thread.\n");
//        trackingInitQuit(&threadHandle);
//    }
//    j = 0;
//    for (i = 0; i < surfaceSetCount; i++) {
//        if (j == 0) ARLOGi("Unloading NFT tracking surfaces.\n");
//        ar2FreeSurfaceSet(&surfaceSet[i]); // Also sets surfaceSet[i] to NULL.
//        j++;
//    }
//    if (j > 0) ARLOGi("Unloaded %d NFT tracking surfaces.\n", j);
//    surfaceSetCount = 0;
//
//    return 0;
//}
//
//
//// References globals: markersNFTCount
//// Modifies globals: threadHandle, surfaceSet[], surfaceSetCount, markersNFT[]
//static int loadNFTData(void)
//{
//    int i;
//    KpmRefDataSet *refDataSet;
//
//    // If data was already loaded, stop KPM tracking thread and unload previously loaded data.
//    if (threadHandle) {
//        ARLOGi("Reloading NFT data.\n");
//        unloadNFTData();
//    } else {
//        ARLOGi("Loading NFT data.\n");
//    }
//
//    refDataSet = NULL;
//
//    for (i = 0; i < markersNFTCount; i++) {
//        // Load KPM data.
//        KpmRefDataSet  *refDataSet2;
//        ARLOGi("Reading %s.fset3\n", markersNFT[i].datasetPathname);
//        if (kpmLoadRefDataSet(markersNFT[i].datasetPathname, "fset3", &refDataSet2) < 0 ) {
//            ARLOGe("Error reading KPM data from %s.fset3\n", markersNFT[i].datasetPathname);
//            markersNFT[i].pageNo = -1;
//            continue;
//        }
//        markersNFT[i].pageNo = surfaceSetCount;
//        ARLOGi("  Assigned page no. %d.\n", surfaceSetCount);
//        if (kpmChangePageNoOfRefDataSet(refDataSet2, KpmChangePageNoAllPages, surfaceSetCount) < 0) {
//            ARLOGe("Error: kpmChangePageNoOfRefDataSet\n");
//            exit(-1);
//        }
//        if (kpmMergeRefDataSet(&refDataSet, &refDataSet2) < 0) {
//            ARLOGe("Error: kpmMergeRefDataSet\n");
//            exit(-1);
//        }
//        ARLOGi("  Done.\n");
//
//        // Load AR2 data.
//        ARLOGi("Reading %s.fset\n", markersNFT[i].datasetPathname);
//
//        if ((surfaceSet[surfaceSetCount] = ar2ReadSurfaceSet(markersNFT[i].datasetPathname, "fset", NULL)) == NULL ) {
//            ARLOGe("Error reading data from %s.fset\n", markersNFT[i].datasetPathname);
//        }
//        ARLOGi("  Done.\n");
//
//        surfaceSetCount++;
//        if (surfaceSetCount == PAGES_MAX) break;
//    }
//    if (kpmSetRefDataSet(kpmHandle, refDataSet) < 0) {
//        ARLOGe("Error: kpmSetRefDataSet\n");
//        exit(-1);
//    }
//    kpmDeleteRefDataSet(&refDataSet);
//
//    // Start the KPM tracking thread.
//    threadHandle = trackingInitInit(kpmHandle);
//    if (!threadHandle) exit(-1);
//
//    ARLOGi("Loading of NFT data complete.\n");
//    return (TRUE);
//}
//
//
//static void cleanup(void)
//{
//    if (markersNFT) deleteMarkers(&markersNFT, &markersNFTCount);
//
//    // NFT cleanup.
//    unloadNFTData();
//    ARLOGd("Cleaning up ARToolKit NFT handles.\n");
//    ar2DeleteHandle(&ar2Handle);
//    kpmDeleteHandle(&kpmHandle);
//    arParamLTFree(&gCparamLT);
//}
//
////---------------------------------------------------------------------
//
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/core/core.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
//
////--------------//#include "utility.h"----------------------
//#include<stdio.h>
//#include<stdlib.h>
//#include<iostream>
//using namespace std;
//template <int N>
//void Print(const ARdouble (&arra)[N])
//{
//    for (const auto e : arra) {
//         std::cout<<std::to_string(e)<<"  ";
//    }
//    std::cout<<endl;
//}
//
//template <int N,int M>
//void PrintTransform(const float (&arra)[N][M])
//{
//    for(int i = 0 ;i < N ;i++){
//        for(int j = 0 ; j < M ; j++){
//            std::cout<<arra[i][j]<<"  ";
//        }
//    }
//    std::cout<<std::endl;
//}
////---------------------------------------------------------
//
//using namespace cv;
//using namespace std;
//
//
////=========Functions========
//static bool initDetection();
//static void runDetection(Mat &input_mat);
//
////====Configuration Value=====
////const char *cparam_name = "Data2/camera_para.dat";
//const char *cparam_name = NULL;
//char vconf[] = "";
//const char markerConfigDataFilename[] = "/Users/knnadkar/Desktop/artoolkit5-master/share/nftSimple/Data2/markers.dat";///Users/knnadkar/Desktop/artoolkit5-master
//
//ARParam            cparam;
//int                xsize = 640 , ysize = 480;
//AR_PIXEL_FORMAT pixFormat  = AR_PIXEL_FORMAT_RGB ;
//
//
//int main() {
//    VideoCapture stream1(0);   //0 is the id of video device.0 if you have only one camera.
//
//    if (!stream1.isOpened()) { //check if video device has been initialised
//        std::cout << "cannot open camera";
//    }
//
//    initDetection();
//    while (true) {
//        Mat cameraFrame;
//        stream1.read(cameraFrame);
//
//        /* detect the markers in the video frame */
//        runDetection(cameraFrame);
//
//        imshow("NFT",cameraFrame);
//        if (waitKey(30) >= 0)
//            break;
//    }
//    return 0;
//}
//
//
//static bool initDetection()
//{
//    // AR init.  ========================================
//
//       //Load camera parameter data
//       if (cparam_name && *cparam_name) {
//           if (arParamLoad(cparam_name, 1, &cparam) < 0) {
//               ARLOGe("initDetection##setupCamera(): Error loading parameter file %s for camera.\n", cparam_name);
//               arVideoClose();
//               return (FALSE);
//           }
//       } else {
//           arParamClearWithFOVy(&cparam, xsize, ysize, M_PI_4); // M_PI_4 radians = 45 degrees.
//           ARLOGw("initDetection##Using default camera parameters for %dx%d image size, 45 degrees vertical field-of-view.\n", xsize, ysize);
//       }
//
//       //Resize the frame
//       if (cparam.xsize != xsize || cparam.ysize != ysize) {
//           ARLOGw("*** Camera Parameter resized from %d, %d. ***\n", cparam.xsize, cparam.ysize);
//           arParamChangeSize(&cparam, xsize, ysize, &cparam);
//       }
//
//       // Prints the camera parameter
//       ARLOG("*** Camera Parameter ***\n");
//       arParamDisp(&cparam);
//
//       //Create a lookuptable from the parameters
//       if ((gCparamLT = arParamLTCreate(&cparam, AR_PARAM_LT_DEFAULT_OFFSET)) == NULL) {
//           ARLOGe("setupCamera(): Error: arParamLTCreate.\n");
//           arVideoClose();
//           return (FALSE);
//       }
//
//    //Initializing NFT
//    if (!initNFT(gCparamLT, pixFormat)) {
//        ARLOG("*** Init error ***\n");
//    }
//
//    // Markers setup. ========================================
//
//    // Load marker(s).
//    newMarkers(markerConfigDataFilename, &markersNFT, &markersNFTCount);
//    if (!markersNFTCount) {
//        ARLOGe("Error loading markers from config. file '%s'.\n", markerConfigDataFilename);
//        cleanup();
//        exit(-1);
//    }
//    ARLOGi("Marker count = %d\n", markersNFTCount);
//
//    // Marker data has been loaded, so now load NFT data.
//    if (!loadNFTData()) {
//        ARLOGe("Error loading NFT data.\n");
//        cleanup();
//        exit(-1);
//    }
//    return TRUE;
//}
//
//
//static void runDetection(Mat &input_mat)
//{
//    ARUint8        *gARTImage = NULL;
//    gARTImage = (uchar *)input_mat.data;
//    float trackingTrans[3][4] = {0,0,0,0,0,0,0,0,0,0,0,0};
//    cv::Mat rot_vec = Mat::zeros(1,3,CV_64F), trn_vec = Mat::zeros(1,3,CV_64F) , rot_mat = Mat::zeros(3,3,CV_64F);
//
//
//    if(true){
//        // NFT results.
//        static int detectedPage = -2; // -2 Tracking not inited, -1 tracking inited OK, >= 0 tracking online on page.
//
//        int i, j, k;
//
//        // Grab a video frame.
//        if (gARTImage != NULL) {
//
//            gCallCountMarkerDetect++; // Increment ARToolKit FPS counter.
//
//            // Run marker detection on frame
//            if (threadHandle) {
//                // Perform NFT tracking.
//                ARLOGi("**Entered threadhandle detected if statement");
//                float            err;
//                int              ret;
//                int              pageNo;
//
//                if( detectedPage == -2 ) {
//                    trackingInitStart( threadHandle, gARTImage );
//                    ARLOGi("**Started tracking setting detectedPage = -1");
//                    detectedPage = -1;
//                }
//                if( detectedPage == -1 ) {
//                    ARLOGi("**Entered detectedPage == -1 if statement");
//                    ret = trackingInitGetResult( threadHandle, trackingTrans, &pageNo);
//                    if( ret == 1 ) {
//                        ARLOGi("**Entered marker detected if statement");
//
//                        if (pageNo >= 0 && pageNo < surfaceSetCount) {
//                            ARLOGd("Detected page %d.\n", pageNo);
//                            detectedPage = pageNo;
//                            ar2SetInitTrans(surfaceSet[detectedPage], trackingTrans);
//                        } else {
//                            ARLOGe("Detected bad page %d.\n", pageNo);
//                            detectedPage = -2;
//                        }
//                    } else if( ret < 0 ) {
//                        ARLOGd("No page detected.\n");
//                        detectedPage = -2;
//                    }
//                }
//                if( detectedPage >= 0 && detectedPage < surfaceSetCount) {
//                    if( ar2Tracking(ar2Handle, surfaceSet[detectedPage], gARTImage, trackingTrans, &err) < 0 ) {
//                        ARLOGd("Tracking lost.\n");
//                        detectedPage = -2;
//                    } else {
//                        ARLOGi("**Entered ar2Tracking tracked page if statement");
//                        ARLOGd("Tracked page %d (max %d).\n", detectedPage, surfaceSetCount - 1);
//                    }
//                }
//            } else {
//                ARLOGe("Error: threadHandle\n");
//                detectedPage = -2;
//            }
//
//            // Update markers.
//            for (i = 0; i < markersNFTCount; i++) {
//                markersNFT[i].validPrev = markersNFT[i].valid;
//                if (markersNFT[i].pageNo >= 0 && markersNFT[i].pageNo == detectedPage) {
//                    markersNFT[i].valid = TRUE;
//                    for (j = 0; j < 3; j++)
//                        for (k = 0; k < 4; k++){
//                            markersNFT[i].trans[j][k] = trackingTrans[j][k];
//                        }
//
//                }
//                else markersNFT[i].valid = FALSE;
//
//
//                if (markersNFT[i].valid) {
//
//                    // Filter the pose estimate.
//                    if (markersNFT[i].ftmi) {
//                        if (arFilterTransMat(markersNFT[i].ftmi, markersNFT[i].trans, !markersNFT[i].validPrev) < 0) {
//                            ARLOGe("arFilterTransMat error with marker %d.\n", i);
//                        }
//                    }
//
//                    if (!markersNFT[i].validPrev) {
//                        // Marker has become visible, tell any dependent objects.
//                        // --->
//                    }
//
//                    // We have a new pose, so set that.
//                    arglCameraViewRH((const ARdouble (*)[4])markersNFT[i].trans, markersNFT[i].pose.T, VIEW_SCALEFACTOR);
//                    ARLOGi("**Set up new pose. Will print transform matrix");
//
//                    //Print(markersNFT[i].pose.T);
//                    PrintTransform(trackingTrans);
//
//                    //************Rotation Matrix************
//
////                    rot_mat.at<float>(0,0) = trackingTrans[0][0];
////                    rot_mat.at<float>(0,1) = trackingTrans[0][1];
////                    rot_mat.at<float>(0,2) = trackingTrans[0][2];
////                    rot_mat.at<float>(1,0) = trackingTrans[1][0];
////                    rot_mat.at<float>(1,1) = trackingTrans[1][1];
////                    rot_mat.at<float>(1,2) = trackingTrans[1][2];
////                    rot_mat.at<float>(2,0) = trackingTrans[2][0];
////                    rot_mat.at<float>(2,1) = trackingTrans[2][1];
////                    rot_mat.at<float>(2,2) = trackingTrans[2][2];
//
//                    rot_mat.at<float>(0,0) = trackingTrans[0][0];
//                    rot_mat.at<float>(0,1) = trackingTrans[0][0];
//                    rot_mat.at<float>(0,2) = trackingTrans[0][0];
//                    rot_mat.at<float>(1,0) = trackingTrans[0][0];
//                    rot_mat.at<float>(1,1) = trackingTrans[0][0];
//                    rot_mat.at<float>(1,2) = trackingTrans[0][0];
//                    rot_mat.at<float>(2,0) = trackingTrans[0][0];
//                    rot_mat.at<float>(2,1) = trackingTrans[0][0];
//                    rot_mat.at<float>(2,2) = trackingTrans[0][0];
//
//                    Rodrigues(rot_mat, rot_vec);
//                    ARLOGi("**Performed Rodrigues");
//                    //************Translation Matrix***********
//                    trn_vec.at<double>(0,0) = trackingTrans[0][3];
//                    trn_vec.at<double>(0,1) = trackingTrans[1][3];
//                    trn_vec.at<double>(0,2) = trackingTrans[2][3];
//
//                    std::cout<<trn_vec<<endl;
//
//                    //************Camera Matrix*****************
//                    cv::Mat intrisicMat(3, 3, cv::DataType<double>::type); // Intrisic matrix
//                    intrisicMat.at<double>(0, 0) = 674.171631;
//                    intrisicMat.at<double>(1, 0) = 0;
//                    intrisicMat.at<double>(2, 0) = 0;
//
//                    intrisicMat.at<double>(0, 1) = 0;
//                    intrisicMat.at<double>(1, 1) = 633.898087;
//                    intrisicMat.at<double>(2, 1) = 0;
//
//                    intrisicMat.at<double>(0, 2) = 318.297791;
//                    intrisicMat.at<double>(1, 2) = 237.900467;
//                    intrisicMat.at<double>(2, 2) = 1;
//
//                    //************Dist Matrix**********************
//                    cv::Mat distCoeffs(5, 1, cv::DataType<double>::type);   // Distortion vector
//                    distCoeffs.at<double>(0) = 0.1147807688;
//                    distCoeffs.at<double>(1) = -0.5208189487;
//                    distCoeffs.at<double>(2) = -0.0002069871;
//                    distCoeffs.at<double>(3) = -0.0040593124;
//                    distCoeffs.at<double>(4) = 0;
//
//
//                    std::vector<cv::Point3d> inputPnt ;
//                    std::vector<cv::Point2d> outPnt;
//
//                    cv::Point3d pnt = Point3d(0,0,0);
//                    std::cout<<pnt<<endl;
//
//                    //cv::Point3d pnt = Point3d(markersNFT[i].pose.T[12],markersNFT[i].pose.T[13],markersNFT[i].pose.T[14]);
//
//                    inputPnt.push_back(pnt);
//
//                    cv::projectPoints(inputPnt,rot_vec,trn_vec,intrisicMat,distCoeffs,outPnt);
//                    std::cout<<outPnt<<endl;
//
//                    // Convert from ARToolkit Image type to OpenCV Mat
//                    IplImage* iplImg;
//                    iplImg = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U,3);
//
//                    // Get video frame from ARToolkit
//                    iplImg->imageData = (char *)gARTImage;
//                    input_mat = cv::cvarrToMat(iplImg);
//                    ARLOGi("**opencv image created, drawing circle");
//                    //std::cout<<outPnt<<"**********"<<endl;
//                    //****Draw a Circle using the pose data****
//                    cv::circle(input_mat,Point(outPnt[0].x,outPnt[0].y),30,Scalar(255,255,255) ,4);
//
//                } else {
//
//                    if (markersNFT[i].validPrev) {
//                        // Marker has ceased to be visible, tell any dependent objects.
//                        // --->
//                        ARLOGi("**Marker has become invalid");
//                    }
//                }
//            }
//        }
//    }
//}
//
