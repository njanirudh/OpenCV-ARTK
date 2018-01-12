#include "ar_toolkit.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "utility.h"

using namespace cv;
using namespace std;


//=========Functions========
static bool initDetection();
static void runDetection(Mat &input_mat);

//====Configuration Value=====
const char *cparam_name = "Data2/camera_para.dat";
char vconf[] = "";
const char markerConfigDataFilename[] = "Data2/markers.dat";

ARParam			cparam;
int				xsize = 640 , ysize = 480;
AR_PIXEL_FORMAT pixFormat  = AR_PIXEL_FORMAT_RGB ;


int main() {
    VideoCapture stream1(0);   //0 is the id of video device.0 if you have only one camera.

    if (!stream1.isOpened()) { //check if video device has been initialised
        cout << "cannot open camera";
    }

    initDetection();
    while (true) {
        Mat cameraFrame;
        stream1.read(cameraFrame);

        /* detect the markers in the video frame */
        runDetection(cameraFrame);

        imshow("NFT",cameraFrame);
        if (waitKey(30) >= 0)
            break;
    }
    return 0;
}


static bool initDetection()
{
    // AR init.  ========================================

    //Load camera parameter data
    if (arParamLoad(cparam_name, 1, &cparam) < 0) {
        ARLOGe("setupCamera(): Error loading parameter file %s for camera.\n", cparam_name);
        arVideoClose();
        return (FALSE);
    }

    //Resize the frame
    if (cparam.xsize != xsize || cparam.ysize != ysize) {
        ARLOGw("*** Camera Parameter resized from %d, %d. ***\n", cparam.xsize, cparam.ysize);
        arParamChangeSize(&cparam, xsize, ysize, &cparam);
    }

    // Prints the camera parameter
    ARLOG("*** Camera Parameter ***\n");
    arParamDisp(&cparam);

    //Create a lookuptable from the parameters
    if ((gCparamLT = arParamLTCreate(&cparam, AR_PARAM_LT_DEFAULT_OFFSET)) == NULL) {
        ARLOGe("Creates a lookup table.\n", cparam_name);
    }

    //Initializing NFT
    if (!initNFT(gCparamLT, pixFormat)) {
        ARLOG("*** Init error ***\n");
    }

    // Markers setup. ========================================

    // Load marker(s).
    newMarkers(markerConfigDataFilename, &markersNFT, &markersNFTCount);
    if (!markersNFTCount) {
        ARLOGe("Error loading markers from config. file '%s'.\n", markerConfigDataFilename);
        cleanup();
        exit(-1);
    }
    ARLOGi("Marker count = %d\n", markersNFTCount);

    // Marker data has been loaded, so now load NFT data.
    if (!loadNFTData()) {
        ARLOGe("Error loading NFT data.\n");
        cleanup();
        exit(-1);
    }

}


static void runDetection(Mat &input_mat)
{
    ARUint8		*gARTImage = NULL;
    gARTImage = (uchar *)input_mat.data;
    static float trackingTrans[3][4];

    if(true){
        // NFT results.
        static int detectedPage = -2; // -2 Tracking not inited, -1 tracking inited OK, >= 0 tracking online on page.

        int i, j, k;

        // Grab a video frame.
        if (gARTImage != NULL) {

            gCallCountMarkerDetect++; // Increment ARToolKit FPS counter.

            // Run marker detection on frame
            if (threadHandle) {
                // Perform NFT tracking.
                float            err;
                int              ret;
                int              pageNo;

                if( detectedPage == -2 ) {
                    trackingInitStart( threadHandle, gARTImage );
                    detectedPage = -1;
                }
                if( detectedPage == -1 ) {
                    ret = trackingInitGetResult( threadHandle, trackingTrans, &pageNo);
                    if( ret == 1 ) {
                        if (pageNo >= 0 && pageNo < surfaceSetCount) {
                            ARLOGd("Detected page %d.\n", pageNo);
                            detectedPage = pageNo;
                            ar2SetInitTrans(surfaceSet[detectedPage], trackingTrans);
                        } else {
                            ARLOGe("Detected bad page %d.\n", pageNo);
                            detectedPage = -2;
                        }
                    } else if( ret < 0 ) {
                        ARLOGd("No page detected.\n");
                        detectedPage = -2;
                    }
                }
                if( detectedPage >= 0 && detectedPage < surfaceSetCount) {
                    if( ar2Tracking(ar2Handle, surfaceSet[detectedPage], gARTImage, trackingTrans, &err) < 0 ) {
                        ARLOGd("Tracking lost.\n");
                        detectedPage = -2;
                    } else {
                        ARLOGd("Tracked page %d (max %d).\n", detectedPage, surfaceSetCount - 1);
                    }
                }
            } else {
                ARLOGe("Error: threadHandle\n");
                detectedPage = -2;
            }

            // Update markers.
            for (i = 0; i < markersNFTCount; i++) {
                markersNFT[i].validPrev = markersNFT[i].valid;
                if (markersNFT[i].pageNo >= 0 && markersNFT[i].pageNo == detectedPage) {
                    markersNFT[i].valid = TRUE;
                    for (j = 0; j < 3; j++)
                        for (k = 0; k < 4; k++){
                            markersNFT[i].trans[j][k] = trackingTrans[j][k];
                        }

                }
                else markersNFT[i].valid = FALSE;


                if (markersNFT[i].valid) {

                    // Filter the pose estimate.
                    if (markersNFT[i].ftmi) {
                        if (arFilterTransMat(markersNFT[i].ftmi, markersNFT[i].trans, !markersNFT[i].validPrev) < 0) {
                            ARLOGe("arFilterTransMat error with marker %d.\n", i);
                        }
                    }

                    if (!markersNFT[i].validPrev) {
                        // Marker has become visible, tell any dependent objects.
                        // --->
                    }

                    // We have a new pose, so set that.
                    Print(markersNFT[i].pose.T);
                    arglCameraViewRH((const ARdouble (*)[4])markersNFT[i].trans, markersNFT[i].pose.T, VIEW_SCALEFACTOR);

                    // Convert from ARToolkit Image type to OpenCV Mat
                    IplImage* iplImg;
                    iplImg = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U,3);

                    // Get video frame from ARToolkit
                    iplImg->imageData = (char *)gARTImage;
                    input_mat = cv::cvarrToMat(iplImg);

                    //****Draw a Circle using the pose data****
                    cv::circle(input_mat,Point(markersNFT[i].pose.T[12] ,-markersNFT[i].pose.T[13]),30,Scalar(255,255,255) ,4);

                } else {

                    if (markersNFT[i].validPrev) {
                        // Marker has ceased to be visible, tell any dependent objects.
                        // --->
                    }
                }
            }
        }
    }
}
