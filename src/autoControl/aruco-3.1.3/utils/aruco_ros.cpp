/**
Copyright 2017 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
*/

#include "aruco.h"
#include "cvdrawingutils.h"
#include <fstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <string>
#include <stdexcept>

#if  CV_MAJOR_VERSION >= 4
#define CV_CAP_PROP_FRAME_COUNT cv::CAP_PROP_FRAME_COUNT
#define CV_CAP_PROP_POS_FRAMES cv::CAP_PROP_POS_FRAMES
#endif
using namespace std;
using namespace cv;
using namespace aruco;

int aruco_search[5]={106,219,615,999,759};
int write_aruco[5] ={0,0,0,0,0};
int aruco_searchArea[5] = {0,0,0,0,0};


bool refreshAruco = false;

MarkerDetector MDetector;
VideoCapture TheVideoCapturer;
vector<Marker> TheMarkers;
Mat TheInputImage,TheInputImageGrey, TheInputImageCopy;
CameraParameters TheCameraParameters;
void cvTackBarEvents(int pos, void*);
string dictionaryString;
int iDetectMode=0,iMinMarkerSize=0,iCorrectionRate=0,iShowAllCandidates=0,iEnclosed=0,iThreshold,iCornerMode,iDictionaryIndex,iTrack=0;

int waitTime = 0;
bool showMennu=false,bPrintHelp=false,isVideo=false;
class CmdLineParser{int argc;char** argv;public:CmdLineParser(int _argc, char** _argv): argc(_argc), argv(_argv){}   bool operator[](string param)    {int idx = -1;  for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param)idx = i;return (idx != -1);}    string operator()(string param, string defvalue = "-1")    {int idx = -1;for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param)idx = i;if (idx == -1)return defvalue;else return (argv[idx + 1]);}};
struct   TimerAvrg{std::vector<double> times;size_t curr=0,n; std::chrono::high_resolution_clock::time_point begin,end;   TimerAvrg(int _n=30){n=_n;times.reserve(n);   }inline void start(){begin= std::chrono::high_resolution_clock::now();    }inline void stop(){end= std::chrono::high_resolution_clock::now();double duration=double(std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count())*1e-6;if ( times.size()<n) times.push_back(duration);else{ times[curr]=duration; curr++;if (curr>=times.size()) curr=0;}}double getAvrg(){double sum=0;for(auto t:times) sum+=t;return sum/double(times.size());}};

TimerAvrg Fps;

cv::Mat resize(const cv::Mat& in, cv::Size s){
if(s.width==-1 || s.height==-1)return in;
cv::Mat im2;
cv::resize(in, im2, s);
return im2;
}



cv::Mat resize(const cv::Mat& in, int width)
{
    if (in.size().width <= width)
        return in;
    float yf = float(width) / float(in.size().width);
    cv::Mat im2;
    cv::resize(in, im2, cv::Size(width, static_cast<int>(in.size().height * yf)));
    return im2;
}
cv::Mat resizeImage(cv::Mat &in,float resizeFactor){
    if (fabs(1-resizeFactor)<1e-3 )return in;
    float nc=float(in.cols)*resizeFactor;
    float nr=float(in.rows)*resizeFactor;
    cv::Mat imres;
    cv::resize(in,imres,cv::Size(nc,nr));
    cout<<"Imagesize="<<imres.size()<<endl;
    return imres;
}
/************************************
 *
 *
 *
 *
 ************************************/
void setParamsFromGlobalVariables(aruco::MarkerDetector &md){


    md.setDetectionMode((DetectionMode)iDetectMode,float(iMinMarkerSize)/1000.);
    md.getParameters().setCornerRefinementMethod( (aruco::CornerRefinementMethod) iCornerMode);

    md.getParameters().detectEnclosedMarkers(iEnclosed);
    md.getParameters().ThresHold=iThreshold;
    md.getParameters().trackingMinDetections=(iTrack?3:0);
    if ( aruco::Dictionary::getTypeFromString( md.getParameters().dictionary)!=Dictionary::CUSTOM)
            md.setDictionary((aruco::Dictionary::DICT_TYPES) iDictionaryIndex,float(iCorrectionRate)/10. );  // sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)
}

void createMenu(){
    cv::createTrackbar("Dictionary", "menu", &iDictionaryIndex, 13, cvTackBarEvents);
   cv::createTrackbar("DetectMode", "menu", &iDetectMode, 2, cvTackBarEvents);
   cv::createTrackbar("CornerMode", "menu", &iCornerMode, 2, cvTackBarEvents);
   cv::createTrackbar("Track", "menu", &iTrack,1, cvTackBarEvents);

   cv::createTrackbar("MinMarkerSize", "menu", &iMinMarkerSize, 1000, cvTackBarEvents);
   cv::createTrackbar("Threshold", "menu", &iThreshold, 40, cvTackBarEvents);
   cv::createTrackbar("ErrorRate", "menu", &iCorrectionRate, 10, cvTackBarEvents);
   cv::createTrackbar("Enclosed", "menu", &iEnclosed, 1, cvTackBarEvents);
   cv::createTrackbar("ShowAll", "menu", &iShowAllCandidates, 1, cvTackBarEvents);
   iThreshold=MDetector.getParameters().ThresHold;
   iCornerMode= MDetector.getParameters().cornerRefinementM;
}

void putText(cv::Mat &im,string text,cv::Point p,float size){
    float fact=float(im.cols)/float(640);
    if (fact<1) fact=1;

    cv::putText(im,text,p,FONT_HERSHEY_SIMPLEX, size,cv::Scalar(0,0,0),3*fact);
    cv::putText(im,text,p,FONT_HERSHEY_SIMPLEX, size,cv::Scalar(125,255,255),1*fact);

}
void printHelp(cv::Mat &im)
{
    float fs=float(im.cols)/float(1200);

    putText(im,"'m': show/hide menu",cv::Point(10,fs*60),fs*0.5f);
    putText(im,"'s': start/stop video capture",cv::Point(10,fs*80),fs*0.5f);
    putText(im,"'w': write image to file",cv::Point(10,fs*100),fs*0.5f);
    putText(im,"'t': do a speed test",cv::Point(10,fs*120),fs*0.5f);
    putText(im,"'f': saves current configuration to file 'arucoConfig.yml'",cv::Point(10,fs*140),fs*0.5f);
}

void printInfo(cv::Mat &im){
    float fs=float(im.cols)/float(1200);
    putText(im,"fps="+to_string(1./Fps.getAvrg()),cv::Point(10,fs*20),fs*0.5f);
    putText(im,"'h': show/hide help",cv::Point(10,fs*40),fs*0.5f);
    if(bPrintHelp) printHelp(im);
}

void printMenuInfo(){
        cv::Mat image(200,400,CV_8UC3);
        image=cv::Scalar::all(255);
        string str="Dictionary="+aruco::Dictionary::getTypeString((aruco::Dictionary::DICT_TYPES) iDictionaryIndex) ;

        cv::putText(image,str,cv::Size(10,20),FONT_HERSHEY_SIMPLEX, 0.35,cv::Scalar(0,0,0),1);

        str="Detection Mode="+MarkerDetector::Params::toString(MDetector.getParameters().detectMode);
        cv::putText(image,str,cv::Size(10,40),FONT_HERSHEY_SIMPLEX, 0.35,cv::Scalar(0,0,0),1);
        str="Corner Mode="+MarkerDetector::Params::toString(MDetector.getParameters().cornerRefinementM);;
        cv::putText(image,str,cv::Size(10,60),FONT_HERSHEY_SIMPLEX, 0.35,cv::Scalar(0,0,0),1);
        // cv::imshow("menu",image);
}


/************************************
 *
 *
 *
 *
 ************************************/
cv::Size parseSize(const string &strsize  ){
    if(strsize.size()==0)return cv::Size(-1,-1);
    cv::Size s;
    string ssaux=strsize;
    for(auto &c:ssaux){
        if(c==':'){
            c=' ';
        }
    }
    stringstream sstr;sstr<<ssaux;
    if( sstr>>s.width>>s.height)
        return s;
    return cv::Size(-1,-1);

}
int main(int argc, char** argv)
{
    try
    {
        CmdLineParser cml(argc, argv);
        if (argc < 2 || cml["-h"])
        {
            cerr << "Invalid number of arguments" << endl;
            cerr << "Usage: (in.avi|live[:camera_index(e.g 0 or 1)]) [-c camera_params.yml] [-s  marker_size_in_meters] [-d "
                    "dictionary:ALL_DICTS by default] [-h] [-ws w:h] [-skip frames]"
                 << endl;
            cerr << "\tDictionaries: ";
            for (auto dict : aruco::Dictionary::getDicTypes())
                cerr << dict << " ";
            cerr << endl;
            cerr << "\t Instead of these, you can directly indicate the path to a file with your own generated "
                    "dictionary"
                 << endl;
            return false;
        }

        ///////////  PARSE ARGUMENTS
        string TheInputVideo = argv[1];
        // read camera parameters if passed
        if (cml["-c"])
            TheCameraParameters.readFromXMLFile(cml("-c"));

        float TheMarkerSize = std::stof(cml("-s", "-1"));
        //resize factor
        float resizeFactor=stof(cml("-rf","1"));

        iMinMarkerSize=stof(cml("-mms","0.0"));


        ///////////  OPEN VIDEO
        // read from camera or from  file
        if (TheInputVideo.find("live") != string::npos)
        {
            int vIdx = 0;
            // check if the :idx is here
            char cad[100];
            if (TheInputVideo.find(":") != string::npos)
            {
                std::replace(TheInputVideo.begin(), TheInputVideo.end(), ':', ' ');
                sscanf(TheInputVideo.c_str(), "%s %d", cad, &vIdx);
            }
            cout << "Opening camera index " << vIdx << endl;
            TheVideoCapturer.open(vIdx);
            waitTime = 30;
            isVideo=true;
        }
        else{
            TheVideoCapturer.open(TheInputVideo);
            if ( TheVideoCapturer.get(CV_CAP_PROP_FRAME_COUNT)>=2) isVideo=true;
            if(cml["-skip"])
                TheVideoCapturer.set(CV_CAP_PROP_POS_FRAMES,stoi(cml("-skip")));

        }
        // check video is open
        if (!TheVideoCapturer.isOpened())
            throw std::runtime_error("Could not open video");


        // //create windows
        // if(cml["-fs"]){
        //     cv::namedWindow("in", cv::WINDOW_FULLSCREEN);
        //     cv::namedWindow("thres", cv::WINDOW_FULLSCREEN);
        // }
        // else if(cml["-ws"])
        // {
        //     cv::namedWindow("in",cv::WINDOW_NORMAL);
        //     cv::Size s=parseSize(cml("-ws"));
        //     cv::resizeWindow("in",s.width,s.height);
        //     cv::namedWindow("thres",cv::WINDOW_NORMAL);
        //     resizeWindow("thres",s.width,s.height);

        // }

        // else {
        //     cv::namedWindow("in",cv::WINDOW_NORMAL);
        //     cv::resizeWindow("in",640,480);
        //     float w=std::min(int(1920),int(TheInputImage.cols));
        //     float f=w/float(TheInputImage.cols);
        //     resizeWindow("in",w,float(TheInputImage.rows)*f);
        // }

        ///// CONFIGURE DATA
        // read first image to get the dimensions
        TheVideoCapturer >> TheInputImage;
        if (TheCameraParameters.isValid())
            TheCameraParameters.resize(TheInputImage.size());
        dictionaryString=cml("-d", "ALL_DICTS");
        iDictionaryIndex=(uint64_t)aruco::Dictionary::getTypeFromString(dictionaryString);
         MDetector.setDictionary(dictionaryString,float(iCorrectionRate)/10. );  // sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)
         iThreshold=MDetector.getParameters().ThresHold;
         iCornerMode= MDetector.getParameters().cornerRefinementM;


        setParamsFromGlobalVariables(MDetector);


        // go!
        char key = 0;
        int index = 0,indexSave=0;
        // capture until press ESC or until the end of the video

        do
        {

            TheVideoCapturer.retrieve(TheInputImage);
             std::cout<<"Frame:"<<TheVideoCapturer.get(CV_CAP_PROP_POS_FRAMES)<<std::endl;
            TheInputImage=resizeImage(TheInputImage,resizeFactor);
            // copy image
            Fps.start();
            TheMarkers = MDetector.detect(TheInputImage, TheCameraParameters, TheMarkerSize);
            Fps.stop();
            // chekc the speed by calculating the mean speed of all iterations
            cout << "\rTime detection=" << Fps.getAvrg()*1000 << " milliseconds nmarkers=" << TheMarkers.size() <<" images resolution="<<TheInputImage.size() <<std::endl;

            // print marker info and draw the markers in image
            TheInputImage.copyTo(TheInputImageCopy);

            if (iShowAllCandidates){
                auto candidates=MDetector.getCandidates();
                for(auto cand:candidates)
                    Marker(cand,-1).draw(TheInputImageCopy, Scalar(255, 0, 255));
            }

            
            for (unsigned int i = 0; i < TheMarkers.size(); i++)
            {
                /////////////////////////////////////////////////////////////////////////////////
                /////////////////////////////////////////////////////////////////////////////////
                cout << TheMarkers[i].id<<","<<TheMarkers[i].getArea()<<","<<TheMarkers[i].getCenter().x<<","<<TheMarkers[i].getCenter().y<< endl;
                for(unsigned int j=0;j<5;j++)
                {
                    if(TheMarkers[i].id == aruco_search[j]) {
                        if(aruco_searchArea[j] < TheMarkers[i].getArea()) {
                            aruco_searchArea[j]=TheMarkers[i].getArea();
                            string aruco_NUM = std::to_string(j+1);
                            string aruco_ID = std::to_string(TheMarkers[i].id);
                            string imname ="("+aruco_NUM+")"+"-("+aruco_ID+")"+".png";
                            cv::circle(TheInputImage,Point((int)TheMarkers[i].getCenter().x,(int)TheMarkers[i].getCenter().y),8,CV_RGB(0,255,0));
                            cv::imwrite(imname,TheInputImage);
                        }
                    }
                }
            }
            
            refreshAruco = false;
           
            // DONE! Easy, right?
            // show input with augmented information and  the thresholded image
            printInfo(TheInputImageCopy);
            if(showMennu)printMenuInfo();

            // cv::imshow("thres", resize(MDetector.getThresholdedImage(), 1024));

            // cv::imshow("in",  TheInputImageCopy);

            key = cv::waitKey(waitTime);  // wait for key to be pressed
            

            if (isVideo)
                if ( TheVideoCapturer.grab()==false) key=27;
        } while (key != 27 );
    }
    catch (std::exception& ex)

    {
        cout << "Exception :" << ex.what() << endl;
    }
}


void cvTackBarEvents(int pos, void*)
{
    (void)(pos);


    setParamsFromGlobalVariables(MDetector);

    // recompute
        Fps.start();
        TheMarkers=MDetector.detect(TheInputImage);
        Fps.stop();
    // chekc the speed by calculating the mean speed of all iterations
    TheInputImage.copyTo(TheInputImageCopy);
    if (iShowAllCandidates){
        auto candidates=MDetector.getCandidates();
        for(auto cand:candidates)
            Marker(cand,-1).draw(TheInputImageCopy, Scalar(255, 0, 255),1);
    }

    for (unsigned int i = 0; i < TheMarkers.size(); i++){
        cout << TheMarkers[i] << endl;
        TheMarkers[i].draw(TheInputImageCopy, Scalar(0, 0, 255),2);
    }

    // draw a 3d cube in each marker if there is 3d info
    if (TheCameraParameters.isValid())
        for (unsigned int i = 0; i < TheMarkers.size(); i++)
            CvDrawingUtils::draw3dCube(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
    cv::putText(TheInputImageCopy,"fps="+to_string(1./Fps.getAvrg() ),cv::Point(10,20),FONT_HERSHEY_SIMPLEX, 0.5f,cv::Scalar(125,255,255),2);

    // cv::imshow("in",  TheInputImageCopy );
    // cv::imshow("thres", resize(MDetector.getThresholdedImage(), 1024));
    if(showMennu)printMenuInfo();

}
