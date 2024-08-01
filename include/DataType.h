#ifndef __ORB_SLAM_SEMANTIC_DATA_TYPES_HEADER__
#define __ORB_SLAM_SEMANTIC_DATA_TYPES_HEADER__
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
using namespace std;

class OCRDetection{
    private:
        string content_;

        cv::Rect roi_;
    public:
        OCRDetection();

        OCRDetection(const OCRDetection& ocr);

        OCRDetection(const cv::Rect& roi, const string& content);

        string getContent() const;

        cv::Rect getRoI() const;

        OCRDetection& operator=(const OCRDetection& ocr);
};

class Detection{
    protected:
        cv::Rect roi_;
        cv::Mat mask_;
        string name_;
        string content_;
    public: 
        Detection();

        Detection(const cv::Rect& roi, const cv::Mat& mask, const string& name, string content = "");

        ~Detection();   

        cv::Rect getRoI() const;

        cv::Mat getMask() const;

        string getClassName() const;

        string getContent() const; // only for room number

        void copyContent(const OCRDetection& ocr_output);
};




class Object{
    protected:
        string name_;

        int room_number_;
    public:
        Object();

        Object(const string& name, int room_number = -1);

        string getClassName() const;

        int getRoomNumber() const;
};



#endif