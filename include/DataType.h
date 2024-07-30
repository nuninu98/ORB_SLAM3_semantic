#ifndef __ORB_SLAM_SEMANTIC_DATA_TYPES_HEADER__
#define __ORB_SLAM_SEMANTIC_DATA_TYPES_HEADER__
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
class Detection{
    protected:
        cv::Rect roi_;
        cv::Mat mask_;
        size_t id_;
    public: 
        Detection();

        Detection(const cv::Rect& roi, const cv::Mat& mask, const size_t& id);

        ~Detection();   

        cv::Rect getRoI() const;

        cv::Mat getMask() const;

        size_t getClassID() const;
};


class OCRDetection: public Detection{
    public:
        OCRDetection();

        OCRDetection(const cv::Rect& roi, const size_t& id);
};

class Object{
    protected:
        size_t id_;
    public:
        Object();

        Object(const size_t& id);

        size_t getClassID() const;
};

class Door: public Object{
    private:
        size_t room_number_;
        Eigen::Matrix4d pose_;
    public:
        Door();

        Door(size_t room_number, const Eigen::Matrix4d& pose);

        size_t getNumber() const;
};

#endif