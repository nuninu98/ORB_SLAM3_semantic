#ifndef __ORB_SLAM_SEMANTIC_DATA_TYPES_HEADER__
#define __ORB_SLAM_SEMANTIC_DATA_TYPES_HEADER__
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <Eigen/StdVector>
using namespace std;
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4f)
class OCRDetection{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        OCRDetection();

        OCRDetection(const OCRDetection& ocr);

        OCRDetection(const cv::Rect& roi, const string& content);

        string getContent() const;

        cv::Rect getRoI() const;

        OCRDetection& operator=(const OCRDetection& ocr);

    private:
        string content_;

        cv::Rect roi_;
};

class Object{
    
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Object();

        Object(const Object& obj);
        
        Object(const string& name, const pcl::PointCloud<pcl::PointXYZRGB>& cloud);

        string getClassName() const;

        bool getEstBbox(const Eigen::Matrix3f& K, const Eigen::Matrix4f& cam_in_map, cv::Rect& output) const;

        void setCloud(const pcl::PointCloud<pcl::PointXYZRGB>& input);

        void getCloud(pcl::PointCloud<pcl::PointXYZRGB>& output) const;
    
    private:
        string name_;
        pcl::PointCloud<pcl::PointXYZRGB> cloud_;  // future work : remove this and replace as keyframe pointers
};

class Detection{
    
    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Detection();

        Detection(const cv::Rect& roi, const cv::Mat& mask, const string& name, string content = "");

        ~Detection();   

        cv::Rect getRoI() const;

        cv::Mat getMask() const;

        string getClassName() const;

        string getContent() const; // only for room number

        void copyContent(const OCRDetection& ocr_output);

        void setCorrespondence(Object* obj);


        Object* getObject() const;

        void generateCloud(const cv::Mat& color_mat, const cv::Mat& depth_mat, const Eigen::Matrix3f& K);

        void getCloud(pcl::PointCloud<pcl::PointXYZRGB>& output) const;
    private:
        cv::Rect roi_;
        cv::Mat mask_;
        string name_;
        string content_;
        Object* object_;
        pcl::PointCloud<pcl::PointXYZRGB> cloud_;

        
};

class DetectionGroup{
    
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW      
    private:
        double stamp_;
        cv::Mat color_img;
        cv::Mat depth_img;
        Eigen::Matrix4f sensor_pose_;
        Eigen::Matrix3f K_;
        vector<Detection> detections_;
    public:
        DetectionGroup();

        DetectionGroup(const DetectionGroup& dg);

        DetectionGroup(const cv::Mat& color, const cv::Mat& depth, const Eigen::Matrix4f& sensor_pose,
        const vector<Detection>& detections, const Eigen::Matrix3f& K, double stamp);

        ~DetectionGroup();

        double stamp() const;

        void detections(vector<Detection>& output) const;

        Eigen::Matrix4f getSensorPose() const;

        Eigen::Matrix3f getIntrinsic() const;
};







#endif