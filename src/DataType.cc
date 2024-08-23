#include "DataType.h"
namespace ORB_SLAM3{
    Detection::Detection(){

    }

    Detection::Detection(const cv::Rect& roi, const cv::Mat& mask, const string& name, string content): roi_(roi), mask_(mask), name_(name), content_(content), dg_(nullptr){

    }

    Detection::~Detection(){

    }

    cv::Rect Detection::getRoI() const{
        return roi_;
    }

    cv::Mat Detection::getMask() const{
        return mask_;
    }

    string Detection::getClassName() const{
        return name_;
    }

    string Detection::getContent() const{
        return content_;
    }

    void Detection::copyContent(const OCRDetection& ocr_output){
        content_ = ocr_output.getContent();
    }

    void Detection::generateCloud(const cv::Mat& color_mat, const cv::Mat& depth_mat, const Eigen::Matrix3f& K){
        if(!cloud_.empty()){
            return;
        }
        vector<pcl::PointXYZRGB> raw_cloud;
        for(int r = 0; r < color_mat.rows; ++r){
            for(int c = 0; c < color_mat.cols; ++c){
                float depth = depth_mat.at<float>(r, c);
                if(isnanf(depth) || depth < 1.0e-4){
                    continue;
                }
                if(!roi_.contains(cv::Point2i(c, r))){
                    continue;
                }
                Eigen::Vector3f pix(c, r, 1.0);
                float x = (c - K(0, 2)) * depth / K(0, 0);
                float y = (r - K(1, 2)) * depth / K(1, 1);
                pcl::PointXYZRGB pt;
                pt.x = x;
                pt.y = y;
                pt.z = depth;
                pt.r = color_mat.at<cv::Vec3b>(r, c)[2];
                pt.g = color_mat.at<cv::Vec3b>(r, c)[1];
                pt.b = color_mat.at<cv::Vec3b>(r, c)[0];
                raw_cloud.push_back(pt);
            }
        }
        sort(raw_cloud.begin(), raw_cloud.end(), [](const pcl::PointXYZRGB& pt1, const pcl::PointXYZRGB& pt2){
            return pt1.z < pt2.z;
        });
        int left = 0;//(float)raw_cloud.size() * 0.1;
        int right = raw_cloud.size()-1;//(float)raw_cloud.size() * 0.9;
        for(int i = left; i <= min((int)raw_cloud.size()-1, right); ++i){
            cloud_.push_back(raw_cloud[i]);
        }
    }

    void Detection::getCloud(pcl::PointCloud<pcl::PointXYZRGB>& output) const{
        output.clear();
        output = cloud_;
    }

    void Detection::setDetectionGroup(DetectionGroup* dg){
        dg_ = dg;
    }

    const DetectionGroup* Detection::getDetectionGroup() const{
        return dg_;
    }


    //=====================OCR DETECTION======================
    OCRDetection::OCRDetection(){
        
    }

    OCRDetection::OCRDetection(const OCRDetection& ocr){
        content_ = ocr.content_;
        roi_ = ocr.roi_;
    }

    OCRDetection::OCRDetection(const cv::Rect& roi, const string& content): roi_(roi){
        
        content_ = content;
    }


    string OCRDetection::getContent() const{
        return content_;
    }

    cv::Rect OCRDetection::getRoI() const{
        return roi_;
    }

    OCRDetection& OCRDetection::operator=(const OCRDetection& ocr){
        content_ = ocr.content_;
        roi_ = ocr.roi_;
        return *this;
    }
    //======================OBJECT==============================

    Object::Object(){

    }

    Object::Object(const string& name, const pcl::PointCloud<pcl::PointXYZRGB>& cloud): name_(name), cloud_(cloud){

    }

    Object::Object(const Object& obj): name_(obj.name_), cloud_(obj.cloud_){
    }

    string Object::getClassName() const{
        return name_;
    }

    bool Object::getEstBbox(const Eigen::Matrix3f& K, const Eigen::Matrix4f& cam_in_map, cv::Rect& output) const{
        Eigen::Matrix4f ext = cam_in_map.inverse();
        float radii = 0.0;
        Eigen::Vector3f center = Eigen::Vector3f::Zero();
        for(const auto& pt : cloud_){
            Eigen::Vector4f pt_v(pt.x, pt.y, pt.z, 1.0);
            Eigen::Vector4f pt_cam = ext * pt_v;
            center += pt_cam.block<3, 1>(0, 0);
        }
        center = center / (float)cloud_.size();
        if(center(2) < 0.0){
            return false;
        }

        Eigen::MatrixXf P = K* Eigen::MatrixXf::Identity(3, 4);
        float xmin = 10000.0;
        float ymin = 10000.0;
        float xmax = 0.0;
        float ymax = 0.0;
        for(const auto& pt : cloud_){
            Eigen::Vector4f pt_v(pt.x, pt.y, pt.z, 1.0);
            Eigen::Vector4f pt_cam = ext * pt_v;
            if(pt_cam(2) < 0.0){
                continue;
            }
            Eigen::Vector3f pix = P * pt_cam;
            pix /= pix(2);
            xmin = min(pix(0), xmin);
            ymin = min(pix(1), ymin);
            xmax = max(pix(0), xmax);
            ymax = max(pix(1), ymax);
        }
        if(xmin >= xmax || ymin >= ymax){
            return false;
        }
        output = cv::Rect(cv::Point(xmin, ymin), cv::Point(xmax, ymax));
        return true;
    }

    void Object::setCloud(const pcl::PointCloud<pcl::PointXYZRGB>& input){
        cloud_ = input;
    }
            
    void Object::getCloud(pcl::PointCloud<pcl::PointXYZRGB>& output) const{
        output.clear();
        //output = cloud_;
        for(const auto& det_seen : seens_){
            pcl::PointCloud<pcl::PointXYZRGB> det_cloud, cloud_tf;
            det_seen->getCloud(det_cloud);
            Eigen::Matrix4f base_in_map = det_seen->getDetectionGroup()->getKeyFrame()->GetPoseInverse().matrix();
            Eigen::Matrix4f cam_in_base = det_seen->getDetectionGroup()->getSensorPose();
            Eigen::Matrix4f cam_in_map = base_in_map * cam_in_base;
            pcl::transformPointCloud(det_cloud, cloud_tf, cam_in_map);
            output += cloud_tf;
        }
    }

    void Object::addDetection(const Detection* det){
        seens_.push_back(det);
    }
    //=====================DetectionGroup================
    DetectionGroup::DetectionGroup(){}

    DetectionGroup::DetectionGroup(const DetectionGroup& dg) : color_img(dg.color_img), depth_img(dg.depth_img), 
    sensor_pose_(dg.sensor_pose_), detections_(dg.detections_), K_(dg.K_), stamp_(dg.stamp_), kf_(dg.kf_){
        for(auto& elem : detections_){
            elem.setDetectionGroup(this);
        }
    }

    DetectionGroup::DetectionGroup(const cv::Mat& color, const cv::Mat& depth, const Eigen::Matrix4f& sensor_pose,
    const vector<Detection>& detections, const Eigen::Matrix3f& K, double stamp): color_img(color), depth_img(depth),
    sensor_pose_(sensor_pose), stamp_(stamp), K_(K), detections_(detections), kf_(nullptr)
    {
    // detection sibal
        for(auto& elem : detections_){
            elem.generateCloud(color, depth, K);
            elem.setDetectionGroup(this);
        }
    }

    DetectionGroup::~DetectionGroup(){ //fix this!!!!

    }

    double DetectionGroup::stamp() const{
        return stamp_;
    }

    void DetectionGroup::detections(vector<const Detection*>& output) const{
        output.clear();
        for(int i = 0; i < detections_.size(); ++i){
            output.push_back(&detections_[i]);
        }
    }

    Eigen::Matrix4f DetectionGroup::getSensorPose() const{
        return sensor_pose_;
    }

    Eigen::Matrix3f DetectionGroup::getIntrinsic() const{
        return K_;
    }

    void DetectionGroup::setKeyFrame(KeyFrame* kf){
        kf_ = kf;    
    }

    KeyFrame* DetectionGroup::getKeyFrame() const{
        return kf_;
    }
}
