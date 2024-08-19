#include "DataType.h"
Detection::Detection(): object_(nullptr){

}

Detection::Detection(const cv::Rect& roi, const cv::Mat& mask, const string& name, string content): roi_(roi), mask_(mask), name_(name), content_(content), object_(nullptr){

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

void Detection::setCorrespondence(Object* obj){
    object_ = obj;
}


Object* Detection::getObject() const{
    return object_;
}

// Eigen::Matrix4f Detection::getSensorPose() const{
//     return sensor_pose_;
// }


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

Object::Object(const string& name): name_(name){

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
//=====================DetectionGroup================
DetectionGroup::DetectionGroup(){}

DetectionGroup::DetectionGroup(const DetectionGroup& dg) : color_img(dg.color_img), depth_img(dg.depth_img), 
sensor_pose_(dg.sensor_pose_), detections_(dg.detections_), K_(dg.K_), stamp_(dg.stamp_){

}

DetectionGroup::DetectionGroup(const cv::Mat& color, const cv::Mat& depth, const Eigen::Matrix4f& sensor_pose,
const vector<Detection>& detections, const Eigen::Matrix3f& K, double stamp): color_img(color), depth_img(depth),
 sensor_pose_(sensor_pose), stamp_(stamp), K_(K), detections_(detections)
{
   // detection sibal
    
}

DetectionGroup::~DetectionGroup(){ //fix this!!!!

}

double DetectionGroup::stamp() const{
    return stamp_;
}

void DetectionGroup::detections(vector<Detection>& output) const{
    output.clear();
    output = detections_;
}

Eigen::Matrix4f DetectionGroup::getSensorPose() const{
    return sensor_pose_;
}