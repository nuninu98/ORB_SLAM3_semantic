#include "DataType.h"
Detection::Detection(){

}

Detection::Detection(const cv::Rect& roi, const cv::Mat& mask, const size_t& id): roi_(roi), mask_(mask), id_(id){

}

Detection::~Detection(){

}

cv::Rect Detection::getRoI() const{
    return roi_;
}

cv::Mat Detection::getMask() const{
    return mask_;
}

size_t Detection::getClassID() const{
    return id_;
}

OCRDetection::OCRDetection(){
    
}

OCRDetection::OCRDetection(const cv::Rect& roi, const size_t& id){
    roi_ = roi;
    id_ = id;
}

Object::Object(){

}

Object::Object(const size_t& id): id_(id){

}

size_t Object::getClassID() const{
    return id_;
}


Door::Door(){
    id_ = 5000;
}

Door::Door(size_t room_number, const Eigen::Matrix4d& pose): room_number_(room_number), pose_(pose){
    id_ = 5000;
}

size_t Door::getNumber() const{
    return room_number_;
}