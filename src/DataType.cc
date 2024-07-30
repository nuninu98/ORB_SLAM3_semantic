#include "DataType.h"
Detection::Detection(){

}

Detection::Detection(const cv::Rect& roi, const cv::Mat& mask, const string& name): roi_(roi), mask_(mask), name_(name){

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

OCRDetection::OCRDetection(){
    
}

OCRDetection::OCRDetection(const cv::Rect& roi, const string& content){
    roi_ = roi;
    name_ = content;
}

Object::Object(){

}

Object::Object(const string& name, int room_number): name_(name), room_number_(room_number){

}

int Object::getRoomNumber() const{
    return room_number_;
}

string Object::getClassName() const{
    return name_;
}
