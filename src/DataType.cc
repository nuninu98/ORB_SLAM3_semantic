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

Object::Object(const string& name, const Eigen::Matrix4f& pose): name_(name), pose_(pose){

}

string Object::getClassName() const{
    return name_;
}
