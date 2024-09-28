#include "LoopQuery.h"
namespace ORB_SLAM3{
    LoopQuery::LoopQuery(size_t qid, size_t tid, const Eigen::Matrix4f& drift): id_query(qid), id_target(tid), drift(drift){

    }

    LoopQuery::~LoopQuery(){

    }

    LoopQuery::LoopQuery(const LoopQuery& lq): id_query(lq.id_query), id_target(lq.id_target), drift(lq.drift){

    }
}
