#ifndef __LOOP_QUERY_H__
#define __LOOP_QUERY_H__

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
namespace ORB_SLAM3{
    struct LoopQuery{
        size_t id_query;
        size_t id_target;
        Eigen::Matrix4f drift;

        LoopQuery(size_t qid, size_t tid, const Eigen::Matrix4f& drift);

        LoopQuery(const LoopQuery& lq);
        
        ~LoopQuery();
    };
}



#endif