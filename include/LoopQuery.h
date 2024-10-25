#ifndef __LOOP_QUERY_H__
#define __LOOP_QUERY_H__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
using namespace std;
namespace ORB_SLAM3{
    namespace LOOP_TYPE{
        const int BAG_OF_WORDS = 1;
        const int SEMANTIC = 2;
    }
    
    struct LoopQuery{
        int type;
        size_t id_query;
        size_t id_target;
        Eigen::Matrix4f drift;

        std::vector<pair<float, size_t>> candidates;

        LoopQuery(int type, size_t qid, size_t tid, const Eigen::Matrix4f& drift);

        LoopQuery(const LoopQuery& lq);
        
        ~LoopQuery();
    };
}



#endif