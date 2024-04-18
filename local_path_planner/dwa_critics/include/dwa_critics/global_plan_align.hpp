#ifndef GLOBAL_PLAN_ALIGN_HPP_
#define GLOBAL_PLAN_ALIGN_HPP_

#include <vector>
#include <queue>

#include "dwa_critics/base_critic.hpp"
#include "dwa_critics/critic_score_normalizer.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"

namespace dwa_critics
{
/**
 * @class GlobalPlanAlignCritic
 * @brief A critic that gives lower points to local plans that align with the global plan
*/
class GlobalPlanAlignCritic : public BaseCritic 
{
public:
    GlobalPlanAlignCritic();
    /**
     * @param plan Global plan in map frame
    */
    double scorePlan(const nav_2d_msgs::msg::Path2D& plan) override;
    virtual ~GlobalPlanAlignCritic() = default;

private:
    void on_initialize() override;
    void makeAlignmentMap(const nav_2d_msgs::msg::Path2D& plan);
    std::vector<unsigned char> alignment_map_;
    std::queue<unsigned int> q_;
    std::vector<bool> visited_;
    unsigned int size_x_;
    unsigned int size_y_;
    CriticScoreNormalizer score_normalizer_;
};
}

#endif