#include "nav2_costmap_2d/costmap_2d.hpp"

#include "dwa_critics/global_plan_align.hpp"

namespace dwa_critics
{
    int directions[8][2] {
        {1,0},
        {1,1},
        {0,1},
        {-1,1},
        {-1,0},
        {-1,-1},
        {0,-1},
        {1,-1}
    };
 
    GlobalPlanAlignCritic::GlobalPlanAlignCritic()
    : BaseCritic()
    , score_normalizer_(true)
    {
        name_ = "GlobalPlanAlign";
        score_normalizer_.setMinScore(0.0);
    }

    void GlobalPlanAlignCritic::on_initialize()
    {
        nh_->declare_parameter(name_ + ".weight", rclcpp::ParameterValue(1.0));
        weight_ = nh_->get_parameter(name_ + ".weight").as_double();

        size_x_ = costmap_->getSizeInCellsX();
        size_y_ = costmap_->getSizeInCellsY();
        visited_ = std::vector<bool>(size_x_*size_y_, false);
        alignment_map_ = std::vector<unsigned char>(size_x_*size_y_, 0);
    }

    double GlobalPlanAlignCritic::scorePlan(const nav_2d_msgs::msg::Path2D& plan)
    {
        score_normalizer_.setMaxScore(
            static_cast<double>(UCHAR_MAX) * static_cast<double>(plan.poses.size()));
        makeAlignmentMap(plan);

        double score = 0.0;
        for (auto& pose : plan.poses)
        {
            unsigned int x, y;
            if (costmap_->worldToMap(pose.x, pose.y, x, y))
            {
                unsigned int idx = costmap_->getIndex(x, y);
                score += static_cast<double>(alignment_map_[idx]);
            }
        }
        return weight_ * score_normalizer_.getNormalizedScore(score);
    }

    void GlobalPlanAlignCritic::makeAlignmentMap(const nav_2d_msgs::msg::Path2D& plan)
    {
        // in the alignment_map_, all the cells that are on the global plan will have value=0
        // other cells with have value proportional to distance to nearest plan cell
        
        // first push all plan cells into queue
        for (auto& pose : plan.poses)
        {
            unsigned int x, y, idx;
            if (costmap_->worldToMap(pose.x, pose.y, x, y))
            {
                idx = costmap_->getIndex(x, y);
                q_.push(idx);
            }
        }

        // level-order traversal
        unsigned char dist = 0;
        while (!q_.empty())
        {
            for (int i=0; i<q_.size(); ++i)
            {
                unsigned int idx = q_.back();
                q_.pop();

                alignment_map_[idx] = dist;
                for (int j=0; j<8; ++j)
                {
                    unsigned int x, y;
                    costmap_->indexToCells(idx, x, y);
                    unsigned int new_x = x + directions[j][0];
                    unsigned int new_y = y + directions[j][1];
                    idx = costmap_->getIndex(new_x, new_y);
                    if (!visited_[idx] && new_x < size_x_ && new_y < size_y_)
                    {
                        q_.push(idx);
                        visited_[idx] = true;
                    }
                }
            }

            if (dist < UCHAR_MAX)
            {
                ++dist;
            }
        }
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dwa_critics::GlobalPlanAlignCritic, dwa_critics::BaseCritic)