#ifndef CRITIC_SCORE_NORMALIZER_HPP_
#define CRITIC_SCORE_NORMALIZER_HPP_

namespace dwa_critics
{
/**
 * @class CriticScoreNormalizer
 * @brief A simple class to normalize the score from a dwa critic
*/
class CriticScoreNormalizer
{
public:
    CriticScoreNormalizer(bool invert)
    : invert_(invert)
    {}

    void setMaxScore(double max_score)
    {
        max_score_ = max_score;
    }

    void setMinScore(double min_score)
    {
        min_score_ = min_score;
    }

    double getNormalizedScore(double score)
    {
        double normalized_score = (score - min_score_) / (max_score_ - min_score_);
        if (invert_) 
        {
            return 1.0 - normalized_score;
        }
        return normalized_score;
    }

private:
    double max_score_;
    double min_score_;
    bool invert_;
};
}

#endif