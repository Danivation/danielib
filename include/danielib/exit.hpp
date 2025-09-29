#pragma once

namespace danielib {
class ExitCondition {
    public:
        ExitCondition(const float exitRange, const float exitTime);
        bool isDone();
        /**
         * @brief Update the exit condition with the specified input
         * 
         * @param input current state of the motion to check vs the exit range
         */
        bool update(const float input);
        /**
         * @brief Reset the exit timer
         */
        void reset();
    protected:
        const float exitRange;
        const float exitTime;
        int startTime = -1;
        bool done = false;
};
} // namespace danielib