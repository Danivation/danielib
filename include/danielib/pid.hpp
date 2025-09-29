#pragma once

namespace danielib {
class PID {
    public:
        /**
         * @brief Creates a new PID with specified parameters
         * 
         * @param kP proportional gain parameter
         * @param kI integral gain parameter
         * @param kD derivative gain parameter
         * @param timeout time after which PID will exit regardless of error
         * @param windupRange range in which the integral term has an effect
         * @param exitRange range in which the PID will exit after the specified exitTime
         * @param exitTime time that the error must be within exitRange for the PID to exit
         */
        PID(float kP, float kI, float kD, float timeout = 0, float windupRange = 0, float exitRange = 0, float exitTime = 0);

        /**
         * @brief Updates the PID with the specified target and error parameters
         * 
         * @param target target value for the PID to reach
         * @param current current sensor value
         */
        float update(float target, float current);

        /**
         * @brief Reset the PID's internal calculations
         */
        void reset();
    protected:
        const float kP;
        const float kI;
        const float kD;

        const float timeout;
        const float windupRange;

        const float exitRange;
        const float exitTime;

        float integral = 0;
        float prevError = 0;
};
}