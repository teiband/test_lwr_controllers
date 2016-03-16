#ifndef MOTION_GENERATION_H_
#define MOTION_GENERATION_H_

using namespace std;

class MotionGenerator
{
private:
    float time_;

public:

    MotionGenerator() : time_(0.0)
    {}

    void swing(float min, float max, double& act, float& inc)
    {     
        if (act > max || act < min)
            inc = -inc;

        act += inc;
    }

    double sineWave(float amp, float init_pos, float freq, float loop_rate)
    {
        double act = init_pos + amp * sin(2.0*M_PI* freq * time_);
        time_ += 1/(float)loop_rate;

        return act;
    }

    void reset() { time_ = 0.0; }

    virtual ~MotionGenerator() {}

};

#endif
