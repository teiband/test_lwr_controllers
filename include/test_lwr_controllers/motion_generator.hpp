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

    void sineWave(float amp, float init_pos, double& act, float freq)
    {
        const float f = 0.05; //Hz
        act = init_pos + amp * sin(2.0*M_PI* f * time_);
        time_ += 1/(float)freq;
    }

    void reset() { time_ = 0.0; }

    virtual ~MotionGenerator() {}

};

#endif
