#ifndef CRAAL_SAMPLER_H_
#define CRAAL_SAMPLER_H_

#include<vector>

class Sampler {
public:

    Sampler();
    Sampler(int numBins);

    virtual ~Sampler();

    void init();

    void addValue(float value);

    float getSample();
    int findCeil(std::vector<int> prefix, int r, int l, int h);

    inline float getMin() { return min;}
    inline float getMax() { return max;}
    inline float getInterval() { return interval;}

    void printValueVec();

    void printelementCount();

private:
    std::vector<float> valueVec;
    std::vector<int> binElementCount;
    std::vector<float> binCenterValue;
    float min;
    float max;
    float interval;
    int numBins;
};

#endif /* CRAAL_SAMPLER_H_ */
