#include "../include/Sampler.h"

#include <algorithm>    
#include <iostream>
#include <random>

Sampler::Sampler() : valueVec(0), min(0), max(0), numBins(10), binElementCount(0), binCenterValue(0) {

}

Sampler::Sampler(int numBins) : valueVec(0), min(0), max(0), numBins(numBins), binElementCount(0), binCenterValue(numBins) {

    if(numBins < 1)
        std::cout<<"number of bins needs to be greater than 0"<<std::endl;
}


Sampler::~Sampler() {}


void
Sampler::addValue(float value) {

    valueVec.push_back(value);
}


void 
Sampler::init() {

    for(int i = 0; i< numBins; ++i)
        binElementCount.push_back(0);

    std::sort(valueVec.begin(), valueVec.end());
    min = valueVec.front();
    max = valueVec.back();
    interval = (max - min) / numBins;

    for(int i=0; i<numBins; ++i) {
        if( i == 0)
            binCenterValue[i] = min + interval/2;
        else
            binCenterValue[i] = binCenterValue[i-1] + interval;
    }

    for(size_t i=0; i<valueVec.size(); ++i) {

        if(i == 0) {
            binElementCount.front()++;
            continue;
        }

        if(i == valueVec.size() - 1) {
            binElementCount.back()++;
            continue;
        }

        for(int j=0; j<numBins; ++j) {

            if(valueVec[i] < min + (j+1)*interval) {
                binElementCount[j]++;
                break;
            }
        }
    }
}


// Utility function to find ceiling of r in arr[l..h]
int 
Sampler::findCeil(std::vector<int> prefix, int r, int l, int h)
{
    int mid;
    while (l < h)
    {
        mid = l + ((h - l) >> 1);  // Same as mid = (l+h)/2
        (r > prefix[mid]) ? (l = mid + 1) : (h = mid);
    }
    return (prefix[l] >= r) ? l : -1;
}


float
Sampler::getSample() {

    // Create and fill prefix array
    std::vector<int> prefix(numBins);
    
    prefix[0] = binElementCount[0];
    for (int i = 1; i < numBins; ++i)
        prefix[i] = prefix[i - 1] + binElementCount[i];

    // prefix[n-1] is sum of all frequencies. Generate a random number
    // with value from 1 to this sum
    
    // old version rand()
    //int r = (rand() % prefix[numBins - 1]) + 1;

    // C++11 feature
    std::random_device rd;
    std::mt19937 engine(rd());
    std::uniform_int_distribution<int> dist(1, prefix[numBins - 1]);
    int r = dist(engine);

    // Find index of ceiling of r in prefix arrat
    int indexc = findCeil(prefix, r, 0, numBins - 1);
    return binCenterValue[indexc];
}

void 
Sampler::printValueVec() {
    for(size_t i = 0; i < valueVec.size(); ++i)
        std::cout<<valueVec[i]<<" ";
    std::cout<<std::endl;
}


void 
Sampler::printelementCount() {
    for(size_t i = 0; i < binElementCount.size(); ++i)
        std::cout<<binElementCount[i]<<" ";
    std::cout<<std::endl;
}
