#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "histogram.h"
#include<algorithm>

Histogram::Histogram(float min, float max, float binwidth){
    int binCount = (int)((max - min) / binwidth)+1;
    mBinwidth = binwidth;
    mBuckets.resize(binCount);
    mMin=min;
}

void Histogram::record(float datum){
    if(datum>=10){
        mBuckets[mBuckets.size()-1]+=1;
    }
    else
    {
        int binindex = (int)((datum - mMin) / mBinwidth);
        mBuckets[binindex]+=1;
    }

}

int Histogram::bins() const{
    return mBuckets.size();
}

int Histogram::count(int bin) const{
    return mBuckets[bin];
}

int Histogram::getMaxIndex() const{
    auto iter = std::max_element(mBuckets.begin(), mBuckets.end());//type of iter: std::vector<int>::iterator
    int index = std::distance(mBuckets.begin(), iter);
    return index;
}

Histogram::~Histogram()          // デストラクタ
{
    mBuckets.clear();
    std::vector<int>(mBuckets).swap(mBuckets);
}