#include <vector>

class Histogram{
    public:
        //constructor
        Histogram(float min, float max, float binWidth);
        void record(float datum);
        int bins() const; //runqiu: get the number of bins
        int count(int bin) const; //runqiu: return the number in a bin
        int getMaxIndex() const;
        ~Histogram();        // デストラクタ

    private:
        float mMin;
        float mBinwidth;
        std::vector<int> mBuckets;

};