#pragma once

#include <vector>
#include <cstdint>
#include <cmath>

using namespace std;

class AveragingFilter
{
private:
    // an array of the last FILTER_SIZE data points; this will be used as a circular buffer
    // with the index being updated for each new data point, until it rolls over
    vector<double> data;

    uint8_t filterIndex = 0;

    const int dataSize;
    const double dataInit;

public:
    bool isFilledOnce = false;

    explicit AveragingFilter(int size, double value=0.f) : dataSize(size), dataInit(value) {data.resize(dataSize); fillWith(dataInit);}

    void fillWith(double value);

    void addDatum(double value);
    double addAndReturnAverage(double value);
    double getAverage();
    double getAbsAverage();
    double getStdDev();
    double addAndReturnMedian(double value); // TODO: implement median filter
    void reset() {fillWith(dataInit); filterIndex = 0; isFilledOnce = false; }
};