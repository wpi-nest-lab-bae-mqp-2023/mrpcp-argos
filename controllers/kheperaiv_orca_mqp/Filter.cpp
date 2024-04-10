#include "Filter.h"

void AveragingFilter::addDatum(double value)
{
    //add to the circular buffer
    data[filterIndex++] = value;
    if(filterIndex == dataSize) filterIndex = 0;
    if (filterIndex == 0) {
        isFilledOnce = true;
    }
}

double AveragingFilter::addAndReturnAverage(double value)
{
    addDatum(value);
    return getAverage();
}

double AveragingFilter::getAverage()
{
    double  average = 0;
    for(int i = 0; i < dataSize; i++)
    {
        average += data[i];
    }

    return average / dataSize;
}

double AveragingFilter::getAbsAverage()
{
    double  average = 0;
    for(int i = 0; i < dataSize; i++)
    {
        average += fabs(data[i]);
    }

    return average / dataSize;
}

double AveragingFilter::getStdDev()
{
    double average = getAverage();
    double stdDev = 0;
    for(int i = 0; i < dataSize; i++)
    {
        stdDev += pow(data[i] - average, 2);
    }

    return sqrt(stdDev / dataSize);
}

double AveragingFilter::addAndReturnMedian(double value)
{
    addDatum(value);

    //we're going to sort the last dataSize data points, but note
    //that we don't sort the original array, as that would mess up our time series  
    //this is a brute force method -- there are better ways 

    //first copy data into a temporary array 
    double tempArray[dataSize];
    for(int i = 0; i < dataSize; i++)
    {
        tempArray[i] = data[i];
    }

    //now brute force sort the temporary array
    for(int i = 0; i < dataSize; i++)
    {
        for(int j = i + 1; j < dataSize; j++)
        {
            if(tempArray[j] < tempArray[i]) std::swap(tempArray[i], tempArray[j]);
        }
    }

    return tempArray[dataSize / 2];
}

void AveragingFilter::fillWith(double value)
{
    for (int i = 0; i < dataSize; ++i) {
        data[i] = value;
    }
}
