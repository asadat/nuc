#include <stdio.h>
#include <vector>
#include <math.h>

using namespace std;

double mean(vector<float> &v)
{
    if(v.empty())
        return 0;

    float sum=0;
    for(unsigned int i=0; i<v.size(); i++)
    {
        sum += v[i];
    }

    return sum/v.size();
}

float var(vector<float> &v)
{
    if(v.size() <= 1)
        return 0;

    float m=mean(v);
    float difsum;
    for(unsigned int i=0; i<v.size(); i++)
    {
        difsum = (m-v[i])*(m-v[i]);
    }

    return sqrt(difsum/(v.size()-1));
}

int main(int argc, char* argv[])
{
    vector<float> list;
    FILE* f = fopen(argv[1],"r");

    float x;

    while(fscanf(f,"%f", &x) != EOF)
    {
        list.push_back(x);
        //printf("%f \n", x);
    }

    fclose(f);

    printf("%f %f\n", mean(list), var(list));

    return 0;
}
