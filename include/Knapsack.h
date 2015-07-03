#ifndef _KNAPSACK_
#define _KNAPSACK_

#include <set>
#include <vector>
#include <bitset>
#include <iostream>
#include "ros/ros.h"

#define SET_MAX_SIZE    256

using namespace std;

template<class ITEM>
class Knapsack
{
    public:
        Knapsack();
       ~Knapsack();

        void AddItem(ITEM* item, const double value, const double weight);
        double Solve(const double weight, set<ITEM*> & solution);

    private:

        double Solve_recursive(const int idx, const double weight, bitset<SET_MAX_SIZE> &bs);

        typedef struct
        {
            ITEM* item;
            double value;
            double weight;
        } Tuple;

       vector<Tuple> items;
};

template <class ITEM>
Knapsack<ITEM>::Knapsack()
{

}

template <class ITEM>
Knapsack<ITEM>::~Knapsack()
{

}

template <class ITEM>
void Knapsack<ITEM>::AddItem(ITEM *item, const double value, const double weight)
{
    Tuple tp;
    tp.item = item;
    tp.value = value;
    tp.weight = weight;

    items.push_back(tp);
}

template <class ITEM>
double Knapsack<ITEM>::Solve(const double weight, set<ITEM *> &solution)
{
    for(size_t i = 0; i < items.size(); i++)
    {
        ROS_INFO("Item #%d value: %f  weight: %f", (int)i, items[i].value, items[i].weight);
    }

    bitset<SET_MAX_SIZE> bs;
    bs.reset();

    double v = Solve_recursive(items.size(), weight, bs);

    solution.clear();

    for(size_t i = 0; i < bs.size(); i++)
    {
        if(bs.test(i))
        {
            solution.insert(items[i].item);
        }
    }

    return v;
}

template <class ITEM>
double Knapsack<ITEM>::Solve_recursive(const int idx, const double weight, bitset<SET_MAX_SIZE> &bs)
{
    if(idx <= 0 || weight <=0)
        return 0;

    bitset<SET_MAX_SIZE> bs0, bs1;
    bs0 = bs;
    bs1 = bs;

    double v0 = Solve_recursive(idx-1, weight, bs0);
    double v1 = 0;
    if(weight-items[idx-1].weight >=0)
        v1 = Solve_recursive(idx-1, weight-items[idx-1].weight, bs1)+items[idx-1].value;

    if(v1 > v0)
    {
        bs = bs1;
        bs.set(idx-1, true);
        //cout << "selected item: " << idx-1 << endl;
        return v1;
    }
    else
    {
        bs = bs0;
        bs.set(idx-1, false);
        //cout << "Not selected item: " << idx-1 << endl;
        return v0;
    }
}

#endif
