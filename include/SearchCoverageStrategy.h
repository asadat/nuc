#ifndef _SEARCHCOVERAGE_STRATEGY_H
#define _SEARCHCOVERAGE_STRATEGY_H
#include "TraversalStrategy.h"
#include "CNode.h"


class SearchCoverageStrategy: public TraversalStrategy
{
public:
    SearchCoverageStrategy(CNode* root);
    ~SearchCoverageStrategy();

    void glDraw();
    CNode* GetNextNode();
    void hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey);

private:
    CNode * GetNode(int i, int j);
    void SetupGrid(CNode* root);
    void GenerateLawnmower();

    std::vector<CNode*> nodeStack;
    std::vector<CNode*> grid;

    CNode* tree;
    int s;
};

#endif
