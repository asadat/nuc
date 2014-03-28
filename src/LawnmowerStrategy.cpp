#include "LawnmowerStrategy.h"

LawnmowerStrategy::LawnmowerStrategy(CNode *root)
{
    nodeStack.push_back(root);
}

LawnmowerStrategy::~LawnmowerStrategy()
{
    nodeStack.clear();
}

CNode* LawnmowerStrategy::GetNextNode()
{

}
