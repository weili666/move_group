#ifndef _BACKHANDTREE_H
#define _BACKHANDTREE_H
#include <fstream>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "PositionAttitude.h"
#include "BackTreeNode.h"
#include "StartTreeNode.h"

#define M_pi 3.141592653
using namespace std;
class BackHandTree
{
public:
    BackHandTree(){root=new BackTreeNode;}
    BackHandTree(PositionAttitude &t):target(t)
    {
        root=new BackTreeNode;
        root->setData(t);
        root->setLevel(1);
        root->ptrs_of_backtree.push_back(root);
    }
    ~BackHandTree(){}
    bool IsEmpty() const
    {
        return((root)?false:true);
    }
    void AddNode(const PositionAttitude& element,BackTreeNode *f);
    BackTreeNode* getRoot();
    void setRoot(const PositionAttitude&);
    void CreateHandTree();

   private:
    BackTreeNode* root;
    PositionAttitude target;

};
#endif
