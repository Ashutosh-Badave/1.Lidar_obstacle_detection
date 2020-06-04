//
// Created by ashutosh on 18.03.20.
//

#ifndef PLAYBACK_KDTREE3D_H
#define PLAYBACK_KDTREE3D_H

#include "render/render.h"
#include "processPointClouds.h"

// Structure to represent node of kd tree
struct Node
{
    pcl::PointXYZI point;
    int id;
    Node* left;
    Node* right;
    Node(pcl::PointXYZI arr, int setId)
            :	point(arr), id(setId), left(NULL), right(NULL)
    {}
};

struct KdTree
{
    Node* root;

    KdTree()
            : root(NULL)
    {}

    void insertHelper(Node **node,u_int depth,const pcl::PointXYZI point, int id)
    {
        if(*node==NULL){
            *node = new Node(point,id);
        }
        else{
            std::vector<float> check_point {(*node)->point.x,(*node)->point.y,(*node)->point.z};
            std::vector<float> new_point {point.x,point.y,point.z};
            u_int cd = depth % 3;
            if(new_point[cd] < (check_point[cd]))
                insertHelper(&(*node)->left,depth+1,point,id);
            else
                insertHelper(&(*node)->right,depth+1,point,id);
        }
    }

    void insert(const pcl::PointXYZI point, int id)
    {
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root
        insertHelper(&root,0,point,id);

    }

    void searchHelper(Node* node,pcl::PointXYZI target,u_int depth,float distanceTol,std::vector<int>& ids)
    {
        if (node!=NULL)
        {
            if (node->point.x <= (target.x + distanceTol) &&
                node->point.x >= (target.x - distanceTol) &&
                node->point.y <= (target.y + distanceTol) &&
                node->point.y >= (target.y - distanceTol)  &&
                node->point.z <= (target.z + distanceTol) &&
                node->point.z >= (target.z - distanceTol)   )
            {

                float A = node->point.x - target.x;
                float B = node->point.y - target.y;
                float C = node->point.z - target.z;

                float dist = sqrt(pow(A,2) + pow(B,2) + pow(C,2));

                if (dist <= distanceTol)
                {
                    ids.push_back(node->id);
                    // std::cout << node->id << std::endl;
                }
            }
            std::vector<float> check_point {node->point.x,node->point.y,node->point.z};
            std::vector<float> new_point {target.x,target.y,target.z};
            u_int cd = depth % 3;
            if (check_point[cd] > (new_point[cd] - distanceTol))
                searchHelper(node->left,target, depth + 1, distanceTol, ids);

            if (check_point[cd] < (new_point[cd] + distanceTol))
                searchHelper(node->right,target, depth + 1,distanceTol, ids);

        }
    }
    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(pcl::PointXYZI target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(root,target,0,distanceTol,ids);
        return ids;
    }
};


#endif //PLAYBACK_KDTREE3D_H
