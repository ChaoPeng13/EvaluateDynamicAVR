#ifndef _AVL_TREE_HPP_
#define _AVL_TREE_HPP_

/*
  Revised from http://www.cnblogs.com/skywang12345/p/3577360.html
*/

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "DigraphSearchItem.h"

using namespace std;

class AVLTreeNode{
    public:
        DigraphSearchItem key;                // left key         
        int height;         // height
        AVLTreeNode *left;    // left child
        AVLTreeNode *right;    // right child

        AVLTreeNode(DigraphSearchItem value, AVLTreeNode *l, AVLTreeNode *r):
            key(value), height(0),left(l),right(r) {}
};

class AVLTree {
    private:
        AVLTreeNode *mRoot;    // 根结点

    public:
        AVLTree();
        ~AVLTree();

        // 获取树的高度
        int height();

        // 前序遍历"AVL树"
        void preOrder();
        // 中序遍历"AVL树"
        void inOrder();
        // 后序遍历"AVL树"
        void postOrder();

        // (递归实现)查找"AVL树"中键值为key的节点
        AVLTreeNode* search(DigraphSearchItem key);
		// recursively find all the nodes
		void search(vector<AVLTreeNode*>& vec, DigraphSearchItem key);
		// check whether we need to search this item and update
		bool check(vector<DigraphSearchItem>& vec, DigraphSearchItem key);

        // (非递归实现)查找"AVL树"中键值为key的节点
        AVLTreeNode* iterativeSearch(DigraphSearchItem key);

        // 查找最小结点：返回最小结点的键值。
        DigraphSearchItem minimum();
        // 查找最大结点：返回最大结点的键值。
        DigraphSearchItem maximum();

        // 将结点(key为节点键值)插入到AVL树中
        void insert(DigraphSearchItem key);

        // 删除结点(key为节点键值)
        void remove(DigraphSearchItem key);

        // 销毁AVL树
        void destroy();

        // 打印AVL树
        void print();

		// Dot
		void write_graphviz(ostream& out);
    private:
        // 获取树的高度
        int height(AVLTreeNode* tree) ;

        // 前序遍历"AVL树"
        void preOrder(AVLTreeNode* tree) const;
        // 中序遍历"AVL树"
        void inOrder(AVLTreeNode* tree) const;
        // 后序遍历"AVL树"
        void postOrder(AVLTreeNode* tree) const;

        // (递归实现)查找"AVL树x"中键值为key的节点
        AVLTreeNode* search(AVLTreeNode* x, DigraphSearchItem key) const;
		// recursively find all the nodes
		void search(vector<AVLTreeNode*>& vec, AVLTreeNode* x, DigraphSearchItem key) const;

		bool check(vector<DigraphSearchItem>& vec, AVLTreeNode* x, DigraphSearchItem key);

        // (非递归实现)查找"AVL树x"中键值为key的节点
        AVLTreeNode* iterativeSearch(AVLTreeNode* x, DigraphSearchItem key) const;

        // 查找最小结点：返回tree为根结点的AVL树的最小结点。
        AVLTreeNode* minimum(AVLTreeNode* tree);
        // 查找最大结点：返回tree为根结点的AVL树的最大结点。
        AVLTreeNode* maximum(AVLTreeNode* tree);

        // LL：左左对应的情况(左单旋转)。
        AVLTreeNode* leftLeftRotation(AVLTreeNode* k2);

        // RR：右右对应的情况(右单旋转)。
        AVLTreeNode* rightRightRotation(AVLTreeNode* k1);

        // LR：左右对应的情况(左双旋转)。
        AVLTreeNode* leftRightRotation(AVLTreeNode* k3);

        // RL：右左对应的情况(右双旋转)。
        AVLTreeNode* rightLeftRotation(AVLTreeNode* k1);

        // 将结点(z)插入到AVL树(tree)中
        AVLTreeNode* insert(AVLTreeNode* &tree, DigraphSearchItem key);

        // 删除AVL树(tree)中的结点(z)，并返回被删除的结点
        AVLTreeNode* remove(AVLTreeNode* &tree, AVLTreeNode* z);

        // 销毁AVL树
        void destroy(AVLTreeNode* &tree);

        // 打印AVL树
        void print(AVLTreeNode* tree, DigraphSearchItem key, int direction);

		// DOT
		void write_graphviz_nodes(ostream& out, AVLTreeNode* tree, int direction);
		void write_graphviz_edges(ostream& out, AVLTreeNode* tree, int direction);
};

#endif