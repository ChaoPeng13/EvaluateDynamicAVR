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
        AVLTreeNode *mRoot;    // �����

    public:
        AVLTree();
        ~AVLTree();

        // ��ȡ���ĸ߶�
        int height();

        // ǰ�����"AVL��"
        void preOrder();
        // �������"AVL��"
        void inOrder();
        // �������"AVL��"
        void postOrder();

        // (�ݹ�ʵ��)����"AVL��"�м�ֵΪkey�Ľڵ�
        AVLTreeNode* search(DigraphSearchItem key);
		// recursively find all the nodes
		void search(vector<AVLTreeNode*>& vec, DigraphSearchItem key);
		// check whether we need to search this item and update
		bool check(vector<DigraphSearchItem>& vec, DigraphSearchItem key);

        // (�ǵݹ�ʵ��)����"AVL��"�м�ֵΪkey�Ľڵ�
        AVLTreeNode* iterativeSearch(DigraphSearchItem key);

        // ������С��㣺������С���ļ�ֵ��
        DigraphSearchItem minimum();
        // ��������㣺���������ļ�ֵ��
        DigraphSearchItem maximum();

        // �����(keyΪ�ڵ��ֵ)���뵽AVL����
        void insert(DigraphSearchItem key);

        // ɾ�����(keyΪ�ڵ��ֵ)
        void remove(DigraphSearchItem key);

        // ����AVL��
        void destroy();

        // ��ӡAVL��
        void print();

		// Dot
		void write_graphviz(ostream& out);
    private:
        // ��ȡ���ĸ߶�
        int height(AVLTreeNode* tree) ;

        // ǰ�����"AVL��"
        void preOrder(AVLTreeNode* tree) const;
        // �������"AVL��"
        void inOrder(AVLTreeNode* tree) const;
        // �������"AVL��"
        void postOrder(AVLTreeNode* tree) const;

        // (�ݹ�ʵ��)����"AVL��x"�м�ֵΪkey�Ľڵ�
        AVLTreeNode* search(AVLTreeNode* x, DigraphSearchItem key) const;
		// recursively find all the nodes
		void search(vector<AVLTreeNode*>& vec, AVLTreeNode* x, DigraphSearchItem key) const;

		bool check(vector<DigraphSearchItem>& vec, AVLTreeNode* x, DigraphSearchItem key);

        // (�ǵݹ�ʵ��)����"AVL��x"�м�ֵΪkey�Ľڵ�
        AVLTreeNode* iterativeSearch(AVLTreeNode* x, DigraphSearchItem key) const;

        // ������С��㣺����treeΪ������AVL������С��㡣
        AVLTreeNode* minimum(AVLTreeNode* tree);
        // ��������㣺����treeΪ������AVL��������㡣
        AVLTreeNode* maximum(AVLTreeNode* tree);

        // LL�������Ӧ�����(����ת)��
        AVLTreeNode* leftLeftRotation(AVLTreeNode* k2);

        // RR�����Ҷ�Ӧ�����(�ҵ���ת)��
        AVLTreeNode* rightRightRotation(AVLTreeNode* k1);

        // LR�����Ҷ�Ӧ�����(��˫��ת)��
        AVLTreeNode* leftRightRotation(AVLTreeNode* k3);

        // RL�������Ӧ�����(��˫��ת)��
        AVLTreeNode* rightLeftRotation(AVLTreeNode* k1);

        // �����(z)���뵽AVL��(tree)��
        AVLTreeNode* insert(AVLTreeNode* &tree, DigraphSearchItem key);

        // ɾ��AVL��(tree)�еĽ��(z)�������ر�ɾ���Ľ��
        AVLTreeNode* remove(AVLTreeNode* &tree, AVLTreeNode* z);

        // ����AVL��
        void destroy(AVLTreeNode* &tree);

        // ��ӡAVL��
        void print(AVLTreeNode* tree, DigraphSearchItem key, int direction);

		// DOT
		void write_graphviz_nodes(ostream& out, AVLTreeNode* tree, int direction);
		void write_graphviz_edges(ostream& out, AVLTreeNode* tree, int direction);
};

#endif