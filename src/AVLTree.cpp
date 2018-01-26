#include "AVLTree.h"

/* 
 * ���캯��
 */
AVLTree::AVLTree():mRoot(NULL)
{
}

/* 
 * ��������
 */
AVLTree::~AVLTree() 
{
    destroy(mRoot);
}

/*
 * ��ȡ���ĸ߶�
 */
int AVLTree::height(AVLTreeNode* tree) 
{
    if (tree != NULL)
        return tree->height;

    return 0;
}

int AVLTree::height() 
{
    return height(mRoot);
}

/*
 * ǰ�����"AVL��"
 */
void AVLTree::preOrder(AVLTreeNode* tree) const
{
    if(tree != NULL)
    {
        cout<< tree->key << " " ;
        preOrder(tree->left);
        preOrder(tree->right);
    }
}

void AVLTree::preOrder() 
{
    preOrder(mRoot);
}

/*
 * �������"AVL��"
 */
void AVLTree::inOrder(AVLTreeNode* tree) const
{
    if(tree != NULL)
    {
        inOrder(tree->left);
        cout<< tree->key << " " ;
        inOrder(tree->right);
    }
}

void AVLTree::inOrder() 
{
    inOrder(mRoot);
}

/*
 * �������"AVL��"
 */
void AVLTree::postOrder(AVLTreeNode* tree) const
{
    if(tree != NULL)
    {
        postOrder(tree->left);
        postOrder(tree->right);
        cout<< tree->key << " " ;
    }
}

void AVLTree::postOrder() 
{
    postOrder(mRoot);
}

/*
 * (�ݹ�ʵ��)����"AVL��x"�м�ֵΪkey�Ľڵ�
 */
AVLTreeNode* AVLTree::search(AVLTreeNode* x, DigraphSearchItem key) const
{
    if (x==NULL || x->key==key)
        return x;

    if (key < x->key)
        return search(x->left, key);
    else
        return search(x->right, key);
}

AVLTreeNode* AVLTree::search(DigraphSearchItem key) 
{
    return search(mRoot, key);
}

void AVLTree::search(vector<AVLTreeNode*>& vec, AVLTreeNode* x, DigraphSearchItem key) const {
	while ((x!=NULL) && (x->key!=key))
	{
		if (key < x->key)
			x = x->left;
		else
			x = x->right;
	}

	if (x==NULL) return;

	vec.push_back(x);
	search(vec,x->left,key);
	search(vec,x->right,key);
}

void AVLTree::search(vector<AVLTreeNode*>& vec, DigraphSearchItem key)
{
	search(vec,mRoot,key);
}

bool AVLTree::check(vector<DigraphSearchItem>& vec, AVLTreeNode* x, DigraphSearchItem key) {
	vector<AVLTreeNode*> foundItems;
	search(foundItems,x,key);

	if (foundItems.empty()) {
		insert(mRoot,key);
		return false;
	}

	if (foundItems.size() == 1) {
		AVLTreeNode* item = foundItems.front();
		if (item->key.lKey <= key.lKey && item->key.rKey >= key.rKey && item->key.value >= key.value)
			return true;
		if (item->key.lKey == key.lKey && item->key.rKey == key.rKey) 
			item->key.value = key.value;
	}

	return false;
}

bool AVLTree::check(vector<DigraphSearchItem>& vec, DigraphSearchItem key) {
	return check(vec,mRoot,key);
}

/*
 * (�ǵݹ�ʵ��)����"AVL��x"�м�ֵΪkey�Ľڵ�
 */
AVLTreeNode* AVLTree::iterativeSearch(AVLTreeNode* x, DigraphSearchItem key) const
{
    while ((x!=NULL) && (x->key!=key))
    {
        if (key < x->key)
            x = x->left;
        else
            x = x->right;
    }

    return x;
}

AVLTreeNode* AVLTree::iterativeSearch(DigraphSearchItem key)
{
    return iterativeSearch(mRoot, key);
}

/* 
 * ������С��㣺����treeΪ������AVL������С��㡣
 */
AVLTreeNode* AVLTree::minimum(AVLTreeNode* tree)
{
    if (tree == NULL)
        return NULL;

    while(tree->left != NULL)
        tree = tree->left;
    return tree;
}

DigraphSearchItem AVLTree::minimum()
{
    AVLTreeNode *p = minimum(mRoot);
    if (p != NULL)
        return p->key;

    return DigraphSearchItem();
}
 
/* 
 * ��������㣺����treeΪ������AVL��������㡣
 */
AVLTreeNode* AVLTree::maximum(AVLTreeNode* tree)
{
    if (tree == NULL)
        return NULL;

    while(tree->right != NULL)
        tree = tree->right;
    return tree;
}

DigraphSearchItem AVLTree::maximum()
{
    AVLTreeNode *p = maximum(mRoot);
    if (p != NULL)
        return p->key;

    return DigraphSearchItem();
}

/*
 * LL�������Ӧ�����(����ת)��
 *
 * ����ֵ����ת��ĸ��ڵ�
 */
AVLTreeNode* AVLTree::leftLeftRotation(AVLTreeNode* k2)
{
    AVLTreeNode* k1;

    k1 = k2->left;
    k2->left = k1->right;
    k1->right = k2;

    k2->height = max( height(k2->left), height(k2->right)) + 1;
    k1->height = max( height(k1->left), k2->height) + 1;

    return k1;
}

/*
 * RR�����Ҷ�Ӧ�����(�ҵ���ת)��
 *
 * ����ֵ����ת��ĸ��ڵ�
 */
AVLTreeNode* AVLTree::rightRightRotation(AVLTreeNode* k1)
{
    AVLTreeNode* k2;

    k2 = k1->right;
    k1->right = k2->left;
    k2->left = k1;

    k1->height = max( height(k1->left), height(k1->right)) + 1;
    k2->height = max( height(k2->right), k1->height) + 1;

    return k2;
}

/*
 * LR�����Ҷ�Ӧ�����(��˫��ת)��
 *
 * ����ֵ����ת��ĸ��ڵ�
 */
AVLTreeNode* AVLTree::leftRightRotation(AVLTreeNode* k3)
{
    k3->left = rightRightRotation(k3->left);

    return leftLeftRotation(k3);
}

/*
 * RL�������Ӧ�����(��˫��ת)��
 *
 * ����ֵ����ת��ĸ��ڵ�
 */
AVLTreeNode* AVLTree::rightLeftRotation(AVLTreeNode* k1)
{
    k1->right = leftLeftRotation(k1->right);

    return rightRightRotation(k1);
}

/* 
 * �������뵽AVL���У������ظ��ڵ�
 *
 * ����˵����
 *     tree AVL���ĸ����
 *     key ����Ľ��ļ�ֵ
 * ����ֵ��
 *     ���ڵ�
 */
AVLTreeNode* AVLTree::insert(AVLTreeNode* &tree, DigraphSearchItem key)
{
    if (tree == NULL) 
    {
        // �½��ڵ�
        tree = new AVLTreeNode(key, NULL, NULL);
        if (tree==NULL)
        {
            cout << "ERROR: create avltree node failed!" << endl;
            return NULL;
        }
    }
    else if (key < tree->key) // Ӧ�ý�key���뵽"tree��������"�����
    {
        tree->left = insert(tree->left, key);
        // ����ڵ����AVL��ʧȥƽ�⣬�������Ӧ�ĵ��ڡ�
        if (height(tree->left) - height(tree->right) == 2)
        {
            if (key < tree->left->key)
                tree = leftLeftRotation(tree);
            else
                tree = leftRightRotation(tree);
        }
    }
    else if (key > tree->key) // Ӧ�ý�key���뵽"tree��������"�����
    {
        tree->right = insert(tree->right, key);
        // ����ڵ����AVL��ʧȥƽ�⣬�������Ӧ�ĵ��ڡ�
        if (height(tree->right) - height(tree->left) == 2)
        {
            if (key > tree->right->key)
                tree = rightRightRotation(tree);
            else
                tree = rightLeftRotation(tree);
        }
    }
    else //key == tree->key)
    {
        cout << "���ʧ�ܣ������������ͬ�Ľڵ㣡" << endl;
    }

    tree->height = max( height(tree->left), height(tree->right)) + 1;

    return tree;
}

void AVLTree::insert(DigraphSearchItem key)
{
    insert(mRoot, key);
}

/* 
 * ɾ�����(z)�����ظ��ڵ�
 *
 * ����˵����
 *     tree AVL���ĸ����
 *     z ��ɾ���Ľ��
 * ����ֵ��
 *     ���ڵ�
 */
AVLTreeNode* AVLTree::remove(AVLTreeNode* &tree, AVLTreeNode* z)
{
    // ��Ϊ�� ���� û��Ҫɾ���Ľڵ㣬ֱ�ӷ���NULL��
    if (tree==NULL || z==NULL)
        return NULL;

    if (z->key < tree->key)        // ��ɾ���Ľڵ���"tree��������"��
    {
        tree->left = remove(tree->left, z);
        // ɾ���ڵ����AVL��ʧȥƽ�⣬�������Ӧ�ĵ��ڡ�
        if (height(tree->right) - height(tree->left) == 2)
        {
            AVLTreeNode *r =  tree->right;
            if (height(r->left) > height(r->right))
                tree = rightLeftRotation(tree);
            else
                tree = rightRightRotation(tree);
        }
    }
    else if (z->key > tree->key)// ��ɾ���Ľڵ���"tree��������"��
    {
        tree->right = remove(tree->right, z);
        // ɾ���ڵ����AVL��ʧȥƽ�⣬�������Ӧ�ĵ��ڡ�
        if (height(tree->left) - height(tree->right) == 2)
        {
            AVLTreeNode *l =  tree->left;
            if (height(l->right) > height(l->left))
                tree = leftRightRotation(tree);
            else
                tree = leftLeftRotation(tree);
        }
    }
    else    // tree�Ƕ�ӦҪɾ���Ľڵ㡣
    {
        // tree�����Һ��Ӷ��ǿ�
        if ((tree->left!=NULL) && (tree->right!=NULL))
        {
            if (height(tree->left) > height(tree->right))
            {
                // ���tree�����������������ߣ�
                // ��(01)�ҳ�tree���������е����ڵ�
                //   (02)�������ڵ��ֵ��ֵ��tree��
                //   (03)ɾ�������ڵ㡣
                // ����������"tree�������������ڵ�"��"tree"������
                // �������ַ�ʽ�ĺô��ǣ�ɾ��"tree�������������ڵ�"֮��AVL����Ȼ��ƽ��ġ�
                AVLTreeNode* max = maximum(tree->left);
                tree->key = max->key;
                tree->left = remove(tree->left, max);
            }
            else
            {
                // ���tree��������������������(��������ȣ�������������������1)
                // ��(01)�ҳ�tree���������е���С�ڵ�
                //   (02)������С�ڵ��ֵ��ֵ��tree��
                //   (03)ɾ������С�ڵ㡣
                // ����������"tree������������С�ڵ�"��"tree"������
                // �������ַ�ʽ�ĺô��ǣ�ɾ��"tree������������С�ڵ�"֮��AVL����Ȼ��ƽ��ġ�
                AVLTreeNode* min = maximum(tree->right);
                tree->key = min->key;
                tree->right = remove(tree->right, min);
            }
        }
        else
        {
            AVLTreeNode* tmp = tree;
            tree = (tree->left!=NULL) ? tree->left : tree->right;
            delete tmp;
        }
    }

    return tree;
}

void AVLTree::remove(DigraphSearchItem key)
{
    AVLTreeNode* z; 

    if ((z = search(mRoot, key)) != NULL)
        mRoot = remove(mRoot, z);
}

/* 
 * ����AVL��
 */
void AVLTree::destroy(AVLTreeNode* &tree)
{
    if (tree==NULL)
        return ;

    if (tree->left != NULL)
        destroy(tree->left);
    if (tree->right != NULL)
        destroy(tree->right);

    delete tree;
}

void AVLTree::destroy()
{
    destroy(mRoot);
}

/*
 * ��ӡ"���������"
 *
 * key        -- �ڵ�ļ�ֵ 
 * direction  --  0����ʾ�ýڵ��Ǹ��ڵ�;
 *               -1����ʾ�ýڵ������ĸ���������;
 *                1����ʾ�ýڵ������ĸ������Һ��ӡ�
 */
void AVLTree::print(AVLTreeNode* tree, DigraphSearchItem key, int direction)
{
    if(tree != NULL)
    {
        if(direction==0)    // tree�Ǹ��ڵ�
            cout << setw(2) << tree->key << " is root" << endl;
        else                // tree�Ƿ�֧�ڵ�
            cout << setw(2) << tree->key << " is " << setw(2) << key << "'s "  << setw(12) << (direction==1?"right child" : "left child") << endl;

        print(tree->left, tree->key, -1);
        print(tree->right,tree->key,  1);
    }
}

void AVLTree::print()
{
    if (mRoot != NULL)
        print(mRoot, mRoot->key, 0);
}

void AVLTree::write_graphviz_nodes(ostream& out, AVLTreeNode* tree, int direction) {
	if (tree != NULL) {
		DigraphSearchItem item = tree->key;
		if(direction == 0) // tree root node
			out << "n" << tree << " [label=\" (" << item.lKey << ","<< item.rKey << "): " << item.value << " \", color = red, style=filled];" << endl; 
		else if (direction == 1) // tree right node
			out << "n" << tree << " [label=\" (" << item.lKey << ","<< item.rKey << "): " << item.value << " \", color = lightblue2, style=filled];" << endl; 
		else
			out << "n" << tree << " [label=\" (" << item.lKey << ","<< item.rKey << "): " << item.value << " \", color = lightyellow2, style=filled];" << endl; 
	
		write_graphviz_nodes(out,tree->left,-1);
		write_graphviz_nodes(out,tree->right,1);
	}
}

void AVLTree::write_graphviz_edges(ostream& out, AVLTreeNode* tree, int direction) {
	if (tree != NULL) {
		if (tree->left != NULL) {
			out << "n" << tree << " -> " << "n" << tree->left << ";" << endl;
			write_graphviz_edges(out,tree->left,-1);
		}

		if (tree->right != NULL) {
			out << "n" << tree << " -> " << "n" << tree->right << ";" << endl;
			write_graphviz_edges(out,tree->right,1);
		}
	}
}

void AVLTree::write_graphviz(ostream& out)
{
	out << "digraph G {" <<endl;
	// write nodes
	if (mRoot != NULL)
		write_graphviz_nodes(out,mRoot,0);

	// write edges
	if (mRoot != NULL)
		write_graphviz_edges(out,mRoot,0);

	out << "}" <<endl;
}
