#include "AVLTree.h"

/* 
 * 构造函数
 */
AVLTree::AVLTree():mRoot(NULL)
{
}

/* 
 * 析构函数
 */
AVLTree::~AVLTree() 
{
    destroy(mRoot);
}

/*
 * 获取树的高度
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
 * 前序遍历"AVL树"
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
 * 中序遍历"AVL树"
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
 * 后序遍历"AVL树"
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
 * (递归实现)查找"AVL树x"中键值为key的节点
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
 * (非递归实现)查找"AVL树x"中键值为key的节点
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
 * 查找最小结点：返回tree为根结点的AVL树的最小结点。
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
 * 查找最大结点：返回tree为根结点的AVL树的最大结点。
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
 * LL：左左对应的情况(左单旋转)。
 *
 * 返回值：旋转后的根节点
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
 * RR：右右对应的情况(右单旋转)。
 *
 * 返回值：旋转后的根节点
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
 * LR：左右对应的情况(左双旋转)。
 *
 * 返回值：旋转后的根节点
 */
AVLTreeNode* AVLTree::leftRightRotation(AVLTreeNode* k3)
{
    k3->left = rightRightRotation(k3->left);

    return leftLeftRotation(k3);
}

/*
 * RL：右左对应的情况(右双旋转)。
 *
 * 返回值：旋转后的根节点
 */
AVLTreeNode* AVLTree::rightLeftRotation(AVLTreeNode* k1)
{
    k1->right = leftLeftRotation(k1->right);

    return rightRightRotation(k1);
}

/* 
 * 将结点插入到AVL树中，并返回根节点
 *
 * 参数说明：
 *     tree AVL树的根结点
 *     key 插入的结点的键值
 * 返回值：
 *     根节点
 */
AVLTreeNode* AVLTree::insert(AVLTreeNode* &tree, DigraphSearchItem key)
{
    if (tree == NULL) 
    {
        // 新建节点
        tree = new AVLTreeNode(key, NULL, NULL);
        if (tree==NULL)
        {
            cout << "ERROR: create avltree node failed!" << endl;
            return NULL;
        }
    }
    else if (key < tree->key) // 应该将key插入到"tree的左子树"的情况
    {
        tree->left = insert(tree->left, key);
        // 插入节点后，若AVL树失去平衡，则进行相应的调节。
        if (height(tree->left) - height(tree->right) == 2)
        {
            if (key < tree->left->key)
                tree = leftLeftRotation(tree);
            else
                tree = leftRightRotation(tree);
        }
    }
    else if (key > tree->key) // 应该将key插入到"tree的右子树"的情况
    {
        tree->right = insert(tree->right, key);
        // 插入节点后，若AVL树失去平衡，则进行相应的调节。
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
        cout << "添加失败：不允许添加相同的节点！" << endl;
    }

    tree->height = max( height(tree->left), height(tree->right)) + 1;

    return tree;
}

void AVLTree::insert(DigraphSearchItem key)
{
    insert(mRoot, key);
}

/* 
 * 删除结点(z)，返回根节点
 *
 * 参数说明：
 *     tree AVL树的根结点
 *     z 待删除的结点
 * 返回值：
 *     根节点
 */
AVLTreeNode* AVLTree::remove(AVLTreeNode* &tree, AVLTreeNode* z)
{
    // 根为空 或者 没有要删除的节点，直接返回NULL。
    if (tree==NULL || z==NULL)
        return NULL;

    if (z->key < tree->key)        // 待删除的节点在"tree的左子树"中
    {
        tree->left = remove(tree->left, z);
        // 删除节点后，若AVL树失去平衡，则进行相应的调节。
        if (height(tree->right) - height(tree->left) == 2)
        {
            AVLTreeNode *r =  tree->right;
            if (height(r->left) > height(r->right))
                tree = rightLeftRotation(tree);
            else
                tree = rightRightRotation(tree);
        }
    }
    else if (z->key > tree->key)// 待删除的节点在"tree的右子树"中
    {
        tree->right = remove(tree->right, z);
        // 删除节点后，若AVL树失去平衡，则进行相应的调节。
        if (height(tree->left) - height(tree->right) == 2)
        {
            AVLTreeNode *l =  tree->left;
            if (height(l->right) > height(l->left))
                tree = leftRightRotation(tree);
            else
                tree = leftLeftRotation(tree);
        }
    }
    else    // tree是对应要删除的节点。
    {
        // tree的左右孩子都非空
        if ((tree->left!=NULL) && (tree->right!=NULL))
        {
            if (height(tree->left) > height(tree->right))
            {
                // 如果tree的左子树比右子树高；
                // 则(01)找出tree的左子树中的最大节点
                //   (02)将该最大节点的值赋值给tree。
                //   (03)删除该最大节点。
                // 这类似于用"tree的左子树中最大节点"做"tree"的替身；
                // 采用这种方式的好处是：删除"tree的左子树中最大节点"之后，AVL树仍然是平衡的。
                AVLTreeNode* max = maximum(tree->left);
                tree->key = max->key;
                tree->left = remove(tree->left, max);
            }
            else
            {
                // 如果tree的左子树不比右子树高(即它们相等，或右子树比左子树高1)
                // 则(01)找出tree的右子树中的最小节点
                //   (02)将该最小节点的值赋值给tree。
                //   (03)删除该最小节点。
                // 这类似于用"tree的右子树中最小节点"做"tree"的替身；
                // 采用这种方式的好处是：删除"tree的右子树中最小节点"之后，AVL树仍然是平衡的。
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
 * 销毁AVL树
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
 * 打印"二叉查找树"
 *
 * key        -- 节点的键值 
 * direction  --  0，表示该节点是根节点;
 *               -1，表示该节点是它的父结点的左孩子;
 *                1，表示该节点是它的父结点的右孩子。
 */
void AVLTree::print(AVLTreeNode* tree, DigraphSearchItem key, int direction)
{
    if(tree != NULL)
    {
        if(direction==0)    // tree是根节点
            cout << setw(2) << tree->key << " is root" << endl;
        else                // tree是分支节点
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
