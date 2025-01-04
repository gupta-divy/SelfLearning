#include <iostream>
using namespace std;

// NodeStructure
struct Node
{
    double data;
    Node* left;
    Node* right;
    
    Node(int val){
        data = val;
    }
};

class binaryTree
{
private:
    Node* root;

    void insert(double data, Node*& node){
        if(node==nullptr){node = new Node(data);return;}
        if(data <= node->data){insert(data, node->left);}
        else{insert(data,node->right);}
    }

public:
    binaryTree(Node* x=nullptr){root = x;}

    void printData(Node* x){cout<<x->data;}

    int maxDepth(Node* x){
        if(!x) return 0;
        int maxL = maxDepth(x->left);
        int maxR = maxDepth(x->right);
        return max(maxL,maxR) + 1;
    }

    void preTraversal(Node* x){
        if(!x){return;}
        printData(x);
        preTraversal(x->left);
        preTraversal(x->right);
    }

    void postTraversal(Node* x){
        if(!x){return;}
        postTraversal(x->left);
        postTraversal(x->right);
        printData(x);
    }

    void inTraversal(Node* x){
        if(!x){return;}
        inTraversal(x->left);
        printData(x);
        inTraversal(x->right);
    }

    void insertNode(double data){
        insert(data,root);
    }

};