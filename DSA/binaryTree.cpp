#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
using namespace std;

// NodeStructure
struct Node
{
    double data;
    Node* left;
    Node* right;
    
    Node(int val){
        data = val;
        left = nullptr;
        right = nullptr;
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

    bool isValidBST(Node* x, double lVal = INT_MIN, double uVal = INT_MAX){
        if(!x) return true;
        if(x==root || (x->data > lVal && x->data < uVal)){
            bool leftValid = isValidBST(x->left, lVal, x->data);
            bool rightValid = isValidBST(x->right, x->data, uVal);
            return leftValid && rightValid;
        }
        else return false;
    }

    Node* search(Node* x, double target){
        if(x==root){
            if(!isValidBST(root)){cout << "Invalid Tree Structure: Not a Binary Search Tree, method invalid"<<endl;return nullptr;}
        }
        
        if(!x){cout<< "Not found" << endl; return nullptr;}
        
        if(x->data == target){return x;}        
        if(x->data < target){search(x->left,target);}
        if(x->data > target){search(x->right,target);}
    }

    Node* lowestCommonAncestor(Node* x, Node* p, Node* q){
        if(!x || x==p || x==q)return x;
        Node* left = lowestCommonAncestor(x->left,p,q);
        Node* right = lowestCommonAncestor(x->left,p,q);

        return !left ? right : !right ? left : root;
    }

    void arrayImplementation(Node* x, std::vector<double>& ans) {
    std::queue<Node*> q;
    q.push(x);

    while (!q.empty()) {
        Node* current = q.front();
        q.pop();
        if(current!=nullptr){
            ans.push_back(current->data); // Add the current node's value
            q.push(current->left);       // Push left child (can be nullptr)
            q.push(current->right);      // Push right child (can be nullptr)
        } else {
            ans.push_back(NAN); // Add NaN for null children
        }
    }
    int n = ans.size()-1;
    // Clean trailing NaN
    for(int i=n; i>0; i--){if(isnan(ans[i])){ans.pop_back();}else{break;}}
    }
};

void printVector(const std::vector<double>& vec) {
    for (double val : vec) {
        if (!val) {
            std::cout << "NaN ";
        } else {
            std::cout << val << " ";
        }
    }
    std::cout << std::endl;
}

int main(){
    Node* root = new Node(1);
    root->left = new Node(2);
    root->right = new Node(3);
    root->left->left = new Node(4);
    root->left->right = new Node(5);
    std::vector<double> ans;
    binaryTree T1 = binaryTree(root);
    T1.arrayImplementation(root, ans);
    printVector(ans);
}