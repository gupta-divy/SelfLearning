#include <iostream>
using namespace std;

// NodeStructure
struct Node
{
    int data;
    Node* next;
    
    Node(int val, Node* link = nullptr){
        data = val;
        if(link != nullptr) next = link;
    }
};

class LinkedList
{   
public:
    Node* head;
    Node* tail;
    int len = 0;

    LinkedList() : head(nullptr), tail(nullptr) {}

    void append(int value){
        Node* newNode = new Node(value);
        if(head == nullptr){
            head = newNode;
            tail = head;
        }
        else{
            tail->next = newNode;
            tail = newNode;
        }
        len++;
    }

    void insert(int value, int idx){
        if(idx > len) {
            cout << "Invalid Index" << endl;
            return;
        }
        if(idx == 0){
            Node* newNode = new Node(value, head);
            head = newNode;
            if(tail == nullptr){ tail = newNode; }
        }
        else{
            int i = 0;
            Node* temp = head;
            while(i < idx - 1){
                temp = temp->next;
                i++;
            }
            Node* newNode = new Node(value, temp->next);
            temp->next = newNode;
            if(newNode->next == nullptr){ tail = newNode; }
        }
        len++;
    }

    void deleteNode(int idx){
        if(idx < 1 || idx >= len){
            cout << "Not Allowed" << endl;
            return;
        }
        Node* temp = head;
        for(int i = 0; i < idx - 1; i++) { temp = temp->next; }
        Node* toDelete = temp->next;
        temp->next = temp->next->next;
        if(temp->next == nullptr) { tail = temp; }
        delete toDelete;
        len--;
    }

    int fetch(int idx){
        int i = 0;
        Node* temp = head;
        while(i < idx) { temp = temp->next; i++; }
        return temp->data;
    }

    void display(){
        if(head == nullptr) {
            cout << "The list is empty." << endl;
            return;
        }
        
        Node* temp = head;
        while(temp != nullptr){
            cout << temp->data << "->";
            temp = temp->next;
        }
        cout << "NULL" << endl;
    }
};

int main(){
    LinkedList list;
    list.append(10);
    list.append(20);
    list.append(30);
    list.insert(25, 0);
    cout << "Linked List: ";
    list.display();
    cout << list.fetch(1) << endl;
    list.deleteNode(1);
    cout << "After deletion: ";
    list.display();
}
