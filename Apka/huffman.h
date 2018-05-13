#ifndef HUFFMAN_H
#define HUFFMAN_H

#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>

typedef unsigned char Byte;

Byte setBit(Byte x, unsigned int index);

bool getBit(Byte x, unsigned int index);

class Node {
public:
    static bool greaterThan(Node *a, Node *b) {
        if(a != NULL && b != NULL) {
            return a->count > b->count;
        } else if(a != NULL) {
            return true;
        } else {
            return false;
        }
    }

    static bool lessThan(Node *a, Node *b) {
        if(a != NULL && b != NULL) {
            return a->count < b->count;
        } else if(b != NULL) {
            return true;
        } else {
            return false;
        }
    }

    static void removeTree(Node *root) {
        if(root == NULL) {
            return;
        }
        removeTree(root->left);
        removeTree(root->right);
        delete root;
    }

    Node(Byte b, Node *l = NULL, Node *r = NULL, unsigned long c = 0) : left(l), right(r), count(c), byte(b) {
    }

    bool isLeaf() const {
        return this->left == NULL && this->right == NULL;
    }

    Node *left, *right;
    unsigned long count;
    Byte byte;
};

void mapTree(Node *root, std::vector<bool> *codes, std::vector<bool> &prefix = std::vector<bool>());
void saveTree(Node *root, std::ostream &output, Byte &accumulator, unsigned int &bitIndex);
bool loadTree(std::istream &input, Byte &accumulator, unsigned int &bitIndex, Node *&root);
void compress(std::istream &input, std::ostream &output);
bool decompress(std::istream &input, std::ostream &output);


#endif // HUFFMAN_H
