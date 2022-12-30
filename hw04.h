/*
 * John Mathews
 * hw04 - Optimal Path Finding
 * CPSC 2120
 */

#ifndef HW04_H
#define HW04_H

//include libraries
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
using namespace std;

//structures

typedef struct pixelStruct{

    unsigned char r;
    unsigned char g;
    unsigned char b;
} pixel;

typedef struct pointStruct{

    int x;
    int y;

} point;

typedef struct NodeInformation{

    point currentLocation;
    point previousLocation;
    int passengers;

} Node;

typedef struct ppmInformation{

    string ppmName;
    Node startPoint;
    Node goalPoint;
    point profDean;
    point zhihongLin;
    point johnPerez;
    point johnReeder;
    int T; //threshold that govern which pixels are valid
    int eucMin;
    int eucMax;
    int M; //max degrees the train can turn
    int D; //distance specifying min distance from an obj

} config;

//function prototypes
void readConfig(char* argv[]);
void readPPMPicture(char* argv[]);
pixel &pix(int x, int y);
void getNeighbors(void);
void dijkstra(void);
void printPath(char* argv[]);
void writePath(Node startPoint, Node goalPoint, FILE* textFile);
bool pixelValidation(pixel pix);
double distance(double x1, double x2, double y1, double y2);
bool degrees(Node previous, Node current, Node next);
pair<double, double> unitVectors(pair<int, int> v);
double findDotProduct(pair<double, double> a, pair<double, double> b);
void getPassengers(Node &temp);

//point overrides
bool operator<(point a, point b){

    if (a.x == b.x) return a.y < b.y;
    else return a.x < b.x;
}
bool operator==(point a, point b){

    if (a.x == b.x && a.y == b.y) return true;
    return false;
}
bool operator!=(point a, point b){

    if (a.x != b.x || a.y != b.y) return true;
    return false;
}

//Node overrides
bool operator==(Node a, Node b){

    return a.currentLocation == b.currentLocation && a.previousLocation == b.previousLocation && a.passengers == b.passengers;
}
bool operator!=(Node a, Node b){

    return !(a == b);
}
bool operator<(Node a, Node b){

    if (a.currentLocation.x == b.currentLocation.x && a.currentLocation.y == b.currentLocation.y){

        if (a.previousLocation != b.previousLocation) return a.previousLocation < b.previousLocation;
        else return a.passengers < b.passengers;
    }
    else{

        if (a.currentLocation.x == b.currentLocation.x) return a.currentLocation.y < b.currentLocation.y;
        else return a.currentLocation.x < b.currentLocation.x;
    }
}

#endif