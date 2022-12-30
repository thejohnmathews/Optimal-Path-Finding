/*
 * John Mathews
 * hw04 - Optimal Path Finding
 * CPSC 2120
 */

//include statement
#include "hw04.h"
#include <map>
#include <vector>
#include <queue>
#include <math.h>

//global variables
config info;

//dijkstra global vars
map<Node, int> dist;
map<Node, Node> pred;
typedef pair<double, Node> pin;
int inf = 999999;

//pixel global variables
int width;
int height;
pixel *Picture;
pixel clemsonOrange = { 245, 102 , 0 };

//call main()
int main(int argc, char* argv[]){

    //configure information
    readConfig(argv);
    readPPMPicture(argv);

    //find the shortest path with dijkstra
    cout << "starting dijkstra's algorithm." << endl;
    dijkstra();

    //print results to ppm Picture
    cout << "printing path to .ppm & .txt" << endl;
    printPath(argv);

    cout << "end of program" << endl;

    return 0;
}

//function definitions:

//pix(): takes two ints and corresponds to a point in Picture
pixel &pix(int x, int y){

    return Picture[width * y + x];
}

//pixelValidation(): ensure the pixels do not go past threshold T
bool pixelValidation(pixel pix){

    if (pix.r + pix.g + pix.b > info.T) return true;
    return false;
}

//distance(): use the distance equation for 2 points
double distance(double x1, double x2, double y1, double y2){

    return (sqrt(pow((x2 - x1), 2) + pow(y2 - y1, 2)));
}

//unitVectors(): calculate the unit vector of a vector given
pair<double, double> unitVectors(pair<int, int> v){

    //unitVectors() local variables
    double length = 0;

    length = sqrt(pow(v.first, 2) + pow(v.second, 2));
    return make_pair(v.first / length, v.second / length);
}

//findDotProduct(): take the two unit vectors & multiply together
double findDotProduct(pair<double, double> a, pair<double, double> b){

    return a.first * b.first + a.second * b.second;
}

//degrees(): determines if the angle is too much for the train
bool degrees(Node previous, Node current, Node next) {

    //degrees() local variables
    double dotProduct = 0;
    double angle = 0;
    pair<int, int> vector1 = make_pair(current.currentLocation.x - previous.currentLocation.x, current.currentLocation.y - previous.currentLocation.y);
    pair<int, int> vector2 = make_pair(next.currentLocation.x - current.currentLocation.x, next.currentLocation.y - current.currentLocation.y);

    //to find dot product, multiply the unit vectors together
    dotProduct = findDotProduct(unitVectors(vector1), unitVectors(vector2));

    //calculate the angle
    angle = info.M * (M_PI / 180.0);

    //return if dotProduct is >= cos(angle)
    return dotProduct >= cos(angle);
}

//getPassengers(): checks to see if passengers are in range of current point
void getPassengers(Node &temp){

    //pick up passengers
    if (distance(temp.currentLocation.x, info.profDean.x, temp.currentLocation.y, info.profDean.y) <= info.D) temp.passengers |= 1;
    if (distance(temp.currentLocation.x, info.zhihongLin.x, temp.currentLocation.y, info.zhihongLin.y) <= info.D) temp.passengers |= 2;
    if (distance(temp.currentLocation.x, info.johnPerez.x, temp.currentLocation.y, info.johnPerez.y) <= info.D
        || distance(temp.currentLocation.x, info.johnReeder.x, temp.currentLocation.y, info.johnReeder.y) <= info.D) temp.passengers |= 4;
}

//getNeighbors(): look at neighbors around a pixel
vector<Node> getNeighbors(Node node){

    //getNeighbors() local variables
    vector<Node> neighbors;
    Node temp;
    Node prev;
    double dist = 0;

    //look at square around node
    for (int i = max(0, node.currentLocation.y - info.eucMax); i <= node.currentLocation.y + info.eucMax && i < height; i++){
        for (int j = max(0, node.currentLocation.x - info.eucMax); j <= node.currentLocation.x + info.eucMax && j < width; j++){

            //validate point before continuing
            if (pixelValidation(pix(j,i))) continue;

            //set node data
            temp.currentLocation.x = j;
            temp.currentLocation.y = i;
            temp.previousLocation.x = node.currentLocation.x;
            temp.previousLocation.y = node.currentLocation.y;
            temp.passengers = node.passengers;
            prev.currentLocation.x = node.previousLocation.x;
            prev.currentLocation.y = node.previousLocation.y;

            //validate distance before continuing
            dist = distance(node.currentLocation.x, temp.currentLocation.x, node.currentLocation.y, temp.currentLocation.y);
            if (dist < info.eucMin || dist > info.eucMax) continue;

            //ensure point being looked at is not start Node and passes angle constraint
            if (node != info.startPoint && !degrees(prev, node, temp)) continue;

            //check to see if passengers can be picked up
            getPassengers(temp);

            //place neighbors into vector
            neighbors.push_back(temp);
        }
    }

    //return neighbor vector to dijkstra
    return neighbors;
}

//dijkstra(): shortest path algorithm between 2 Nodes
void dijkstra(void){

    //dijkstra() local variables
    dist[info.startPoint] = 0;
    priority_queue <pin, vector<pin>, greater<pin>> to_visit;
    to_visit.push(make_pair(0, info.startPoint));
    pred[info.startPoint] = info.startPoint;

    //loop while there are pixels to look at
    while(!to_visit.empty()){

        Node x = to_visit.top().second;
        to_visit.pop();

        //end search when end point is found along with passengers
        if (x.passengers == 7 && distance(x.currentLocation.x, info.goalPoint.currentLocation.x, x.currentLocation.y, info.goalPoint.currentLocation.y) <= info.D){

            cout << "dijkstra found the end point (" << x.currentLocation.x << ", " << x.currentLocation.y <<  ") and picked up all passengers." << endl;
            pred[info.goalPoint] = x;
            return;
        }

        //for every possible node, check its neighbors
        for (Node n : getNeighbors(x)){

            if (dist.find(n) == dist.end()) dist[n] = inf;
            double weight = distance(x.currentLocation.x, n.currentLocation.x, x.currentLocation.y, n.currentLocation.y);

            if (dist[x] + weight < dist[n]){

                dist[n] = dist[x] + weight;
                pred[n] = x;
                to_visit.push(make_pair(dist[n], n));
            }
        }
    }
}

//writePath(): place red pixels onto ppm Picture and write coords to file
void writePath(Node start, Node goal, FILE* textFile){

    //put 3x3 red pixels on path
    for (int i = -2; i < 2; i++){
        for (int j = -1; j < 1; j++){

            Picture[width * goal.currentLocation.y+i + goal.currentLocation.x+j] = clemsonOrange;
            Picture[width * goal.currentLocation.y+i + goal.currentLocation.x-j] = clemsonOrange;
            Picture[width * goal.currentLocation.y-i + goal.currentLocation.x+j] = clemsonOrange;
            Picture[width * goal.currentLocation.y-i + goal.currentLocation.x-j] = clemsonOrange;
        }
    }

    //recursively backtrace through pred map
    if (pred[goal] != goal){

        writePath(start, pred[goal], textFile);
    }

    //print points into textFile
    if (&pred[goal] !=  &pred[start]){

        fprintf(textFile, "%d %d\n", pred[goal].currentLocation.x, pred[goal].currentLocation.y);
    }
}

//printPath(): print and create output
void printPath(char* argv[]){

    //adjust Picture and text file
    FILE* textFile = fopen("path.txt","w");
    FILE* ppmFile = fopen("path.ppm", "w");
    if (textFile == NULL) cout << "there was an error opening the text file." << endl;
    if (ppmFile == NULL) cout << "there was an error opening the ppm file." << endl;

    //adjust pixels and write
    writePath(info.startPoint, info.goalPoint, textFile);
    fprintf (ppmFile, "P6\n%d %d\n255\n", width, height);
    fwrite (Picture, width * height, sizeof(pixel), ppmFile);

    //close files
    fclose(textFile);
    fclose(ppmFile);
}

//readPPMPicture(): read contents of PPM Picture
void readPPMPicture(char* argv[]){

    //readPPMPicture() local variables
    FILE* img = fopen(info.ppmName.c_str(), "r");

    //read contents of PPM Picture
    fscanf(img, "P6\n%d %d\n255%*c", &width, &height);
    Picture = new pixel[width * height];
    fread(Picture, width * height, sizeof(pixel), img);
    fclose(img);
}

//initializeFromFile(): read file input
void readConfig(char* argv[]){

    //readConfig() local variables
    ifstream configFile (argv[1]);

    //check file integrity
    if (configFile.is_open()){

        //place data into global struct
        configFile >> info.ppmName
                   >> info.startPoint.currentLocation.x >> info.startPoint.currentLocation.y
                   >> info.goalPoint.currentLocation.x >> info.goalPoint.currentLocation.y
                   >> info.profDean.x >> info.profDean.y
                   >> info.zhihongLin.x >> info.zhihongLin.y
                   >> info.johnPerez.x >> info.johnPerez.y
                   >> info.johnReeder.x >> info.johnReeder.y
                   >> info.T >> info.eucMin >> info.eucMax
                   >> info.M >> info.D;
    }
    //if file cannot be opened/corrupted exit program
    else{

        cout << "There was an error opening the file." << endl;
        exit(-1);
    }
}