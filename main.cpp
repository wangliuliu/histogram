#include <iostream>
#include <stdlib.h>
#include <pcl/io/pcd_io.h>
#include <vector>

using namespace std;


const int B = 20;
const double MIN = 0.0;
const double MAX = 100.0;

double calculateW(vector<double> H, vector<double> G){
    double result = 0.0;
    double last = 0.0;
    for(int i=0; i<H.size(); i++){
        last += abs(H[i]-G[i]);
        result += last;
    }
    if(H.size()>0)
        result /= H.size();
    return result;
}

void calculateHistogram(string path1, string path2, vector<double> & H, vector<double>&G,int b, double min, double delta_b){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(path1,*cloud1);
    pcl::io::loadPCDFile(path2,*cloud2);
    for(int i=0;i<cloud1->points.size();i++){
        double d = cloud1->points[i].x * cloud1->points[i].x+cloud1->points[i].y * cloud1->points[i].y+cloud1->points[i].z * cloud1->points[i].z;
        d = sqrt(d);
        int index = (d-min)/delta_b;
        if(index>=0 && index<100)
            H[index] ++;
    }

    for(int i=0;i<cloud2->points.size();i++){
        double d = cloud2->points[i].x * cloud2->points[i].x+cloud2->points[i].y * cloud2->points[i].y+cloud2->points[i].z * cloud2->points[i].z;
        d = sqrt(d);
        int index = (d-min)/delta_b;
        if(index>=0 && index<100)
            G[index] ++;
    }
    for(int i=0;i<b;i++){
        if(cloud1->points.size()>0)
            H[i] /= cloud1->points.size();
        if(cloud2->points.size()>0)
            G[i] /= cloud2->points.size();
    }
}

int main(int argc, char **argv){
    string path1 = argv[1];
    string path2 = argv[2];
    vector<double> H(B,0.0);
    vector<double> G(B,0.0);
    double delta_b = (MAX-MIN)/B;
    calculateHistogram(path1,path2,H,G,B,MIN,delta_b);
    cout<<calculateW(H,G)<<endl;
}
