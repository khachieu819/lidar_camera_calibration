#ifndef PREPROCESSING_H_
#define PREPROCESSING_H_

#include <iostream>
#include <opencv4/opencv2/opencv.hpp>

#include <pcl/point_cloud.h> // for PointCloud
#include <pcl/pcl_macros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

#include <pcl/common/transforms.h>

struct PointXYZRI {
    PCL_ADD_POINT4D;
    float intensity;
    float r;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;    // ensure proper alignment
};

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRI, 
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, intensity, intensity)
                                    (float, r, r)
)

pcl::PointCloud<PointXYZRI>::Ptr toPointXYZRI(const pcl::PointCloud<pcl::PointXYZI> &point_cloud_ptr){
    pcl::PointCloud<PointXYZRI>::Ptr new_cloud(new pcl::PointCloud <PointXYZRI>);
    for(auto pt = point_cloud_ptr.begin(); pt < point_cloud_ptr.end(); pt ++){
        PointXYZRI myPt;
        myPt.x = pt->x;
        myPt.y = pt->y;
        myPt.z = pt->z;
        myPt.intensity = 0.0;
        myPt.r = 0.0;
        new_cloud->push_back(myPt);
    }
    return new_cloud;

}

pcl::PointCloud<pcl::PointXYZI>::Ptr toPointXYZI(pcl::PointCloud<PointXYZRI> point_cloud){
    pcl::PointCloud<pcl::PointXYZI>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    // for(pcl::PointCloud<PointXYZRI>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++){
    //     new_cloud->push_back(pcl::PointXYZI(pt->x, pt->y, pt->z, pt->intensity));
    // }

    for(pcl::PointCloud<PointXYZRI>::iterator 
        pt = point_cloud.begin(); pt < point_cloud.end(); pt++){
            new_cloud->push_back(pcl::PointXYZI(pt->x, pt->y, pt->z, pt->intensity));
        }

    return new_cloud;

}

pcl::PointCloud<PointXYZRI> normalizeIntensity(pcl::PointCloud<PointXYZRI> point_cloud, float min, float max)
{
    float min_found = 10e6;
    float max_found = -10e6;

    for(pcl::PointCloud<PointXYZRI>::iterator pt = point_cloud.begin(); pt < point_cloud.end(); pt++)
    {
        max_found = std::max(max_found, pt->intensity);
        min_found = std::min(min_found, pt->intensity);
    }

    for(pcl::PointCloud<PointXYZRI>::iterator pt = point_cloud.begin(); pt < point_cloud.end(); pt++){
        pt->intensity = (pt->intensity - min_found) / (max_found - min_found) * (max - min) + min;
        // std::cout << pt->intensity << std::endl;
    }

    return point_cloud;
}

pcl::PointCloud <PointXYZRI> intensityByRangeDiff(pcl::PointCloud <PointXYZRI> point_cloud){

        std::vector <std::vector<PointXYZRI*>> rings(16);       // khai bao mot mang vector con tro co kieu du lieu PointXYZRI

        // in SICK Multi Scan layer 6 and 14 has 0.125 degree/round -> 2880 data, the other has 1 degree/round -> 360 data
        int i = 1;
        for(pcl::PointCloud<PointXYZRI>::iterator pt = point_cloud.begin(); pt < point_cloud.end(); pt++)
        {
            pt->r = (pt->x * pt->x + pt->y * pt->y + pt->z * pt->z);

            if(i <= 360)
                rings[0].push_back(&(*pt));
            else if(i > 360 && i <= 720)
                rings[1].push_back(&(*pt));
            else if(i > 720 && i <= 1080)
                rings[2].push_back(&(*pt));
            else if(i > 1080 && i <= 1440)
                rings[3].push_back(&(*pt));
            else if(i > 1440 && i <= 1800)
                rings[4].push_back(&(*pt));
            else if(i > 1800 && i <= 4680)
                rings[5].push_back(&(*pt));
            else if(i > 4680 && i <= 5040)
                rings[6].push_back(&(*pt));
            else if(i > 5040 && i <= 5400)
                rings[7].push_back(&(*pt));
            else if(i > 5400 && i <= 5760)
                rings[8].push_back(&(*pt));
            else if(i > 5760 && i <= 6120)
                rings[9].push_back(&(*pt));
            else if(i > 6120 && i <= 6480)
                rings[10].push_back(&(*pt));
            else if(i > 6480 && i <= 6840)
                rings[11].push_back(&(*pt));
            else if(i > 6840 && i <= 7200)
                rings[12].push_back(&(*pt));
            else if(i > 7200 && i <= 10080)
                rings[13].push_back(&(*pt));
            else if(i > 10080 && i <= 10440)
                rings[14].push_back(&(*pt));
            else if(i > 10440 && i <= 10800)
                rings[15].push_back(&(*pt));

            i++;
        }

        for(std::vector <std::vector<PointXYZRI* >>::iterator ring = rings.begin(); ring < rings.end(); ring++)
        {
            PointXYZRI *prev, *next;

            if (ring->empty()){
                continue;
            }

            (*ring->begin())->intensity = 0;
            (*(ring->end()-1))->intensity = 0;

            for(std::vector<PointXYZRI*>::iterator pt = ring->begin()+1; pt < ring->end()-1; pt++){
                prev = *(pt-1);  // con tro iterator
                next = *(pt+1);

                (*pt)->intensity = std::max( std::max (prev->r - (*pt)->r, next->r - (*pt)->r), float(0.0)) * 10;
            }
        }

        point_cloud = normalizeIntensity(point_cloud, 0.0, 1.0);

        pcl::PointCloud<PointXYZRI> filtered;

        for(pcl::PointCloud<PointXYZRI>::iterator pt = point_cloud.begin(); pt < point_cloud.points.end(); pt++)
        {
            if(pt->intensity > 0.03){
                if(pt->x >0 && pt->x <=2 && pt->y > -0.5 && pt->y < 0.5 && pt->z >-1 && pt->z <=2)
                    filtered.push_back(*(pt));
            }
        }
        return filtered;
    }

pcl::PointCloud <PointXYZRI> transform(pcl::PointCloud<PointXYZRI> pc, float x, float y, float z, float rot_x, float rot_y, float rot_z)
{
    Eigen::Affine3f transf = pcl::getTransformation(x, y, z, rot_x, rot_y, rot_z);
    pcl::PointCloud<PointXYZRI> new_cloud;
    pcl::transformPointCloud(pc, new_cloud, transf);
    return new_cloud;
}


void onMouse(int event, int x, int y, int f, void *g){
    cv::Point *P = static_cast<cv::Point*> (g);
    // std::cout << "hello" << std::endl;
    switch(event){
        case cv::EVENT_LBUTTONDOWN  :
            P->x = x;
            P->y = y;
            break;
        
        case cv::EVENT_LBUTTONUP    :
            P->x = x;
            P->y = y;
            break;
        
        default                     :
            break;
    }
    // std::cout << P->x << std::endl;
    // std::cout << P->y << std::endl;
}



#endif