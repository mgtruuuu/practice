#include <liblas/liblas.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>


#include <iostream>
#include <unordered_map>
#include <vector>




enum Options
{
    PlaneModelSegmentation,     // PMS      1
    EuclideanClusterExtraction, // ECE

    IsWrong
};

Options cvtString2Enum(const std::string& input)
{
    static const std::unordered_map<std::string, Options> table =
    {
        { "PMS", PlaneModelSegmentation },
        { "ECE", EuclideanClusterExtraction }
    };
    
    auto it = table.find(input);
    if (it != table.end())
    {
        return it->second;
    }
    else
    {
        return IsWrong;
    }
}

void planeModelSegmentation();
void euclideanClusterExtraction();

Options cvtString2Enum(const std::string& input);





int main (int argc, char** argv)
{
    std::cout << "What do you want?" << std::endl;
    std::string input;
    std::cin >> input;

    Options option = cvtString2Enum(input);

    switch(option)
    {
        case PlaneModelSegmentation:
            planeModelSegmentation();
            break;

        



        case IsWrong:
            std::cout << "default\n";
            break;
    }



    return 0;
}





void planeModelSegmentation()
{
    /*
    **  1. Floor Removal (RANSAC : Random Sample Consensus)
    **             
    */

    // Plane model segmentation
    // http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inlierPoints (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Object for storing the plane model coefficients.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());


    std::ifstream ifs ("../las_example.las", std::ios::in | std::ios::binary);
    liblas::Reader reader(ifs);
    int Idx = 0;
    double shift_x, shift_y, shift_z;
    while (reader.ReadNextPoint())
    {   
        if (Idx == 0)
        {
            shift_x = reader.GetPoint().GetX();
            shift_y = reader.GetPoint().GetY();
            shift_z = reader.GetPoint().GetZ();
            //std::cout << "Idx == 0: " << shift_x << ' ' << shift_y << ' ' << shift_z << std::endl;
        }

        pcl::PointXYZRGB pt;

        pt.x = reader.GetPoint().GetX() - shift_x;
        pt.y = reader.GetPoint().GetY() - shift_y;
        pt.z = reader.GetPoint().GetZ() - shift_z;
        pt.r = reader.GetPoint().GetColor().GetRed();
        pt.g = reader.GetPoint().GetColor().GetGreen();
        pt.b = reader.GetPoint().GetColor().GetBlue();

        // if( Idx % 20000 == 0 )
        // {
        //     std::cout << "x, y, z: " << pt.x << ' ' << pt.y << ' ' << pt.z << std::endl;
        //     std::cout << "r, g, b: " << pt.r << ' ' << pt.g << ' ' << pt.b << std::endl;
        // }

        cloud->points.push_back(pt);

        ++Idx;
    }
    
    std::cout << "Input : " << cloud->points.size() 
        << " (" << pcl::getFieldsList(*cloud) << ")" << std::endl;








    // ???????????? ?????? Create the segmentation object.

    
    //const double para1 = 0.01;
    const double para1 = 0.1;
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);       // (??????) // Enable model coefficient refinement (optional).
                                              // sign of 'a' ....
    seg.setInputCloud (cloud);                // ?????? 
    seg.setModelType (pcl::SACMODEL_PLANE);   // ?????? ??????  // Configure the object to look for a plane.
    seg.setMethodType (pcl::SAC_RANSAC);      // ?????? ??????   // Use RANSAC method.
    seg.setMaxIterations (1000);              // ?????? ?????? ???
    seg.setDistanceThreshold (para1);         // inlier??? ????????? ?????? ??????   // Set the maximum allowed distance to the model.
    //seg.setRadiusLimits(0, 0.1);            // cylinder??????, Set minimum and maximum radii of the cylinder.
    seg.segment (*inliers, *coefficients);    // ?????????????????? ?????? 

    std::vector<double> vars;
    vars.push_back(para1);


    //????????? ?????? ???????????? ?????? (eg. ax + by + cz + d = 0 ).
    std::cerr << "Model coefficients (ax+by+cz+d=0): " 
              << coefficients->values[0] << " " 
              << coefficients->values[1] << " "
              << coefficients->values[2] << " " 
              << coefficients->values[3] << std::endl;

    pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud, *inliers, *inlierPoints);
    
    std::cout << "Output : " << inlierPoints->points.size () 
        << " (" << pcl::getFieldsList(*inlierPoints)  << ")" << std::endl;







    std::string outputName = "../output/PlaneModelSegmentation/PMS";
    for (const auto& para : vars)
    {
        outputName += "_" + std::to_string(para);
    }
    outputName += ".las";
    std::ofstream ofs (outputName, std::ios::out | std::ios::binary);


    liblas::Header header;
    header.SetScale(0.001, 0.001, 0.001);       // NOTE!!!
    liblas::Writer writer(ofs, header);
    
    for (int ptIdx = 0; ptIdx < inlierPoints->points.size(); ++ptIdx)
    {
        liblas::Point Point(&header);

        double x = static_cast<double>(inlierPoints->points[ptIdx].x) + shift_x;
        double y = static_cast<double>(inlierPoints->points[ptIdx].y) + shift_y;
        double z = static_cast<double>(inlierPoints->points[ptIdx].z) + shift_z;

        Point.SetX(x);
        Point.SetY(y);
        Point.SetZ(z);

        writer.WritePoint(Point);
    }
}



void euclideanClusterExtraction()
{

    /*
    **  2. ????????? (Euclidean-PCL)
    */

    // Euclidean Cluster Extraction
    // http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

    // ????????? ?????? KdTree ???????????? ?????? //Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud);  //KdTree ?????? 



    std::ifstream ifs ("../las_example.las", std::ios::in | std::ios::binary);
    liblas::Reader reader(ifs);
    int Idx = 0;
    double shift_x, shift_y, shift_z;
    while (reader.ReadNextPoint())
    {   
        if (Idx == 0)
        {
            shift_x = reader.GetPoint().GetX();
            shift_y = reader.GetPoint().GetY();
            shift_z = reader.GetPoint().GetZ();
        }

        pcl::PointXYZRGB pt;

        pt.x = reader.GetPoint().GetX() - shift_x;
        pt.y = reader.GetPoint().GetY() - shift_y;
        pt.z = reader.GetPoint().GetZ() - shift_z;
        pt.r = reader.GetPoint().GetColor().GetRed();
        pt.g = reader.GetPoint().GetColor().GetGreen();
        pt.b = reader.GetPoint().GetColor().GetBlue();

        cloud->points.push_back(pt);

        ++Idx;
    }

    std::cout << "Input : " << cloud->points.size() 
        << " (" << pcl::getFieldsList(*cloud) << ")" << std::endl;




    std::vector<pcl::PointIndices> cluster_indices;       // ???????????? ???????????? Index ??????, ?????? ????????? ????????? cluster_indices[0] ????????? ?????? 
    // ????????? ???????????? ??????  
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setInputCloud (cloud);       // ??????   
    ec.setClusterTolerance (0.02);  // 2cm  
    ec.setMinClusterSize (100);     // ?????? ????????? ??? 
    ec.setMaxClusterSize (25000);   // ?????? ????????? ???
    ec.setSearchMethod (tree);      // ????????? ????????? ?????? ?????? ?????? 
    ec.extract (cluster_indices);   // ????????? ?????? 

    // ??????????????? ?????? ??????, ??????, ?????? 
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
         it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloud_cluster->points.push_back (cloud->points[*pit]); 
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // ???????????? ??????
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;

        // // ??????????????? ?????? ?????? ??? ?????? 
        // std::stringstream ss;
        // ss << "cloud_cluster_" << j << ".pcd";
        // pcl::PCDWriter writer;
        // writer.write<pcl::PointXYZRGB> (ss.str(), *cloud_cluster, false); //*
        // ++j;

        // ??????????????? ?????? ?????? ??? ?????? 
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZRGB> (ss.str(), *cloud_cluster, false); //*
        ++j;

        

    }


}













