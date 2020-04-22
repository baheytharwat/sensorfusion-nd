// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <pcl/filters/crop_box.h>
#include <unordered_set>


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
  typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (filterRes, filterRes, filterRes);
  sor.filter (*cloud_filtered);
  
  typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT>);
  
  pcl::CropBox< PointT > roi(true);
  roi.setMin(minPoint);
  roi.setMax(maxPoint);
  roi.setInputCloud(cloud_filtered);
  roi.filter(*cloud_region);
  
  std::vector<int> indices;
  
  pcl::CropBox< PointT > roof(true);
  roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1 ));
  roof.setMax(Eigen::Vector4f(2.6, 1.6, -0.4, 1 ));
  roof.setInputCloud(cloud_region);
  roof.filter(indices);
  
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  for(int point : indices)
    inliers->indices.push_back(point);
  
  pcl::ExtractIndices<PointT> extract;
  
  extract.setInputCloud (cloud_region);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*cloud_region);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
 typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());
 typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ()) ;
  
  for(int index : inliers ->indices)
    planeCloud->points.push_back(cloud->points[index]);
  
  pcl::ExtractIndices<PointT> extract;
  
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*obstCloud);
  
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
  
  
  std::unordered_set<int> inliersResult;
  for (int i=0; i< maxIterations; i++){
      
      	std::unordered_set<int> inliers;

      while(inliers.size() < 3)
        inliers.insert(rand()%(cloud->points.size()));
      
      
      auto itr = inliers.begin();
      
      	float x1,y1,z1,x2,y2,z2,x3,y3,z3;
      
      	x1 = cloud->points[*itr].x;
      	y1 = cloud->points[*itr].y;
      	z1 = cloud->points[*itr].z;
      	itr++;
        x2 = cloud->points[*itr].x;
      	y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
      	itr++;
        x3 = cloud->points[*itr].x;
      	y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;
      
      	float v1[3] = {x2-x1, y2-y1, z2-z1};
      	float v2[3] = {x3-x1, y3-y1, z3-z1};
      	float v[3];
    
	v[0] = v1[1] * v2[2] - v1[2] * v2[1]; 
    v[1] = v1[2] * v2[0] - v1[0] * v2[2]; 
    v[2] = v1[0] * v2[1] - v1[1] * v2[0];   
    
  	    float a = v[0];
        float b = v[1];
        float c = v[2];
      	float d = -1* ( a*x1 + b*y1 + c*z1 );
      
  		for (int index =0; index< cloud->points.size(); index++){
    
          	if (inliers.count(index) >0)
              continue;
              
    		float x = cloud->points[index].x;
            float y = cloud->points[index].y;
          	float z = cloud->points[index].z;
            float dist = fabs((a*x + b*y+ c*z + d) / sqrt(a*a + b*b + c*c));
          
          if (dist <= distanceThreshold)
           	 inliers.insert(index);
          
    	}
      
      if(inliers.size() > inliersResult.size())
        inliersResult = inliers;
    }

  
  
  
  
  
  pcl::PointIndices::Ptr inlierss (new pcl::PointIndices ());
  
  inlierss->indices.insert(inlierss->indices.end(), inliersResult.begin(), inliersResult.end());
  
    if (inlierss->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inlierss,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
  
  
KdTree* tree = new KdTree;
std::vector<std::vector<float>> vec_points;
  for(int i = 0; i < cloud->points.size(); i++)
  {
    PointT point = cloud->points[i];
    std::vector<float> vec = {point.x, point.y, point.z};
    vec_points.push_back(vec);
    tree->insert(vec, i);
  }
  
  std::vector<std::vector<int>> clusts;
  clusts =euclideanCluster(vec_points, tree, clusterTolerance);
  
      
    for(auto clust : clusts)
  {
    if(clust.size() < minSize || clust.size() > maxSize)
    {
        continue; 
    }

  typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);
    
    for(auto index : clust)
    {
      cluster->points.push_back(cloud->points[index]);
    }
  
    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;
  
    clusters.push_back(cluster);
  }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}