#ifndef CLUSTER_H_
#define CLUSTER_H_

#include <chrono>
#include <string>


inline void Proximity(int i, const std::vector<std::vector<float>>& points, std::vector<int> &cluster, std::vector<bool> &processed, KdTree* tree, float distanceTol){
  
  processed[i] = true;
  cluster.push_back(i);
  
  std::vector<int> ids;
  ids = tree->search(points[i], distanceTol);
  
  for (int id: ids){
    
    if(processed[id])
      continue;
      
    Proximity(id, points, cluster, processed, tree, distanceTol);
  }
  
  
}

inline std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size(), false);

  	
  	for(int i=0; i<points.size(); i++){
      
		if (processed[i])
          continue;
          
       std::vector<int> cluster;
      	Proximity(i, points, cluster, processed, tree, distanceTol);
      clusters.push_back(cluster);
    }
 
	return clusters;

}
#endif