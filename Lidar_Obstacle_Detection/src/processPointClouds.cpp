// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
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
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered  (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes,filterRes,filterRes);
    sor.filter (*cloud_filtered);

    typename pcl::CropBox<PointT> ROI(true);
    ROI.setInputCloud(cloud_filtered);
    ROI.setMin(minPoint);
    ROI.setMax(maxPoint);
    ROI.filter(*cloud_filtered2);
    
    typename pcl::CropBox<PointT> Remove_roof(true);
    Remove_roof.setInputCloud(cloud_filtered2);
    std::vector<int> indices;
    Remove_roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    Remove_roof.setMax(Eigen::Vector4f(2.7,1.8,-.4,1));
    Remove_roof.filter(indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for (int index : indices)
        inliers->indices.push_back(index);
    
    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud (cloud_filtered2);
    extract.setIndices (inliers);

    extract.setNegative (true);
    extract.filter(*cloud_filtered2);



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered2;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

    for (int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);
    typename pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud (cloud);
    extract.setIndices (inliers);

    extract.setNegative (true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	/*pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    typename pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers,*coefficients);
    if(inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planner model" <<std::endl;
    }*/
    std::unordered_set<int> inliersRes;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    while(maxIterations--) {
        std::unordered_set<int> inliers;
        while (inliers.size() < 3) {
            inliers.insert(rand() % cloud->points.size());
        }

        float x1, x2, x3, y1, y2, y3, z1, z2, z3;

        auto itr = inliers.begin();

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

        float i, j, k;

        i = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        j = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        k = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);

        std::vector<float> v1{x2 - x1, y2 - y1, z2 - z1};
        std::vector<float> v2{x3 - x1, y3 - y1, z3 - z1};
        std::vector<float> cross_prod{i, j, k};

        float A = i;
        float B = j;
        float C = k;
        float D = -(i * x1 + j * y1 + k * z1);

        for (int index = 0; index <= cloud->points.size(); index++) {
            if (inliers.count(index) > 0)
                continue;
            typename pcl::PointCloud<PointT>::PointType point = cloud->points[index];
            float x{point.x}, y{point.y}, z{point.z};
            float num = fabs(A * x + B * y + C * z + D);
            float Den = sqrt(A * A + B * B + C * C);
            float Dist = num / Den;

            if (Dist <= distanceThreshold)
                inliers.insert(index);
        }
        if (inliers.size() > inliersRes.size())
            inliersRes = inliers;
    }
    auto itr = inliersRes.begin();
    for (itr; itr != inliersRes.end(); itr++)
    {
        inliers->indices.push_back(*itr);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(int indice,typename pcl::PointCloud<PointT>::Ptr &cloud_cluster,typename pcl::PointCloud<PointT>::Ptr cloud,KdTree* tree, float& distanceTol,std::vector<bool>& processed)
{
    processed[indice]=true;//mark point as processed
    cloud_cluster->push_back(cloud->points[indice]);
    std::vector<int> nearest = tree->search(cloud->points[indice],distanceTol);

    for (int id : nearest)
    {
        if(!processed[id])
            proximity(id,cloud_cluster,cloud,tree,distanceTol,processed);
    }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>ProcessPointClouds<PointT>:: euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float& distanceTol,int &minSize,int &maxSize)
{

    // TODO: Fill out this function to return list of indices for each cluster
    std::vector<typename pcl::PointCloud<PointT>::Ptr> euclusters;

    std::vector<bool> processed(cloud->points.size(),false);
    int i=0;
    //std::cout<<"x: "<<point[0]<<std::endl;
    while(i<cloud->points.size()) {
        if (processed[i]) //point has not been processed
        {
            i++;
            continue;
        }
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new typename pcl::PointCloud<PointT>);//create cluster
        proximity(i, cloud_cluster, cloud, tree, distanceTol, processed);
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        if(cloud_cluster->points.size()>=minSize)
            euclusters.push_back(cloud_cluster);
        i++;
    }
    return euclusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    KdTree* tree = new KdTree;

    for (int index = 0; index <= cloud->points.size(); index++)
        tree->insert(cloud->points[index],index);

    clusters = euclideanCluster(cloud,tree,clusterTolerance,minSize,maxSize);

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
   /*  typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    //std::cout<<"max cluster size: "<< ec.getMaxClusterSize()<<std::endl;
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for (pcl::PointIndices getIndices : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (int index :getIndices.indices)
            cloud_cluster->points.push_back (cloud->points[index]);
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }*/
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
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  

    // Find bounding box for one of the clusters
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new typename pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
// Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);   
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
    /*Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;
    */
   BoxQ boxq;
   boxq.bboxTransform = bboxTransform;
   boxq.bboxQuaternion = bboxQuaternion;
   boxq.cube_length = maxPoint.x - minPoint.x;
   boxq.cube_width = maxPoint.y - minPoint.y;
   boxq.cube_height = maxPoint.z - minPoint.z;

    return boxq;
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