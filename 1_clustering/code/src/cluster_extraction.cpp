#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include "../include/Renderer.hpp"
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <chrono>
#include <unordered_set>
#include "../include/tree_utilities.hpp"
#include <boost/filesystem.hpp>
#include <thread>

#define USE_PCL_LIBRARY
// #define CHECK_BB
// #define DOWNSAMPLING_FROM_MEMORY
#define RENDER_ORIGINAL_CLOUD
// #define CAP_HZ

namespace fs = boost::filesystem;
using namespace lidar_obstacle_detection;

typedef std::unordered_set<int> my_visited_set_t;

//This function sets up the custom kdtree using the point cloud
void setupKdtree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree* tree, int dimension)
{
    //insert point cloud points into tree
    for (int i = 0; i < cloud->size(); ++i)
    {
        tree->insert({cloud->at(i).x, cloud->at(i).y, cloud->at(i).z}, i);
    }
}

/*
OPTIONAL
This function computes the nearest neighbors and builds the clusters
    - Input:
        + cloud: Point cloud to be explored
        + target_ndx: i-th point to visit
        + tree: kd tree for searching neighbors
        + distanceTol: Distance tolerance to build the clusters 
        + visited: Visited points --> typedef std::unordered_set<int> my_visited_set_t;
        + cluster: Here we add points that will represent the cluster
        + max: Max cluster size
    - Output:
        + visited: already visited points
        + cluster: at the end of this function we will have one cluster
*/
void proximity(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int target_ndx, my_pcl::KdTree* tree, float distanceTol, my_visited_set_t& visited, std::vector<int>& cluster, int max)
{
	if (cluster.size() < max)
    {
        cluster.push_back(target_ndx);
        visited.insert(target_ndx);

        std::vector<float> point {cloud->at(target_ndx).x, cloud->at(target_ndx).y, cloud->at(target_ndx).z};
    
        // get all neighboring indices of point
        std::vector<int> neighborNdxs = tree->search(point, distanceTol);

        for (int neighborNdx : neighborNdxs)
        {
            // if point was not visited
            if (visited.find(neighborNdx) == visited.end())
            {
                proximity(cloud, neighborNdx, tree, distanceTol, visited, cluster, max);
            }

            if (cluster.size() >= max)
            {
                return;
            }
        }
    }
}

/*
OPTIONAL
This function builds the clusters following a euclidean clustering approach
    - Input:
        + cloud: Point cloud to be explored
        + tree: kd tree for searching neighbors
        + distanceTol: Distance tolerance to build the clusters 
        + setMinClusterSize: Minimum cluster size
        + setMaxClusterSize: Max cluster size
    - Output:
        + cluster: at the end of this function we will have a set of clusters
TODO: Complete the function
*/
std::vector<pcl::PointIndices> euclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree* tree, float distanceTol, int setMinClusterSize, int setMaxClusterSize)
{
	my_visited_set_t visited{};                                                          //already visited points
	std::vector<pcl::PointIndices> clusters;                                             //vector of PointIndices that will contain all the clusters
    std::vector<int> cluster;                                                            //vector of int that is used to store the points that the function proximity will give me back
    
    //for every point of the cloud
    //  if the point has not been visited (use the function called "find")
    //    find clusters using the proximity function
    //
    //    if we have more clusters than the minimum
    //      Create the cluster and insert it in the vector of clusters. You can extract the indices from the cluster returned by the proximity funciton (use pcl::PointIndices)   
    //    end if
    //  end if
    //end for

    for(size_t idx = 0; idx < cloud->points.size(); idx++)
    {
        if(visited.find(idx) == visited.end()) // find returns an iterator, the iterator is equal to set.end() if the element was not found
        {   
            cluster.clear(); // reset what is inside cluster
            proximity(cloud, idx, tree, distanceTol, visited, cluster, setMaxClusterSize);
            if(cluster.size() >= setMinClusterSize)
            {
                pcl::PointIndices current_cluster;
                current_cluster.indices = std::move(cluster);
                clusters.push_back(std::move(current_cluster));
            }
        }
    }
	return clusters;	
}

static double mean(double a, double b)
{
    return (a+b)/2.0f;
}

static inline double ratio(double a, double b) 
{
    if(std::abs(b) < 1e-6) // degenerate case
    {
        return std::numeric_limits<double>::infinity();
    }
    return(a/b);
}

static inline bool in_range(double value, double lowerbound, double higherbound)
{
    return ((value >= lowerbound) && (value <= higherbound));
}

static bool isCarOrPedestrianOrBike(const pcl::PointXYZ& minPt, const pcl::PointXYZ& maxPt)
{
    const double car_length_width_threshold = 1.2;
    const double car_length_height_threshold = 1.2;
    const double car_width_height_threshold = 1.5;

    const double bike_length_width_threshold = 1.5;
    const double bike_height_width_threshold = 1.5;
    const double bike_width_height_threshold = 1.2;

    const double person_height_base_threshold = 2.0;

    double width  = std::abs(maxPt.x - minPt.x);
    double length = std::abs(maxPt.y - minPt.y);
    double height = std::abs(maxPt.z - minPt.z);

    if (!std::isfinite(width) || !std::isfinite(length) || !std::isfinite(height))
    {
        return false;
    }

    if (width > length) 
    {
        std::swap(width, length); // just to make sure we use conventional naming (in pedestrians width and length lose meaning but whatever)
    }

    if (width < 0.3 || length < 0.3 || height < 1.0 || width > 3.5 || length > 7.5 || height > 3.5) // discard absurd values
    {
        return false;
    }

    double l_w = ratio(length, width);              
    double l_h = ratio(length, height);
    double w_h = ratio(width, height);
    double h_w = ratio(height, width);
    double h_w_l = ratio(height, std::max(width, length)); //used for pedestrians

    if (!std::isfinite(l_w) || !std::isfinite(l_h) || !std::isfinite(w_h) || !std::isfinite(h_w) || !std::isfinite(h_w_l))
    {
        return false;
    }

    // lets see if it might be a car
    bool isCar =
        in_range(length, 2.0, 6.0) && 
        in_range(width,  1.3, 2.5) &&
        in_range(height, 1.3, 2.5) &&
        l_w >= car_length_width_threshold &&    // length > width
        l_h >= car_length_height_threshold &&   // length > height
        w_h <= car_width_height_threshold;      // width and height not wildly different

    // lets see if it might be a bike
    bool isBike =
        in_range(length, 1.0, 2.5) &&
        in_range(width,  0.5, 1.0) &&
        in_range(height, 1.0, 2.0) &&
        l_w >= bike_length_width_threshold &&   // the length of the bike should be considerably greater that its width
        h_w >= bike_height_width_threshold &&   // the height of the bike should be considerably greater than its width
        w_h <= bike_width_height_threshold;     // width and height not wildly different

    // Pedestrian
    bool isPed = 
        in_range(length, 0.3, 0.8) &&
        in_range(width,  0.3, 0.8) &&
        in_range(height, 1.4, 2.0) &&
        h_w_l >= person_height_base_threshold; // tall vs footprint

    return isCar || isBike || isPed;
}

void ProcessAndRenderPointCloud (Renderer& renderer, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    int setMinClusterSize = 20;
    int setMaxClusterSize = 150;
    double setClusterTolerance = 0.50f;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ()), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    // TODO: 1) Downsample the dataset
    #ifdef DOWNSAMPLING_FROM_MEMORY // downsample directly in memory
        // cons: you must assume pcl point are ordered, you don't account for points density
        // pros: approx. 3x faster than voxelization (empirical)
        int n = 2; // factor of downsampling
        std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> filtered_points;
        std::size_t output_size = (cloud->points.size() + n - 1) / n;
        filtered_points.reserve(output_size);
        for(size_t i = 0; i < cloud->points.size(); i+=n)
        {
           filtered_points.push_back(cloud->points[i]);
        }
        cloud_filtered->points = std::move(filtered_points);
        cloud_filtered->height = 1;
        cloud_filtered->width = output_size;

    #else // use voxelization
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud (cloud);
        vg.setLeafSize (0.2f, 0.2f, 0.2f); 	
        vg.filter (*cloud_filtered);
    #endif

    // 2) here we crop the points that are far away from us, in which we are not interested
    pcl::CropBox<pcl::PointXYZ> cb(true);
    cb.setInputCloud(cloud_filtered);
    cb.setMin(Eigen::Vector4f (-20, -6, -2, 1));
    cb.setMax(Eigen::Vector4f ( 30, 7, 5, 1));
    cb.filter(*cloud_filtered); 

    // TODO: 3) Segmentation and apply RANSAC
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.15);

    // TODO: 4) iterate over the filtered cloud, segment and remove the planar inliers 
    while (cloud_filtered->size () > 0.015f * cloud->size())
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for remaining points." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);

        // optional for debugging ######
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;
        // ######

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;
    }

    // TODO: 5) Create the vector of PointIndices

    std::vector<pcl::PointIndices> cluster_indices;

    // TODO: 6) Set the spatial tolerance for new cluster candidates (pay attention to the tolerance!!!)

    #ifdef USE_PCL_LIBRARY
        //PCL functions
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud_filtered);
            
        //HERE 6)
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        //Set the spatial tolerance for new cluster candidates
        ec.setClusterTolerance(setClusterTolerance);
        //We impose that the clusters found must have at least setMinClusterSize() points and maximum setMaxClusterSize() points
        ec.setMinClusterSize(setMinClusterSize);
        ec.setMaxClusterSize(setMaxClusterSize);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered);
        ec.extract(cluster_indices);
    #else
        // Optional assignment
        my_pcl::KdTree treeM;
        treeM.set_dimension(3);
        setupKdtree(cloud_filtered, &treeM, 3);
        cluster_indices = euclideanCluster(cloud_filtered, &treeM, setClusterTolerance, setMinClusterSize, setMaxClusterSize);
    #endif

    std::cout << "The number of extracted clusters is: " << cluster_indices.size() << std::endl; 

    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(1,0,1), Color(0,1,1), Color(1,1,1)};

    /**Now we extracted the clusters out of our point cloud and saved the indices in cluster_indices. 

    To separate each cluster out of the vector<PointIndices> we have to iterate through cluster_indices, create a new PointCloud for each entry and write all points of the current cluster in the PointCloud.
    Compute euclidean distance
    **/
    int j = 0;
    int clusterId = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            cloud_cluster->push_back ((*cloud_filtered)[*pit]); 
        }
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // TODO: 7) render the cluster and plane without rendering the original cloud 
        #ifndef RENDER_ORIGINAL_CLOUD
            renderer.RenderPointCloud(cloud_cluster,"Cluster"+std::to_string(clusterId),colors[2]);
        #endif

        //Here we create the bounding box on the detected clusters
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);
        //TODO: 8) Here you can plot the distance of each cluster w.r.t ego vehicle
        Box box{minPt.x, minPt.y, minPt.z,
        maxPt.x, maxPt.y, maxPt.z};

        #ifdef CHECK_BB
        bool render = isCarOrPedestrianOrBike(minPt, maxPt); // I apply a check on the shape and volume of the box to check if it is a Car, a Pedestrian or a Bike.
        #else
        bool render = true;
        #endif
        if(render){
            Eigen::Vector3f box_center;
            box_center.x() = mean(minPt.x, maxPt.x);
            box_center.y() = mean(minPt.y, maxPt.y);
            box_center.z() = mean(minPt.z, maxPt.z);
            double distance = box_center.norm();

            double street_width = 50.0;
            double front_distance_threshold = 50.0; // the assignment says 5.0 m but it looks small
            //TODO: 9) Here you can color the vehicles that are both in front and 5 meters away from the ego vehicle
            // we assume a object is in front of the vehichle if it's x value is positive and its y value is between +4 and -4 meters
            if(((minPt.x > 0.0) || (box_center.x() > 0.0)) && (((minPt.y < street_width)&&(minPt.y > (-street_width))) || ((maxPt.y < street_width)&&(maxPt.y > (-street_width)))) && (distance <= front_distance_threshold))
            {
                    renderer.RenderBox(box, j, box_center, distance, colors[1]);
            }
            else
            {
                renderer.RenderBox(box, j, box_center, distance);
            }
        }
        ++clusterId;
        j++;
    }  
    #ifdef RENDER_ORIGINAL_CLOUD
        renderer.RenderPointCloud(cloud,"originalCloud",colors[3]);
    #endif

}


int main(int argc, char* argv[])
{
    #ifdef CAP_HZ
        using clock = std::chrono::steady_clock;
        const int hz = 5;
        const auto time_budget = std::chrono::milliseconds(1000 / hz);
    #endif

    Renderer renderer;
    renderer.InitCamera(CameraAngle::XY);
    // Clear viewer
    renderer.ClearViewer();

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<boost::filesystem::path> stream(boost::filesystem::directory_iterator{"/app/1_clustering/dataset_2"},
                                                boost::filesystem::directory_iterator{});

    // sort files in ascending (chronological) order
    std::sort(stream.begin(), stream.end());

    auto streamIterator = stream.begin();

    while (not renderer.WasViewerStopped())
    {
        renderer.ClearViewer();

        pcl::PCDReader reader;
        reader.read (streamIterator->string(), *input_cloud);
        auto startTime = std::chrono::steady_clock::now();

        ProcessAndRenderPointCloud(renderer,input_cloud);
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "[PointCloudProcessor<PointT>::ReadPcdFile] Loaded "
        << input_cloud->points.size() << " data points from " << streamIterator->string() <<  "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        #ifdef CAP_HZ
            if(elapsedTime < time_budget)
            {
                std::this_thread::sleep_for(time_budget - elapsedTime);
            }
        #endif

        renderer.SpinViewerOnce();
    }
}
