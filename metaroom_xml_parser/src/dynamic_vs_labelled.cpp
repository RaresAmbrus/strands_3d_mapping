#include "metaroom_xml_parser/load_utilities.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef pcl::search::KdTree<PointType> Tree;

using namespace std;

double getMatchAccuracy(CloudPtr object, CloudPtr cluster){

    // check if it's a match
    CloudPtr difference(new Cloud());
    pcl::SegmentDifferences<PointType> segment;
    segment.setInputCloud(object);
    segment.setTargetCloud(cluster);
    segment.setDistanceThreshold(0.001);
    typename Tree::Ptr tree (new pcl::search::KdTree<PointType>);
    tree->setInputCloud (cluster);
    segment.setSearchMethod(tree);
    segment.segment(*difference);

    bool matches;
    double accuracy;
    if (difference->points.size() > 0.9*object->points.size())
    {
        return -1.0;
    } else {
        double percentage1 = (double)(object->points.size() - difference->points.size())/ (double)object->points.size();
        segment.setInputCloud(cluster);
        segment.setTargetCloud(object);
        typename Tree::Ptr tree2 (new pcl::search::KdTree<PointType>);
        tree->setInputCloud (object);
        segment.setSearchMethod(tree2);
        segment.segment(*difference);
        double percentage2 = (double)( cluster->points.size() - difference->points.size() )/ (double)cluster->points.size();

        return (percentage1+percentage2)/2;
    }
}

vector<CloudPtr> clusterPointCloud(CloudPtr input_cloud, double tolerance, int min_cluster_size, int max_cluster_size)
{
    typename Tree::Ptr tree (new pcl::search::KdTree<PointType>);
    tree->setInputCloud (input_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance (tolerance);
    ec.setMinClusterSize (min_cluster_size);
    ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (input_cloud);
    ec.extract (cluster_indices);

    std::vector<CloudPtr> toRet;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        CloudPtr cloud_cluster (new Cloud());
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
            cloud_cluster->points.push_back (input_cloud->points[*pit]); //*
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;

        toRet.push_back(cloud_cluster);

    }

    return toRet;
}

void findLabels(vector<CloudPtr> input_segmented_dynamics, semantic_map_load_utilties::LabelledData<PointType> labelled_clusters,
                vector<pair<CloudPtr, string>>& labelled_segmented_dynamics, vector<CloudPtr>& crap_dynamics){
    for (CloudPtr segmented_dynamic : input_segmented_dynamics){
        bool found = false;
        for (size_t i=0; i<labelled_clusters.objectClouds.size();i++){
            double accuracy = getMatchAccuracy(segmented_dynamic, labelled_clusters.objectClouds[i]);
            if (accuracy != -1){
                labelled_segmented_dynamics.push_back(make_pair(segmented_dynamic,labelled_clusters.objectLabels[i] ));
                found =true;
                break;
            }
        }

        if (!found){
              crap_dynamics.push_back(segmented_dynamic);
        }
    }
}

int main(int argc, char** argv)
{
    string waypId = "WayPoint16"; // the only one for which there is labelled data
    bool visualize_all = true; // change this
    bool visualize_individual_clusters = true; // change this
    int min_cluster_size = 50;
    string dataPath = "";

    if (argc == 4)
    {
        dataPath = argv[1];
        waypId = argv[2];
        min_cluster_size = atoi(argv[3]);
    } else {
        cout<<"Usage: dynamic_vs_labelled path waypId min_cluster_size"<<endl;
        return -1;
    }

    pcl::visualization::PCLVisualizer *p = new pcl::visualization::PCLVisualizer (argc, argv, "Dynamic_vs_labelled");
    p->addCoordinateSystem();

    vector<string> matchingObservations = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointType>(dataPath, waypId);
    ROS_INFO_STREAM("Observation matches for waypoint "<<matchingObservations.size());

    for (size_t i=0; i<matchingObservations.size();i++)
    {
        // load labelled objects
        semantic_map_load_utilties::LabelledData<PointType> data = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointType>(matchingObservations[i], false);
        if (data.objectClouds.size() == 0) continue; // no labelled objects

        // load dynamic clusters
        auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(matchingObservations[i], std::vector<std::string>{"RoomDynamicClusters"},false);
        if (sweep.dynamicClusterCloud->points.size() == 0) continue; // no dynamic clusters detected
        CloudPtr dynamicClusters = sweep.dynamicClusterCloud;

        // Transform to the map frame of reference:
        static tf::StampedTransform world_transform = data.transformToGlobal;
        pcl_ros::transformPointCloud(*data.completeCloud, *data.completeCloud,world_transform);
        pcl_ros::transformPointCloud(*dynamicClusters, *dynamicClusters,world_transform);
        for (auto object: data.objectClouds)
        {
            pcl_ros::transformPointCloud(*object, *object,world_transform);
        }

        // Cluster segmented dynamics into individual clusters
        vector<CloudPtr> segmented_dynamic_clusters = clusterPointCloud(dynamicClusters,0.05, min_cluster_size,50000);

        // separate segmented dynamics into labelled segmented and false positives
        vector<CloudPtr> false_positives;
        vector<pair<CloudPtr, string>> labelled_segmented_dynamics;
        findLabels(segmented_dynamic_clusters, data, labelled_segmented_dynamics, false_positives);


        cout<<"Now looking at "<<matchingObservations[i]<<"  acquisition time "<<data.sweepTime<<"   labelled objects  "<<data.objectClouds.size()<<endl;
        if (visualize_all) // visualize the whole clouds
        {
            CloudPtr all_labelled(new Cloud());
            CloudPtr all_segmented_labelled(new Cloud());
            CloudPtr all_false_positives(new Cloud());
            for (CloudPtr cloud : data.objectClouds){
                *all_labelled += *cloud;
            }

            for (auto cloud_with_label : labelled_segmented_dynamics){
                *all_segmented_labelled += *(cloud_with_label.first);
            }

            for (CloudPtr cloud : false_positives){
                *all_false_positives += *cloud;
            }


            pcl::visualization::PointCloudColorHandlerCustom<PointType> all_labelled_handler (all_labelled, 255, 0, 0);
            pcl::visualization::PointCloudColorHandlerCustom<PointType> all_segmented_labelled_handler (all_segmented_labelled, 255, 0, 0);
            pcl::visualization::PointCloudColorHandlerCustom<PointType> all_false_positives_handler (all_false_positives, 0, 0, 255);

            p->addPointCloud (all_segmented_labelled,all_segmented_labelled_handler,"all_segmented_labelled");
            p->addPointCloud (all_false_positives,all_false_positives_handler,"all_false_positives");
            p->addPointCloud (all_labelled,"all_labelled");

            p->spin();
            p->removeAllPointClouds();
        }

        if (visualize_individual_clusters) // visualize individual clouds
        {
            for ( size_t j=0; j<labelled_segmented_dynamics.size(); j++ )
            {
                CloudPtr object = labelled_segmented_dynamics[j].first;
                string label = labelled_segmented_dynamics[j].second;
                stringstream ss;ss<<"segmented"<<j;
                pcl::visualization::PointCloudColorHandlerCustom<PointType> segmented_labelled_handler (object, 255, 0, 0);
                p->addPointCloud(object,segmented_labelled_handler,ss.str());
                cout<<"Corresponding label for this segmented object is "<<label<<endl;

                // find matching labelled object
                for (size_t k=0; k<data.objectClouds.size();k++){
                    double accuracy = getMatchAccuracy(object, data.objectClouds[k]);
                    if (accuracy != -1){
                        stringstream ss2;ss2<<"labelled"<<k;
                        p->addPointCloud(data.objectClouds[k],ss2.str());
                        break;
                    }
                }

                p->spin();
                p->removeAllPointClouds();
            }


        }

    }
}
