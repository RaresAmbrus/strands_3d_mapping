#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vector>
#include <QDir>



#include <metaroom_xml_parser/load_utilities.h>
#include <object_manager/dynamic_object.h>
#include <object_manager/dynamic_object_xml_parser.h>

#include "labeller.h"


typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;


using namespace std;

int main(int argc, char** argv)
{
    string waypId = "";
    string path = "";
    string original_label = "";
    string new_label = "";
    if (argc == 5)
    {
        path = argv[1];
        waypId = argv[2];
        original_label = argv[3];
        new_label = argv[4];
    } else {
        cout<<"Please provide path and waypoint as argument"<<endl;
        return -1;
    }

    vector<string> matchingObservations = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointType>(path, waypId);
    ROS_INFO_STREAM("Observation matches for waypoint "<<matchingObservations.size());

    DynamicObjectXMLParser parser;
    sort(matchingObservations.begin(), matchingObservations.end());

    for (size_t i=0; i<matchingObservations.size();i++)
    {
        ROS_INFO_STREAM("Parsing "<<matchingObservations[i]);

        // find label image files in this folder
        unsigned found = matchingObservations[i].find_last_of("/");
        std::string base_path = matchingObservations[i].substr(0,found+1);
        QStringList xmlFiles = QDir(base_path.c_str()).entryList(QStringList("*label*.xml"));

        cout<<"Found "<<xmlFiles.size()<<" labelled objects."<<endl;

        for (size_t i=0; i<xmlFiles.size(); i++)
        {
            DynamicObject::Ptr parsed = parser.loadFromXML(base_path+xmlFiles[i].toStdString());
            if (parsed->m_label == original_label){
                unsigned found = (xmlFiles[i].toStdString()).find_last_of(".");
                std::string pcd_file = (xmlFiles[i].toStdString()).substr(0,found+1) + "pcd";
                parsed->m_label = new_label;

                DynamicObjectXMLParser temp_parser(base_path);
                temp_parser.saveAsXML(parsed, xmlFiles[i].toStdString(), pcd_file);

                // change label file as well
                string label_file =base_path+(xmlFiles[i].toStdString()).substr(0,found+1) + "txt";
                ofstream label_stream(label_file);
                label_stream<<new_label;
                label_stream.close();


                cout<<"Xml file: "<<xmlFiles[i].toStdString()<<" ||| pcd file: "<<pcd_file<<" ||| label: "<<label_file<<endl;
            }
        }
    }
}
