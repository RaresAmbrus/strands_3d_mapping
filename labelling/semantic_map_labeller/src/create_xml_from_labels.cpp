#include <vector>
#include <QDir>
#include <QFile>

#include <pcl/io/pcd_io.h>

#include <metaroom_xml_parser/load_utilities.h>
#include <object_manager/dynamic_object_xml_parser.h>
#include <semantic_map/room_xml_parser.h>

#include "labeller.h"

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;


using namespace std;

int main(int argc, char** argv)
{
   string waypId = "";
   string path ="";
   if (argc == 3)
   {
       path = argv[1];
      waypId = argv[2];
   } else {
      cout<<"Please provide path and waypoint as argument"<<endl;
      return -1;
   }


   vector<string> matchingObservations = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointType>(path, waypId);
   ROS_INFO_STREAM("Observation matches for waypoint "<<matchingObservations.size());


   sort(matchingObservations.begin(), matchingObservations.end());

   for (size_t i=0; i<matchingObservations.size();i++)
   {
      ROS_INFO_STREAM("Parsing "<<matchingObservations[i]);
      SemanticRoom<PointType> aRoom = SemanticRoomXMLParser<PointType>::loadRoomFromXML(matchingObservations[i],false);

      // find label image files in this folder
      unsigned found = matchingObservations[i].find_last_of("/");
      std::string base_path = matchingObservations[i].substr(0,found+1);
      QStringList imageFiles = QDir(base_path.c_str()).entryList(QStringList("*label*.jpg"));
      QStringList pcdFiles = QDir(base_path.c_str()).entryList(QStringList("*label*.pcd"));
      QStringList labelFiles = QDir(base_path.c_str()).entryList(QStringList("*label*.txt"));

      cout<<"Found "<<imageFiles.size()<<" labelled objects."<<endl;
      if ((imageFiles.size() != pcdFiles.size()) || (imageFiles.size() != labelFiles.size()))
      {
         cout<<"Folder "<<matchingObservations[i]<<" number of files doesn't correspond"<<endl;
         cout<<imageFiles.size()<<"  "<<pcdFiles.size()<<"  "<<labelFiles.size()<<endl;
         cout<<"------------------------------------------------------------------------"<<endl;
         return;
//         continue;
      }

      DynamicObjectXMLParser parser(base_path);
      for (size_t i=0; i<labelFiles.size(); i++)
      {
         DynamicObject::Ptr parsed = DynamicObject::Ptr (new DynamicObject());
         ifstream in(base_path+labelFiles[i].toStdString());
         string label;
         in >> label;
         cout<<"Object label "<<label<<" object file "<<base_path+labelFiles[i].toStdString()<<endl;

//         if (label != "water_boiler"){
//             continue;
//         }

         QFile file((base_path+pcdFiles[i].toStdString()).c_str());
         if (!file.exists())
         {
            cout<<"----------------- Label file "<<base_path+labelFiles[i].toStdString()<<" doesn't have a corresponding pcd file "<<endl;
            continue;
         }

         pcl::PCDReader reader;
         CloudPtr cloud (new Cloud);
         reader.read (base_path+pcdFiles[i].toStdString(), *cloud);
         parsed->setCloud(cloud);
         parsed->m_label = label;
         parsed->setTime(aRoom.getRoomLogStartTime());

         unsigned found2 = (labelFiles[i].toStdString()).find_last_of(".");
         std::string xml_file = (labelFiles[i].toStdString()).substr(0,found2+1) + "xml";

         cout<<"Saving dynamic object at "<<xml_file<<"  cloud size "<<cloud->points.size()<<endl;
         parser.saveAsXML(parsed, xml_file, pcdFiles[i].toStdString());
      }

   }

}
