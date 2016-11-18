#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <string.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "eigen_conversions/eigen_msg.h"
#include "tf_conversions/tf_eigen.h"

#include "metaroom_xml_parser/simple_xml_parser.h"
#include "metaroom_xml_parser/simple_summary_parser.h"

#include <tf_conversions/tf_eigen.h>

#include "ros/ros.h"
#include <metaroom_xml_parser/load_utilities.h>
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>

#include "metaroom_xml_parser/load_utilities.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>

#include <tf_conversions/tf_eigen.h>

#include "quasimodo_msgs/model.h"
#include "quasimodo_msgs/rgbd_frame.h"
#include "quasimodo_msgs/model_from_frame.h"
#include "quasimodo_msgs/index_frame.h"
#include "quasimodo_msgs/fuse_models.h"
#include "quasimodo_msgs/get_model.h"
#include "quasimodo_msgs/segment_model.h"
#include "quasimodo_msgs/metaroom_pair.h"

#include "ros/ros.h"
#include <quasimodo_msgs/query_cloud.h>
#include <quasimodo_msgs/visualize_query.h>
#include <metaroom_xml_parser/load_utilities.h>
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>

#include "metaroom_xml_parser/load_utilities.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>

#include "quasimodo_msgs/model.h"
#include "quasimodo_msgs/rgbd_frame.h"
#include "quasimodo_msgs/model_from_frame.h"
#include "quasimodo_msgs/index_frame.h"
#include "quasimodo_msgs/fuse_models.h"
#include "quasimodo_msgs/get_model.h"

#include "metaroom_xml_parser/simple_xml_parser.h"
#include "metaroom_xml_parser/load_utilities.h"

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>

#include "modelupdater/ModelUpdater.h"
#include "core/RGBDFrame.h"
#include "Util/Util.h"

// Services
#include <object_manager_msgs/DynamicObjectsService.h>
#include <object_manager_msgs/GetDynamicObjectService.h>
#include <object_manager_msgs/ProcessDynamicObjectService.h>

// Registration service
#include <observation_registration_services/ObjectAdditionalViewRegistrationService.h>

// Additional view mask service
#include <object_manager_msgs/DynamicObjectComputeMaskService.h>


#include <sys/time.h>
#include <sys/resource.h>

// Custom messages
#include "object_manager/dynamic_object.h"
#include "object_manager/dynamic_object_xml_parser.h"
#include "object_manager/dynamic_object_utilities.h"
#include "object_manager/dynamic_object_mongodb_interface.h"

#include <object_manager_msgs/DynamicObjectTracks.h>
#include <object_manager_msgs/DynamicObjectTrackingData.h>


#include <semantic_map_msgs/RoomObservation.h>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#include <quasimodo_conversions/conversions.h>

#include <soma_llsd/GetScene.h>
#include <soma_llsd/InsertScene.h>


std::vector< ros::ServiceServer > m_DynamicObjectsServiceServers;
std::vector< ros::ServiceServer > m_GetDynamicObjectServiceServers;

typedef typename object_manager_msgs::DynamicObjectsService::Request DynamicObjectsServiceRequest;
typedef typename object_manager_msgs::DynamicObjectsService::Response DynamicObjectsServiceResponse;

typedef typename object_manager_msgs::GetDynamicObjectService::Request GetDynamicObjectServiceRequest;
typedef typename object_manager_msgs::GetDynamicObjectService::Response GetDynamicObjectServiceResponse;

typedef typename object_manager_msgs::ProcessDynamicObjectService::Request ProcessDynamicObjectServiceRequest;
typedef typename object_manager_msgs::ProcessDynamicObjectService::Response ProcessDynamicObjectServiceResponse;

ros::ServiceClient segmentation_client;
using namespace std;
using namespace semantic_map_load_utilties;

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef pcl::search::KdTree<PointType> Tree;
typedef semantic_map_load_utilties::DynamicObjectData<PointType> ObjectData;

using namespace std;
using namespace semantic_map_load_utilties;

std::string overall_folder = "";
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
int visualization_lvl			= 0;
int visualization_lvl_regref	= 0;
int visualization_lvl_regini	= 0;
std::string outtopic			= "";
std::string modelouttopic		= "";
ros::Publisher out_pub;
ros::Publisher model_pub;

std::vector< ros::Publisher > m_PublisherStatuss;
std::vector< ros::Publisher > out_pubs;
std::vector< ros::Publisher > model_pubs;
ros::Publisher roomObservationCallback_pubs;

std::string saveVisuals = "";

std::string posepath = "testposes.xml";
std::vector<Eigen::Matrix4d> sweepPoses;
reglib::Camera * basecam;

ros::NodeHandle * np;

bool recomputeRelativePoses = false;
bool do_last_and_send = true;


void sendMetaroomToServer(std::string path);
bool testDynamicObjectServiceCallback(std::string path);
bool dynamicObjectsServiceCallback(DynamicObjectsServiceRequest &req, DynamicObjectsServiceResponse &res);
void processSweepForDatabase(std::string path, std::string savePath);

void signal_callback_handler(int signum){
	printf("Caught signal %d\n",signum);
	exit(signum);
}

std::string replaceAll(std::string str, std::string from, std::string to){
	std::size_t found = str.find(from);
	while(found!=std::string::npos){
		str.replace(found,from.size(),to);
		found = str.find(from);
	}
	return str;
}


void remove_old_seg(std::string sweep_folder){
	printf("remove_old_seg: %s\n",sweep_folder.c_str());
	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir (sweep_folder.c_str())) != NULL) {
		/* print all the files and directories within directory */
		while ((ent = readdir (dir)) != NULL) {
			std::string file = std::string(ent->d_name);

			if (file.find("dynamic_obj") !=std::string::npos && (file.find(".xml") !=std::string::npos || file.find(".pcd") !=std::string::npos)){
				printf ("removing %s\n", ent->d_name);
				std::remove((sweep_folder+"/"+file).c_str());
			}

			if (file.find("dynamicmask") !=std::string::npos && file.find(".png") !=std::string::npos){
				printf ("removing %s\n", ent->d_name);
				std::remove((sweep_folder+"/"+file).c_str());
			}

			if (file.find("moving_obj") !=std::string::npos && (file.find(".xml") !=std::string::npos || file.find(".pcd") !=std::string::npos)){
				printf ("removing %s\n", ent->d_name);
				std::remove((sweep_folder+"/"+file).c_str());
			}

			if (file.find("movingmask") !=std::string::npos && file.find(".png") !=std::string::npos){
				printf ("removing %s\n", ent->d_name);
				std::remove((sweep_folder+"/"+file).c_str());
			}

			//			if (file.find("_object_") !=std::string::npos && file.find(".xml") !=std::string::npos){
			//				printf ("removing %s\n", ent->d_name);
			//				std::remove((sweep_folder+"/"+file).c_str());
			//			}
			//			if (file.find("_object_") !=std::string::npos && file.find(".pcd") !=std::string::npos){
			//				printf ("maybe: removing %s\n", ent->d_name);
			//				printf("%i\n",file.find("_additional_view_") == std::string::npos);
			//				//file.find("_additional_view_") == std::string::npos;
			//				//std::remove((sweep_folder+"/"+file).c_str());
			//			}
		}
		closedir (dir);
	}

	printf("done remove_old_seg\n");
}

void writePose(QXmlStreamWriter* xmlWriter, Eigen::Matrix4d pose){
	Eigen::Quaterniond q = Eigen::Quaterniond ((Eigen::Affine3d (pose)).rotation());

	xmlWriter->writeStartElement("Translation");
	xmlWriter->writeStartElement("x");
	xmlWriter->writeCharacters(QString::number(pose(0,3)));
	xmlWriter->writeEndElement();
	xmlWriter->writeStartElement("y");
	xmlWriter->writeCharacters(QString::number(pose(1,3)));
	xmlWriter->writeEndElement();
	xmlWriter->writeStartElement("z");
	xmlWriter->writeCharacters(QString::number(pose(2,3)));
	xmlWriter->writeEndElement();
	xmlWriter->writeEndElement(); // Translation

	xmlWriter->writeStartElement("Rotation");
	xmlWriter->writeStartElement("w");
	xmlWriter->writeCharacters(QString::number(q.w()));
	xmlWriter->writeEndElement();
	xmlWriter->writeStartElement("x");
	xmlWriter->writeCharacters(QString::number(q.x()));
	xmlWriter->writeEndElement();
	xmlWriter->writeStartElement("y");
	xmlWriter->writeCharacters(QString::number(q.y()));
	xmlWriter->writeEndElement();
	xmlWriter->writeStartElement("z");
	xmlWriter->writeCharacters(QString::number(q.z()));
	xmlWriter->writeEndElement();
	xmlWriter->writeEndElement(); //Rotation
}

void writeXml(std::string xmlFile, std::vector<reglib::RGBDFrame *> & frames, std::vector<Eigen::Matrix4d> & poses){
	int slash_pos = xmlFile.find_last_of("/");
	std::string sweep_folder = xmlFile.substr(0, slash_pos) + "/";

	QFile file(xmlFile.c_str());
	if (file.exists()){file.remove();}


	if (!file.open(QIODevice::ReadWrite | QIODevice::Text))
	{
		std::cerr<<"Could not open file "<<sweep_folder<<"additionalViews.xml to save views as XML"<<std::endl;
		return;
	}

	QXmlStreamWriter* xmlWriter = new QXmlStreamWriter();
	xmlWriter->setDevice(&file);

	xmlWriter->writeStartDocument();
	xmlWriter->writeStartElement("Views");
	xmlWriter->writeAttribute("number_of_views", QString::number(frames.size()));
	for(unsigned int i = 0; i < frames.size(); i++){
		reglib::RGBDFrame * frame = frames[i];
		char buf [1024];

		xmlWriter->writeStartElement("View");
		sprintf(buf,"%s/view_RGB%10.10i.png",sweep_folder.c_str(),i);
		cv::imwrite(buf, frame->rgb );

		sprintf(buf,"view_RGB%10.10i.png",i);
		xmlWriter->writeAttribute("RGB", QString(buf));


		sprintf(buf,"%s/view_DEPTH%10.10i.png",sweep_folder.c_str(),i);
		cv::imwrite(buf, frame->depth );
		sprintf(buf,"view_DEPTH%10.10i.png",i);
		xmlWriter->writeAttribute("DEPTH", QString(buf));

		long nsec = 1e9*((frame->capturetime)-std::floor(frame->capturetime));
		long sec = frame->capturetime;
		xmlWriter->writeStartElement("Stamp");

		xmlWriter->writeStartElement("sec");
		xmlWriter->writeCharacters(QString::number(sec));
		xmlWriter->writeEndElement();

		xmlWriter->writeStartElement("nsec");
		xmlWriter->writeCharacters(QString::number(nsec));
		xmlWriter->writeEndElement();

		xmlWriter->writeEndElement(); // Stamp

		xmlWriter->writeStartElement("Camera");

		xmlWriter->writeStartElement("fx");
		xmlWriter->writeCharacters(QString::number(frame->camera->fx));
		xmlWriter->writeEndElement();

		xmlWriter->writeStartElement("fy");
		xmlWriter->writeCharacters(QString::number(frame->camera->fy));
		xmlWriter->writeEndElement();

		xmlWriter->writeStartElement("cx");
		xmlWriter->writeCharacters(QString::number(frame->camera->cx));
		xmlWriter->writeEndElement();

		xmlWriter->writeStartElement("cy");
		xmlWriter->writeCharacters(QString::number(frame->camera->cy));
		xmlWriter->writeEndElement();

		xmlWriter->writeEndElement(); // camera

		xmlWriter->writeStartElement("RegisteredPose");
		writePose(xmlWriter,poses[i]);
		xmlWriter->writeEndElement();

		xmlWriter->writeStartElement("Pose");
		writePose(xmlWriter,frame->pose);
		xmlWriter->writeEndElement();

		xmlWriter->writeEndElement();
	}
	xmlWriter->writeEndElement(); // Semantic Room
	xmlWriter->writeEndDocument();
	delete xmlWriter;
}

Eigen::Matrix4d getPose(QXmlStreamReader * xmlReader){
	QXmlStreamReader::TokenType token = xmlReader->readNext();//Translation
	QString elementName = xmlReader->name().toString();

	token = xmlReader->readNext();//fx
	double tx = atof(xmlReader->readElementText().toStdString().c_str());

	token = xmlReader->readNext();//fy
	double ty = atof(xmlReader->readElementText().toStdString().c_str());

	token = xmlReader->readNext();//cx
	double tz = atof(xmlReader->readElementText().toStdString().c_str());

	token = xmlReader->readNext();//Translation
	elementName = xmlReader->name().toString();

	token = xmlReader->readNext();//Rotation
	elementName = xmlReader->name().toString();

	token = xmlReader->readNext();//qw
	double qw = atof(xmlReader->readElementText().toStdString().c_str());

	token = xmlReader->readNext();//qx
	double qx = atof(xmlReader->readElementText().toStdString().c_str());

	token = xmlReader->readNext();//qy
	double qy = atof(xmlReader->readElementText().toStdString().c_str());

	token = xmlReader->readNext();//qz
	double qz = atof(xmlReader->readElementText().toStdString().c_str());

	token = xmlReader->readNext();//Rotation
	elementName = xmlReader->name().toString();

	Eigen::Matrix4d regpose = (Eigen::Affine3d(Eigen::Quaterniond(qw,qx,qy,qz))).matrix();
	regpose(0,3) = tx;
	regpose(1,3) = ty;
	regpose(2,3) = tz;

	return regpose;
}

int readNumberOfViews(std::string xmlFile){
	QFile file(xmlFile.c_str());
	if (!file.exists()){return -1;}

	file.open(QIODevice::ReadOnly);
	QXmlStreamReader* xmlReader = new QXmlStreamReader(&file);


	int count = 0;
	while (!xmlReader->atEnd() && !xmlReader->hasError()){
		QXmlStreamReader::TokenType token = xmlReader->readNext();
		if (token == QXmlStreamReader::StartDocument)
			continue;

		if (xmlReader->hasError()){return -1;}

		if (token == QXmlStreamReader::StartElement){
			if (xmlReader->name() == "View"){
				count++;
			}
		}
	}
	delete xmlReader;
	return count;
}

void readViewXML(std::string xmlFile, std::vector<reglib::RGBDFrame *> & frames, std::vector<Eigen::Matrix4d> & poses, bool compute_edges = true, std::string savePath = ""){
	//printf("readViewXML: compute_edges: %i\n",compute_edges);
	QFile file(xmlFile.c_str());

	if (!file.exists())
	{
		ROS_ERROR("Could not open file %s to load room.",xmlFile.c_str());
		return;
	}

	QString xmlFileQS(xmlFile.c_str());
	int index = xmlFileQS.lastIndexOf('/');
	std::string roomFolder = xmlFileQS.left(index).toStdString();


	file.open(QIODevice::ReadOnly);
	ROS_INFO_STREAM("Parsing xml file: "<<xmlFile.c_str());

	QXmlStreamReader* xmlReader = new QXmlStreamReader(&file);


	while (!xmlReader->atEnd() && !xmlReader->hasError())
	{
		QXmlStreamReader::TokenType token = xmlReader->readNext();
		if (token == QXmlStreamReader::StartDocument)
			continue;

		if (xmlReader->hasError())
		{
			ROS_ERROR("XML error: %s",xmlReader->errorString().toStdString().c_str());
			return;
		}

		QString elementName = xmlReader->name().toString();

		if (token == QXmlStreamReader::StartElement)
		{

			if (xmlReader->name() == "View")
			{
				cv::Mat rgb;
				cv::Mat depth;
				//printf("elementName: %s\n",elementName.toStdString().c_str());
				QXmlStreamAttributes attributes = xmlReader->attributes();
				if (attributes.hasAttribute("RGB"))
				{
					std::string imgpath = attributes.value("RGB").toString().toStdString();
					printf("rgb filename: %s\n",(roomFolder+"/"+imgpath).c_str());
					rgb = cv::imread(roomFolder+"/"+imgpath, CV_LOAD_IMAGE_UNCHANGED);

					//QString rgbpath = attributes.value("RGB").toString();
					//rgb = cv::imread(roomFolder+"/"+(rgbpath.toStdString().c_str()), CV_LOAD_IMAGE_UNCHANGED);
					//rgb = cv::imread((rgbpath.toStdString()).c_str(), CV_LOAD_IMAGE_UNCHANGED);
				}else{break;}


				if (attributes.hasAttribute("DEPTH"))
				{
					std::string imgpath = attributes.value("DEPTH").toString().toStdString();
					printf("depth filename: %s\n",(roomFolder+"/"+imgpath).c_str());
					depth = cv::imread(roomFolder+"/"+imgpath, CV_LOAD_IMAGE_UNCHANGED);
					//QString depthpath = attributes.value("DEPTH").toString();
					//printf("depth filename: %s\n",depthpath.toStdString().c_str());
					//depth = cv::imread((roomFolder+"/"+depthpath.toStdString()).c_str(), CV_LOAD_IMAGE_UNCHANGED);
					//depth = cv::imread(roomFolder+"/"+(depthpath.toStdString().c_str()), CV_LOAD_IMAGE_UNCHANGED);
				}else{break;}


				token = xmlReader->readNext();//Stamp
				elementName = xmlReader->name().toString();

				token = xmlReader->readNext();//sec
				elementName = xmlReader->name().toString();
				int sec = atoi(xmlReader->readElementText().toStdString().c_str());

				token = xmlReader->readNext();//nsec
				elementName = xmlReader->name().toString();
				int nsec = atoi(xmlReader->readElementText().toStdString().c_str());
				token = xmlReader->readNext();//end stamp

				token = xmlReader->readNext();//Camera
				elementName = xmlReader->name().toString();

				reglib::Camera * cam = new reglib::Camera();

				token = xmlReader->readNext();//fx
				cam->fx = atof(xmlReader->readElementText().toStdString().c_str());

				token = xmlReader->readNext();//fy
				cam->fy = atof(xmlReader->readElementText().toStdString().c_str());

				token = xmlReader->readNext();//cx
				cam->cx = atof(xmlReader->readElementText().toStdString().c_str());

				token = xmlReader->readNext();//cy
				cam->cy = atof(xmlReader->readElementText().toStdString().c_str());

				token = xmlReader->readNext();//Camera
				elementName = xmlReader->name().toString();

				double time = double(sec)+double(nsec)/double(1e9);

				token = xmlReader->readNext();//RegisteredPose
				elementName = xmlReader->name().toString();

				Eigen::Matrix4d regpose = getPose(xmlReader);

				token = xmlReader->readNext();//RegisteredPose
				elementName = xmlReader->name().toString();


				token = xmlReader->readNext();//Pose
				elementName = xmlReader->name().toString();

				Eigen::Matrix4d pose = getPose(xmlReader);

				token = xmlReader->readNext();//Pose
				elementName = xmlReader->name().toString();

				reglib::RGBDFrame * frame = new reglib::RGBDFrame(cam,rgb,depth, time, regpose,true,savePath,compute_edges);
				frames.push_back(frame);
				poses.push_back(pose);
			}
		}
	}
	delete xmlReader;
}


void setBaseSweep(std::string path){
	printf("setBaseSweep(%s)\n",path.c_str());
	SimpleXMLParser<pcl::PointXYZRGB> parser;
	SimpleXMLParser<pcl::PointXYZRGB>::RoomData roomData  = parser.loadRoomFromXML(path);//, std::vector<std::string>(),false,false);
	printf("loaded room data\n");
	image_geometry::PinholeCameraModel baseCameraModel = roomData.vIntermediateRoomCloudCamParamsCorrected.front();
	if(basecam != 0){delete basecam;}
	basecam		= new reglib::Camera();
	basecam->fx = baseCameraModel.fx();
	basecam->fy = baseCameraModel.fy();
	basecam->cx = baseCameraModel.cx();
	basecam->cy = baseCameraModel.cy();

	sweepPoses.clear();
	for (size_t i=0; i<roomData.vIntermediateRoomCloudTransformsRegistered.size(); i++){
		sweepPoses.push_back(quasimodo_brain::getMat(roomData.vIntermediateRoomCloudTransformsRegistered[i]));
		std::cout << "sweepPoses" << i << std::endl;
		std::cout << sweepPoses.back() << std::endl;
	}
}

reglib::Model * processAV(std::string path, bool compute_edges = true, std::string savePath = ""){
	printf("processAV: %s\n",path.c_str());

	int slash_pos = path.find_last_of("/");
	std::string sweep_folder = path.substr(0, slash_pos) + "/";

	std::vector<cv::Mat> viewrgbs;
	std::vector<cv::Mat> viewdepths;
	std::vector<tf::StampedTransform > viewtfs;

	QStringList objectFiles = QDir(sweep_folder.c_str()).entryList(QStringList("*object*.xml"));
	for (auto objectFile : objectFiles){
		auto object = loadDynamicObjectFromSingleSweep<PointType>(sweep_folder+objectFile.toStdString(),false);
		for (unsigned int i=0; i<object.vAdditionalViews.size(); i++){
			CloudPtr cloud = object.vAdditionalViews[i];

			cv::Mat rgb;
			rgb.create(cloud->height,cloud->width,CV_8UC3);
			unsigned char * rgbdata = (unsigned char *)rgb.data;

			cv::Mat depth;
			depth.create(cloud->height,cloud->width,CV_16UC1);
			unsigned short * depthdata = (unsigned short *)depth.data;

			unsigned int nr_data = cloud->height * cloud->width;
			for(unsigned int j = 0; j < nr_data; j++){
				PointType p = cloud->points[j];
				rgbdata[3*j+0]	= p.b;
				rgbdata[3*j+1]	= p.g;
				rgbdata[3*j+2]	= p.r;
				depthdata[j]	= short(5000.0 * p.z);
			}

			viewrgbs.push_back(rgb);
			viewdepths.push_back(depth);
			viewtfs.push_back(object.vAdditionalViewsTransforms[i]);
		}
	}

	reglib::Model * sweep = quasimodo_brain::load_metaroom_model(path,savePath);
	if(sweep->frames.size() == 0){
		printf("no frames in sweep, returning\n");
		return sweep;
	}

	for(unsigned int i = 0; (i < sweep->frames.size()) && (sweep->frames.size() == sweepPoses.size()) ; i++){
		sweep->frames[i]->pose	= sweep->frames.front()->pose * sweepPoses[i];
		sweep->relativeposes[i] = sweepPoses[i];
		if(basecam != 0){
			delete sweep->frames[i]->camera;
			sweep->frames[i]->camera = basecam->clone();
		}
	}

	std::vector<reglib::RGBDFrame *> frames;
	std::vector<reglib::ModelMask *> masks;
	std::vector<Eigen::Matrix4d> unrefined;

	std::vector<Eigen::Matrix4d> both_unrefined;
	both_unrefined.push_back(Eigen::Matrix4d::Identity());
	std::vector<double> times;
	for(unsigned int i = 0; i < 3000 &&  i < viewrgbs.size(); i++){
		printf("additional view: %i\n",i);
		geometry_msgs::TransformStamped msg;
		tf::transformStampedTFToMsg(viewtfs[i], msg);
		long sec = msg.header.stamp.sec;
		long nsec = msg.header.stamp.nsec;
		double time = double(sec)+1e-9*double(nsec);

		Eigen::Matrix4d m = quasimodo_brain::getMat(viewtfs[i]);

		cout << m << endl << endl;

		unrefined.push_back(m);
		times.push_back(time);

		cv::Mat fullmask;
		fullmask.create(480,640,CV_8UC1);
		unsigned char * maskdata = (unsigned char *)fullmask.data;
		for(int j = 0; j < 480*640; j++){maskdata[j] = 255;}
		masks.push_back(new reglib::ModelMask(fullmask));

		reglib::Camera * cam		= sweep->frames.front()->camera->clone();
		reglib::RGBDFrame * frame	= new reglib::RGBDFrame(cam,viewrgbs[i],viewdepths[i],time, m,true,savePath);//a.matrix());
		frames.push_back(frame);

		both_unrefined.push_back(sweep->frames.front()->pose.inverse()*m);
	}
//printf("%s::%i\n",__PRETTY_FUNCTION__,__LINE__);
	reglib::RegistrationRandom *	reg	= new reglib::RegistrationRandom();
	reglib::ModelUpdaterBasicFuse * mu	= new reglib::ModelUpdaterBasicFuse( sweep, reg);
	mu->occlusion_penalty               = 15;
	mu->massreg_timeout                 = 60*4;
	mu->viewer							= viewer;

	sweep->recomputeModelPoints();
//printf("%s::%i\n",__PRETTY_FUNCTION__,__LINE__);
	reglib::Model * fullmodel = new reglib::Model();
	fullmodel->savePath = savePath+"/";
	fullmodel->frames = sweep->frames;
	fullmodel->relativeposes = sweep->relativeposes;
	fullmodel->modelmasks = sweep->modelmasks;
//printf("%s::%i\n",__PRETTY_FUNCTION__,__LINE__);
	if(frames.size() > 0){

		reglib::RegistrationRefinement * refinement = new reglib::RegistrationRefinement();
		refinement->viewer	= viewer;
		refinement->visualizationLvl	= visualization_lvl_regini;
		refinement->normalize_matchweights = false;
		refinement->target_points = 4000;
		////double register_setup_start = getTime();

		//printf("%s::%i\n",__PRETTY_FUNCTION__,__LINE__);
		reglib::CloudData * cd1 = sweep->getCD(sweep->points.size());
		refinement->setDst(cd1);

		fullmodel->points = frames.front()->getSuperPoints(Eigen::Matrix4d::Identity(),1,false);
		reglib::CloudData * cd2	= fullmodel->getCD(fullmodel->points.size());
		refinement->setSrc(cd2);
		//printf("%s::%i\n",__PRETTY_FUNCTION__,__LINE__);
		//////printf("register_setup_start:          %5.5f\n",getTime()-register_setup_start);
		////double register_compute_start = getTime();
		Eigen::Matrix4d guess = both_unrefined[1]*both_unrefined[0].inverse();

		reglib::FusionResults fr = refinement->getTransform(guess);
		Eigen::Matrix4d offset = fr.guess*guess.inverse();
		for(unsigned int i = 1; i < both_unrefined.size(); i++){
			both_unrefined[i] = offset*both_unrefined[i];
		}

		delete refinement;

		fullmodel->points.clear();
		delete cd1;
		delete cd2;

		reglib::MassRegistrationPPR2 * bgmassreg = new reglib::MassRegistrationPPR2(0.25);
		if(savePath.size() != 0){
			bgmassreg->savePath = savePath+"/processAV_"+std::to_string(fullmodel->id);
		}
		//printf("%s::%i\n",__PRETTY_FUNCTION__,__LINE__);

		bgmassreg->timeout = 300;
		bgmassreg->viewer = viewer;
		bgmassreg->use_surface = true;
		bgmassreg->use_depthedge = false;
		bgmassreg->visualizationLvl = visualization_lvl_regref;
		bgmassreg->maskstep = 5;
		bgmassreg->nomaskstep = 5;
		bgmassreg->nomask = true;
		bgmassreg->stopval = 0.0005;
		bgmassreg->addModel(sweep);
		bgmassreg->setData(frames,masks);
//		exit(0);
		//printf("%s::%i\n",__PRETTY_FUNCTION__,__LINE__);

		reglib::MassFusionResults bgmfr = bgmassreg->getTransforms(both_unrefined);

		delete bgmassreg;

		for(unsigned int i = 0; i < frames.size(); i++){frames[i]->pose = sweep->frames.front()->pose * bgmfr.poses[i+1];}

		for(unsigned int i = 0; i < frames.size(); i++){
			fullmodel->frames.push_back(frames[i]);
			fullmodel->modelmasks.push_back(masks[i]);
			fullmodel->relativeposes.push_back(bgmfr.poses[i+1]);
		}
		fullmodel->recomputeModelPoints();

		if(savePath.size() != 0){
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cld = fullmodel->getPCLnormalcloud(1, false);
			pcl::io::savePCDFileBinaryCompressed (savePath+"/processAV_"+std::to_string(fullmodel->id)+"_fused.pcd", *cld);
		}
		//pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Model::getPCLnormalcloud(int step, bool color){

	}else{
		fullmodel->points = sweep->points;
	}

	delete reg;
	delete mu;
	delete sweep;

	return fullmodel;
}

void savePoses(std::string xmlFile, std::vector<Eigen::Matrix4d> poses, int maxposes = -1){
	QFile file(xmlFile.c_str());
	if (file.exists()){file.remove();}

	if (!file.open(QIODevice::ReadWrite | QIODevice::Text)){
		std::cerr<<"Could not open file "<< xmlFile <<" to save views as XML"<<std::endl;
		return;
	}

	QXmlStreamWriter* xmlWriter = new QXmlStreamWriter();
	xmlWriter->setDevice(&file);

	xmlWriter->writeStartDocument();
	xmlWriter->writeStartElement("Poses");
	//	xmlWriter->writeAttribute("number_of_poses", QString::number(poses.size()));
	for(unsigned int i = 0; i < poses.size() && (maxposes == -1 || int(i) < maxposes); i++){
		printf("saving %i\n",i);
		xmlWriter->writeStartElement("Pose");
		writePose(xmlWriter,poses[i]);
		xmlWriter->writeEndElement();
	}
	xmlWriter->writeEndElement(); // Poses
	xmlWriter->writeEndDocument();
	delete xmlWriter;
}

std::vector<Eigen::Matrix4d> readPoseXML(std::string xmlFile){
	std::vector<Eigen::Matrix4d> poses;
	QFile file(xmlFile.c_str());

	if (!file.exists()){
		ROS_ERROR("Could not open file %s to load poses.",xmlFile.c_str());
		return poses;
	}

	file.open(QIODevice::ReadOnly);
	ROS_INFO_STREAM("Parsing xml file: "<<xmlFile.c_str());

	QXmlStreamReader* xmlReader = new QXmlStreamReader(&file);

	while (!xmlReader->atEnd() && !xmlReader->hasError()){
		QXmlStreamReader::TokenType token = xmlReader->readNext();
		if (token == QXmlStreamReader::StartDocument)
			continue;

		if (xmlReader->hasError()){
			ROS_ERROR("XML error: %s",xmlReader->errorString().toStdString().c_str());
			return poses;
		}

		QString elementName = xmlReader->name().toString();

		if (token == QXmlStreamReader::StartElement){
			if (xmlReader->name() == "Pose"){
				Eigen::Matrix4d pose = getPose(xmlReader);

				token = xmlReader->readNext();//Pose
				elementName = xmlReader->name().toString();

				std::cout << pose << std::endl << std::endl;
				poses.push_back(pose);
			}
		}
	}
	delete xmlReader;
	printf("done readPoseXML\n");
	return poses;
}

reglib::Model * getAVMetaroom(std::string path, bool compute_edges = true, std::string saveVisuals_sp = ""){
	printf("processing: %s\n",path.c_str());

	if ( ! boost::filesystem::exists( path ) ){return 0;}

	int slash_pos = path.find_last_of("/");
	std::string sweep_folder = path.substr(0, slash_pos) + "/";

	int viewgroup_nrviews = readNumberOfViews(sweep_folder+"ViewGroup.xml");

	printf("sweep_folder: %s\n",sweep_folder.c_str());

	int additional_nrviews = 0;
	QStringList objectFiles = QDir(sweep_folder.c_str()).entryList(QStringList("*object*.xml"));
	for (auto objectFile : objectFiles){
		printf("objectFile: %s\n",(sweep_folder+objectFile.toStdString()).c_str());
		auto object = loadDynamicObjectFromSingleSweep<PointType>(sweep_folder+objectFile.toStdString(),false);
		additional_nrviews += object.vAdditionalViews.size();
	}

	SimpleXMLParser<pcl::PointXYZRGB> parser;
	SimpleXMLParser<pcl::PointXYZRGB>::RoomData current_roomData  = parser.loadRoomFromXML(path,std::vector<std::string>{"RoomIntermediateCloud","IntermediatePosition"});
	int metaroom_nrviews = current_roomData.vIntermediateRoomClouds.size();


	printf("viewgroup_nrviews: %i\n",viewgroup_nrviews);
	printf("additional_nrviews: %i\n",additional_nrviews);
	printf("metaroom_nrviews: %i\n",metaroom_nrviews);

	reglib::Model * fullmodel;
	if(viewgroup_nrviews == (additional_nrviews+metaroom_nrviews) && !recomputeRelativePoses){
		printf("time to read old\n");
		fullmodel = new reglib::Model();
		fullmodel->savePath = saveVisuals_sp+"/";

		for(unsigned int i = 0; i < viewgroup_nrviews; i++){
			cv::Mat fullmask;
			fullmask.create(480,640,CV_8UC1);
			unsigned char * maskdata = (unsigned char *)fullmask.data;
			for(int j = 0; j < 480*640; j++){maskdata[j] = 255;}
			fullmodel->modelmasks.push_back(new reglib::ModelMask(fullmask));
		}

		readViewXML(sweep_folder+"ViewGroup.xml",fullmodel->frames,fullmodel->relativeposes,compute_edges,saveVisuals_sp);
		fullmodel->recomputeModelPoints();
	}else{
		//printf("%s::%i\n",__PRETTY_FUNCTION__,__LINE__);
		fullmodel = processAV(path,compute_edges,saveVisuals_sp);
		//printf("%s::%i\n",__PRETTY_FUNCTION__,__LINE__);
		writeXml(sweep_folder+"ViewGroup.xml",fullmodel->frames,fullmodel->relativeposes);
	}
	//printf("%s::%i\n",__PRETTY_FUNCTION__,__LINE__);

	return fullmodel;
}

int totalcounter = 0;


int processMetaroom(CloudPtr dyncloud, std::string path, bool store_old_xml = true){
	path = replaceAll(path, "//", "/");

	quasimodo_brain::cleanPath(path);
	int returnval = 0;
	printf("processMetaroom: %s\n",path.c_str());

	int slash_pos = path.find_last_of("/");
	std::string sweep_folder = path.substr(0, slash_pos) + "/";

	if ( ! boost::filesystem::exists( path ) ){return 0;}

	QStringList objectFiles = QDir(sweep_folder.c_str()).entryList(QStringList("*object*.xml"));
	store_old_xml = objectFiles.size() == 0;

	SimpleXMLParser<pcl::PointXYZRGB> parser;

	SimpleXMLParser<pcl::PointXYZRGB>::RoomData current_roomData  = parser.loadRoomFromXML(path,std::vector<std::string>());
//	if(current_roomData.roomWaypointId.compare("ReceptionDesk") != 0){return 0;}

	current_roomData  = parser.loadRoomFromXML(path,std::vector<std::string>{"RoomIntermediateCloud","IntermediatePosition"});
	if(current_roomData.vIntermediateRoomClouds.size() != 17){return 0;}



	reglib::Model * fullmodel = getAVMetaroom(path,true,saveVisuals);

	reglib::RegistrationRandom *	reg	= new reglib::RegistrationRandom();
	reglib::ModelUpdaterBasicFuse * mu	= new reglib::ModelUpdaterBasicFuse( fullmodel, reg);
	mu->occlusion_penalty               = 15;
	mu->massreg_timeout                 = 60*4;
	mu->viewer							= viewer;

	std::vector<Eigen::Matrix4d> po;
	std::vector<reglib::RGBDFrame*> fr;
	std::vector<reglib::ModelMask*> mm;
	fullmodel->getData(po, fr, mm);

	if(fr.size() == 0){
		printf("no frames in model, returning\n");
		fullmodel->fullDelete();
		delete fullmodel;
		return 0;
	}

	DynamicObjectXMLParser objectparser(sweep_folder, true);

	std::string current_waypointid = current_roomData.roomWaypointId;

	if(overall_folder.back() == '/'){overall_folder.pop_back();}

	int prevind = -1;
	std::vector<std::string> sweep_xmls = semantic_map_load_utilties::getSweepXmls<pcl::PointXYZRGB>(overall_folder);
	for (unsigned int i = 0; i < sweep_xmls.size(); i++){
		sweep_xmls[i] = replaceAll(sweep_xmls[i], "//", "/");
		SimpleXMLParser<pcl::PointXYZRGB>::RoomData other_roomData  = parser.loadRoomFromXML(sweep_xmls[i],std::vector<std::string>(),false,false);
		std::string other_waypointid = other_roomData.roomWaypointId;

		if(sweep_xmls[i].compare(path) == 0){break;}
		if(other_waypointid.compare(current_waypointid) == 0){prevind = i;}
	}

	printf("prevind: %i\n",prevind);
	printf("current: %s\n",path.c_str());

	int nextind = sweep_xmls.size();
	for (int i = int(sweep_xmls.size()-1); i >= 0 ; i--){
		if(i < 0){break;}
		SimpleXMLParser<pcl::PointXYZRGB>::RoomData other_roomData  = parser.loadRoomFromXML(sweep_xmls[i],std::vector<std::string>(),false,false);
		std::string other_waypointid = other_roomData.roomWaypointId;

		if(sweep_xmls[i].compare(path) == 0){break;}
		if(other_waypointid.compare(current_waypointid) == 0){nextind = i;}
	}

	std::vector< reglib::Model * > models;
	models.push_back(fullmodel);

	std::vector< std::vector< cv::Mat > > internal;
	std::vector< std::vector< cv::Mat > > external;
	std::vector< std::vector< cv::Mat > > dynamic;

	std::vector< reglib::Model * > bgs;
	if(prevind != -1){
		std::string prev = sweep_xmls[prevind];
		printf("prev: %s\n",prev.c_str());
		reglib::Model * bg = getAVMetaroom(prev);
		if(bg->frames.size() == 0){
			printf("no frames in bg\n");
			bg->fullDelete();
			delete bg;
		}else{
			bgs.push_back(bg);
		}
	}else{
		printf("no previous...\n");
	}

	if(bgs.size() > 0){
		auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(path, std::vector<std::string>{},false);

		printf("models.front()->frames.size() = %i\n",models.front()->frames.size());
		quasimodo_brain::segment(bgs,models,internal,external,dynamic,visualization_lvl,saveVisuals);

		remove_old_seg(sweep_folder);

		if(models.size() == 0){	returnval = 2;}
		else{					returnval = 3;}

		for(unsigned int i = 0; i < models.size(); i++){
			printf("processing model %i\n",i);
			std::vector<cv::Mat> internal_masks = internal[i];
			std::vector<cv::Mat> external_masks = external[i];
			std::vector<cv::Mat> dynamic_masks	= dynamic[i];
			reglib::Model * model = models[i];

			std::vector<Eigen::Matrix4d> mod_po;
			std::vector<reglib::RGBDFrame*> mod_fr;
			std::vector<reglib::ModelMask*> mod_mm;
			model->getData(mod_po, mod_fr, mod_mm);

			int dynamicCounter = 1;
			while(true){
				double sum = 0;
				double sumx = 0;
				double sumy = 0;
				double sumz = 0;
				std::vector<int> imgnr;
				std::vector<cv::Mat> masks;

				CloudPtr cloud_cluster (new Cloud());

				Eigen::Matrix4d first = model->frames.front()->pose;

				for(unsigned int j = 0; j < mod_fr.size(); j++){
					reglib::RGBDFrame * frame = mod_fr[j];
					//std::cout << first*mod_po[j] << std::endl;
					Eigen::Matrix4d p = frame->pose;//first*mod_po[j];//*bg->frames.front()->pose;
					unsigned char  * rgbdata		= (unsigned char	*)(frame->rgb.data);
					unsigned short * depthdata		= (unsigned short	*)(frame->depth.data);
					float		   * normalsdata	= (float			*)(frame->normals.data);

					reglib::Camera * camera = frame->camera;

					unsigned char * internalmaskdata = (unsigned char *)(internal_masks[j].data);
					unsigned char * externalmaskdata = (unsigned char *)(external_masks[j].data);
					unsigned char * dynamicmaskdata = (unsigned char *)(dynamic_masks[j].data);

					float m00 = p(0,0); float m01 = p(0,1); float m02 = p(0,2); float m03 = p(0,3);
					float m10 = p(1,0); float m11 = p(1,1); float m12 = p(1,2); float m13 = p(1,3);
					float m20 = p(2,0); float m21 = p(2,1); float m22 = p(2,2); float m23 = p(2,3);

					const float idepth			= camera->idepth_scale;
					const float cx				= camera->cx;
					const float cy				= camera->cy;
					const float ifx				= 1.0/camera->fx;
					const float ify				= 1.0/camera->fy;
					const unsigned int width	= camera->width;
					const unsigned int height	= camera->height;

					cv::Mat mask;
					mask.create(height,width,CV_8UC1);
					unsigned char * maskdata = (unsigned char *)(mask.data);


					bool containsData = false;
					for(unsigned int w = 0; w < width;w++){
						for(unsigned int h = 0; h < height;h++){
							int ind = h*width+w;

							if(dynamicmaskdata[ind] == dynamicCounter){
								maskdata[ind] = 255;
								containsData = true;
								float z = idepth*float(depthdata[ind]);
								if(z > 0){
									float x = (float(w) - cx) * z * ifx;
									float y = (float(h) - cy) * z * ify;

									PointType p;
									p.x += m00*x + m01*y + m02*z + m03;
									p.y += m10*x + m11*y + m12*z + m13;
									p.z += m20*x + m21*y + m22*z + m23;
									p.r  = rgbdata[3*ind+2];
									p.g  = rgbdata[3*ind+1];
									p.b  = rgbdata[3*ind+0];
									cloud_cluster->points.push_back (p);

									sumx += p.x;
									sumy += p.y;
									sumz += p.z;
									sum ++;
								}
							}else{
								maskdata[ind] = 0;
							}
						}
					}

					if(containsData){
						masks.push_back(mask);
						imgnr.push_back(j);
					}
				}

				cloud_cluster->width = cloud_cluster->points.size ();
				cloud_cluster->height = 1;
				cloud_cluster->is_dense = true;

				if(masks.size() > 0){


					if(store_old_xml){
						std::stringstream ss;
						ss << "object_";
						ss << (dynamicCounter-1);

						// create and save dynamic object
						DynamicObject::Ptr roomObject(new DynamicObject());
						roomObject->setCloud(cloud_cluster);
						roomObject->setTime(sweep.roomLogStartTime);
						roomObject->m_roomLogString = sweep.roomLogName;
						roomObject->m_roomStringId = sweep.roomWaypointId;
						roomObject->m_roomRunNumber = sweep.roomRunNumber;
						//				        // create label from room log time; could be useful later on, and would resolve ambiguities
						std::stringstream ss_obj;
						ss_obj<<boost::posix_time::to_simple_string(sweep.roomLogStartTime);
						ss_obj<<"_object_";ss_obj<<(dynamicCounter-1);
						std::string tmp = ss_obj.str();
						printf("ss_obj.str(): %s\n",tmp.c_str());
						//roomObject->m_label = tmp;
						roomObject->setLabel(tmp);
						std::string xml_file = objectparser.saveAsXML(roomObject);
						printf("xml_file: %s\n",xml_file.c_str());
					}

					char buf [1024];
					sprintf(buf,"%s/dynamic_obj%10.10i.pcd",sweep_folder.c_str(),dynamicCounter-1);
					pcl::io::savePCDFileBinaryCompressed(std::string(buf),*cloud_cluster);

					*dyncloud += *cloud_cluster;


					//					std::string objectpcd = std::string(buf);//object.substr(0,object.size()-4);
					//					std::cout << objectpcd.substr(0,objectpcd.size()-4) << std::endl;

					sprintf(buf,"%s/dynamic_obj%10.10i.xml",sweep_folder.c_str(),dynamicCounter-1);
					printf("saving dynamic objec: %s\n",buf);
					QFile file(buf);
					if (file.exists()){file.remove();}
					if (!file.open(QIODevice::ReadWrite | QIODevice::Text)){std::cerr<<"Could not open file "<< buf <<" to save dynamic object as XML"<<std::endl;}
					QXmlStreamWriter* xmlWriter = new QXmlStreamWriter();
					xmlWriter->setDevice(&file);

					xmlWriter->writeStartDocument();
					xmlWriter->writeStartElement("Object");
					xmlWriter->writeAttribute("object_number", QString::number(dynamicCounter-1));
					xmlWriter->writeAttribute("classname", QString(""));
					xmlWriter->writeAttribute("instancename", QString(""));
					xmlWriter->writeAttribute("tags", QString(""));
					xmlWriter->writeStartElement("Mean");
					xmlWriter->writeAttribute("x", QString::number(sumx/sum));
					xmlWriter->writeAttribute("y", QString::number(sumy/sum));
					xmlWriter->writeAttribute("z", QString::number(sumz/sum));
					xmlWriter->writeEndElement();


					for(unsigned int j = 0; j < masks.size(); j++){
						char buf [1024];
						sprintf(buf,"%s/dynamicmask_%i_%i.png",sweep_folder.c_str(),dynamicCounter-1,imgnr[j]);
						cv::imwrite(buf, masks[j] );

						int width = masks[j].cols;
						int height = masks[j].rows;

						int maxw = 0;
						int minw = width;
						int maxh = 0;
						int minh = height;
						for(int w = 0; w < width; w++){
							for(int h = 0; h < height; h++){
								if(masks[j].data[h*width+w] != 0){
									maxw = std::max(maxw,w);
									minw = std::min(minw,w);
									maxh = std::max(maxh,h);
									minh = std::min(minh,h);
								}
							}
						}

						double ratio = 0.15;
						int diffw = maxw-minw;
						int diffh = maxh-minh;

						maxw = std::min(width-1	,int(maxw+ratio*diffw));
						minw = std::max(0		,int(minw-ratio*diffw));
						maxh = std::min(height-1,int(maxh+ratio*diffh));
						minh = std::max(0		,int(minh-ratio*diffh));

						diffw = maxw-minw;
						diffh = maxh-minh;

						cv::Mat image = mod_fr[imgnr[j]]->rgb.clone();
						cv::Rect myROI(minw, minh, diffw, diffh);
						cv::Mat localimg = image(myROI);


						char buf2 [1024];
						sprintf(buf2,"/home/johane/imgregion/region%10.10i.png",totalcounter++);
						cv::imwrite(buf2, localimg );

						printf("saving dynamic mask: dynamicmask_%i_%i.png\n",dynamicCounter-1,imgnr[j]);

						sprintf(buf,"dynamicmask_%i_%i.png",dynamicCounter-1,imgnr[j]);
						xmlWriter->writeStartElement("Mask");
						xmlWriter->writeAttribute("filename", QString(buf));
						xmlWriter->writeAttribute("image_number", QString::number(imgnr[j]));
						xmlWriter->writeEndElement();
					}

					xmlWriter->writeEndElement();
					xmlWriter->writeEndDocument();
					delete xmlWriter;
					dynamicCounter++;
				}else{break;}
			}

			int movingCounter = 1;
			while(true){
				double sum = 0;
				double sumx = 0;
				double sumy = 0;
				double sumz = 0;
				std::vector<int> imgnr;
				std::vector<cv::Mat> masks;

				for(unsigned int j = 0; j < mod_fr.size(); j++){
					reglib::RGBDFrame * frame = mod_fr[j];
					//std::cout << mod_po[j] << std::endl;
					Eigen::Matrix4d p = mod_po[j]*bgs.front()->frames.front()->pose;
					unsigned char  * rgbdata		= (unsigned char	*)(frame->rgb.data);
					unsigned short * depthdata		= (unsigned short	*)(frame->depth.data);
					float		   * normalsdata	= (float			*)(frame->normals.data);

					reglib::Camera * camera = frame->camera;

					unsigned char * internalmaskdata = (unsigned char *)(internal_masks[j].data);
					unsigned char * externalmaskdata = (unsigned char *)(external_masks[j].data);
					unsigned char * dynamicmaskdata = (unsigned char *)(dynamic_masks[j].data);

					float m00 = p(0,0); float m01 = p(0,1); float m02 = p(0,2); float m03 = p(0,3);
					float m10 = p(1,0); float m11 = p(1,1); float m12 = p(1,2); float m13 = p(1,3);
					float m20 = p(2,0); float m21 = p(2,1); float m22 = p(2,2); float m23 = p(2,3);

					const float idepth			= camera->idepth_scale;
					const float cx				= camera->cx;
					const float cy				= camera->cy;
					const float ifx				= 1.0/camera->fx;
					const float ify				= 1.0/camera->fy;
					const unsigned int width	= camera->width;
					const unsigned int height	= camera->height;

					cv::Mat mask;
					mask.create(height,width,CV_8UC1);
					unsigned char * maskdata = (unsigned char *)(mask.data);

					bool containsData = false;
					for(unsigned int w = 0; w < width;w++){
						for(unsigned int h = 0; h < height;h++){
							int ind = h*width+w;

							if(internalmaskdata[ind] == movingCounter){
								maskdata[ind] = 255;
								containsData = true;
								float z = idepth*float(depthdata[ind]);
								if(z > 0){
									float x = (float(w) - cx) * z * ifx;
									float y = (float(h) - cy) * z * ify;

									sumx += m00*x + m01*y + m02*z + m03;
									sumy += m10*x + m11*y + m12*z + m13;
									sumz += m20*x + m21*y + m22*z + m23;
									sum ++;
								}
							}else{
								maskdata[ind] = 0;
							}
						}
					}

					if(containsData){
						masks.push_back(mask);
						imgnr.push_back(j);
					}
				}
				if(masks.size() > 0){
					char buf [1024];
					sprintf(buf,"%s/moving_obj%10.10i.xml",sweep_folder.c_str(),movingCounter-1);
					QFile file(buf);
					if (file.exists()){file.remove();}
					if (!file.open(QIODevice::ReadWrite | QIODevice::Text)){std::cerr<<"Could not open file "<< buf <<" to save dynamic object as XML"<<std::endl;}
					QXmlStreamWriter* xmlWriter = new QXmlStreamWriter();
					xmlWriter->setDevice(&file);

					xmlWriter->writeStartDocument();
					xmlWriter->writeStartElement("Object");
					xmlWriter->writeAttribute("object_number", QString::number(movingCounter-1));
					xmlWriter->writeAttribute("label", QString(""));
					xmlWriter->writeStartElement("Mean");
					xmlWriter->writeAttribute("x", QString::number(sumx/sum));
					xmlWriter->writeAttribute("y", QString::number(sumy/sum));
					xmlWriter->writeAttribute("z", QString::number(sumz/sum));
					xmlWriter->writeEndElement();

					for(unsigned int j = 0; j < masks.size(); j++){
						char buf [1024];
						sprintf(buf,"%s/movingmask_%i_%i.png",sweep_folder.c_str(),movingCounter-1,imgnr[j]);
						cv::imwrite(buf, masks[j] );
						sprintf(buf,"movingmask_%i_%i.png",movingCounter-1,imgnr[j]);
						xmlWriter->writeStartElement("Mask");
						xmlWriter->writeAttribute("filename", QString(buf));
						xmlWriter->writeAttribute("image_number", QString::number(imgnr[j]));
						xmlWriter->writeEndElement();
					}

					xmlWriter->writeEndElement();
					xmlWriter->writeEndDocument();
					delete xmlWriter;
					movingCounter++;
				}else{break;}
			}
		}
	}else{
		returnval = 1;
	}

	if(dyncloud->points.size()){
		dyncloud->width = dyncloud->points.size();
		dyncloud->height = 1;
		pcl::io::savePCDFileBinaryCompressed(sweep_folder+"/dynamic_clusters.pcd",*dyncloud);
	}

	for(unsigned int i = 0; i < bgs.size(); i++){
		bgs[i]->fullDelete();
		delete bgs[i];
	}

	fullmodel->fullDelete();
	delete fullmodel;
	//	delete bgmassreg;
	delete reg;
	delete mu;
	printf("publishing file %s to %s\n",path.c_str(),outtopic.c_str());

	std_msgs::String msg;
	msg.data = path;
	for(unsigned int i = 0; i < out_pubs.size(); i++){out_pubs[i].publish(msg);}
	ros::spinOnce();

	//sendMetaroomToServer(path);
	return returnval;
}

void chatterCallback(const std_msgs::String::ConstPtr& msg){
	CloudPtr dyncloud (new Cloud());
	processMetaroom(dyncloud,msg->data);
}

void trainMetaroom(std::string path){
	printf("processing: %s\n",path.c_str());

	if(posepath.compare("")==0){
		printf("posepath not set, set before training\n");
		return ;
	}

	int slash_pos = path.find_last_of("/");
	std::string sweep_folder = path.substr(0, slash_pos) + "/";

	reglib::Model * fullmodel = processAV(path);

	//Not needed if metaroom well calibrated
	reglib::MassRegistrationPPR2 * bgmassreg = new reglib::MassRegistrationPPR2(0.01);
	bgmassreg->timeout = 60*10;
	bgmassreg->viewer = viewer;
	bgmassreg->use_surface = true;
	bgmassreg->use_depthedge = false;//true;
	bgmassreg->visualizationLvl = visualization_lvl;//0;
	bgmassreg->maskstep = 5;
	bgmassreg->nomaskstep = 5;
	bgmassreg->nomask = true;
	bgmassreg->stopval = 0.0005;
	bgmassreg->setData(fullmodel->frames,fullmodel->modelmasks);
	reglib::MassFusionResults bgmfr = bgmassreg->getTransforms(fullmodel->relativeposes);
exit(0);
	savePoses(overall_folder+"/"+posepath,bgmfr.poses,17);
	fullmodel->fullDelete();
	delete fullmodel;
	delete bgmassreg;
}

std::vector<reglib::Model *> loadModels(std::string path){
	std::vector<reglib::Model *> models;
	int slash_pos = path.find_last_of("/");
	std::string sweep_folder = path.substr(0, slash_pos) + "/";

	std::vector<reglib::RGBDFrame *> frames;
	std::vector<Eigen::Matrix4d> poses;
	readViewXML(sweep_folder+"ViewGroup.xml",frames,poses,false);

	QStringList objectFiles = QDir(sweep_folder.c_str()).entryList(QStringList("dynamic_obj*.xml"));
	for (auto objectFile : objectFiles){
		std::string object = sweep_folder+objectFile.toStdString();
		printf("object: %s\n",object.c_str());

		QFile file(object.c_str());

		if (!file.exists()){
			ROS_ERROR("Could not open file %s to masks.",object.c_str());
			continue;
		}

		file.open(QIODevice::ReadOnly);
		//ROS_INFO_STREAM("Parsing xml file: "<<object.c_str());

		reglib::Model * mod = new reglib::Model();
		QXmlStreamReader* xmlReader = new QXmlStreamReader(&file);

		while (!xmlReader->atEnd() && !xmlReader->hasError()){
			QXmlStreamReader::TokenType token = xmlReader->readNext();
			if (token == QXmlStreamReader::StartDocument)
				continue;

			if (xmlReader->hasError()){
				ROS_ERROR("XML error: %s",xmlReader->errorString().toStdString().c_str());
				break;
			}

			QString elementName = xmlReader->name().toString();

			if (token == QXmlStreamReader::StartElement){
				if (xmlReader->name() == "Mask"){
					int number = 0;
					cv::Mat mask;
					QXmlStreamAttributes attributes = xmlReader->attributes();
					if (attributes.hasAttribute("filename")){
						QString maskpath = attributes.value("filename").toString();
						//printf("mask filename: %s\n",(sweep_folder+maskpath.toStdString()).c_str());
						mask = cv::imread(sweep_folder+"/"+(maskpath.toStdString().c_str()), CV_LOAD_IMAGE_UNCHANGED);
						//mask = cv::imread((maskpath.toStdString()).c_str(), CV_LOAD_IMAGE_UNCHANGED);
					}else{break;}


					if (attributes.hasAttribute("image_number")){
						QString depthpath = attributes.value("image_number").toString();
						number = atoi(depthpath.toStdString().c_str());
						//printf("number: %i\n",number);
					}else{break;}

					mod->frames.push_back(frames[number]->clone());
					mod->relativeposes.push_back(poses[number]);
					mod->modelmasks.push_back(new reglib::ModelMask(mask));
				}
			}
		}

		delete xmlReader;
		models.push_back(mod);
	}

	for(unsigned int i = 0; i < frames.size(); i++){delete frames[i];}
	return models;
}

void addModelToModelServer(reglib::Model * model){
	printf("addModelToModelServer\n");
	for(unsigned int i = 0; i < model_pubs.size(); i++){model_pubs[i].publish(quasimodo_brain::getModelMSG(model));}
	ros::spinOnce();
}

void sendMetaroomToServer(std::string path){
	std::vector<reglib::Model *> models = loadModels(path);
	for(unsigned int i = 0; i < models.size(); i++){
		printf("%i\n",i);
		addModelToModelServer(models[i]);
		models[i]->fullDelete();
		delete models[i];
	}
}

void sendCallback(const std_msgs::String::ConstPtr& msg){sendMetaroomToServer(msg->data);}

void processAndSendCallback(const std_msgs::String::ConstPtr& msg){
	printf("================================================================================\n");
	printf("============================processAndSendCallback==============================\n");
	printf("================================================================================\n");
	CloudPtr dyncloud (new Cloud());
	processMetaroom(dyncloud,msg->data);
	sendCallback(msg);
}

bool dynamicObjectsServiceCallback(DynamicObjectsServiceRequest &req, DynamicObjectsServiceResponse &res){
	printf("bool dynamicObjectsServiceCallback(DynamicObjectsServiceRequest &req, DynamicObjectsServiceResponse &res)\n");
	std::string current_waypointid = req.waypoint_id;

	printf("current_waypointid: %i\n",current_waypointid.c_str());


	if(overall_folder.back() == '/'){overall_folder.pop_back();}

	SimpleXMLParser<pcl::PointXYZRGB> parser;
	int prevind = -1;
	std::vector<std::string> sweep_xmls = semantic_map_load_utilties::getSweepXmls<pcl::PointXYZRGB>(overall_folder);
	for (unsigned int i = 0; i < sweep_xmls.size(); i++){
		SimpleXMLParser<pcl::PointXYZRGB>::RoomData other_roomData  = parser.loadRoomFromXML(sweep_xmls[i],std::vector<std::string>(),false,false);
		std::string other_waypointid = other_roomData.roomWaypointId;
		if(other_waypointid.compare(current_waypointid) == 0){prevind = i;}
	}
	if(prevind == -1){return false;}
	std::string path = sweep_xmls[prevind];
	//processMetaroom(path);

	printf("path: %s\n",path.c_str());
	int slash_pos = path.find_last_of("/");
	std::string sweep_folder = path.substr(0, slash_pos) + "/";

	printf("sweep_folder: %s\n",sweep_folder.c_str());

	QStringList objectFiles = QDir(sweep_folder.c_str()).entryList(QStringList("dynamic_object*.xml"));
	for (auto objectFile : objectFiles){
		std::string object = sweep_folder+objectFile.toStdString();
		printf("object: %s\n",object.c_str());

		QFile file(object.c_str());

		if (!file.exists()){
			ROS_ERROR("Could not open file %s to masks.",object.c_str());
			continue;
		}

		file.open(QIODevice::ReadOnly);
		ROS_INFO_STREAM("Parsing xml file: "<<object.c_str());

		QXmlStreamReader* xmlReader = new QXmlStreamReader(&file);

		while (!xmlReader->atEnd() && !xmlReader->hasError()){
			QXmlStreamReader::TokenType token = xmlReader->readNext();
			if (token == QXmlStreamReader::StartDocument)
				continue;

			if (xmlReader->hasError()){
				ROS_ERROR("XML error: %s",xmlReader->errorString().toStdString().c_str());
				break;
			}

			QString elementName = xmlReader->name().toString();
			if (token == QXmlStreamReader::StartElement){
				if (xmlReader->name() == "Mean"){
					QXmlStreamAttributes attributes = xmlReader->attributes();
					double x = atof(attributes.value("x").toString().toStdString().c_str());
					double y = atof(attributes.value("y").toString().toStdString().c_str());
					double z = atof(attributes.value("z").toString().toStdString().c_str());
					printf("mean: %f %f %f\n",x,y,z);

					std::string objectpcd = object.substr(0,object.size()-4);
					std::cout << objectpcd << std::endl;

					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
					pcl::io::loadPCDFile<pcl::PointXYZRGB> (objectpcd+".pcd", *cloud);

					printf("cloud->points.size() = %i\n",cloud->points.size());

					sensor_msgs::PointCloud2 msg_objects;
					pcl::toROSMsg(*cloud, msg_objects);
					msg_objects.header.frame_id="/map";

					geometry_msgs::Point p;
					p.x = x;
					p.y = y;
					p.z = z;

					res.object_id.push_back(object);
					res.objects.push_back(msg_objects);
					res.centroids.push_back(p);
				}
			}
		}
		delete xmlReader;
	}

	return true;
}

bool getDynamicObjectServiceCallback(GetDynamicObjectServiceRequest &req, GetDynamicObjectServiceResponse &res){
	std::string current_waypointid = req.waypoint_id;

	if(overall_folder.back() == '/'){overall_folder.pop_back();}

	SimpleXMLParser<pcl::PointXYZRGB> parser;
	int prevind = -1;
	std::vector<std::string> sweep_xmls = semantic_map_load_utilties::getSweepXmls<pcl::PointXYZRGB>(overall_folder);
	for (unsigned int i = 0; i < sweep_xmls.size(); i++){
		SimpleXMLParser<pcl::PointXYZRGB>::RoomData other_roomData  = parser.loadRoomFromXML(sweep_xmls[i],std::vector<std::string>(),false,false);
		std::string other_waypointid = other_roomData.roomWaypointId;
		if(other_waypointid.compare(current_waypointid) == 0){prevind = i;}
	}
	if(prevind == -1){return false;}
	std::string path = sweep_xmls[prevind];

	printf("path: %s\n",path.c_str());
	int slash_pos = path.find_last_of("/");
	std::string sweep_folder = path.substr(0, slash_pos) + "/";

	printf("sweep_folder: %s\n",sweep_folder.c_str());
	printf("req.object_id: %s\n",req.object_id.c_str());
	char buf [1024];
	std::string fullpath = std::string(buf);

	return true;
}

bool segmentRaresFiles(std::string path, bool resegment){
	printf("bool segmentRaresFiles(%s)\n",path.c_str());

	vector<string> sweep_xmls = semantic_map_load_utilties::getSweepXmls<PointType>(path);
	for (auto sweep_xml : sweep_xmls) {
		printf("sweep_xml: %s\n",sweep_xml.c_str());

		quasimodo_brain::cleanPath(sweep_xml);
		int slash_pos = sweep_xml.find_last_of("/");
		std::string sweep_folder = sweep_xml.substr(0, slash_pos) + "/";
		QStringList segoutput = QDir(sweep_folder.c_str()).entryList(QStringList("segoutput.txt"));

		printf("segoutput %i\n",segoutput.size());
		if(resegment || segoutput.size() == 0){

			std::ofstream myfile;
			myfile.open (sweep_folder+"segoutput.txt");
			myfile << "dummy";
			myfile.close();

			CloudPtr dyncloud (new Cloud());
			processMetaroom(dyncloud,sweep_xml);
		}

	}
	return false;
}

bool testDynamicObjectServiceCallback(std::string path){
	printf("bool getDynamicObjectServiceCallback(GetDynamicObjectServiceRequest &req, GetDynamicObjectServiceResponse &res)\n");
	DynamicObjectsServiceRequest req;
	req.waypoint_id = path;
	DynamicObjectsServiceResponse res;
	return dynamicObjectsServiceCallback(req,res);
}


void processSweep(std::string path, std::string savePath){


	CloudPtr dyncloud (new Cloud());
	int ret = processMetaroom(dyncloud,path);

	std_msgs::String msg;
	if(ret == 0){
		ROS_ERROR_STREAM("Xml file does not exist. Aborting.");
		msg.data	= "error_processing_observation";
	}else{msg.data	= "finished_processing_observation";}

	if(ret == 1){ROS_ERROR_STREAM("First metaroom.");}
	if(ret == 2){ROS_ERROR_STREAM("No moving objects found.");}
	if(ret == 3){ROS_ERROR_STREAM("Moving objects found.");}

	sensor_msgs::PointCloud2 input;
	pcl::toROSMsg (*dyncloud,input);
	input.header.frame_id = "/map";
	roomObservationCallback_pubs.publish(input);

	for(unsigned int i = 0; i < m_PublisherStatuss.size(); i++){m_PublisherStatuss[i].publish(msg);}

	//Send the previous room to the modelserver...
	path = replaceAll(path, "//", "/");
	int prevind = -1;
	std::vector<std::string> sweep_xmls = semantic_map_load_utilties::getSweepXmls<pcl::PointXYZRGB>(overall_folder);
	for (unsigned int i = 0; i < sweep_xmls.size(); i++){
		sweep_xmls[i] = replaceAll(sweep_xmls[i], "//", "/");
		if(sweep_xmls[i].compare(path) == 0){break;}
		prevind = i;
	}
	if(prevind >= 0){//Submit last metaroom results
		sendMetaroomToServer(sweep_xmls[prevind]);
	}
}

void roomObservationCallback(const semantic_map_msgs::RoomObservationConstPtr& obs_msg) {

	std::cout<<"Room obs message received"<<std::endl;
	processSweep(obs_msg->xml_file_name,"");
	//processMetaroom
}

std::vector<reglib::superpoint> getRoomSuperPoints(std::string path, std::string savePath){
	std::vector<reglib::superpoint> spvec;
	int slash_pos = path.find_last_of("/");
	std::string sweep_folder = path.substr(0, slash_pos) + "/";

	std::streampos size;
	char * memblock;

	std::ifstream file (sweep_folder+"/superpoints.bin", ios::in|ios::binary|ios::ate);
	if (file.is_open())	{
		size = file.tellg();
		if(size == 0){return spvec;}

		memblock = new char [size];
		file.seekg (0, ios::beg);
		file.read (memblock, size);
		file.close();

		cout << "the entire file content is in memory";

		long nr_points = size / (sizeof(float)*12);
		printf("loading %i points\n",nr_points);

		float * data = (float *)memblock;

		spvec.resize(nr_points);
		long count = 0;
		for(unsigned long i = 0; i < nr_points; i++){
			reglib::superpoint & p = spvec[i];
			p.point(0)			= data[count++];
			p.point(1)			= data[count++];
			p.point(2)			= data[count++];
			p.normal(0)			= data[count++];
			p.normal(1)			= data[count++];
			p.normal(2)			= data[count++];
			p.feature(0)		= data[count++];
			p.feature(1)		= data[count++];
			p.feature(2)		= data[count++];
			p.point_information	= data[count++];
			p.normal_information = data[count++];
			p.feature_information = data[count++];
		}

		delete[] memblock;
	}else{
		std::cout << "Unable to open file superpoint file, go process that data" << std::endl;
	}

	return spvec;
}

void transformSuperPoints(std::vector<reglib::superpoint> & spvec, Eigen::Matrix4d cp){
	float m00 = cp(0,0); float m01 = cp(0,1); float m02 = cp(0,2); float m03 = cp(0,3);
	float m10 = cp(1,0); float m11 = cp(1,1); float m12 = cp(1,2); float m13 = cp(1,3);
	float m20 = cp(2,0); float m21 = cp(2,1); float m22 = cp(2,2); float m23 = cp(2,3);

	for(unsigned long i = 0; i < spvec.size(); i++){
		reglib::superpoint & p = spvec[i];

		float x		= p.point(0);
		float y		= p.point(1);
		float z		= p.point(2);
		float nx	= p.normal(0);
		float ny	= p.normal(1);
		float nz	= p.normal(2);

		float tx	= m00*x + m01*y + m02*z + m03;
		float ty	= m10*x + m11*y + m12*z + m13;
		float tz	= m20*x + m21*y + m22*z + m23;

		float tnx	= m00*nx + m01*ny + m02*nz;
		float tny	= m10*nx + m11*ny + m12*nz;
		float tnz	= m20*nx + m21*ny + m22*nz;

		p.point(0)	= tx;
		p.point(1)	= ty;
		p.point(2)	= tz;
		p.normal(0) = tnx;
		p.normal(1) = tny;
		p.normal(2) = tnz;
	}
}

void saveSuperPoints(std::string path, std::vector<reglib::superpoint> & spvec, Eigen::Matrix4d pose, float ratio_keep = 0.1){
	printf("saveSuperPoints(%s)\n",path.c_str());
	transformSuperPoints(spvec,pose);
	//XYZ RGB NXNYNZ W1 W2
	long sizeofSuperPoint = 3*(3+1);

	std::ofstream file;
	file.open(path, ios::out | ios::binary);
	if(spvec.size() != 0){
		float * data = new float[spvec.size() * sizeofSuperPoint];
		long added = 0;
		long count = 0;
		for(unsigned long i = 0; i < spvec.size(); i++){
			if(double(rand() % 1000)*0.001 > ratio_keep ){continue;}
			reglib::superpoint & p = spvec[i];
			data[count++] = p.point(0);
			data[count++] = p.point(1);
			data[count++] = p.point(2);
			data[count++] = p.normal(0);
			data[count++] = p.normal(1);
			data[count++] = p.normal(2);
			data[count++] = p.feature(0);
			data[count++] = p.feature(1);
			data[count++] = p.feature(2);
			data[count++] = p.point_information;
			data[count++] = p.normal_information;
			data[count++] = p.feature_information;
			added++;
		}
		if(added > 0){
			printf("saving %i points\n",added);
			file.write( (char*)data, added*sizeofSuperPoint*sizeof(float));
		}
		delete[] data;
	}
	file.close();
}

void processSweepForDatabase(std::string path, std::string savePath){
	path = replaceAll(path, "//", "/");
	if ( ! boost::filesystem::exists( path ) ){return;}

	int slash_pos = path.find_last_of("/");
	std::string sweep_folder = path.substr(0, slash_pos) + "/";


	SimpleXMLParser<pcl::PointXYZRGB> parser;
	SimpleXMLParser<pcl::PointXYZRGB>::RoomData roomData  = parser.loadRoomFromXML(sweep_folder+"/room.xml",std::vector<std::string>(),false,false);

	std::string current_waypointid = roomData.roomWaypointId;


	if(current_waypointid.compare("ReceptionDesk") != 0){return;}
	//if(current_waypointid.compare("WayPoint5") != 0){return;}


	printf("------------------------START--------------------------\n");
	printf("processSweepForDatabase(%s)\n",path.c_str());
	//printf("sweep_folder: %s\n",sweep_folder.c_str());
	printf("current_waypointid: %s\n",current_waypointid.c_str());


	//Send the previous room to the modelserver...
	int firstind = -1;
	int prevind = -1;
	std::vector<std::string> sweep_xmls = semantic_map_load_utilties::getSweepXmls<pcl::PointXYZRGB>(overall_folder,false);
	for (unsigned int i = 0; i < sweep_xmls.size(); i++){
		sweep_xmls[i] = replaceAll(sweep_xmls[i], "//", "/");
		SimpleXMLParser<pcl::PointXYZRGB>::RoomData other_roomData  = parser.loadRoomFromXML(sweep_xmls[i],std::vector<std::string>(),false,false);

		//printf("sweep: %s\n",sweep_xmls[i].c_str());

		if(sweep_xmls[i].compare(path) == 0){break;}
		std::string other_waypointid = other_roomData.roomWaypointId;

		std::string other_path = sweep_xmls[i];
		int slash_pos = other_path.find_last_of("/");
		std::string other_sweep_folder = other_path.substr(0, slash_pos) + "/";
		std::ifstream file (other_sweep_folder+"/superpoints.bin", ios::in|ios::binary|ios::ate);
		if (file.is_open())	{
			file.close();

			//Check if it has the superpoint file
			if(other_waypointid.compare(current_waypointid) == 0){
				if(firstind == -1){firstind = i;}
				prevind = i;
			}
		}
	}

	reglib::Model * sweep = quasimodo_brain::load_metaroom_model(path,savePath);
	Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d change = Eigen::Matrix4d::Identity();
	if(firstind != -1){
		std::vector<reglib::superpoint> backgroundsp = getRoomSuperPoints(sweep_xmls[firstind],savePath);
		if(prevind != -1 && prevind != firstind){
			std::vector<reglib::superpoint> prev_vec = getRoomSuperPoints(sweep_xmls[prevind],savePath);
			backgroundsp.insert(backgroundsp.end(), prev_vec.begin(), prev_vec.end());
		}

		Eigen::Matrix4d currentpose = Eigen::Matrix4d::Identity();
		roomData  = parser.loadRoomFromXML(sweep_folder+"/room.xml");
		if(roomData.vIntermediateRoomCloudTransforms.size() != 0){
			currentpose = quasimodo_brain::getMat(roomData.vIntermediateRoomCloudTransforms[0]);
		}


		Eigen::Matrix4d firstpose = Eigen::Matrix4d::Identity();
		SimpleXMLParser<pcl::PointXYZRGB>::RoomData first_roomData  = parser.loadRoomFromXML(sweep_xmls[firstind]);
		if(first_roomData.vIntermediateRoomCloudTransforms.size() != 0){
			firstpose = quasimodo_brain::getMat(first_roomData.vIntermediateRoomCloudTransforms[0]);
		}

		std::vector<Eigen::Matrix4d> poses;
		poses.push_back(Eigen::Matrix4d::Identity());
		poses.push_back(sweep->frames.front()->pose);//Eigen::Matrix4d::Identity());

		reglib::Model * bgdata = new reglib::Model();
		bgdata->points = backgroundsp;

		reglib::MassRegistrationPPR2 * bgmassreg = new reglib::MassRegistrationPPR2(0.1);
		bgmassreg->timeout = 600;
		bgmassreg->viewer = viewer;
		bgmassreg->use_surface = true;
		bgmassreg->use_depthedge = true;
		bgmassreg->visualizationLvl = visualization_lvl;//0;
		bgmassreg->maskstep = 5;
		bgmassreg->nomaskstep = 5;
		bgmassreg->nomask = true;
		bgmassreg->stopval = 0.0005;
		bgmassreg->addModel(bgdata);
		bgmassreg->addModel(sweep);
		reglib::MassFusionResults bgmfr = bgmassreg->getTransforms(poses);
		change = bgmfr.poses.back() * poses.back().inverse();
		pose = bgmfr.poses.back();
	}else{//this is the first room, set to whatever odometry is
		printf("this is the first room, set to whatever odometry is!\n");
		roomData  = parser.loadRoomFromXML(sweep_folder+"/room.xml");
		if(roomData.vIntermediateRoomCloudTransforms.size() != 0){
			pose = quasimodo_brain::getMat(roomData.vIntermediateRoomCloudTransforms[0]);
		}
	}

	saveSuperPoints(sweep_folder+"/superpoints.bin",sweep->points,pose,1);
	for(unsigned int i = 0; i < sweep->frames.size(); i++){
		sweep->frames[i]->pose *= change;
	}

//	string meta_data
//	uint32 timestamp
//	tf/tfMessage transform
//	---
//	bool result
//	soma_llsd_msgs/Scene response

	ros::ServiceClient client = np->serviceClient<soma_llsd::InsertScene>("/soma_llsd/insert_scene");
	ROS_INFO("Waiting for /soma_llsd/insert_scene service...");
	if (!client.waitForExistence(ros::Duration(1.0))) {
		ROS_INFO("Failed to get /soma_llsd/insert_scene service!");
		//return;
	}else{
		ROS_INFO("Got /soma_llsd/insert_scene service");
	}

	soma_llsd_msgs::Segment sweepsegment;
	sweepsegment.id = "sweep"+roomData.roomRunNumber;

	for(unsigned int i = 0; i < sweep->frames.size(); i++){
		reglib::RGBDFrame * frame = sweep->frames[i];
		geometry_msgs::Pose		pose;
		tf::poseEigenToMsg (Eigen::Affine3d(frame->pose), pose);

		geometry_msgs::Pose		segpose;
		tf::poseEigenToMsg (Eigen::Affine3d(sweep->relativeposes[i]), segpose);

		std::cout << sweep->relativeposes[i] << std::endl;

		cv_bridge::CvImage rgbBridgeImage;
		rgbBridgeImage.image = frame->rgb;
		rgbBridgeImage.encoding = "bgr8";

		cv_bridge::CvImage depthBridgeImage;
		depthBridgeImage.image = frame->depth;
		depthBridgeImage.encoding = "mono16";

		sensor_msgs::PointCloud2 input;
		pcl::toROSMsg (*(roomData.vIntermediateRoomClouds[i]),input);//, *transformed_cloud);
		input.header.frame_id = "/map";

		soma_llsd::InsertScene scene;
		scene.request.rgb_img = *(rgbBridgeImage.toImageMsg());
		scene.request.depth_img = *(depthBridgeImage.toImageMsg());
		scene.request.camera_info.K[0] = frame->camera->fx;
		scene.request.camera_info.K[4] = frame->camera->fy;
		scene.request.camera_info.K[2] = frame->camera->cx;
		scene.request.camera_info.K[5] = frame->camera->cy;
		scene.request.robot_pose = pose;
		scene.request.cloud = input;
		scene.request.waypoint = current_waypointid;
		scene.request.episode_id = roomData.roomRunNumber;

//		if (!client.call(scene)) {
//			ROS_ERROR("Failed to call service /soma_llsd/insert_scene");
//			return;
//		}


		cv_bridge::CvImage maskBridgeImage;
		maskBridgeImage.image		= sweep->modelmasks[i]->getMask();
		maskBridgeImage.encoding	= "mono8";

		soma_llsd_msgs::Observation obs;
		obs.scene_id = scene.response.response.id;
		//obs.camera_cloud = model.clouds[counter]; //Dont add clouds...
		obs.image_mask = *(maskBridgeImage.toImageMsg());
		obs.pose = segpose;
		obs.id = "frame"+std::to_string(frame->id);
//        obs.timestamp = ros::Time::now().nsec;
		sweepsegment.observations.push_back(obs);
	}


	//	int startsize = msg.local_poses.size();
	//	msg.local_poses.resize(startsize+model->relativeposes.size());
	//	msg.frames.resize(startsize+model->frames.size());
	//	msg.masks.resize(startsize+model->modelmasks.size());
	//	for(unsigned int i = 0; i < model->relativeposes.size(); i++){
	//		geometry_msgs::Pose		pose1;
	//		tf::poseEigenToMsg (Eigen::Affine3d(model->relativeposes[i])*rp, pose1);
	//		geometry_msgs::Pose		pose2;
	//		tf::poseEigenToMsg (Eigen::Affine3d(model->frames[i]->pose)*rp, pose2);
	//		cv_bridge::CvImage rgbBridgeImage;
	//		rgbBridgeImage.image = model->frames[i]->rgb;
	//		rgbBridgeImage.encoding = "bgr8";
	//		cv_bridge::CvImage depthBridgeImage;
	//		depthBridgeImage.image = model->frames[i]->depth;
	//		depthBridgeImage.encoding = "mono16";
	//		cv_bridge::CvImage maskBridgeImage;
	//		maskBridgeImage.image			= model->modelmasks[i]->getMask();
	//		maskBridgeImage.encoding		= "mono8";
	//		msg.local_poses[startsize+i]			= pose1;
	//		msg.frames[startsize+i].capture_time	= ros::Time();
	//		msg.frames[startsize+i].pose			= pose2;
	//		msg.frames[startsize+i].frame_id		= model->frames[i]->id;
	//		msg.frames[startsize+i].rgb				= *(rgbBridgeImage.toImageMsg());
	//		msg.frames[startsize+i].depth			= *(depthBridgeImage.toImageMsg());
	//		msg.masks[startsize+i]					= *(maskBridgeImage.toImageMsg());

	//		msg.frames[startsize+i].camera.K[0] = model->frames[i]->camera->fx;
	//		msg.frames[startsize+i].camera.K[4] = model->frames[i]->camera->fy;
	//		msg.frames[startsize+i].camera.K[2] = model->frames[i]->camera->cx;
	//		msg.frames[startsize+i].camera.K[5] = model->frames[i]->camera->cy;

	//		sensor_msgs::PointCloud2 output;
	//		if(addClouds){pcl::toROSMsg(*(model->frames[i]->getPCLcloud()), output);}
	//		msg.clouds.push_back(output);
	//	}
	//	for(unsigned int i = 0; i < model->submodels_relativeposes.size(); i++){
	//		addToModelMSG(msg,model->submodels[i],Eigen::Affine3d(model->submodels_relativeposes[i])*rp,addClouds);
	//	}


	sweep->fullDelete();
	delete sweep;
	//	exit(0);
	//		int additional_nrviews = 0;
	//		QStringList objectFiles = QDir(sweep_folder.c_str()).entryList(QStringList("*object*.xml"));
	//		for (auto objectFile : objectFiles){
	//			printf("objectFile: %s\n",(sweep_folder+objectFile.toStdString()).c_str());
	//			auto object = loadDynamicObjectFromSingleSweep<PointType>(sweep_folder+objectFile.toStdString(),false);
	//			additional_nrviews += object.vAdditionalViews.size();
	//		}

	//		SimpleXMLParser<pcl::PointXYZRGB> parser;
	//		SimpleXMLParser<pcl::PointXYZRGB>::RoomData current_roomData  = parser.loadRoomFromXML(path,std::vector<std::string>{"RoomIntermediateCloud","IntermediatePosition"});
	//		int metaroom_nrviews = current_roomData.vIntermediateRoomClouds.size();


	//		printf("viewgroup_nrviews: %i\n",viewgroup_nrviews);
	//		printf("additional_nrviews: %i\n",additional_nrviews);
	//		printf("metaroom_nrviews: %i\n",metaroom_nrviews);

	//		reglib::Model * fullmodel;
	//		if(viewgroup_nrviews == (additional_nrviews+metaroom_nrviews) && !recomputeRelativePoses){
	//			printf("time to read old\n");
	//			fullmodel = new reglib::Model();
	//			fullmodel->savePath = saveVisuals_sp+"/";

	//			for(unsigned int i = 0; i < viewgroup_nrviews; i++){
	//				cv::Mat fullmask;
	//				fullmask.create(480,640,CV_8UC1);
	//				unsigned char * maskdata = (unsigned char *)fullmask.data;
	//				for(int j = 0; j < 480*640; j++){maskdata[j] = 255;}
	//				fullmodel->modelmasks.push_back(new reglib::ModelMask(fullmask));
	//			}

	//			readViewXML(sweep_folder+"ViewGroup.xml",fullmodel->frames,fullmodel->relativeposes,compute_edges,saveVisuals_sp);
	//			fullmodel->recomputeModelPoints();
	//		}else{
	//			fullmodel = processAV(path,compute_edges,saveVisuals_sp);
	//			writeXml(sweep_folder+"ViewGroup.xml",fullmodel->frames,fullmodel->relativeposes);
	//		}

	//		return fullmodel;
	//	}



	//	quasimodo_msgs::model	model;
	//	soma_llsd_msgs::Segment segment;
	//model_to_soma_segment(ros::NodeHandle& n, const quasimodo_msgs::model& model, soma_llsd_msgs::Segment& segment);
	printf("------------------------STOP--------------------------\n");
}

bool testPath(std::string path){
	printf("bool testPath(%s)\n",path.c_str());

	vector<string> sweep_xmls = semantic_map_load_utilties::getSweepXmls<PointType>(path);
	for (auto sweep_xml : sweep_xmls) {
		//printf("sweep_xml: %s\n",sweep_xml.c_str());
		//processSweep(sweep_xml,"");
		processSweepForDatabase(sweep_xml,"");
	}
	return false;
}


void setLargeStack(){
	const rlim_t kStackSize = 256 * 1024 * 1024;   // min stack size = 256 MB
	struct rlimit rl;
	unsigned long result;

	result = getrlimit(RLIMIT_STACK, &rl);
	if (result == 0){
		if (rl.rlim_cur < kStackSize){
			rl.rlim_cur = kStackSize;
			result = setrlimit(RLIMIT_STACK, &rl);
			if (result != 0){fprintf(stderr, "setrlimit returned result = %d\n", int(result));}
		}
	}
}

int main(int argc, char** argv){
	bool baseSetting = true;
	bool once = false;

	overall_folder	= std::string(getenv ("HOME"))+"/.semanticMap/";
	outtopic		= "/some/topic";
	modelouttopic	= "/model/out/topic";
	posepath		= "testposes.xml";

	ros::init(argc, argv, "metaroom_additional_view_processing");
	ros::NodeHandle n;
	np = &n;

	std::vector<ros::Subscriber> segsubs;
	std::vector<ros::Subscriber> sendsubs;
	std::vector<ros::Subscriber> roomObservationSubs;
	std::vector<ros::Subscriber> processAndSendsubs;


	std::vector<std::string> trainMetarooms;
	std::vector<std::string> sendMetaroomToServers;
	std::vector<std::string> processMetarooms;
	std::vector<std::string> testpaths;

	std::vector<bool>		 raresfiles_resegment;
	std::vector<std::string> raresfiles;

	bool resegment					= false;

	int inputstate = 0;
	for(int i = 1; i < argc;i++){
		printf("input: %s\n",argv[i]);
		if(		std::string(argv[i]).compare("-intopic") == 0){						inputstate = 0;}
		else if(std::string(argv[i]).compare("-outtopic") == 0){					inputstate = 1;}
		else if(std::string(argv[i]).compare("-file") == 0){						inputstate = 2;}
		else if(std::string(argv[i]).compare("-v") == 0){	visualization_lvl = 1;	inputstate = 3;}
		else if(std::string(argv[i]).compare("-folder") == 0){						inputstate	= 4;}
		else if(std::string(argv[i]).compare("-train") == 0){						inputstate	= 5;}
		else if(std::string(argv[i]).compare("-posepath") == 0){					inputstate	= 6;}
		else if(std::string(argv[i]).compare("-loadposes") == 0){					inputstate	= 7;}
		else if(std::string(argv[i]).compare("-sendModel") == 0){					inputstate	= 8;}
		else if(std::string(argv[i]).compare("-sendSub") == 0)	{					inputstate	= 9;}
		else if(std::string(argv[i]).compare("-sendTopic") == 0){					inputstate	= 10;}
		else if(std::string(argv[i]).compare("-roomObservationTopic") == 0){		inputstate	= 11;}
		else if(std::string(argv[i]).compare("-DynamicObjectsService") == 0){		inputstate	= 12;}
		else if(std::string(argv[i]).compare("-GetDynamicObjectService") == 0){		inputstate	= 13;}
		else if(std::string(argv[i]).compare("-statusmsg") == 0){					inputstate	= 14;}
		else if(std::string(argv[i]).compare("-files") == 0){						inputstate	= 15;}
		else if(std::string(argv[i]).compare("-baseSweep") == 0){					inputstate	= 16;}
		else if(std::string(argv[i]).compare("-resegment") == 0){					resegment	= true;}
		else if(std::string(argv[i]).compare("-once") == 0){						once		= true;}
		else if(std::string(argv[i]).compare("-nobase") == 0){						baseSetting = false;}
		else if(std::string(argv[i]).compare("-recomputeRelativePoses") == 0){		recomputeRelativePoses = true;}
		else if(std::string(argv[i]).compare("-v_init") == 0){	visualization_lvl_regini = 1;inputstate = 17;}
		else if(std::string(argv[i]).compare("-v_reg") == 0){	visualization_lvl_regref = 1;inputstate = 18;}
		else if(std::string(argv[i]).compare("-saveVisuals") == 0){					inputstate = 19;}
		else if(std::string(argv[i]).compare("-testpaths") == 0 || std::string(argv[i]).compare("-testpath") == 0 || std::string(argv[i]).compare("-testPaths") == 0 || std::string(argv[i]).compare("-testPath") == 0){					inputstate = 20;}
		else if(inputstate == 0){
			segsubs.push_back(n.subscribe(std::string(argv[i]), 1000, chatterCallback));
		}else if(inputstate == 1){
			out_pubs.push_back(n.advertise<std_msgs::String>(std::string(argv[i]), 1000));
		}else if(inputstate == 2){
			processMetarooms.push_back(std::string(argv[i]));
		}else if(inputstate == 3){
			visualization_lvl = atoi(argv[i]);
		}else if(inputstate == 4){
			overall_folder = std::string(argv[i]);
		}else if(inputstate == 5){
			trainMetarooms.push_back(std::string(argv[i]));
		}else if(inputstate == 6){
			posepath = std::string(argv[i]);
		}else if(inputstate == 7){
			sweepPoses = readPoseXML(std::string(argv[i]));
		}else if(inputstate == 8){
			sendMetaroomToServers.push_back(std::string(argv[i]));
		}else if(inputstate == 9){
			sendsubs.push_back(n.subscribe(std::string(argv[i]), 1000, sendCallback));
		}else if(inputstate == 10){
			model_pubs.push_back(n.advertise<quasimodo_msgs::model>(std::string(argv[i]), 1000));
		}else if(inputstate == 11){
			roomObservationSubs.push_back(n.subscribe(std::string(argv[i]), 1000, roomObservationCallback));
		}else if(inputstate == 12){
			m_DynamicObjectsServiceServers.push_back(n.advertiseService(std::string(argv[i]), dynamicObjectsServiceCallback));
		}else if(inputstate == 13){
			m_GetDynamicObjectServiceServers.push_back(n.advertiseService(std::string(argv[i]), getDynamicObjectServiceCallback));
		}else if(inputstate == 14){
			m_PublisherStatuss.push_back(n.advertise<std_msgs::String>(std::string(argv[i]), 1000));
		}else if(inputstate == 15){
			raresfiles.push_back(std::string(argv[i]));
			raresfiles_resegment.push_back(resegment);
		}else if(inputstate == 16){
			setBaseSweep(std::string(argv[i]));
		}else if(inputstate == 17){
			visualization_lvl_regini = atoi(argv[i]);
		}else if(inputstate == 18){
			visualization_lvl_regref = atoi(argv[i]);
		}else if(inputstate == 19){
			saveVisuals = std::string(argv[i]);
		}else if(inputstate == 20){
			testpaths.push_back(std::string(argv[i]));
		}
	}

	printf("done reading commands.\n");

	if(visualization_lvl != 0 || visualization_lvl_regref != 0 || visualization_lvl_regini != 0){
		viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
		viewer->setBackgroundColor (0.5, 0, 0.5);
		viewer->addCoordinateSystem (1.0);
		viewer->initCameraParameters ();
	}

	if(baseSetting){
		if(m_PublisherStatuss.size() == 0){
			m_PublisherStatuss.push_back(n.advertise<std_msgs::String>("/local_metric_map/status", 1000));
		}

		if(segsubs.size() == 0){
			segsubs.push_back(n.subscribe("/quasimodo/segmentation/in", 1000, chatterCallback));
		}

		if(out_pubs.size() == 0){
			out_pubs.push_back(n.advertise<std_msgs::String>("/quasimodo/segmentation/out/path", 1000));
		}

		if(model_pubs.size() == 0){
			model_pubs.push_back(n.advertise<quasimodo_msgs::model>("/quasimodo/segmentation/out/model", 1000));
		}

		if(sendsubs.size() == 0){
			sendsubs.push_back(n.subscribe("/quasimodo/segmentation/send", 1000, sendCallback));
		}

		if(roomObservationSubs.size() == 0){
			roomObservationSubs.push_back(n.subscribe("/local_metric_map/room_observations", 1000, roomObservationCallback));
		}

		if(processAndSendsubs.size() == 0){
			processAndSendsubs.push_back(n.subscribe("/object_learning/learned_object_xml", 1000, chatterCallback));//processAndSendCallback));
		}
	}


	roomObservationCallback_pubs = n.advertise<sensor_msgs::PointCloud2>("/quasimodo/segmentation/roomObservation/dynamic_clusters", 1000);

	printf("overall_folder: %s\n",overall_folder.c_str());

	printf("testpaths: %i\n",testpaths.size());
	for(unsigned int i = 0; i < testpaths.size(); i++){
		testPath(testpaths[i]);
	}


	for(unsigned int i = 0; i < raresfiles.size(); i++){segmentRaresFiles(		raresfiles[i], raresfiles_resegment[i]);}
	for(unsigned int i = 0; i < trainMetarooms.size(); i++){		trainMetaroom(			trainMetarooms[i]);}
	for(unsigned int i = 0; i < processMetarooms.size(); i++){
		CloudPtr dyncloud (new Cloud());
		processMetaroom(dyncloud,processMetarooms[i]);
	}
	for(unsigned int i = 0; i < sendMetaroomToServers.size(); i++){
		vector<string> sweep_xmls = semantic_map_load_utilties::getSweepXmls<PointType>(sendMetaroomToServers[i]);
		for (auto sweep_xml : sweep_xmls) {
			printf("sweep_xml: %s\n",sweep_xml.c_str());
			quasimodo_brain::cleanPath(sweep_xml);
			int slash_pos = sweep_xml.find_last_of("/");
			std::string sweep_folder = sweep_xml.substr(0, slash_pos) + "/";
			sendMetaroomToServer(sweep_folder);
		}
	}

	if(!once){ros::spin();}
	printf("done...\n");
	return 0;
}
