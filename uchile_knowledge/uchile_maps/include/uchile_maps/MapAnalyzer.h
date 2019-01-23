/*
 * MapAnalyzer.h
 *
 *  Created on: January, 2015
 *      Author: matias.pavez.b@gmail.com
 */

#ifndef SEMANTICMAPSERVER_H_
#define SEMANTICMAPSERVER_H_

// C, C++
#include <string>
#include <vector>
#include <map>
#include <limits>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <uchile_util/ParameterServerWrapper.h>

// ROS messages and services
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <uchile_srvs/ValidPoint.h>
#include <uchile_srvs/StringArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <uchile_srvs/PointCloud2.h>
#include <uchile_srvs/PointCloud2Trasnform.h>
#include <visualization_msgs/MarkerArray.h>

// pcl
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/publisher.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>

namespace uchile_knowledge {

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class MapAnalyzer {

private:

	std::string _name;

	// map polygons
	std::vector<std::string> _rooms;
	geometry_msgs::Polygon _map_polygon;
	geometry_msgs::Polygon _map_bounding_box;
	std::map<std::string, geometry_msgs::Polygon> _room_polygons;

	std::string _map_frame;
	tf::TransformListener _tf_listener;

	// publishers
	ros::Publisher _rviz_pub;

	// services
	ros::ServiceServer _get_rooms_server;
	ros::ServiceServer _check_point_server;
	ros::ServiceServer _mask_point_cloud2_server;

	// (testing)
	bool _debug_enabled;
	ros::Subscriber _cloud_testing_sub;
	ros::Publisher _cloud_testing_pub;

public:
	MapAnalyzer(std::string name);
	virtual ~MapAnalyzer();

private:

	// services
	bool getRoomsService(uchile_srvs::StringArray::Request &req, uchile_srvs::StringArray::Response& res);
	bool checkPointService(uchile_srvs::ValidPoint::Request &req, uchile_srvs::ValidPoint::Response& res);
	bool checkPointCloud2Service(uchile_srvs::PointCloud2Trasnform::Request &req, uchile_srvs::PointCloud2Trasnform::Response& res);

	// publishing
	void publishPolygons();
	void polygonToMarker(const geometry_msgs::Polygon &polygon, visualization_msgs::Marker &marker);

	// subscribing
	void testCloudCallback(const sensor_msgs::PointCloud2& cloud);

	//  checking methods
	bool checkInsideBoundingBox(float x, float y);
	bool checkInsidePolygon(float x, float y, geometry_msgs::Polygon &polygon);
	bool checkPoint(float x, float y);

	static const float invalid_pcl_point;
};

const float MapAnalyzer::invalid_pcl_point = std::numeric_limits<float>::quiet_NaN();

} /* namespace uchile_knowledge */

#endif /* SEMANTICMAPSERVER_H_ */
