#include "algorithm"
#include "vector"
#include "ros/ros.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/Proximity.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_srvs/Trigger.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "ariac_entry/Bins.h"
#include "ariac_entry/Bin.h"

std::vector<osrf_gear::Order> order_vector;
std::vector<ariac_entry::Bin> bin_vector;
std::vector<ariac_entry::Bin> agv_vector;
std::vector<ariac_entry::Bin> fault_vector;

ariac_entry::Bins bin_contents_pub;
ros::ServiceClient find_bin;

bool sort_agv1 = false;
bool sort_agv2 = false;

bool sort_bin1 = false;
bool sort_bin2 = false;
bool sort_bin3 = false;
bool sort_bin4 = false;
bool sort_bin5 = false;
bool sort_bin6 = false;

bool sort_faulty1 = false;
bool sort_faulty2 = false;

//Starts the competition
void start_competition(ros::NodeHandle &node) {
  order_vector.clear();
  ros::ServiceClient begin_client = node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  std_srvs::Trigger begin_comp;
  int service_call_succeeded;
  
  if(!begin_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    begin_client.waitForExistence();
    ROS_INFO("Competition is now ready");
  }
  ROS_INFO("Request competition start...");
  service_call_succeeded = begin_client.call(begin_comp);
  if(!service_call_succeeded) {
    ROS_ERROR_STREAM("Failed to start the competion: " << begin_comp.response.message);
  } else {
    ROS_INFO("Competition started succesfully: %s", begin_comp.response.message.c_str());
  }
}

//Identifies the bin of the first product in the first shipment of the first order
void get_bin_of_first(ros::NodeHandle & node) {
  ros::ServiceClient get_bin_of_first_client = node.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
  osrf_gear::GetMaterialLocations get_bin;
  get_bin.request.material_type = order_vector.front().shipments.front().products.front().type.c_str();
  get_bin_of_first_client.call(get_bin);

  for(const osrf_gear::StorageUnit& storage_unit : get_bin.response.storage_units) {
    std::string bin ("bin");
    std::string unit_id = storage_unit.unit_id.c_str();
    std::string material_type = get_bin.request.material_type.c_str();
    if(unit_id.find(bin) != std::string::npos) {
      ROS_INFO("The first product of the first shipment in the first order is '%s' and is found in '%s'", get_bin.request.material_type.c_str(), unit_id.c_str());
    } 
  }
}

//Prints Each product's type, where that product type is located, and the pose for every product in a received order
void order_callback(const osrf_gear::Order::ConstPtr& order_msg) {
  ROS_INFO_STREAM("Received order: " << *order_msg);
  order_vector.push_back(*order_msg);

  osrf_gear::Order current_order = *order_msg;
  osrf_gear::GetMaterialLocations get_bin;

  for(const osrf_gear::Shipment& shipment : current_order.shipments) {
    for(const osrf_gear::Product& product : shipment.products) {
      get_bin.request.material_type = product.type.c_str();
      find_bin.call(get_bin);
      for(const osrf_gear::StorageUnit& storage_unit : get_bin.response.storage_units) {
         ROS_WARN("The '%s' is found in: %s", get_bin.request.material_type.c_str(), storage_unit.unit_id.c_str());
      }
      ROS_WARN_STREAM("The pose of the product is:\n " << product.pose);
    }
  }
  
}

//Logical camera of AGV1 that takes in camera image
void log_cam_agv1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  osrf_gear::LogicalCameraImage temp_msg = *cam_msg;
  ariac_entry::Bin agv1;
  agv1.image = temp_msg;
  if(!sort_agv1) {
    if(!temp_msg.models.empty()){
      agv1.bin_name = "AGV 1";
      agv1.material_type = "any";
      agv_vector.push_back(agv1);
      sort_agv1 = true;
    } else {
      agv1.bin_name = "AGV 1";
      agv1.material_type = "any";
      agv_vector.push_back(agv1);
      sort_agv1 = true;
      ROS_WARN_ONCE("Nothing in agv1");
    }
  }
}

//Logical camera of AGV2 that takes in camera image
void log_cam_agv2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  osrf_gear::LogicalCameraImage temp_msg = *cam_msg;
  ariac_entry::Bin agv2;
  agv2.image = temp_msg;
  if(!sort_agv2) {
    if(!temp_msg.models.empty()){
      agv2.bin_name = "AGV 2";
      agv2.material_type = "any";
      agv_vector.push_back(agv2);
      sort_agv2 = true;
    } else {
      agv2.bin_name = "AGV 2";
      agv2.material_type = "any";
      agv_vector.push_back(agv2);
      sort_agv2 = true;
      ROS_WARN_ONCE("Nothing in agv2");
    }
  }
}

void log_cam_bin1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  osrf_gear::LogicalCameraImage temp_msg = *cam_msg;
  ariac_entry::Bin bin1;
  bin1.image = temp_msg;
  if(!sort_bin1) {
    if(!temp_msg.models.empty()) {
        bin1.bin_name = "Bin 1";
        bin1.material_type = temp_msg.models.front().type.c_str();
        bin_vector.push_back(bin1);
        sort_bin1 = true;
    } else {
        bin1.bin_name = "Bin 1";
        bin1.material_type = "Empty";
        bin_vector.push_back(bin1);
        sort_bin1 = true;
        ROS_WARN_ONCE("Nothing in bin 1");
    }
  }
}

void log_cam_bin2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  osrf_gear::LogicalCameraImage temp_msg = *cam_msg;
  ariac_entry::Bin bin2;
  bin2.image = temp_msg;
  if(!sort_bin2) {
    if(!temp_msg.models.empty()) {
        bin2.bin_name = "Bin 2";
        bin2.material_type = temp_msg.models.front().type.c_str();
        bin_vector.push_back(bin2);
        sort_bin2 = true;
    } else {
        bin2.bin_name = "Bin 2";
        bin2.material_type = "Empty";
        bin_vector.push_back(bin2);
        sort_bin2 = true;
        ROS_WARN_ONCE("Nothing in bin 2");
    }
  }
}

void log_cam_bin3_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  osrf_gear::LogicalCameraImage temp_msg = *cam_msg;
  ariac_entry::Bin bin3;
  bin3.image = temp_msg;
  if(!sort_bin3) {
    if(!temp_msg.models.empty()) {
        bin3.bin_name = "Bin 3";
        bin3.material_type = temp_msg.models.front().type.c_str();
        bin_vector.push_back(bin3);
        sort_bin3 = true;
    } else {
        bin3.bin_name = "Bin 3";
        bin3.material_type = "Empty";
        bin_vector.push_back(bin3);
        sort_bin3 = true;
        ROS_WARN_ONCE("Nothing in bin 3");
    }
  }
}

void log_cam_bin4_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  osrf_gear::LogicalCameraImage temp_msg = *cam_msg;
  ariac_entry::Bin bin4;
  bin4.image = temp_msg;
  if(!sort_bin4) {
    if(!temp_msg.models.empty()) {
        bin4.bin_name = "Bin 4";
        bin4.material_type = temp_msg.models.front().type.c_str();
        bin_vector.push_back(bin4);
        sort_bin4 = true;
    } else {
        bin4.bin_name = "Bin 4";
        bin4.material_type = "Empty";
        bin_vector.push_back(bin4);
        sort_bin4 = true;
        ROS_WARN_ONCE("Nothing in bin 4");
    }
  }
}

void log_cam_bin5_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  osrf_gear::LogicalCameraImage temp_msg = *cam_msg;
  ariac_entry::Bin bin5;
  bin5.image = temp_msg;
  if(!sort_bin5) {
    if(!temp_msg.models.empty()) {
        bin5.bin_name = "Bin 5";
        bin5.material_type = temp_msg.models.front().type.c_str();
        bin_vector.push_back(bin5);
        sort_bin5 = true;
      
    } else {
        bin5.bin_name = "Bin 5";
        bin5.material_type = "Empty";
        bin_vector.push_back(bin5);
        ROS_WARN_ONCE("Nothing in bin 5");
    }
  }
}

void log_cam_bin6_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  osrf_gear::LogicalCameraImage temp_msg = *cam_msg;
  ariac_entry::Bin bin6;
  bin6.image = temp_msg;
  if(!sort_bin6) {
    if(!temp_msg.models.empty()) {
        bin6.bin_name = "Bin 6";
        bin6.material_type = temp_msg.models.front().type.c_str();
        bin_vector.push_back(bin6);
        sort_bin6 = true;
    } else {
        bin6.bin_name = "Bin 6";
        bin6.material_type = "Empty";
        bin_vector.push_back(bin6);
        ROS_WARN_ONCE("Nothing in bin 6");
        sort_bin6 = true;
    }
  }
}

void log_cam_faulty1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  osrf_gear::LogicalCameraImage temp_msg = *cam_msg;
  ariac_entry::Bin faulty1;
  faulty1.image = temp_msg;
  if(!sort_faulty1) {
    if(!temp_msg.models.empty()){
      faulty1.bin_name = "Qaulity Control 1";
      faulty1.material_type = "any";
      agv_vector.push_back(faulty1);
      sort_faulty1 = true;
    } else {
      faulty1.bin_name = "Quality Control 1";
      faulty1.material_type = "any";
      agv_vector.push_back(faulty1);
      sort_faulty1 = true;
      ROS_WARN_ONCE("Nothing in Quality Control 1");
    }
  }
}

void log_cam_faulty2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  osrf_gear::LogicalCameraImage temp_msg = *cam_msg;
  ariac_entry::Bin faulty2;
  faulty2.image = temp_msg;
  if(!sort_faulty2) {
    if(!temp_msg.models.empty()){
      faulty2.bin_name = "Qaulity Control 2";
      faulty2.material_type = "any";
      agv_vector.push_back(faulty2);
      sort_faulty2 = true;
    } else {
      faulty2.bin_name = "Quality Control 2";
      faulty2.material_type = "any";
      agv_vector.push_back(faulty2);
      sort_faulty2 = true;
      ROS_WARN_ONCE("Nothing in Quality Control 2");
    }
  }
}


int main(int argc, char **argv)
{
  bool found_bin = false;
  ros::init(argc, argv, "ariac_entry_node");
  ros::NodeHandle node;
  ros::Subscriber orders_sub = node.subscribe("/ariac/orders", 10, order_callback);
  ros::Publisher bin_of_first = node.advertise<std_msgs::String>("bin_of_first", 10);
  ros::Publisher bin_contents = node.advertise<ariac_entry::Bins>("bin_contents", 10);
  
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped tfStamped;
  geometry_msgs::PoseStamped part_pose, goal_pose;
  try {
        tfStamped = tfBuffer.lookupTransform("arm1_base_link", "logical_camera_bin4_frame", ros::Time(0.0), ros::Duration(1.0));
        ROS_WARN_STREAM_ONCE("tfStamped" << tfStamped);
        ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
  } catch (tf2::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
  }

  find_bin = node.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");

  ros::Subscriber log_cam_agv1_sub = node.subscribe("/ariac/logical_camera_agv1", 10, log_cam_agv1_callback);
  ros::Subscriber log_cam_agv2_sub = node.subscribe("/ariac/logical_camera_agv2", 10, log_cam_agv2_callback);

  ros::Subscriber log_cam_bin1_sub = node.subscribe("/ariac/logical_camera_bin1", 10, log_cam_bin1_callback);
  ros::Subscriber log_cam_bin2_sub = node.subscribe("/ariac/logical_camera_bin2", 10, log_cam_bin2_callback);
  ros::Subscriber log_cam_bin3_sub = node.subscribe("/ariac/logical_camera_bin3", 10, log_cam_bin3_callback);
  ros::Subscriber log_cam_bin4_sub = node.subscribe("/ariac/logical_camera_bin4", 10, log_cam_bin4_callback);
  ros::Subscriber log_cam_bin5_sub = node.subscribe("/ariac/logical_camera_bin5", 10, log_cam_bin5_callback);
  ros::Subscriber log_cam_bin6_sub = node.subscribe("/ariac/logical_camera_bin6", 10, log_cam_bin6_callback);
  
  
  ros::Subscriber log_cam_faulty1_sub = node.subscribe("/ariac/quality_control_sensor_1", 10, log_cam_faulty1_callback);
  ros::Subscriber log_cam_faulty2_sub = node.subscribe("/ariac/quality_control_sensor_2", 10, log_cam_faulty2_callback);

 
  start_competition(node);
  
  ros::Rate loop_rate(10);
  int count = 0;
  
  while(ros::ok()) {

    if (order_vector.size() == 1 && !found_bin){
       get_bin_of_first(node);
       found_bin = true;
    } else {
      ROS_WARN_ONCE("No orders received yet...");
    }
    if(bin_vector.size() == 6) {
      part_pose.pose = bin_vector.at(3).image.models.at(1).pose;
      goal_pose.pose = part_pose.pose;
      goal_pose.pose.position.z += 0.10;
      goal_pose.pose.orientation.w = 0.707;
      goal_pose.pose.orientation.x = 0.0;
      goal_pose.pose.orientation.y = 0.707;
      goal_pose.pose.orientation.w = 0.0;
      tf2::doTransform(part_pose, goal_pose, tfStamped);
    } else {
      ROS_WARN("No data");
    }
    bin_contents_pub.bins = bin_vector;
    bin_contents.publish(bin_contents_pub);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  

  ROS_INFO("Setup complete");

  return 0;
  

}

