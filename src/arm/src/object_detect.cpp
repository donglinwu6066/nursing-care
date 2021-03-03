#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>


ros::Publisher coordinate_pub;
ros::Subscriber box_sub;
ros::Subscriber object_sub;
std::string object = "up";

void boxCB(const darknet_ros_msgs::BoundingBoxes& box){
    int xmid = 0;
    int ymid = 0;
    for(int n=0; n<box.bounding_boxes.size(); n++){
        if(box.bounding_boxes[n].Class == object){
            xmid =(box.bounding_boxes[n].xmin+box.bounding_boxes[n].xmax)/2;
            ymid =(box.bounding_boxes[n].ymin+box.bounding_boxes[n].ymax)/2;
            std_msgs::Int16MultiArray coordinate;
            coordinate.data.push_back(xmid);
            coordinate.data.push_back(ymid);
            coordinate_pub.publish(coordinate);
            break;
        }
    }
}

void objectCB(const std_msgs::String& object_assign){
    object = object_assign.data;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "object_detect");
    ros::NodeHandle nh;
    //nh.getParam("object_detect/object", object);
    object_sub = nh.subscribe("object_detection/object", 1, objectCB);
    box_sub = nh.subscribe("darknet_ros/bounding_boxes", 1, boxCB);
    coordinate_pub = nh.advertise<std_msgs::Int16MultiArray>("object_detection/xy_coordinate",1);
    ros::spin();
    return 0;
}
