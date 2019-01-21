// Copyright 2018 Venkisagunner. All rights reserved.
#include "template_pkg/example_header.h"
ExampleRosClass::ExampleRosClass(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    ROS_INFO("in class constructor of ExampleRosClass");
    loadParameters();
    initializeSubscribers();
    initializePublishers();
    initializeServices();
    initializeDynamicReconfigurations();
    initializeTransformBroadcaster();
}
void ExampleRosClass::loadParameters()
{
    std::string log;
    if(nh.hasParam("example_node/param1"))
    {
        nh.getParam("example_node/param1",param1);
        log = "param1 - loaded, ";
    }
    else
    {
        log = "param1 - not loaded, ";
    }
    if(nh.hasParam("example_node/param2"))
    {
        nh.getParam("example_node/param2",param2);
        log += "param2 - loaded";
    }
    else
    {
        log += "param2 - not loaded";
    }
    ROS_INFO("This is fun %s",log.c_str());
}
void ExampleRosClass::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    minimal_subscriber = nh.subscribe("example_class_input_topic", 1,
                                        &ExampleRosClass::subscriberCallback,
                                        this);
}
void ExampleRosClass::initializeServices()
{
    ROS_INFO("Initializing Services");
    minimal_service = nh.advertiseService("example_minimal_service",
                                                   &ExampleRosClass::serviceCallback,
                                                   this);
}
void ExampleRosClass::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    minimal_publisher = nh.advertise<std_msgs::Float32>("example_class_output_topic",
                                                          100,
                                                          true);
}
void ExampleRosClass::subscriberCallback(const std_msgs::Float32& message_holder)
{
    val_from_subscriber = message_holder.data;
    ROS_INFO("myCallback activated: received value %f", val_from_subscriber);
}
bool ExampleRosClass::serviceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response)
{
    ROS_INFO("service callback activated");
    response.success = true;
    response.message = "here is a response string";
    return true;
}
void ExampleRosClass::initializeDynamicReconfigurations()
{
    server.setCallback(boost::bind(&ExampleRosClass::reconfigCallback, this, _1, _2));
}
void ExampleRosClass::initializeTransformBroadcaster()
{
    ROS_INFO("Initializing Transform Broadcaster");
    tf::Transform transform;
    x_coord = 0.0;
    y_coord = 0.0;
    transform.setOrigin(tf::Vector3(x_coord, y_coord, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
}
void ExampleRosClass::reconfigCallback(template_pkg::ExampleNodeConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: %d %f %s %s %d",
             config.int_param, config.double_param,
             config.str_param.c_str(),
             config.bool_param?"True":"False",
             config.size);
}
void ExampleRosClass::publishMessage(float count)
{
    std_msgs::Float32 msg;
    msg.data = count;
    ROS_INFO("Publishing Message: %f", msg.data);
    /*
    * The publish() function is how you send messages. The parameter
    * is the message object. The type of this object must agree with the type
    * given as a template parameter to the advertise call
    */
    minimal_publisher.publish(msg);
}
void ExampleRosClass::broadcastTransform(const double& dx, const double& dy)
{
    // increment the transform coordinates
    x_coord = x_coord + dx;
    y_coord = y_coord + dy;
    ROS_INFO("Broadcasting Transform: x=%f y=%f", x_coord, y_coord);
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x_coord, y_coord, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
}
void ExampleRosClass::listenTransform()
{
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform("/world", "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    // publish the y coordinate to the example_class_output_topic
    publishMessage(transform.getOrigin().y());
}
/*!
    Orchestrates the entire node
    \return 0
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "exampleRosClass");
    ros::NodeHandle n;
    ROS_INFO("main: instantiating an object of type ExampleRosClass");
    ExampleRosClass exampleRosClass(&n);
    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::Rate r(1.0);  // 1 hz
    const double dx = 0.005; const double dy = 0.01;
    while (ros::ok())
    {
        // broadcast a transform
        exampleRosClass.broadcastTransform(dx, dy);
        // spin for callbacks
        ros::spinOnce();
        r.sleep();
        // listen to a transform
        exampleRosClass.listenTransform();
    }
    return 0;
}
