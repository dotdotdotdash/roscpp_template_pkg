#ifndef EXAMPLE_HEADER_H
#define EXAMPLE_HEADER_H
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <template_pkg/ExampleNodeConfig.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
class ExampleRosClass
{
public:
    /*!
     * A constructor for the class
     */
    explicit ExampleRosClass(ros::NodeHandle* nodehandle);
    /*!
       A method to publish messages
       \param count - Counts the values
     */
    void publishMessage(float count);
    /*!
       Example of broadcasting a transform
       \param dx - change in x coordinate of mobile base in the world frame
       \param dy - change in y coordinate of mobile base in the world frame
    */
    void broadcastTransform(const double& dx, const double& dy);
    /*!
     * A method for listening to the transform
     */
    void listenTransform();
private:
    /*! Advertises message to ___ topic */
    ros::Publisher minimal_publisher;
    /*! Holds the subscription for ___ service */
    ros::ServiceServer minimal_service;
    /*! Holds the subscription for ___ topic */
    ros::Subscriber minimal_subscriber;
    /*! A node handler creates a namespace for the node */
    ros::NodeHandle nh;
    /*! Server for dynamic reconfiguration to change the params on the fly */
    dynamic_reconfigure::Server<template_pkg::ExampleNodeConfig> server;
    /*! Holds the data from the ____ message */
    double val_from_subscriber;
    /*! tf2 broadcaster instance */
    tf::TransformBroadcaster br;
    /*! tf2 listener instance */
    tf::TransformListener listener;
    /*! X coordinate value for tf2 */
    double x_coord;
    /*! Y coordinate value for tf2 */
    double y_coord;
    /*! Parameter from parameter server */
    int param1;
    /*! Parameter from parameter server */
    std::string param2;
    /*!
        A method that loads all the parameters from param server to class members
    */
    void loadParameters();
    /*!
        Initialize the subscribers for the nodehandle and calls subscriberCallback method
    */
    void initializeSubscribers();
    /*!
        Initialize the publishers and start advertising the node
    */
    void initializePublishers();
    /*!
        Initialize the services for the node
    */
    void initializeServices();
    /*!
        Initiliaze the dynamic reconfigurations for the node
    */
    void initializeDynamicReconfigurations();
    /*!
        Initialize a transform broadcaster for the node
    */
    void initializeTransformBroadcaster();
    /*!
        Initialize the subscribers for the nodehandle and calls subscriberCallback method
        \param message_holder - Message from the _____ topic
    */
    void subscriberCallback(const std_msgs::Float32& message_holder);
    /*!
        Method that advertises the service
        \param request - Receives the request from the client
        \param response - Sends out the message to the client
        \return boolean value
    */
    bool serviceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);
    /*!
        A call back method for performing dynamic reconfiguration
        \param config - An instance of the reconfig file
        \param level - An aribitrary level values
    */
    void reconfigCallback(template_pkg::ExampleNodeConfig &config, uint32_t level);
};
#endif  // EXAMPLE_HEADER_H
