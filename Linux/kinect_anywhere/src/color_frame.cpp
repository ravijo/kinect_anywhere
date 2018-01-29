/**
* color_frame.cpp
* Author: Ravi Joshi
* Date: 2018/01/16
*/

#include <ros/ros.h>
#include <zmq.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#define DATA_TYPE_BYTES 4

int main(int argc, char** argv)
{
    ros::init(argc, argv, "color_frame", ros::init_options::AnonymousName);
    ros::NodeHandle n("~");

    std::string host, frame_id;
    int timeout;
    n.getParam("host", host);
    n.getParam("recv_timeout", timeout);
    n.getParam("kinect_frame_id", frame_id);

    if (host.empty())
    {
        ROS_FATAL("Missing 'host' info in launch file");
        exit(-1);
    }

    // Create a ROS publisher for the deserialized stream output.
    image_transport::ImageTransport image_transport(n);
    image_transport::CameraPublisher camera_publisher = image_transport.advertiseCamera("image", 1);

    /*
    camera_info_manager::CameraInfoManager camera_info_manager(n, "rgb");
    camera_info_manager.loadCameraInfo("");
    */

    sensor_msgs::CameraInfo camera_info;
    camera_info.height = 1080;
    camera_info.width = 1920;
    camera_info.distortion_model = "plumb_bob";
    camera_info.D = { 0.0, 0.0, 0.0, 0.0, 0.0 };
    camera_info.K = { 1081.3720703125, 0.0, 959.5, 0.0, 1081.3720703125, 539.5, 0.0, 0.0, 1.0 };
    camera_info.R = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
    camera_info.P = { 1081.3720703125, 0.0, 959.5, 0.0, 0.0, 1081.3720703125, 539.5, 0.0, 0.0, 0.0, 1.0, 0.0 };
    camera_info.binning_x = 0;
    camera_info.binning_y = 0;
    camera_info.roi.x_offset = 0;
    camera_info.roi.y_offset = 0;
    camera_info.roi.height = 0;
    camera_info.roi.width = 0;
    camera_info.roi.do_rectify = false;

    //  Prepare our context and publisher
    zmq::context_t zmq_context(1);
    zmq::socket_t zmq_socket(zmq_context, ZMQ_SUB);

    std::string TOPIC = "";
    zmq_socket.setsockopt(ZMQ_SUBSCRIBE, TOPIC.c_str(), TOPIC.length()); // allow all messages

    // Timeout to get out of the while loop since recv is blocking
    zmq_socket.setsockopt(ZMQ_RCVTIMEO, &timeout, sizeof(timeout));

    int linger = 0; // Proper shutdown ZeroMQ
    zmq_socket.setsockopt(ZMQ_LINGER, &linger, sizeof(linger));

    int conflate = 1;
    zmq_socket.setsockopt(ZMQ_CONFLATE, &conflate, sizeof(conflate));

    int tcp_keepalive = 1;
    zmq_socket.setsockopt(ZMQ_TCP_KEEPALIVE, &tcp_keepalive, sizeof(tcp_keepalive));

    int tcp_keepalive_idle = 30;
    zmq_socket.setsockopt(ZMQ_TCP_KEEPALIVE_IDLE, &tcp_keepalive_idle, sizeof(tcp_keepalive_idle));

    int tcp_keepalive_intvl = 5;
    zmq_socket.setsockopt(ZMQ_TCP_KEEPALIVE_INTVL, &tcp_keepalive_intvl, sizeof(tcp_keepalive_intvl));

    int tcp_keepalive_cnt = 6;
    zmq_socket.setsockopt(ZMQ_TCP_KEEPALIVE_CNT, &tcp_keepalive_cnt, sizeof(tcp_keepalive_cnt));

    std::string socket_address = "tcp://" + host + ":10000";
    zmq_socket.connect(socket_address.c_str());

    ros::Duration duration(0.1); // in seconds (100 ms)
    while (ros::ok())
    {
        zmq::message_t msg;
        int rc = 0;
        try
        {
          rc = zmq_socket.recv(&msg);
        }
        catch(zmq::error_t& e)
        {
          ROS_DEBUG_STREAM("ZMQ Error. " << e.what());
        }
        if (rc)
        {
            unsigned char* byte_ptr = static_cast<unsigned char*>(msg.data());
            const int msg_length = msg.size();

            int width;
            std::memcpy(&width, byte_ptr, DATA_TYPE_BYTES); // integer consumes 4 bytes
            byte_ptr += DATA_TYPE_BYTES;

            int height;
            std::memcpy(&height, byte_ptr, DATA_TYPE_BYTES); // integer consumes 4 bytes
            byte_ptr += DATA_TYPE_BYTES;

            double timestamp = 0.01;
            cv::Mat image(cv::Size(width, height), CV_8UC4, byte_ptr, cv::Mat::AUTO_STEP);
            cv::flip(image, image, 1);

            cv_bridge::CvImage cv_image;
            cv_image.header.frame_id = frame_id;
            cv_image.header.stamp = ros::Time::now();
            cv_image.encoding = "bgra8";
            cv_image.image = image;

            sensor_msgs::Image ros_image;
            cv_image.toImageMsg(ros_image);

            camera_info.header.frame_id = cv_image.header.frame_id;
            camera_info.header.stamp = cv_image.header.stamp;

            camera_publisher.publish(ros_image, camera_info, ros::Time(timestamp));
            ros::spinOnce();
        }
        else
        {
          ROS_DEBUG_STREAM("Color recv() returned 0");
          duration.sleep();
        }
    }

    // Clean up your socket and context here
    zmq_socket.close();
    zmq_context.close();


    return 0;
}
