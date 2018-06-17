/**
* point_cloud.cpp
* Author: Ravi Joshi
* Date: 2018/01/16
*/

#include <ros/ros.h>
#include <zmq.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <chrono>

typedef pcl::PointXYZRGBA Point;
typedef pcl::PointCloud<Point> PointCloud;

#define DATA_TYPE_BYTES 4
#define POINT_FIELD_COUNT 4

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud", ros::init_options::AnonymousName);
    ros::NodeHandle n("~");
    ros::Publisher pub = n.advertise<PointCloud>("points2", 1);

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

    std::string socket_address = "tcp://" + host + ":10002";
    zmq_socket.connect(socket_address.c_str());

    PointCloud::Ptr point_cloud_ros(new PointCloud);
    point_cloud_ros->is_dense = true;
    point_cloud_ros->height = 1;
    point_cloud_ros->header.frame_id = frame_id;

    ros::Duration duration(0.1); // in seconds (100 ms)
    while (ros::ok())
    {
        // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
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
        // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        // std::cout << "time (ms): " << std::chrono::duration_cast<std::chrono::microseconds>(t2 -
        // t1).count()/1000.0 << std::endl;
        if (rc)
        {
            unsigned char* byte_ptr = static_cast<unsigned char*>(msg.data());
            const int msg_length = msg.size();

            unsigned int point_count;
            std::memcpy(&point_count, byte_ptr, DATA_TYPE_BYTES); // integer consumes 4 bytes
            byte_ptr += DATA_TYPE_BYTES;

            bool cheeck_data_integrity = point_count
                == (msg_length - DATA_TYPE_BYTES) / (POINT_FIELD_COUNT * DATA_TYPE_BYTES);

            if (!cheeck_data_integrity)
            {
                continue;
            }

            point_cloud_ros->clear(); // Remove old points from cloud
            point_cloud_ros->width = point_count;
            point_cloud_ros->height = 1;

            // https://answers.ros.org/question/172730/pcl-header-timestamp/
            pcl_conversions::toPCL(ros::Time::now(), point_cloud_ros->header.stamp);

            // Todo: use sensor_msgs::PointCloud2 instead of pcl::PointCloud<pcl::PointXYZRBGA>
            // http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/PointCloud2.html
            for (int i = 0; i < point_count; i++)
            {
                float x;
                std::memcpy(&x, byte_ptr, DATA_TYPE_BYTES);
                byte_ptr += DATA_TYPE_BYTES;

                float y;
                std::memcpy(&y, byte_ptr, DATA_TYPE_BYTES);
                byte_ptr += DATA_TYPE_BYTES;

                float z;
                std::memcpy(&z, byte_ptr, DATA_TYPE_BYTES);
                byte_ptr += DATA_TYPE_BYTES;

                unsigned int bgra;
                std::memcpy(&bgra, byte_ptr, DATA_TYPE_BYTES);
                byte_ptr += DATA_TYPE_BYTES;

                Point point_ros;
                point_ros.x = x;
                point_ros.y = y;
                point_ros.z = z;
                point_ros.b = (bgra >> 24);
                point_ros.g = (bgra >> 16);
                point_ros.r = (bgra >> 8);
                point_ros.a = bgra;

                point_cloud_ros->push_back(point_ros);
            }
            pub.publish(point_cloud_ros);
            ros::spinOnce();
        }
        else
        {
          ROS_DEBUG_STREAM("Pointcloud recv() returned 0");
          duration.sleep();
        }
    }

    // Clean up your socket and context here
    zmq_socket.close();
    zmq_context.close();


    return 0;
}
