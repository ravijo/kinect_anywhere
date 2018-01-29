/**
* body_frame.cpp
* Author: Ravi Joshi
* Date: 2018/01/16
*/

#include <ros/ros.h>
#include <zmq.hpp>
#include <kinect_anywhere/BodyFrame.h>
#include <geometry_msgs/Point.h>

#define DATA_TYPE_BYTES 4
#define BODY_TO_JOINT_OFFSET_BYTES 12
#define JOINT_DATA_BYTES 20

/*
 * BodyFrame = BodyCount (4 bytes) + [BodyData]
 * BodyData = TrackingId (8 bytes) + JointCount (4 bytes) + [JointPositionAndState]
 * JointPositionAndState = TrackingState (4 bytes) + JointType (4 bytes)
 * + x (4 bytes) + y (4 bytes) + z (4 bytes)
 */

int main(int argc, char** argv)
{
    ros::init(argc, argv, "body_frame", ros::init_options::AnonymousName);
    ros::NodeHandle n("~");
    ros::Publisher bodyPublisher = n.advertise<kinect_anywhere::BodyFrame>("bodies", 1);

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

    std::string socket_address = "tcp://" + host + ":10001";
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

            int body_count;
            std::memcpy(&body_count, byte_ptr, DATA_TYPE_BYTES);
            byte_ptr += DATA_TYPE_BYTES;

            if (body_count == 0)
            {
                continue;
            }

            int joint_count;
            std::memcpy(&joint_count, byte_ptr + 2 * DATA_TYPE_BYTES, DATA_TYPE_BYTES);

            int single_body_type_bytes
                = BODY_TO_JOINT_OFFSET_BYTES + joint_count * JOINT_DATA_BYTES;
            int all_body_type_bytes = single_body_type_bytes * body_count + DATA_TYPE_BYTES;

            bool cheeck_data_integrity = msg_length == all_body_type_bytes;
            if (!cheeck_data_integrity)
            {
                continue;
            }

            kinect_anywhere::BodyFrame ros_body_frame;

            for (int i = 0; i < body_count; i++)
            {
                // unsigned long int tracking_id; // 'tracking__id' requires 8 bytes
                uint64_t tracking_id;
                std::memcpy(&tracking_id, byte_ptr, 2 * DATA_TYPE_BYTES);
                byte_ptr += (2 * DATA_TYPE_BYTES);

                kinect_anywhere::Body ros_body;
                ros_body.trackingId = tracking_id;
                ros_body.header.stamp = ros::Time::now();
                ros_body.header.frame_id = frame_id;

                std::memcpy(&joint_count, byte_ptr, DATA_TYPE_BYTES);
                byte_ptr += DATA_TYPE_BYTES;

                for (int j = 0; j < joint_count; j++)
                {
                    int tracking_state;
                    std::memcpy(&tracking_state, byte_ptr, DATA_TYPE_BYTES);
                    byte_ptr += DATA_TYPE_BYTES;

                    int joint_type;
                    std::memcpy(&joint_type, byte_ptr, DATA_TYPE_BYTES);
                    byte_ptr += DATA_TYPE_BYTES;

                    float x;
                    std::memcpy(&x, byte_ptr, DATA_TYPE_BYTES);
                    byte_ptr += DATA_TYPE_BYTES;

                    float y;
                    std::memcpy(&y, byte_ptr, DATA_TYPE_BYTES);
                    byte_ptr += DATA_TYPE_BYTES;

                    float z;
                    std::memcpy(&z, byte_ptr, DATA_TYPE_BYTES);
                    byte_ptr += DATA_TYPE_BYTES;

                    kinect_anywhere::JointPositionAndState jointPositionAndState;
                    jointPositionAndState.trackingState = tracking_state;
                    jointPositionAndState.jointType = joint_type;
                    jointPositionAndState.position.x = x;
                    jointPositionAndState.position.y = y;
                    jointPositionAndState.position.z = z;

                    ros_body.jointPositions.push_back(jointPositionAndState);
                }
                ros_body_frame.bodies.push_back(ros_body);
            }
            bodyPublisher.publish(ros_body_frame);
            ros::spinOnce();
        }
        else
        {
          ROS_DEBUG_STREAM("Body recv() returned 0");
          duration.sleep();
        }
    }

    // Clean up your socket and context here
    zmq_socket.close();
    zmq_context.close();

    return 0;
}
