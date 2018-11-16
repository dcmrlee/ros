#ifndef PROTO_UTILS_PROTOBUF_PUBLISHER_H_
#define PROTO_UTILS_PROTOBUF_PUBLISHER_H_

#include <string>
#include "protobuf_serializer.h"

// Class ProtoPublisher is inherited from the ros publisher and specialized
// for publishing proto messages through ros::ProtoBufAdapter. We use
// inheritance so that this proto publisher has all the public functions from
// ros::Publisher.
//
// Example usage:
// ros::NodeHandle nh;
// proto_util::ProtoPublisher pub =
//     static_cast<proto_util::ProtoPublisher>(
//         nh.advertise<ros::ProtoBufAdapter<ProtoMsgType>>(
//             "/test_proto" /* topic */, 0 /* queue size */));
// pub.PublishProto(proto_message_ptr);
// pub.getNumSubscribers(); // It has normal publisher functions as well.
//
// Note that because internally it uses a boost shared pointer to store the
// proto, we require user pass a pointer and internally swap the message to
// avoid extra copy.
//

namespace proto_utils {
class ProtoPublisher : public ros::Publisher {
    public:
        ProtoPublisher() = default;

        explicit ProtoPublisher(const Publisher& rhs) : Publisher(rhs) {}

        // Publishes the proto. We use a raw pointer input to swap proto and avoid
        // copy
        template <typename ProtoMsgType>
        void PublishProto(ProtoMsgType* proto) const {
            auto msg = boost::make_shared<ros::ProtoBufAdapter<ProtoMsgType>>(proto);
            publish(msg);
        }

        // Publishes the proto when the input is a const reference. We make a copy
        // of the proto inside
        template <typename ProtoMsgType>
        void PublishProto(const ProtoMsgType& proto) const {
            ProtoMsgType proto_copy = proto;
            PublishProto(&proto_copy);
        }

    private:
        // A friend function AdvertiseProto creates this proto publisher.
        template <class ProtoMsgType, typename... Args>
        friend ProtoPublisher AdvertiseProto(ros::NodeHandle* nh, Args&&... args);
};


// Helper function to make it easier to advertise proto. Note that we put
// mutable NodeHandle as the first parameter so that we can forward all the
// parameters to ros::NodeHandle::advertise(...).
// Example usage:
//
// proto_util::ProtoPublisher pub =
//     AdvertiseProto<ProtoType>(
//         &node_handler, "/test_proto" /* topic */, 0 /* queue size */);

template <class ProtoMsgType, typename... Args>
ProtoPublisher AdvertiseProto(ros::NodeHandle* nh, Args&&... args) {
    ProtoPublisher proto_publisher = static_cast<proto_utils::ProtoPublisher>(
            nh->advertise<ros::ProtoBufAdapter<ProtoMsgType>>(std::forward<Args>(args)...));

    const std::string topic = proto_publisher.getTopic();
    //ROS_INFO("topic is %s", topic);
    return proto_publisher;
}
} // namespace proto_utils

#endif // PROTO_UTILS_PROTOBUF_PUBLISHER_H_
