#ifndef PROTO_UTILS_PROTOBUF_SUBSCRIBER_H_
#define PROTO_UTILS_PROTOBUF_SUBSCRIBER_H_

#include "protobuf_serializer.h"

// For subscribers, the main job is to convert a callback that originally
// operates on a proto message (from the user) into a boost callback that
// operate on a const pointer of boost shared_ptr (that is used in ros system).
//
// Therefore we introduce a helper function proto_util::Subscribe() to make this
// conversion. This helper accepts callable objects such as lambda,
// std::function and std::bind() expression. Assuming the proto message is
// ProtoMsgType, the signature of the callable object accepts either:
//   - const ProtoMsgType&; or
//   - const std::shared_ptr<ProtoMsgType>&.
//
// Example usage:
// 1) Using lambda as callback:
//
// // Lambda accepting const ref to proto message:
// ros::Subscriber sub = proto_util::Subscribe<voy::Pose>(
//     &node_handle, topic_name, queue_size,
//     [](const voy::Pose&) { // Do your work. });
//
// // Lambda accepting const ref to std::shared_ptr:
// ros::Subscriber sub = proto_util::Subscribe<voy::Pose>(
//     &node_handle, topic_name, queue_size,
//     [](const std::shared_ptr<voy::Pose>&) { // Do your work. });
//
// As usual, you can capture necessary objects. For example, capturing "this":
//
// ros::Subscriber sub = proto_util::Subscribe<voy::Pose>(
//     &node_handle, topic_name, queue_size,
//     [this](const voy::Pose&) { // Do your work. });
//
// 2) Using std::bind() expression as callback:
//
// // Bind to a non-member function or static member function:
// void PrintProto(const voy::Pose& pose_proto) { // ... print proto... }
//
// ros::Subscriber sub = proto_util::Subscribe<voy::Pose>(
//     &node_handle, topic_name, queue_size,
//     std::bind(&PrintProto, std::placeholders::_1));
//
// NOTE: IMPORTANT!
// When you capture objects by reference or capture pointers in callables,
// please be careful about their lifespan. Capturing them do NOT extend their
// lifespan.
//
// When you register a callback to create a subscriber, you need to make sure
// the objects it refers to outlive the subscription. ros::Subscriber is
// copyable, so you need to be extra careful to avoid creating dangling
// references or pointers.


namespace proto_utils {

template <class ProtoMsgType>
using BoostProtoPtr = 
    boost::shared_ptr<const ros::ProtoBufAdapter<ProtoMsgType>>;

template <class ProtoMsgType>
using BoostProtoPtrCallback = 
    boost::function<void(const proto_utils::BoostProtoPtr<ProtoMsgType>&)>;

namespace detail {
    template <class ProtoMsgType, class CallbackType, class EnableType = void>
    struct AdapterCallbackFactory;

    // Template specialization for callable that is able to accept a ProtoMsgType
    // instance
    template <class ProtoMsgType, typename CallbackType>
    struct AdapterCallbackFactory<ProtoMsgType, CallbackType,
            decltype(std::declval<CallbackType>()(std::declval<ProtoMsgType>()))> {
        inline static BoostProtoPtrCallback<ProtoMsgType> Create(CallbackType&& callback) {
            auto adapter_callback = [callback](const proto_utils::BoostProtoPtr<ProtoMsgType>& pb_adapter_ptr) { callback(pb_adapter_ptr->msg()); };
            return static_cast<BoostProtoPtrCallback<ProtoMsgType>>(adapter_callback);
        }
    };

    // Template specialization for callable that is able to accept a
    // std::shared_ptr<ProtoMsgType>.
    template <class ProtoMsgType, typename CallbackType>
    struct AdapterCallbackFactory<ProtoMsgType, CallbackType,
            decltype(std::declval<CallbackType>()(std::declval<std::shared_ptr<ProtoMsgType>>()))> {
        inline static BoostProtoPtrCallback<ProtoMsgType> Create(CallbackType&& callback) {
            auto adapter_callback = [callback](const proto_utils::BoostProtoPtr<ProtoMsgType>& pb_adapter_ptr) { callback(pb_adapter_ptr->msg_ptr()); };
            return static_cast<BoostProtoPtrCallback<ProtoMsgType>>(adapter_callback);
        }
    };

} // namespace detail


// A convenient helper function to subscribe with a callable object accepting
// a proto message. Please see the comments at the top for more details about
// usage.
//
// This helper also automatically applies common transports hints.
template <class ProtoMsgType, class CallbackType>
ros::Subscriber Subscribe(ros::NodeHandle* node_handle,
        const std::string& topic, uint32_t queue_size,
        CallbackType&& callback) {
    return node_handle->subscribe(
            topic, queue_size,
            detail::AdapterCallbackFactory<ProtoMsgType, CallbackType>::Create(std::forward<CallbackType>(callback)),
            ros::VoidConstPtr(),
            ros::TransportHints().tcpNoDelay() // for performance
            );
}

} // namespace proto_utils

#endif // PROTO_UTILS_PROTOBUF_SUBSCRIBER_H_
