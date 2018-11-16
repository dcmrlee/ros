#ifndef PROTO_UTILS_PROTOBUF_ADAPTER_H_
#define PROTO_UTILS_PROTOBUF_ADAPTER_H_

#include <google/protobuf/message.h>
#include <google/protobuf/stubs/common.h>
#include <ros/ros.h>
#include <cassert>
#include <memory>

// This file includes the necessary class template specialization for
// serializing (writing) and de-serializing (reading) protos.
//
// RefURL: 
// http://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes
//
// Typically for a template specialization, one can implemented as (taking
// MD5Sum as an example):
// Original MD5Sum declaration in message_traits.h in roscpp_traits.
//
// template<typename M>
// struct MD5Sum {... }
//
// Then for a single object-type specialization, e.g. string, one can work as:
//
// template<>
// struct MD5Sum<std::string> { // ... specialization for strings... }
//
// However this only works for one specific type. As in ProtoBuf message
// we specialized all the derived classes of  ::google::protobuf::Message,
// which is not possible given the current interface of MD5Sum, and therefore
// we need to introduce a class ProtoBufAdapter.


namespace ros {

template <class ProtoMsgType>
class ProtoBufAdapter {
    public:
        static_assert(
                std::is_base_of<::google::protobuf::Message, ProtoMsgType>::value,
                "ProtoMsgType must be a google proto message."
                );
        ProtoBufAdapter() : proto_msg_(new ProtoMsgType()) {}

        explicit ProtoBufAdapter(ProtoMsgType *msg) : ProtoBufAdapter() {
            proto_msg_->Swap(msg);
        }

        const ProtoMsgType& msg() const {
            return *proto_msg_;
        }

        ProtoMsgType* mutable_msg() { return proto_msg_.get(); }
        std::shared_ptr<ProtoMsgType> msg_ptr() const { return proto_msg_; }

        // Protobuf's GetCachedSize() is not cached until ByteSize() is first
        // called. Here we consolidate them into one interface.
        int GetByteSize() const {
            const int cached_size = proto_msg_->GetCachedSize();
            return cached_size == 0 ? proto_msg_->ByteSize() : cached_size;
        }
        
    private:
        // The proto message owned by adapter. We need to use a shared pointer
        // instead of a unique_ptr because in ros framework, message (here proto
        // adapter) needs to be copiable.
        std::shared_ptr<ProtoMsgType> proto_msg_;

        void check_ptr() {
            if (proto_msg_ == nullptr) {
                ROS_FATAL("Failed Check proto_msg_ is nullptr");
                abort();
            }
        }
};
} // namespace ros

#endif //PROTO_UTILS_PROTOBUF_ADAPTER_H_
