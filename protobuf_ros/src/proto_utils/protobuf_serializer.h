#ifndef PROTO_UTILS_PROTOBUF_SERIALIZER_H_
#define PROTO_UTILS_PROTOBUF_SERIALIZER_H_

#include <ros/message_traits.h>
#include <ros/serialization.h>
#include "protobuf_adapter.h"

// RefURL: 
// http://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes
// http://wiki.ros.org/roscpp/Overview/MessagesTraits
// 
// Due to the template-based serialization system used by roscpp since ROS 1.1, 
// it is possible to adapt an external type for use with ROS publish/subscribe // without modifying that type at all. 
// This is done through a set of specialized traits classes, 
// as well as a specialized Serializer class.
//
// To define serialization for a type you must define a specialization of 
// the ros::serialization::Serializer class
// 
// template<typename T>
// struct Serializer
// {
//   template<typename Stream>
//   inline static void write(Stream& stream, typename boost::call_traits<T>::param_type t);
//   template<typename Stream>
//   inline static void read(Stream& stream, typename boost::call_traits<T>::reference t);
//   inline static uint32_t serializedLength(typename boost::call_traits<T>::param_type t);
// };

namespace ros {
namespace message_traits {

// md5sum
template <class ProtoMsgType>
struct MD5Sum<ProtoBufAdapter<ProtoMsgType>> {
    static const char* value() { return "proto_md5"; }
    static const char* value(const ProtoBufAdapter<ProtoMsgType>& m) {
        return value();
    }
};

// datatype
template <class ProtoMsgType>
struct DataType<ProtoBufAdapter<ProtoMsgType>> {
    static const char* value() { 
        static std::string* datatype = 
            new std::string("proto_msg/" + ProtoMsgType::descriptor()->name());
        return datatype->c_str();
    }
    static const char* value(const ProtoBufAdapter<ProtoMsgType>& m) {
        return value();
    }
};


// definition 
template <class ProtoMsgType>
struct Definition<ProtoBufAdapter<ProtoMsgType>> {
    static const char* value() { return "proto_definition"; }
    static const char* value(const ProtoBufAdapter<ProtoMsgType>& m) {
        return value();
    }
};

} // namespace message_traits


namespace serialization {
template <class ProtoMsgType>
struct Serializer<ProtoBufAdapter<ProtoMsgType>> {
    // write
    template <typename Stream>
    inline static void write(Stream& stream, const ProtoBufAdapter<ProtoMsgType>& m) {
        // Directly serialize to the stream so that we don't have to memcpy
        // or create a temporary string.
        // Write the proto size first.
        const uint32_t size = static_cast<uint32_t>(m.GetByteSize());
        stream.next(size);
        void* dest = stream.advance(size);
        const ProtoMsgType& msg = m.msg();
        msg.SerializeToArray(dest, size);
        //if (!ok) abort();
    }
    // read
    template <typename Stream>
    inline static void read(Stream& stream, ProtoBufAdapter<ProtoMsgType>& m) {
        // Directly parse from the stream to avoid memcpy.
        uint32_t len;
        stream.next(len);
        m.mutable_msg()->ParseFromArray(stream.advance(len), len);
        //if (!ok) abort();
    }
    // serializedLength
    inline static uint32_t serializedLength(const ProtoBufAdapter<ProtoMsgType>& m) {
        // 4 bytes for writing out the size at beginning
        return m.GetByteSize() + 4;
    }
};
} // namespace serialization
} // namespace ros


#endif // PROTO_UTILS_PROTOBUF_SERIALIZER_H_
