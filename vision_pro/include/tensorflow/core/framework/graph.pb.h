// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: tensorflow/core/framework/graph.proto

#ifndef PROTOBUF_tensorflow_2fcore_2fframework_2fgraph_2eproto__INCLUDED
#define PROTOBUF_tensorflow_2fcore_2fframework_2fgraph_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3003000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3003000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include "tensorflow/core/framework/node_def.pb.h"
#include "tensorflow/core/framework/function.pb.h"
#include "tensorflow/core/framework/versions.pb.h"
// @@protoc_insertion_point(includes)
namespace tensorflow {
class FunctionDef;
class FunctionDefDefaultTypeInternal;
extern FunctionDefDefaultTypeInternal _FunctionDef_default_instance_;
class FunctionDefLibrary;
class FunctionDefLibraryDefaultTypeInternal;
extern FunctionDefLibraryDefaultTypeInternal _FunctionDefLibrary_default_instance_;
class FunctionDef_AttrEntry;
class FunctionDef_AttrEntryDefaultTypeInternal;
extern FunctionDef_AttrEntryDefaultTypeInternal _FunctionDef_AttrEntry_default_instance_;
class FunctionDef_RetEntry;
class FunctionDef_RetEntryDefaultTypeInternal;
extern FunctionDef_RetEntryDefaultTypeInternal _FunctionDef_RetEntry_default_instance_;
class GradientDef;
class GradientDefDefaultTypeInternal;
extern GradientDefDefaultTypeInternal _GradientDef_default_instance_;
class GraphDef;
class GraphDefDefaultTypeInternal;
extern GraphDefDefaultTypeInternal _GraphDef_default_instance_;
class NodeDef;
class NodeDefDefaultTypeInternal;
extern NodeDefDefaultTypeInternal _NodeDef_default_instance_;
class NodeDef_AttrEntry;
class NodeDef_AttrEntryDefaultTypeInternal;
extern NodeDef_AttrEntryDefaultTypeInternal _NodeDef_AttrEntry_default_instance_;
class VersionDef;
class VersionDefDefaultTypeInternal;
extern VersionDefDefaultTypeInternal _VersionDef_default_instance_;
}  // namespace tensorflow

namespace tensorflow {

namespace protobuf_tensorflow_2fcore_2fframework_2fgraph_2eproto {
// Internal implementation detail -- do not call these.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[];
  static const ::google::protobuf::uint32 offsets[];
  static void InitDefaultsImpl();
  static void Shutdown();
};
void AddDescriptors();
void InitDefaults();
}  // namespace protobuf_tensorflow_2fcore_2fframework_2fgraph_2eproto

// ===================================================================

class GraphDef : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:tensorflow.GraphDef) */ {
 public:
  GraphDef();
  virtual ~GraphDef();

  GraphDef(const GraphDef& from);

  inline GraphDef& operator=(const GraphDef& from) {
    CopyFrom(from);
    return *this;
  }

  inline ::google::protobuf::Arena* GetArena() const PROTOBUF_FINAL {
    return GetArenaNoVirtual();
  }
  inline void* GetMaybeArenaPointer() const PROTOBUF_FINAL {
    return MaybeArenaPtr();
  }
  static const ::google::protobuf::Descriptor* descriptor();
  static const GraphDef& default_instance();

  static inline const GraphDef* internal_default_instance() {
    return reinterpret_cast<const GraphDef*>(
               &_GraphDef_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void UnsafeArenaSwap(GraphDef* other);
  void Swap(GraphDef* other);

  // implements Message ----------------------------------------------

  inline GraphDef* New() const PROTOBUF_FINAL { return New(NULL); }

  GraphDef* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const GraphDef& from);
  void MergeFrom(const GraphDef& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(GraphDef* other);
  protected:
  explicit GraphDef(::google::protobuf::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::google::protobuf::Arena* arena);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // repeated .tensorflow.NodeDef node = 1;
  int node_size() const;
  void clear_node();
  static const int kNodeFieldNumber = 1;
  const ::tensorflow::NodeDef& node(int index) const;
  ::tensorflow::NodeDef* mutable_node(int index);
  ::tensorflow::NodeDef* add_node();
  ::google::protobuf::RepeatedPtrField< ::tensorflow::NodeDef >*
      mutable_node();
  const ::google::protobuf::RepeatedPtrField< ::tensorflow::NodeDef >&
      node() const;

  // .tensorflow.FunctionDefLibrary library = 2;
  bool has_library() const;
  void clear_library();
  static const int kLibraryFieldNumber = 2;
  private:
  void _slow_mutable_library();
  void _slow_set_allocated_library(
      ::google::protobuf::Arena* message_arena, ::tensorflow::FunctionDefLibrary** library);
  ::tensorflow::FunctionDefLibrary* _slow_release_library();
  public:
  const ::tensorflow::FunctionDefLibrary& library() const;
  ::tensorflow::FunctionDefLibrary* mutable_library();
  ::tensorflow::FunctionDefLibrary* release_library();
  void set_allocated_library(::tensorflow::FunctionDefLibrary* library);
  ::tensorflow::FunctionDefLibrary* unsafe_arena_release_library();
  void unsafe_arena_set_allocated_library(
      ::tensorflow::FunctionDefLibrary* library);

  // .tensorflow.VersionDef versions = 4;
  bool has_versions() const;
  void clear_versions();
  static const int kVersionsFieldNumber = 4;
  private:
  void _slow_mutable_versions();
  void _slow_set_allocated_versions(
      ::google::protobuf::Arena* message_arena, ::tensorflow::VersionDef** versions);
  ::tensorflow::VersionDef* _slow_release_versions();
  public:
  const ::tensorflow::VersionDef& versions() const;
  ::tensorflow::VersionDef* mutable_versions();
  ::tensorflow::VersionDef* release_versions();
  void set_allocated_versions(::tensorflow::VersionDef* versions);
  ::tensorflow::VersionDef* unsafe_arena_release_versions();
  void unsafe_arena_set_allocated_versions(
      ::tensorflow::VersionDef* versions);

  // int32 version = 3 [deprecated = true];
  GOOGLE_PROTOBUF_DEPRECATED_ATTR void clear_version();
  GOOGLE_PROTOBUF_DEPRECATED_ATTR static const int kVersionFieldNumber = 3;
  GOOGLE_PROTOBUF_DEPRECATED_ATTR ::google::protobuf::int32 version() const;
  GOOGLE_PROTOBUF_DEPRECATED_ATTR void set_version(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:tensorflow.GraphDef)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  friend class ::google::protobuf::Arena;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::google::protobuf::RepeatedPtrField< ::tensorflow::NodeDef > node_;
  ::tensorflow::FunctionDefLibrary* library_;
  ::tensorflow::VersionDef* versions_;
  ::google::protobuf::int32 version_;
  mutable int _cached_size_;
  friend struct protobuf_tensorflow_2fcore_2fframework_2fgraph_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// GraphDef

// repeated .tensorflow.NodeDef node = 1;
inline int GraphDef::node_size() const {
  return node_.size();
}
inline void GraphDef::clear_node() {
  node_.Clear();
}
inline const ::tensorflow::NodeDef& GraphDef::node(int index) const {
  // @@protoc_insertion_point(field_get:tensorflow.GraphDef.node)
  return node_.Get(index);
}
inline ::tensorflow::NodeDef* GraphDef::mutable_node(int index) {
  // @@protoc_insertion_point(field_mutable:tensorflow.GraphDef.node)
  return node_.Mutable(index);
}
inline ::tensorflow::NodeDef* GraphDef::add_node() {
  // @@protoc_insertion_point(field_add:tensorflow.GraphDef.node)
  return node_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::tensorflow::NodeDef >*
GraphDef::mutable_node() {
  // @@protoc_insertion_point(field_mutable_list:tensorflow.GraphDef.node)
  return &node_;
}
inline const ::google::protobuf::RepeatedPtrField< ::tensorflow::NodeDef >&
GraphDef::node() const {
  // @@protoc_insertion_point(field_list:tensorflow.GraphDef.node)
  return node_;
}

// .tensorflow.VersionDef versions = 4;
inline bool GraphDef::has_versions() const {
  return this != internal_default_instance() && versions_ != NULL;
}
inline void GraphDef::clear_versions() {
  if (GetArenaNoVirtual() == NULL && versions_ != NULL) delete versions_;
  versions_ = NULL;
}
inline const ::tensorflow::VersionDef& GraphDef::versions() const {
  // @@protoc_insertion_point(field_get:tensorflow.GraphDef.versions)
  return versions_ != NULL ? *versions_
                         : *::tensorflow::VersionDef::internal_default_instance();
}
inline ::tensorflow::VersionDef* GraphDef::mutable_versions() {
  
  if (versions_ == NULL) {
    _slow_mutable_versions();
  }
  // @@protoc_insertion_point(field_mutable:tensorflow.GraphDef.versions)
  return versions_;
}
inline ::tensorflow::VersionDef* GraphDef::release_versions() {
  // @@protoc_insertion_point(field_release:tensorflow.GraphDef.versions)
  
  if (GetArenaNoVirtual() != NULL) {
    return _slow_release_versions();
  } else {
    ::tensorflow::VersionDef* temp = versions_;
    versions_ = NULL;
    return temp;
  }
}
inline  void GraphDef::set_allocated_versions(::tensorflow::VersionDef* versions) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete versions_;
  }
  if (versions != NULL) {
    _slow_set_allocated_versions(message_arena, &versions);
  }
  versions_ = versions;
  if (versions) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_set_allocated:tensorflow.GraphDef.versions)
}

// int32 version = 3 [deprecated = true];
inline void GraphDef::clear_version() {
  version_ = 0;
}
inline ::google::protobuf::int32 GraphDef::version() const {
  // @@protoc_insertion_point(field_get:tensorflow.GraphDef.version)
  return version_;
}
inline void GraphDef::set_version(::google::protobuf::int32 value) {
  
  version_ = value;
  // @@protoc_insertion_point(field_set:tensorflow.GraphDef.version)
}

// .tensorflow.FunctionDefLibrary library = 2;
inline bool GraphDef::has_library() const {
  return this != internal_default_instance() && library_ != NULL;
}
inline void GraphDef::clear_library() {
  if (GetArenaNoVirtual() == NULL && library_ != NULL) delete library_;
  library_ = NULL;
}
inline const ::tensorflow::FunctionDefLibrary& GraphDef::library() const {
  // @@protoc_insertion_point(field_get:tensorflow.GraphDef.library)
  return library_ != NULL ? *library_
                         : *::tensorflow::FunctionDefLibrary::internal_default_instance();
}
inline ::tensorflow::FunctionDefLibrary* GraphDef::mutable_library() {
  
  if (library_ == NULL) {
    _slow_mutable_library();
  }
  // @@protoc_insertion_point(field_mutable:tensorflow.GraphDef.library)
  return library_;
}
inline ::tensorflow::FunctionDefLibrary* GraphDef::release_library() {
  // @@protoc_insertion_point(field_release:tensorflow.GraphDef.library)
  
  if (GetArenaNoVirtual() != NULL) {
    return _slow_release_library();
  } else {
    ::tensorflow::FunctionDefLibrary* temp = library_;
    library_ = NULL;
    return temp;
  }
}
inline  void GraphDef::set_allocated_library(::tensorflow::FunctionDefLibrary* library) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete library_;
  }
  if (library != NULL) {
    _slow_set_allocated_library(message_arena, &library);
  }
  library_ = library;
  if (library) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_set_allocated:tensorflow.GraphDef.library)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)


}  // namespace tensorflow

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_tensorflow_2fcore_2fframework_2fgraph_2eproto__INCLUDED
