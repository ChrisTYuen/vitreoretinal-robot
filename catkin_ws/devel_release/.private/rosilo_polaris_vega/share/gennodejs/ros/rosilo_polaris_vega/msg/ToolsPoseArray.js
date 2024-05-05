// Auto-generated. Do not edit!

// (in-package rosilo_polaris_vega.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ToolsPoseArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.frame_numbers = null;
      this.port_handles = null;
      this.statuses = null;
      this.poses = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('frame_numbers')) {
        this.frame_numbers = initObj.frame_numbers
      }
      else {
        this.frame_numbers = [];
      }
      if (initObj.hasOwnProperty('port_handles')) {
        this.port_handles = initObj.port_handles
      }
      else {
        this.port_handles = [];
      }
      if (initObj.hasOwnProperty('statuses')) {
        this.statuses = initObj.statuses
      }
      else {
        this.statuses = [];
      }
      if (initObj.hasOwnProperty('poses')) {
        this.poses = initObj.poses
      }
      else {
        this.poses = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ToolsPoseArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [frame_numbers]
    bufferOffset = _arraySerializer.int64(obj.frame_numbers, buffer, bufferOffset, null);
    // Serialize message field [port_handles]
    bufferOffset = _arraySerializer.int32(obj.port_handles, buffer, bufferOffset, null);
    // Serialize message field [statuses]
    bufferOffset = _arraySerializer.int32(obj.statuses, buffer, bufferOffset, null);
    // Serialize message field [poses]
    // Serialize the length for message field [poses]
    bufferOffset = _serializer.uint32(obj.poses.length, buffer, bufferOffset);
    obj.poses.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Pose.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ToolsPoseArray
    let len;
    let data = new ToolsPoseArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [frame_numbers]
    data.frame_numbers = _arrayDeserializer.int64(buffer, bufferOffset, null)
    // Deserialize message field [port_handles]
    data.port_handles = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [statuses]
    data.statuses = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [poses]
    // Deserialize array length for message field [poses]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.poses = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.poses[i] = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 8 * object.frame_numbers.length;
    length += 4 * object.port_handles.length;
    length += 4 * object.statuses.length;
    length += 56 * object.poses.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rosilo_polaris_vega/ToolsPoseArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '297b84503018526d42525e774a191c54';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    int64[] frame_numbers
    int32[] port_handles
    int32[] statuses
    geometry_msgs/Pose[] poses
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ToolsPoseArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.frame_numbers !== undefined) {
      resolved.frame_numbers = msg.frame_numbers;
    }
    else {
      resolved.frame_numbers = []
    }

    if (msg.port_handles !== undefined) {
      resolved.port_handles = msg.port_handles;
    }
    else {
      resolved.port_handles = []
    }

    if (msg.statuses !== undefined) {
      resolved.statuses = msg.statuses;
    }
    else {
      resolved.statuses = []
    }

    if (msg.poses !== undefined) {
      resolved.poses = new Array(msg.poses.length);
      for (let i = 0; i < resolved.poses.length; ++i) {
        resolved.poses[i] = geometry_msgs.msg.Pose.Resolve(msg.poses[i]);
      }
    }
    else {
      resolved.poses = []
    }

    return resolved;
    }
};

module.exports = ToolsPoseArray;
