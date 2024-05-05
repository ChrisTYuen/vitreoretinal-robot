// Auto-generated. Do not edit!

// (in-package robot_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

class ROI {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.roi_image = null;
      this.roi_values = null;
    }
    else {
      if (initObj.hasOwnProperty('roi_image')) {
        this.roi_image = initObj.roi_image
      }
      else {
        this.roi_image = new sensor_msgs.msg.Image();
      }
      if (initObj.hasOwnProperty('roi_values')) {
        this.roi_values = initObj.roi_values
      }
      else {
        this.roi_values = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ROI
    // Serialize message field [roi_image]
    bufferOffset = sensor_msgs.msg.Image.serialize(obj.roi_image, buffer, bufferOffset);
    // Serialize message field [roi_values]
    bufferOffset = _arraySerializer.float64(obj.roi_values, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ROI
    let len;
    let data = new ROI(null);
    // Deserialize message field [roi_image]
    data.roi_image = sensor_msgs.msg.Image.deserialize(buffer, bufferOffset);
    // Deserialize message field [roi_values]
    data.roi_values = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.Image.getMessageSize(object.roi_image);
    length += 8 * object.roi_values.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robot_control/ROI';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4345e91a8d0d0f4b5b72c6369217bad9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    sensor_msgs/Image roi_image
    
    float64[] roi_values
    
    
    ================================================================================
    MSG: sensor_msgs/Image
    # This message contains an uncompressed image
    # (0, 0) is at top-left corner of image
    #
    
    Header header        # Header timestamp should be acquisition time of image
                         # Header frame_id should be optical frame of camera
                         # origin of frame should be optical center of camera
                         # +x should point to the right in the image
                         # +y should point down in the image
                         # +z should point into to plane of the image
                         # If the frame_id here and the frame_id of the CameraInfo
                         # message associated with the image conflict
                         # the behavior is undefined
    
    uint32 height         # image height, that is, number of rows
    uint32 width          # image width, that is, number of columns
    
    # The legal values for encoding are in file src/image_encodings.cpp
    # If you want to standardize a new string format, join
    # ros-users@lists.sourceforge.net and send an email proposing a new encoding.
    
    string encoding       # Encoding of pixels -- channel meaning, ordering, size
                          # taken from the list of strings in include/sensor_msgs/image_encodings.h
    
    uint8 is_bigendian    # is this data bigendian?
    uint32 step           # Full row length in bytes
    uint8[] data          # actual matrix data, size is (step * rows)
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ROI(null);
    if (msg.roi_image !== undefined) {
      resolved.roi_image = sensor_msgs.msg.Image.Resolve(msg.roi_image)
    }
    else {
      resolved.roi_image = new sensor_msgs.msg.Image()
    }

    if (msg.roi_values !== undefined) {
      resolved.roi_values = msg.roi_values;
    }
    else {
      resolved.roi_values = []
    }

    return resolved;
    }
};

module.exports = ROI;
