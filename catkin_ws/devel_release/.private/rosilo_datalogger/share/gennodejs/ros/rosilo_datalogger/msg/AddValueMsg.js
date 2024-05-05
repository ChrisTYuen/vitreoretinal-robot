// Auto-generated. Do not edit!

// (in-package rosilo_datalogger.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class AddValueMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.value = null;
      this.strvalue = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = [];
      }
      if (initObj.hasOwnProperty('strvalue')) {
        this.strvalue = initObj.strvalue
      }
      else {
        this.strvalue = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AddValueMsg
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [value]
    bufferOffset = _arraySerializer.float64(obj.value, buffer, bufferOffset, null);
    // Serialize message field [strvalue]
    bufferOffset = _serializer.string(obj.strvalue, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AddValueMsg
    let len;
    let data = new AddValueMsg(null);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [value]
    data.value = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [strvalue]
    data.strvalue = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.name);
    length += 8 * object.value.length;
    length += _getByteLength(object.strvalue);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rosilo_datalogger/AddValueMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd17a07a93f3e7d7722ef08216d8e72f9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string name
    float64[] value
    string strvalue
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AddValueMsg(null);
    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = []
    }

    if (msg.strvalue !== undefined) {
      resolved.strvalue = msg.strvalue;
    }
    else {
      resolved.strvalue = ''
    }

    return resolved;
    }
};

module.exports = AddValueMsg;
