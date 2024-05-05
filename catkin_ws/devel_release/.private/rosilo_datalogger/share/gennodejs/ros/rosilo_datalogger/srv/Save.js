// Auto-generated. Do not edit!

// (in-package rosilo_datalogger.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SaveRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.filename = null;
    }
    else {
      if (initObj.hasOwnProperty('filename')) {
        this.filename = initObj.filename
      }
      else {
        this.filename = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SaveRequest
    // Serialize message field [filename]
    bufferOffset = _serializer.string(obj.filename, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SaveRequest
    let len;
    let data = new SaveRequest(null);
    // Deserialize message field [filename]
    data.filename = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.filename);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rosilo_datalogger/SaveRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '030824f52a0628ead956fb9d67e66ae9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string filename
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SaveRequest(null);
    if (msg.filename !== undefined) {
      resolved.filename = msg.filename;
    }
    else {
      resolved.filename = ''
    }

    return resolved;
    }
};

class SaveResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.worked = null;
    }
    else {
      if (initObj.hasOwnProperty('worked')) {
        this.worked = initObj.worked
      }
      else {
        this.worked = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SaveResponse
    // Serialize message field [worked]
    bufferOffset = _serializer.bool(obj.worked, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SaveResponse
    let len;
    let data = new SaveResponse(null);
    // Deserialize message field [worked]
    data.worked = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rosilo_datalogger/SaveResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e9488d8ea76faf703564dab31aa285e5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool worked
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SaveResponse(null);
    if (msg.worked !== undefined) {
      resolved.worked = msg.worked;
    }
    else {
      resolved.worked = false
    }

    return resolved;
    }
};

module.exports = {
  Request: SaveRequest,
  Response: SaveResponse,
  md5sum() { return '86a64704c03f5415ea7ff59624fa3411'; },
  datatype() { return 'rosilo_datalogger/Save'; }
};
