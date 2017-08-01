// Auto-generated. Do not edit!

// (in-package mitya_teleop.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class HeadPosition {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.horizontal = null;
      this.vertical = null;
    }
    else {
      if (initObj.hasOwnProperty('horizontal')) {
        this.horizontal = initObj.horizontal
      }
      else {
        this.horizontal = 0.0;
      }
      if (initObj.hasOwnProperty('vertical')) {
        this.vertical = initObj.vertical
      }
      else {
        this.vertical = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HeadPosition
    // Serialize message field [horizontal]
    bufferOffset = _serializer.float32(obj.horizontal, buffer, bufferOffset);
    // Serialize message field [vertical]
    bufferOffset = _serializer.float32(obj.vertical, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HeadPosition
    let len;
    let data = new HeadPosition(null);
    // Deserialize message field [horizontal]
    data.horizontal = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vertical]
    data.vertical = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mitya_teleop/HeadPosition';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '93817796a8d7b54c196b10312493371b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 horizontal
    float32 vertical
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HeadPosition(null);
    if (msg.horizontal !== undefined) {
      resolved.horizontal = msg.horizontal;
    }
    else {
      resolved.horizontal = 0.0
    }

    if (msg.vertical !== undefined) {
      resolved.vertical = msg.vertical;
    }
    else {
      resolved.vertical = 0.0
    }

    return resolved;
    }
};

module.exports = HeadPosition;
