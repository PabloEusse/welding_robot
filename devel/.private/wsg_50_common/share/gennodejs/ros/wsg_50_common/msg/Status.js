// Auto-generated. Do not edit!

// (in-package wsg_50_common.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Status {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.status = null;
      this.width = null;
      this.speed = null;
      this.acc = null;
      this.force = null;
      this.force_finger0 = null;
      this.force_finger1 = null;
    }
    else {
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = '';
      }
      if (initObj.hasOwnProperty('width')) {
        this.width = initObj.width
      }
      else {
        this.width = 0.0;
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0.0;
      }
      if (initObj.hasOwnProperty('acc')) {
        this.acc = initObj.acc
      }
      else {
        this.acc = 0.0;
      }
      if (initObj.hasOwnProperty('force')) {
        this.force = initObj.force
      }
      else {
        this.force = 0.0;
      }
      if (initObj.hasOwnProperty('force_finger0')) {
        this.force_finger0 = initObj.force_finger0
      }
      else {
        this.force_finger0 = 0.0;
      }
      if (initObj.hasOwnProperty('force_finger1')) {
        this.force_finger1 = initObj.force_finger1
      }
      else {
        this.force_finger1 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Status
    // Serialize message field [status]
    bufferOffset = _serializer.string(obj.status, buffer, bufferOffset);
    // Serialize message field [width]
    bufferOffset = _serializer.float32(obj.width, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.float32(obj.speed, buffer, bufferOffset);
    // Serialize message field [acc]
    bufferOffset = _serializer.float32(obj.acc, buffer, bufferOffset);
    // Serialize message field [force]
    bufferOffset = _serializer.float32(obj.force, buffer, bufferOffset);
    // Serialize message field [force_finger0]
    bufferOffset = _serializer.float32(obj.force_finger0, buffer, bufferOffset);
    // Serialize message field [force_finger1]
    bufferOffset = _serializer.float32(obj.force_finger1, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Status
    let len;
    let data = new Status(null);
    // Deserialize message field [status]
    data.status = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [width]
    data.width = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [acc]
    data.acc = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [force]
    data.force = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [force_finger0]
    data.force_finger0 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [force_finger1]
    data.force_finger1 = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.status);
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'wsg_50_common/Status';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '51c764be2e2c02863274a528dd8b3470';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string status
    float32 width
    float32 speed
    float32 acc
    float32 force
    float32 force_finger0
    float32 force_finger1
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Status(null);
    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = ''
    }

    if (msg.width !== undefined) {
      resolved.width = msg.width;
    }
    else {
      resolved.width = 0.0
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0.0
    }

    if (msg.acc !== undefined) {
      resolved.acc = msg.acc;
    }
    else {
      resolved.acc = 0.0
    }

    if (msg.force !== undefined) {
      resolved.force = msg.force;
    }
    else {
      resolved.force = 0.0
    }

    if (msg.force_finger0 !== undefined) {
      resolved.force_finger0 = msg.force_finger0;
    }
    else {
      resolved.force_finger0 = 0.0
    }

    if (msg.force_finger1 !== undefined) {
      resolved.force_finger1 = msg.force_finger1;
    }
    else {
      resolved.force_finger1 = 0.0
    }

    return resolved;
    }
};

module.exports = Status;
