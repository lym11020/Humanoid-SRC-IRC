// Auto-generated. Do not edit!

// (in-package ahra.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class Select_MotionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.finish = null;
    }
    else {
      if (initObj.hasOwnProperty('finish')) {
        this.finish = initObj.finish
      }
      else {
        this.finish = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Select_MotionRequest
    // Serialize message field [finish]
    bufferOffset = _serializer.bool(obj.finish, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Select_MotionRequest
    let len;
    let data = new Select_MotionRequest(null);
    // Deserialize message field [finish]
    data.finish = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ahra/Select_MotionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '474a58dbb494a45bb1ca99544cd64e45';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool finish
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Select_MotionRequest(null);
    if (msg.finish !== undefined) {
      resolved.finish = msg.finish;
    }
    else {
      resolved.finish = false
    }

    return resolved;
    }
};

class Select_MotionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.select_motion = null;
      this.distance = null;
    }
    else {
      if (initObj.hasOwnProperty('select_motion')) {
        this.select_motion = initObj.select_motion
      }
      else {
        this.select_motion = 0;
      }
      if (initObj.hasOwnProperty('distance')) {
        this.distance = initObj.distance
      }
      else {
        this.distance = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Select_MotionResponse
    // Serialize message field [select_motion]
    bufferOffset = _serializer.int8(obj.select_motion, buffer, bufferOffset);
    // Serialize message field [distance]
    bufferOffset = _serializer.float64(obj.distance, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Select_MotionResponse
    let len;
    let data = new Select_MotionResponse(null);
    // Deserialize message field [select_motion]
    data.select_motion = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [distance]
    data.distance = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ahra/Select_MotionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8cc24820dfb2ba90d0f15d2ed14e6cfb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 select_motion
    float64 distance
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Select_MotionResponse(null);
    if (msg.select_motion !== undefined) {
      resolved.select_motion = msg.select_motion;
    }
    else {
      resolved.select_motion = 0
    }

    if (msg.distance !== undefined) {
      resolved.distance = msg.distance;
    }
    else {
      resolved.distance = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: Select_MotionRequest,
  Response: Select_MotionResponse,
  md5sum() { return 'd51f1f90cb25facd79c884f5f7a758e6'; },
  datatype() { return 'ahra/Select_Motion'; }
};
