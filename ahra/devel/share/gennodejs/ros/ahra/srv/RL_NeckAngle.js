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

class RL_NeckAngleRequest {
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
    // Serializes a message object of type RL_NeckAngleRequest
    // Serialize message field [finish]
    bufferOffset = _serializer.bool(obj.finish, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RL_NeckAngleRequest
    let len;
    let data = new RL_NeckAngleRequest(null);
    // Deserialize message field [finish]
    data.finish = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ahra/RL_NeckAngleRequest';
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
    const resolved = new RL_NeckAngleRequest(null);
    if (msg.finish !== undefined) {
      resolved.finish = msg.finish;
    }
    else {
      resolved.finish = false
    }

    return resolved;
    }
};

class RL_NeckAngleResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.rl_neckangle = null;
    }
    else {
      if (initObj.hasOwnProperty('rl_neckangle')) {
        this.rl_neckangle = initObj.rl_neckangle
      }
      else {
        this.rl_neckangle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RL_NeckAngleResponse
    // Serialize message field [rl_neckangle]
    bufferOffset = _serializer.float64(obj.rl_neckangle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RL_NeckAngleResponse
    let len;
    let data = new RL_NeckAngleResponse(null);
    // Deserialize message field [rl_neckangle]
    data.rl_neckangle = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ahra/RL_NeckAngleResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b3f7b8005f2422d2175dbefadbd24dbc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 rl_neckangle
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RL_NeckAngleResponse(null);
    if (msg.rl_neckangle !== undefined) {
      resolved.rl_neckangle = msg.rl_neckangle;
    }
    else {
      resolved.rl_neckangle = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: RL_NeckAngleRequest,
  Response: RL_NeckAngleResponse,
  md5sum() { return '53641ec2b7d57c4826404748be4e88f1'; },
  datatype() { return 'ahra/RL_NeckAngle'; }
};
