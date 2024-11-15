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

class UD_NeckAngleRequest {
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
    // Serializes a message object of type UD_NeckAngleRequest
    // Serialize message field [finish]
    bufferOffset = _serializer.bool(obj.finish, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UD_NeckAngleRequest
    let len;
    let data = new UD_NeckAngleRequest(null);
    // Deserialize message field [finish]
    data.finish = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ahra/UD_NeckAngleRequest';
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
    const resolved = new UD_NeckAngleRequest(null);
    if (msg.finish !== undefined) {
      resolved.finish = msg.finish;
    }
    else {
      resolved.finish = false
    }

    return resolved;
    }
};

class UD_NeckAngleResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ud_neckangle = null;
    }
    else {
      if (initObj.hasOwnProperty('ud_neckangle')) {
        this.ud_neckangle = initObj.ud_neckangle
      }
      else {
        this.ud_neckangle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UD_NeckAngleResponse
    // Serialize message field [ud_neckangle]
    bufferOffset = _serializer.float64(obj.ud_neckangle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UD_NeckAngleResponse
    let len;
    let data = new UD_NeckAngleResponse(null);
    // Deserialize message field [ud_neckangle]
    data.ud_neckangle = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ahra/UD_NeckAngleResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '313e6cd6c4903c731de0626857377bb2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 ud_neckangle
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new UD_NeckAngleResponse(null);
    if (msg.ud_neckangle !== undefined) {
      resolved.ud_neckangle = msg.ud_neckangle;
    }
    else {
      resolved.ud_neckangle = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: UD_NeckAngleRequest,
  Response: UD_NeckAngleResponse,
  md5sum() { return '54d0093350d66ae9f81f65f132656855'; },
  datatype() { return 'ahra/UD_NeckAngle'; }
};
