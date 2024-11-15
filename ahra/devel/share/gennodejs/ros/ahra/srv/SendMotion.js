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

class SendMotionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.SM_finish = null;
      this.TA_finish = null;
      this.UD_finish = null;
      this.RL_finish = null;
      this.EM_finish = null;
      this.ST_finish = null;
      this.walkcount = null;
      this.request_id = null;
    }
    else {
      if (initObj.hasOwnProperty('SM_finish')) {
        this.SM_finish = initObj.SM_finish
      }
      else {
        this.SM_finish = false;
      }
      if (initObj.hasOwnProperty('TA_finish')) {
        this.TA_finish = initObj.TA_finish
      }
      else {
        this.TA_finish = false;
      }
      if (initObj.hasOwnProperty('UD_finish')) {
        this.UD_finish = initObj.UD_finish
      }
      else {
        this.UD_finish = false;
      }
      if (initObj.hasOwnProperty('RL_finish')) {
        this.RL_finish = initObj.RL_finish
      }
      else {
        this.RL_finish = false;
      }
      if (initObj.hasOwnProperty('EM_finish')) {
        this.EM_finish = initObj.EM_finish
      }
      else {
        this.EM_finish = false;
      }
      if (initObj.hasOwnProperty('ST_finish')) {
        this.ST_finish = initObj.ST_finish
      }
      else {
        this.ST_finish = false;
      }
      if (initObj.hasOwnProperty('walkcount')) {
        this.walkcount = initObj.walkcount
      }
      else {
        this.walkcount = 0;
      }
      if (initObj.hasOwnProperty('request_id')) {
        this.request_id = initObj.request_id
      }
      else {
        this.request_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SendMotionRequest
    // Serialize message field [SM_finish]
    bufferOffset = _serializer.bool(obj.SM_finish, buffer, bufferOffset);
    // Serialize message field [TA_finish]
    bufferOffset = _serializer.bool(obj.TA_finish, buffer, bufferOffset);
    // Serialize message field [UD_finish]
    bufferOffset = _serializer.bool(obj.UD_finish, buffer, bufferOffset);
    // Serialize message field [RL_finish]
    bufferOffset = _serializer.bool(obj.RL_finish, buffer, bufferOffset);
    // Serialize message field [EM_finish]
    bufferOffset = _serializer.bool(obj.EM_finish, buffer, bufferOffset);
    // Serialize message field [ST_finish]
    bufferOffset = _serializer.bool(obj.ST_finish, buffer, bufferOffset);
    // Serialize message field [walkcount]
    bufferOffset = _serializer.int32(obj.walkcount, buffer, bufferOffset);
    // Serialize message field [request_id]
    bufferOffset = _serializer.int32(obj.request_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SendMotionRequest
    let len;
    let data = new SendMotionRequest(null);
    // Deserialize message field [SM_finish]
    data.SM_finish = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [TA_finish]
    data.TA_finish = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [UD_finish]
    data.UD_finish = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [RL_finish]
    data.RL_finish = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [EM_finish]
    data.EM_finish = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ST_finish]
    data.ST_finish = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [walkcount]
    data.walkcount = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [request_id]
    data.request_id = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 14;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ahra/SendMotionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4bcb9a6896459a3a3d39e3c375bae71b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool SM_finish
    bool TA_finish
    bool UD_finish
    bool RL_finish
    bool EM_finish
    bool ST_finish
    int32 walkcount
    int32 request_id # Unique request ID
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SendMotionRequest(null);
    if (msg.SM_finish !== undefined) {
      resolved.SM_finish = msg.SM_finish;
    }
    else {
      resolved.SM_finish = false
    }

    if (msg.TA_finish !== undefined) {
      resolved.TA_finish = msg.TA_finish;
    }
    else {
      resolved.TA_finish = false
    }

    if (msg.UD_finish !== undefined) {
      resolved.UD_finish = msg.UD_finish;
    }
    else {
      resolved.UD_finish = false
    }

    if (msg.RL_finish !== undefined) {
      resolved.RL_finish = msg.RL_finish;
    }
    else {
      resolved.RL_finish = false
    }

    if (msg.EM_finish !== undefined) {
      resolved.EM_finish = msg.EM_finish;
    }
    else {
      resolved.EM_finish = false
    }

    if (msg.ST_finish !== undefined) {
      resolved.ST_finish = msg.ST_finish;
    }
    else {
      resolved.ST_finish = false
    }

    if (msg.walkcount !== undefined) {
      resolved.walkcount = msg.walkcount;
    }
    else {
      resolved.walkcount = 0
    }

    if (msg.request_id !== undefined) {
      resolved.request_id = msg.request_id;
    }
    else {
      resolved.request_id = 0
    }

    return resolved;
    }
};

class SendMotionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.select_motion = null;
      this.distance = null;
      this.turn_angle = null;
      this.ud_neckangle = null;
      this.rl_neckangle = null;
      this.emergency = null;
      this.success = null;
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
      if (initObj.hasOwnProperty('turn_angle')) {
        this.turn_angle = initObj.turn_angle
      }
      else {
        this.turn_angle = 0.0;
      }
      if (initObj.hasOwnProperty('ud_neckangle')) {
        this.ud_neckangle = initObj.ud_neckangle
      }
      else {
        this.ud_neckangle = 0.0;
      }
      if (initObj.hasOwnProperty('rl_neckangle')) {
        this.rl_neckangle = initObj.rl_neckangle
      }
      else {
        this.rl_neckangle = 0.0;
      }
      if (initObj.hasOwnProperty('emergency')) {
        this.emergency = initObj.emergency
      }
      else {
        this.emergency = false;
      }
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SendMotionResponse
    // Serialize message field [select_motion]
    bufferOffset = _serializer.int8(obj.select_motion, buffer, bufferOffset);
    // Serialize message field [distance]
    bufferOffset = _serializer.float64(obj.distance, buffer, bufferOffset);
    // Serialize message field [turn_angle]
    bufferOffset = _serializer.float64(obj.turn_angle, buffer, bufferOffset);
    // Serialize message field [ud_neckangle]
    bufferOffset = _serializer.float64(obj.ud_neckangle, buffer, bufferOffset);
    // Serialize message field [rl_neckangle]
    bufferOffset = _serializer.float64(obj.rl_neckangle, buffer, bufferOffset);
    // Serialize message field [emergency]
    bufferOffset = _serializer.bool(obj.emergency, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SendMotionResponse
    let len;
    let data = new SendMotionResponse(null);
    // Deserialize message field [select_motion]
    data.select_motion = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [distance]
    data.distance = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [turn_angle]
    data.turn_angle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ud_neckangle]
    data.ud_neckangle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rl_neckangle]
    data.rl_neckangle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [emergency]
    data.emergency = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 35;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ahra/SendMotionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7d0fd39542efcd01f963016eab8f1bc4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 select_motion
    float64 distance
    float64 turn_angle
    float64 ud_neckangle
    float64 rl_neckangle
    bool emergency
    bool success # Response success status
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SendMotionResponse(null);
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

    if (msg.turn_angle !== undefined) {
      resolved.turn_angle = msg.turn_angle;
    }
    else {
      resolved.turn_angle = 0.0
    }

    if (msg.ud_neckangle !== undefined) {
      resolved.ud_neckangle = msg.ud_neckangle;
    }
    else {
      resolved.ud_neckangle = 0.0
    }

    if (msg.rl_neckangle !== undefined) {
      resolved.rl_neckangle = msg.rl_neckangle;
    }
    else {
      resolved.rl_neckangle = 0.0
    }

    if (msg.emergency !== undefined) {
      resolved.emergency = msg.emergency;
    }
    else {
      resolved.emergency = false
    }

    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: SendMotionRequest,
  Response: SendMotionResponse,
  md5sum() { return '195b77fcb35ce347682e382fd8b4d70b'; },
  datatype() { return 'ahra/SendMotion'; }
};
