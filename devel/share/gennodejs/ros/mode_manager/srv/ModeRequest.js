// Auto-generated. Do not edit!

// (in-package mode_manager.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ModeRequestRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.request_data = null;
    }
    else {
      if (initObj.hasOwnProperty('request_data')) {
        this.request_data = initObj.request_data
      }
      else {
        this.request_data = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ModeRequestRequest
    // Serialize message field [request_data]
    bufferOffset = _serializer.string(obj.request_data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ModeRequestRequest
    let len;
    let data = new ModeRequestRequest(null);
    // Deserialize message field [request_data]
    data.request_data = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.request_data.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mode_manager/ModeRequestRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4d0e41720c8d7bc6e697895f7a38cf0c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string request_data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ModeRequestRequest(null);
    if (msg.request_data !== undefined) {
      resolved.request_data = msg.request_data;
    }
    else {
      resolved.request_data = ''
    }

    return resolved;
    }
};

class ModeRequestResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.response_data = null;
    }
    else {
      if (initObj.hasOwnProperty('response_data')) {
        this.response_data = initObj.response_data
      }
      else {
        this.response_data = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ModeRequestResponse
    // Serialize message field [response_data]
    bufferOffset = _serializer.string(obj.response_data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ModeRequestResponse
    let len;
    let data = new ModeRequestResponse(null);
    // Deserialize message field [response_data]
    data.response_data = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.response_data.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mode_manager/ModeRequestResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8decf635d15fee3900d3c2f56dbbbeeb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string response_data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ModeRequestResponse(null);
    if (msg.response_data !== undefined) {
      resolved.response_data = msg.response_data;
    }
    else {
      resolved.response_data = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: ModeRequestRequest,
  Response: ModeRequestResponse,
  md5sum() { return '0c900dd8a9e0a09ef71b5a7480dbc006'; },
  datatype() { return 'mode_manager/ModeRequest'; }
};
