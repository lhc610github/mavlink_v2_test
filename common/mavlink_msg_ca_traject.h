#pragma once
// MESSAGE CA_TRAJECT PACKING

#define MAVLINK_MSG_ID_CA_TRAJECT 153

MAVPACKED(
typedef struct __mavlink_ca_traject_t {
 uint64_t time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 uint64_t PC_time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 float t[2]; /*< Start time and finish time*/
 float trajectory_coefficient_x[7]; /*< x-axis trajectory coefficient*/
 float trajectory_coefficient_y[7]; /*< Y-axis trajectory coefficient*/
 float trajectory_coefficient_z[7]; /*< Z-axis trajectory coefficient*/
 float trajectory_coefficient_r[7]; /*< YAW-axis trajectory coefficient*/
 uint8_t num_keyframe; /*< Total num of keyframe*/
 uint8_t index_keyframe; /*< Index of keyframe*/
 uint8_t order_p_1; /*< Order plus one*/
}) mavlink_ca_traject_t;

#define MAVLINK_MSG_ID_CA_TRAJECT_LEN 139
#define MAVLINK_MSG_ID_CA_TRAJECT_MIN_LEN 139
#define MAVLINK_MSG_ID_153_LEN 139
#define MAVLINK_MSG_ID_153_MIN_LEN 139

#define MAVLINK_MSG_ID_CA_TRAJECT_CRC 77
#define MAVLINK_MSG_ID_153_CRC 77

#define MAVLINK_MSG_CA_TRAJECT_FIELD_T_LEN 2
#define MAVLINK_MSG_CA_TRAJECT_FIELD_TRAJECTORY_COEFFICIENT_X_LEN 7
#define MAVLINK_MSG_CA_TRAJECT_FIELD_TRAJECTORY_COEFFICIENT_Y_LEN 7
#define MAVLINK_MSG_CA_TRAJECT_FIELD_TRAJECTORY_COEFFICIENT_Z_LEN 7
#define MAVLINK_MSG_CA_TRAJECT_FIELD_TRAJECTORY_COEFFICIENT_R_LEN 7

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CA_TRAJECT { \
    153, \
    "CA_TRAJECT", \
    10, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ca_traject_t, time_usec) }, \
         { "PC_time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_ca_traject_t, PC_time_usec) }, \
         { "t", NULL, MAVLINK_TYPE_FLOAT, 2, 16, offsetof(mavlink_ca_traject_t, t) }, \
         { "trajectory_coefficient_x", NULL, MAVLINK_TYPE_FLOAT, 7, 24, offsetof(mavlink_ca_traject_t, trajectory_coefficient_x) }, \
         { "trajectory_coefficient_y", NULL, MAVLINK_TYPE_FLOAT, 7, 52, offsetof(mavlink_ca_traject_t, trajectory_coefficient_y) }, \
         { "trajectory_coefficient_z", NULL, MAVLINK_TYPE_FLOAT, 7, 80, offsetof(mavlink_ca_traject_t, trajectory_coefficient_z) }, \
         { "trajectory_coefficient_r", NULL, MAVLINK_TYPE_FLOAT, 7, 108, offsetof(mavlink_ca_traject_t, trajectory_coefficient_r) }, \
         { "num_keyframe", NULL, MAVLINK_TYPE_UINT8_T, 0, 136, offsetof(mavlink_ca_traject_t, num_keyframe) }, \
         { "index_keyframe", NULL, MAVLINK_TYPE_UINT8_T, 0, 137, offsetof(mavlink_ca_traject_t, index_keyframe) }, \
         { "order_p_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 138, offsetof(mavlink_ca_traject_t, order_p_1) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CA_TRAJECT { \
    "CA_TRAJECT", \
    10, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ca_traject_t, time_usec) }, \
         { "PC_time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_ca_traject_t, PC_time_usec) }, \
         { "t", NULL, MAVLINK_TYPE_FLOAT, 2, 16, offsetof(mavlink_ca_traject_t, t) }, \
         { "trajectory_coefficient_x", NULL, MAVLINK_TYPE_FLOAT, 7, 24, offsetof(mavlink_ca_traject_t, trajectory_coefficient_x) }, \
         { "trajectory_coefficient_y", NULL, MAVLINK_TYPE_FLOAT, 7, 52, offsetof(mavlink_ca_traject_t, trajectory_coefficient_y) }, \
         { "trajectory_coefficient_z", NULL, MAVLINK_TYPE_FLOAT, 7, 80, offsetof(mavlink_ca_traject_t, trajectory_coefficient_z) }, \
         { "trajectory_coefficient_r", NULL, MAVLINK_TYPE_FLOAT, 7, 108, offsetof(mavlink_ca_traject_t, trajectory_coefficient_r) }, \
         { "num_keyframe", NULL, MAVLINK_TYPE_UINT8_T, 0, 136, offsetof(mavlink_ca_traject_t, num_keyframe) }, \
         { "index_keyframe", NULL, MAVLINK_TYPE_UINT8_T, 0, 137, offsetof(mavlink_ca_traject_t, index_keyframe) }, \
         { "order_p_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 138, offsetof(mavlink_ca_traject_t, order_p_1) }, \
         } \
}
#endif

/**
 * @brief Pack a ca_traject message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param PC_time_usec Timestamp (micros since boot or Unix epoch)
 * @param num_keyframe Total num of keyframe
 * @param index_keyframe Index of keyframe
 * @param order_p_1 Order plus one
 * @param t Start time and finish time
 * @param trajectory_coefficient_x x-axis trajectory coefficient
 * @param trajectory_coefficient_y Y-axis trajectory coefficient
 * @param trajectory_coefficient_z Z-axis trajectory coefficient
 * @param trajectory_coefficient_r YAW-axis trajectory coefficient
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ca_traject_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint64_t PC_time_usec, uint8_t num_keyframe, uint8_t index_keyframe, uint8_t order_p_1, const float *t, const float *trajectory_coefficient_x, const float *trajectory_coefficient_y, const float *trajectory_coefficient_z, const float *trajectory_coefficient_r)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CA_TRAJECT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, PC_time_usec);
    _mav_put_uint8_t(buf, 136, num_keyframe);
    _mav_put_uint8_t(buf, 137, index_keyframe);
    _mav_put_uint8_t(buf, 138, order_p_1);
    _mav_put_float_array(buf, 16, t, 2);
    _mav_put_float_array(buf, 24, trajectory_coefficient_x, 7);
    _mav_put_float_array(buf, 52, trajectory_coefficient_y, 7);
    _mav_put_float_array(buf, 80, trajectory_coefficient_z, 7);
    _mav_put_float_array(buf, 108, trajectory_coefficient_r, 7);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CA_TRAJECT_LEN);
#else
    mavlink_ca_traject_t packet;
    packet.time_usec = time_usec;
    packet.PC_time_usec = PC_time_usec;
    packet.num_keyframe = num_keyframe;
    packet.index_keyframe = index_keyframe;
    packet.order_p_1 = order_p_1;
    mav_array_memcpy(packet.t, t, sizeof(float)*2);
    mav_array_memcpy(packet.trajectory_coefficient_x, trajectory_coefficient_x, sizeof(float)*7);
    mav_array_memcpy(packet.trajectory_coefficient_y, trajectory_coefficient_y, sizeof(float)*7);
    mav_array_memcpy(packet.trajectory_coefficient_z, trajectory_coefficient_z, sizeof(float)*7);
    mav_array_memcpy(packet.trajectory_coefficient_r, trajectory_coefficient_r, sizeof(float)*7);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CA_TRAJECT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CA_TRAJECT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CA_TRAJECT_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECT_LEN, MAVLINK_MSG_ID_CA_TRAJECT_CRC);
}

/**
 * @brief Pack a ca_traject message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param PC_time_usec Timestamp (micros since boot or Unix epoch)
 * @param num_keyframe Total num of keyframe
 * @param index_keyframe Index of keyframe
 * @param order_p_1 Order plus one
 * @param t Start time and finish time
 * @param trajectory_coefficient_x x-axis trajectory coefficient
 * @param trajectory_coefficient_y Y-axis trajectory coefficient
 * @param trajectory_coefficient_z Z-axis trajectory coefficient
 * @param trajectory_coefficient_r YAW-axis trajectory coefficient
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ca_traject_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint64_t PC_time_usec,uint8_t num_keyframe,uint8_t index_keyframe,uint8_t order_p_1,const float *t,const float *trajectory_coefficient_x,const float *trajectory_coefficient_y,const float *trajectory_coefficient_z,const float *trajectory_coefficient_r)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CA_TRAJECT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, PC_time_usec);
    _mav_put_uint8_t(buf, 136, num_keyframe);
    _mav_put_uint8_t(buf, 137, index_keyframe);
    _mav_put_uint8_t(buf, 138, order_p_1);
    _mav_put_float_array(buf, 16, t, 2);
    _mav_put_float_array(buf, 24, trajectory_coefficient_x, 7);
    _mav_put_float_array(buf, 52, trajectory_coefficient_y, 7);
    _mav_put_float_array(buf, 80, trajectory_coefficient_z, 7);
    _mav_put_float_array(buf, 108, trajectory_coefficient_r, 7);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CA_TRAJECT_LEN);
#else
    mavlink_ca_traject_t packet;
    packet.time_usec = time_usec;
    packet.PC_time_usec = PC_time_usec;
    packet.num_keyframe = num_keyframe;
    packet.index_keyframe = index_keyframe;
    packet.order_p_1 = order_p_1;
    mav_array_memcpy(packet.t, t, sizeof(float)*2);
    mav_array_memcpy(packet.trajectory_coefficient_x, trajectory_coefficient_x, sizeof(float)*7);
    mav_array_memcpy(packet.trajectory_coefficient_y, trajectory_coefficient_y, sizeof(float)*7);
    mav_array_memcpy(packet.trajectory_coefficient_z, trajectory_coefficient_z, sizeof(float)*7);
    mav_array_memcpy(packet.trajectory_coefficient_r, trajectory_coefficient_r, sizeof(float)*7);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CA_TRAJECT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CA_TRAJECT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CA_TRAJECT_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECT_LEN, MAVLINK_MSG_ID_CA_TRAJECT_CRC);
}

/**
 * @brief Encode a ca_traject struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ca_traject C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ca_traject_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ca_traject_t* ca_traject)
{
    return mavlink_msg_ca_traject_pack(system_id, component_id, msg, ca_traject->time_usec, ca_traject->PC_time_usec, ca_traject->num_keyframe, ca_traject->index_keyframe, ca_traject->order_p_1, ca_traject->t, ca_traject->trajectory_coefficient_x, ca_traject->trajectory_coefficient_y, ca_traject->trajectory_coefficient_z, ca_traject->trajectory_coefficient_r);
}

/**
 * @brief Encode a ca_traject struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ca_traject C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ca_traject_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ca_traject_t* ca_traject)
{
    return mavlink_msg_ca_traject_pack_chan(system_id, component_id, chan, msg, ca_traject->time_usec, ca_traject->PC_time_usec, ca_traject->num_keyframe, ca_traject->index_keyframe, ca_traject->order_p_1, ca_traject->t, ca_traject->trajectory_coefficient_x, ca_traject->trajectory_coefficient_y, ca_traject->trajectory_coefficient_z, ca_traject->trajectory_coefficient_r);
}

/**
 * @brief Send a ca_traject message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param PC_time_usec Timestamp (micros since boot or Unix epoch)
 * @param num_keyframe Total num of keyframe
 * @param index_keyframe Index of keyframe
 * @param order_p_1 Order plus one
 * @param t Start time and finish time
 * @param trajectory_coefficient_x x-axis trajectory coefficient
 * @param trajectory_coefficient_y Y-axis trajectory coefficient
 * @param trajectory_coefficient_z Z-axis trajectory coefficient
 * @param trajectory_coefficient_r YAW-axis trajectory coefficient
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ca_traject_send(mavlink_channel_t chan, uint64_t time_usec, uint64_t PC_time_usec, uint8_t num_keyframe, uint8_t index_keyframe, uint8_t order_p_1, const float *t, const float *trajectory_coefficient_x, const float *trajectory_coefficient_y, const float *trajectory_coefficient_z, const float *trajectory_coefficient_r)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CA_TRAJECT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, PC_time_usec);
    _mav_put_uint8_t(buf, 136, num_keyframe);
    _mav_put_uint8_t(buf, 137, index_keyframe);
    _mav_put_uint8_t(buf, 138, order_p_1);
    _mav_put_float_array(buf, 16, t, 2);
    _mav_put_float_array(buf, 24, trajectory_coefficient_x, 7);
    _mav_put_float_array(buf, 52, trajectory_coefficient_y, 7);
    _mav_put_float_array(buf, 80, trajectory_coefficient_z, 7);
    _mav_put_float_array(buf, 108, trajectory_coefficient_r, 7);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CA_TRAJECT, buf, MAVLINK_MSG_ID_CA_TRAJECT_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECT_LEN, MAVLINK_MSG_ID_CA_TRAJECT_CRC);
#else
    mavlink_ca_traject_t packet;
    packet.time_usec = time_usec;
    packet.PC_time_usec = PC_time_usec;
    packet.num_keyframe = num_keyframe;
    packet.index_keyframe = index_keyframe;
    packet.order_p_1 = order_p_1;
    mav_array_memcpy(packet.t, t, sizeof(float)*2);
    mav_array_memcpy(packet.trajectory_coefficient_x, trajectory_coefficient_x, sizeof(float)*7);
    mav_array_memcpy(packet.trajectory_coefficient_y, trajectory_coefficient_y, sizeof(float)*7);
    mav_array_memcpy(packet.trajectory_coefficient_z, trajectory_coefficient_z, sizeof(float)*7);
    mav_array_memcpy(packet.trajectory_coefficient_r, trajectory_coefficient_r, sizeof(float)*7);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CA_TRAJECT, (const char *)&packet, MAVLINK_MSG_ID_CA_TRAJECT_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECT_LEN, MAVLINK_MSG_ID_CA_TRAJECT_CRC);
#endif
}

/**
 * @brief Send a ca_traject message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ca_traject_send_struct(mavlink_channel_t chan, const mavlink_ca_traject_t* ca_traject)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ca_traject_send(chan, ca_traject->time_usec, ca_traject->PC_time_usec, ca_traject->num_keyframe, ca_traject->index_keyframe, ca_traject->order_p_1, ca_traject->t, ca_traject->trajectory_coefficient_x, ca_traject->trajectory_coefficient_y, ca_traject->trajectory_coefficient_z, ca_traject->trajectory_coefficient_r);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CA_TRAJECT, (const char *)ca_traject, MAVLINK_MSG_ID_CA_TRAJECT_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECT_LEN, MAVLINK_MSG_ID_CA_TRAJECT_CRC);
#endif
}

#if MAVLINK_MSG_ID_CA_TRAJECT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ca_traject_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint64_t PC_time_usec, uint8_t num_keyframe, uint8_t index_keyframe, uint8_t order_p_1, const float *t, const float *trajectory_coefficient_x, const float *trajectory_coefficient_y, const float *trajectory_coefficient_z, const float *trajectory_coefficient_r)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, PC_time_usec);
    _mav_put_uint8_t(buf, 136, num_keyframe);
    _mav_put_uint8_t(buf, 137, index_keyframe);
    _mav_put_uint8_t(buf, 138, order_p_1);
    _mav_put_float_array(buf, 16, t, 2);
    _mav_put_float_array(buf, 24, trajectory_coefficient_x, 7);
    _mav_put_float_array(buf, 52, trajectory_coefficient_y, 7);
    _mav_put_float_array(buf, 80, trajectory_coefficient_z, 7);
    _mav_put_float_array(buf, 108, trajectory_coefficient_r, 7);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CA_TRAJECT, buf, MAVLINK_MSG_ID_CA_TRAJECT_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECT_LEN, MAVLINK_MSG_ID_CA_TRAJECT_CRC);
#else
    mavlink_ca_traject_t *packet = (mavlink_ca_traject_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->PC_time_usec = PC_time_usec;
    packet->num_keyframe = num_keyframe;
    packet->index_keyframe = index_keyframe;
    packet->order_p_1 = order_p_1;
    mav_array_memcpy(packet->t, t, sizeof(float)*2);
    mav_array_memcpy(packet->trajectory_coefficient_x, trajectory_coefficient_x, sizeof(float)*7);
    mav_array_memcpy(packet->trajectory_coefficient_y, trajectory_coefficient_y, sizeof(float)*7);
    mav_array_memcpy(packet->trajectory_coefficient_z, trajectory_coefficient_z, sizeof(float)*7);
    mav_array_memcpy(packet->trajectory_coefficient_r, trajectory_coefficient_r, sizeof(float)*7);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CA_TRAJECT, (const char *)packet, MAVLINK_MSG_ID_CA_TRAJECT_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECT_LEN, MAVLINK_MSG_ID_CA_TRAJECT_CRC);
#endif
}
#endif

#endif

// MESSAGE CA_TRAJECT UNPACKING


/**
 * @brief Get field time_usec from ca_traject message
 *
 * @return Timestamp (micros since boot or Unix epoch)
 */
static inline uint64_t mavlink_msg_ca_traject_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field PC_time_usec from ca_traject message
 *
 * @return Timestamp (micros since boot or Unix epoch)
 */
static inline uint64_t mavlink_msg_ca_traject_get_PC_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field num_keyframe from ca_traject message
 *
 * @return Total num of keyframe
 */
static inline uint8_t mavlink_msg_ca_traject_get_num_keyframe(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  136);
}

/**
 * @brief Get field index_keyframe from ca_traject message
 *
 * @return Index of keyframe
 */
static inline uint8_t mavlink_msg_ca_traject_get_index_keyframe(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  137);
}

/**
 * @brief Get field order_p_1 from ca_traject message
 *
 * @return Order plus one
 */
static inline uint8_t mavlink_msg_ca_traject_get_order_p_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  138);
}

/**
 * @brief Get field t from ca_traject message
 *
 * @return Start time and finish time
 */
static inline uint16_t mavlink_msg_ca_traject_get_t(const mavlink_message_t* msg, float *t)
{
    return _MAV_RETURN_float_array(msg, t, 2,  16);
}

/**
 * @brief Get field trajectory_coefficient_x from ca_traject message
 *
 * @return x-axis trajectory coefficient
 */
static inline uint16_t mavlink_msg_ca_traject_get_trajectory_coefficient_x(const mavlink_message_t* msg, float *trajectory_coefficient_x)
{
    return _MAV_RETURN_float_array(msg, trajectory_coefficient_x, 7,  24);
}

/**
 * @brief Get field trajectory_coefficient_y from ca_traject message
 *
 * @return Y-axis trajectory coefficient
 */
static inline uint16_t mavlink_msg_ca_traject_get_trajectory_coefficient_y(const mavlink_message_t* msg, float *trajectory_coefficient_y)
{
    return _MAV_RETURN_float_array(msg, trajectory_coefficient_y, 7,  52);
}

/**
 * @brief Get field trajectory_coefficient_z from ca_traject message
 *
 * @return Z-axis trajectory coefficient
 */
static inline uint16_t mavlink_msg_ca_traject_get_trajectory_coefficient_z(const mavlink_message_t* msg, float *trajectory_coefficient_z)
{
    return _MAV_RETURN_float_array(msg, trajectory_coefficient_z, 7,  80);
}

/**
 * @brief Get field trajectory_coefficient_r from ca_traject message
 *
 * @return YAW-axis trajectory coefficient
 */
static inline uint16_t mavlink_msg_ca_traject_get_trajectory_coefficient_r(const mavlink_message_t* msg, float *trajectory_coefficient_r)
{
    return _MAV_RETURN_float_array(msg, trajectory_coefficient_r, 7,  108);
}

/**
 * @brief Decode a ca_traject message into a struct
 *
 * @param msg The message to decode
 * @param ca_traject C-struct to decode the message contents into
 */
static inline void mavlink_msg_ca_traject_decode(const mavlink_message_t* msg, mavlink_ca_traject_t* ca_traject)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ca_traject->time_usec = mavlink_msg_ca_traject_get_time_usec(msg);
    ca_traject->PC_time_usec = mavlink_msg_ca_traject_get_PC_time_usec(msg);
    mavlink_msg_ca_traject_get_t(msg, ca_traject->t);
    mavlink_msg_ca_traject_get_trajectory_coefficient_x(msg, ca_traject->trajectory_coefficient_x);
    mavlink_msg_ca_traject_get_trajectory_coefficient_y(msg, ca_traject->trajectory_coefficient_y);
    mavlink_msg_ca_traject_get_trajectory_coefficient_z(msg, ca_traject->trajectory_coefficient_z);
    mavlink_msg_ca_traject_get_trajectory_coefficient_r(msg, ca_traject->trajectory_coefficient_r);
    ca_traject->num_keyframe = mavlink_msg_ca_traject_get_num_keyframe(msg);
    ca_traject->index_keyframe = mavlink_msg_ca_traject_get_index_keyframe(msg);
    ca_traject->order_p_1 = mavlink_msg_ca_traject_get_order_p_1(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CA_TRAJECT_LEN? msg->len : MAVLINK_MSG_ID_CA_TRAJECT_LEN;
        memset(ca_traject, 0, MAVLINK_MSG_ID_CA_TRAJECT_LEN);
    memcpy(ca_traject, _MAV_PAYLOAD(msg), len);
#endif
}
