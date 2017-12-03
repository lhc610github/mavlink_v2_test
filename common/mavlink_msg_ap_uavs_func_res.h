#pragma once
// MESSAGE AP_UAVS_FUNC_RES PACKING

#define MAVLINK_MSG_ID_AP_UAVS_FUNC_RES 155

MAVPACKED(
typedef struct __mavlink_ap_uavs_func_res_t {
 uint64_t time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 float ap_ctrl_vel[3]; /*< Virtual control virable in velocity*/
 uint8_t num_uavs; /*< Num of neighbour uavs*/
}) mavlink_ap_uavs_func_res_t;

#define MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_LEN 21
#define MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_MIN_LEN 21
#define MAVLINK_MSG_ID_155_LEN 21
#define MAVLINK_MSG_ID_155_MIN_LEN 21

#define MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_CRC 164
#define MAVLINK_MSG_ID_155_CRC 164

#define MAVLINK_MSG_AP_UAVS_FUNC_RES_FIELD_AP_CTRL_VEL_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AP_UAVS_FUNC_RES { \
    155, \
    "AP_UAVS_FUNC_RES", \
    3, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ap_uavs_func_res_t, time_usec) }, \
         { "ap_ctrl_vel", NULL, MAVLINK_TYPE_FLOAT, 3, 8, offsetof(mavlink_ap_uavs_func_res_t, ap_ctrl_vel) }, \
         { "num_uavs", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_ap_uavs_func_res_t, num_uavs) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AP_UAVS_FUNC_RES { \
    "AP_UAVS_FUNC_RES", \
    3, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ap_uavs_func_res_t, time_usec) }, \
         { "ap_ctrl_vel", NULL, MAVLINK_TYPE_FLOAT, 3, 8, offsetof(mavlink_ap_uavs_func_res_t, ap_ctrl_vel) }, \
         { "num_uavs", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_ap_uavs_func_res_t, num_uavs) }, \
         } \
}
#endif

/**
 * @brief Pack a ap_uavs_func_res message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param num_uavs Num of neighbour uavs
 * @param ap_ctrl_vel Virtual control virable in velocity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ap_uavs_func_res_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t num_uavs, const float *ap_ctrl_vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 20, num_uavs);
    _mav_put_float_array(buf, 8, ap_ctrl_vel, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_LEN);
#else
    mavlink_ap_uavs_func_res_t packet;
    packet.time_usec = time_usec;
    packet.num_uavs = num_uavs;
    mav_array_memcpy(packet.ap_ctrl_vel, ap_ctrl_vel, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AP_UAVS_FUNC_RES;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_MIN_LEN, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_LEN, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_CRC);
}

/**
 * @brief Pack a ap_uavs_func_res message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param num_uavs Num of neighbour uavs
 * @param ap_ctrl_vel Virtual control virable in velocity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ap_uavs_func_res_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint8_t num_uavs,const float *ap_ctrl_vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 20, num_uavs);
    _mav_put_float_array(buf, 8, ap_ctrl_vel, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_LEN);
#else
    mavlink_ap_uavs_func_res_t packet;
    packet.time_usec = time_usec;
    packet.num_uavs = num_uavs;
    mav_array_memcpy(packet.ap_ctrl_vel, ap_ctrl_vel, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AP_UAVS_FUNC_RES;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_MIN_LEN, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_LEN, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_CRC);
}

/**
 * @brief Encode a ap_uavs_func_res struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ap_uavs_func_res C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ap_uavs_func_res_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ap_uavs_func_res_t* ap_uavs_func_res)
{
    return mavlink_msg_ap_uavs_func_res_pack(system_id, component_id, msg, ap_uavs_func_res->time_usec, ap_uavs_func_res->num_uavs, ap_uavs_func_res->ap_ctrl_vel);
}

/**
 * @brief Encode a ap_uavs_func_res struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ap_uavs_func_res C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ap_uavs_func_res_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ap_uavs_func_res_t* ap_uavs_func_res)
{
    return mavlink_msg_ap_uavs_func_res_pack_chan(system_id, component_id, chan, msg, ap_uavs_func_res->time_usec, ap_uavs_func_res->num_uavs, ap_uavs_func_res->ap_ctrl_vel);
}

/**
 * @brief Send a ap_uavs_func_res message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param num_uavs Num of neighbour uavs
 * @param ap_ctrl_vel Virtual control virable in velocity
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ap_uavs_func_res_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t num_uavs, const float *ap_ctrl_vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 20, num_uavs);
    _mav_put_float_array(buf, 8, ap_ctrl_vel, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES, buf, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_MIN_LEN, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_LEN, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_CRC);
#else
    mavlink_ap_uavs_func_res_t packet;
    packet.time_usec = time_usec;
    packet.num_uavs = num_uavs;
    mav_array_memcpy(packet.ap_ctrl_vel, ap_ctrl_vel, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES, (const char *)&packet, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_MIN_LEN, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_LEN, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_CRC);
#endif
}

/**
 * @brief Send a ap_uavs_func_res message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ap_uavs_func_res_send_struct(mavlink_channel_t chan, const mavlink_ap_uavs_func_res_t* ap_uavs_func_res)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ap_uavs_func_res_send(chan, ap_uavs_func_res->time_usec, ap_uavs_func_res->num_uavs, ap_uavs_func_res->ap_ctrl_vel);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES, (const char *)ap_uavs_func_res, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_MIN_LEN, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_LEN, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_CRC);
#endif
}

#if MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ap_uavs_func_res_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t num_uavs, const float *ap_ctrl_vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 20, num_uavs);
    _mav_put_float_array(buf, 8, ap_ctrl_vel, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES, buf, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_MIN_LEN, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_LEN, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_CRC);
#else
    mavlink_ap_uavs_func_res_t *packet = (mavlink_ap_uavs_func_res_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->num_uavs = num_uavs;
    mav_array_memcpy(packet->ap_ctrl_vel, ap_ctrl_vel, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES, (const char *)packet, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_MIN_LEN, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_LEN, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_CRC);
#endif
}
#endif

#endif

// MESSAGE AP_UAVS_FUNC_RES UNPACKING


/**
 * @brief Get field time_usec from ap_uavs_func_res message
 *
 * @return Timestamp (micros since boot or Unix epoch)
 */
static inline uint64_t mavlink_msg_ap_uavs_func_res_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field num_uavs from ap_uavs_func_res message
 *
 * @return Num of neighbour uavs
 */
static inline uint8_t mavlink_msg_ap_uavs_func_res_get_num_uavs(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field ap_ctrl_vel from ap_uavs_func_res message
 *
 * @return Virtual control virable in velocity
 */
static inline uint16_t mavlink_msg_ap_uavs_func_res_get_ap_ctrl_vel(const mavlink_message_t* msg, float *ap_ctrl_vel)
{
    return _MAV_RETURN_float_array(msg, ap_ctrl_vel, 3,  8);
}

/**
 * @brief Decode a ap_uavs_func_res message into a struct
 *
 * @param msg The message to decode
 * @param ap_uavs_func_res C-struct to decode the message contents into
 */
static inline void mavlink_msg_ap_uavs_func_res_decode(const mavlink_message_t* msg, mavlink_ap_uavs_func_res_t* ap_uavs_func_res)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ap_uavs_func_res->time_usec = mavlink_msg_ap_uavs_func_res_get_time_usec(msg);
    mavlink_msg_ap_uavs_func_res_get_ap_ctrl_vel(msg, ap_uavs_func_res->ap_ctrl_vel);
    ap_uavs_func_res->num_uavs = mavlink_msg_ap_uavs_func_res_get_num_uavs(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_LEN? msg->len : MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_LEN;
        memset(ap_uavs_func_res, 0, MAVLINK_MSG_ID_AP_UAVS_FUNC_RES_LEN);
    memcpy(ap_uavs_func_res, _MAV_PAYLOAD(msg), len);
#endif
}
