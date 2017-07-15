#pragma once
// MESSAGE CA_TRAJECT_RES PACKING

#define MAVLINK_MSG_ID_CA_TRAJECT_RES 154

MAVPACKED(
typedef struct __mavlink_ca_traject_res_t {
 uint64_t time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 uint64_t PC_time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 float P_d[4]; /*< desire position*/
 float vel_d[4]; /*< desire vel*/
 float acc_d[4]; /*< desire position*/
}) mavlink_ca_traject_res_t;

#define MAVLINK_MSG_ID_CA_TRAJECT_RES_LEN 64
#define MAVLINK_MSG_ID_CA_TRAJECT_RES_MIN_LEN 64
#define MAVLINK_MSG_ID_154_LEN 64
#define MAVLINK_MSG_ID_154_MIN_LEN 64

#define MAVLINK_MSG_ID_CA_TRAJECT_RES_CRC 183
#define MAVLINK_MSG_ID_154_CRC 183

#define MAVLINK_MSG_CA_TRAJECT_RES_FIELD_P_D_LEN 4
#define MAVLINK_MSG_CA_TRAJECT_RES_FIELD_VEL_D_LEN 4
#define MAVLINK_MSG_CA_TRAJECT_RES_FIELD_ACC_D_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CA_TRAJECT_RES { \
    154, \
    "CA_TRAJECT_RES", \
    5, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ca_traject_res_t, time_usec) }, \
         { "PC_time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_ca_traject_res_t, PC_time_usec) }, \
         { "P_d", NULL, MAVLINK_TYPE_FLOAT, 4, 16, offsetof(mavlink_ca_traject_res_t, P_d) }, \
         { "vel_d", NULL, MAVLINK_TYPE_FLOAT, 4, 32, offsetof(mavlink_ca_traject_res_t, vel_d) }, \
         { "acc_d", NULL, MAVLINK_TYPE_FLOAT, 4, 48, offsetof(mavlink_ca_traject_res_t, acc_d) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CA_TRAJECT_RES { \
    "CA_TRAJECT_RES", \
    5, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ca_traject_res_t, time_usec) }, \
         { "PC_time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_ca_traject_res_t, PC_time_usec) }, \
         { "P_d", NULL, MAVLINK_TYPE_FLOAT, 4, 16, offsetof(mavlink_ca_traject_res_t, P_d) }, \
         { "vel_d", NULL, MAVLINK_TYPE_FLOAT, 4, 32, offsetof(mavlink_ca_traject_res_t, vel_d) }, \
         { "acc_d", NULL, MAVLINK_TYPE_FLOAT, 4, 48, offsetof(mavlink_ca_traject_res_t, acc_d) }, \
         } \
}
#endif

/**
 * @brief Pack a ca_traject_res message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param PC_time_usec Timestamp (micros since boot or Unix epoch)
 * @param P_d desire position
 * @param vel_d desire vel
 * @param acc_d desire position
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ca_traject_res_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint64_t PC_time_usec, const float *P_d, const float *vel_d, const float *acc_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CA_TRAJECT_RES_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, PC_time_usec);
    _mav_put_float_array(buf, 16, P_d, 4);
    _mav_put_float_array(buf, 32, vel_d, 4);
    _mav_put_float_array(buf, 48, acc_d, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CA_TRAJECT_RES_LEN);
#else
    mavlink_ca_traject_res_t packet;
    packet.time_usec = time_usec;
    packet.PC_time_usec = PC_time_usec;
    mav_array_memcpy(packet.P_d, P_d, sizeof(float)*4);
    mav_array_memcpy(packet.vel_d, vel_d, sizeof(float)*4);
    mav_array_memcpy(packet.acc_d, acc_d, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CA_TRAJECT_RES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CA_TRAJECT_RES;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CA_TRAJECT_RES_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECT_RES_LEN, MAVLINK_MSG_ID_CA_TRAJECT_RES_CRC);
}

/**
 * @brief Pack a ca_traject_res message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param PC_time_usec Timestamp (micros since boot or Unix epoch)
 * @param P_d desire position
 * @param vel_d desire vel
 * @param acc_d desire position
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ca_traject_res_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint64_t PC_time_usec,const float *P_d,const float *vel_d,const float *acc_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CA_TRAJECT_RES_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, PC_time_usec);
    _mav_put_float_array(buf, 16, P_d, 4);
    _mav_put_float_array(buf, 32, vel_d, 4);
    _mav_put_float_array(buf, 48, acc_d, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CA_TRAJECT_RES_LEN);
#else
    mavlink_ca_traject_res_t packet;
    packet.time_usec = time_usec;
    packet.PC_time_usec = PC_time_usec;
    mav_array_memcpy(packet.P_d, P_d, sizeof(float)*4);
    mav_array_memcpy(packet.vel_d, vel_d, sizeof(float)*4);
    mav_array_memcpy(packet.acc_d, acc_d, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CA_TRAJECT_RES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CA_TRAJECT_RES;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CA_TRAJECT_RES_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECT_RES_LEN, MAVLINK_MSG_ID_CA_TRAJECT_RES_CRC);
}

/**
 * @brief Encode a ca_traject_res struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ca_traject_res C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ca_traject_res_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ca_traject_res_t* ca_traject_res)
{
    return mavlink_msg_ca_traject_res_pack(system_id, component_id, msg, ca_traject_res->time_usec, ca_traject_res->PC_time_usec, ca_traject_res->P_d, ca_traject_res->vel_d, ca_traject_res->acc_d);
}

/**
 * @brief Encode a ca_traject_res struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ca_traject_res C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ca_traject_res_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ca_traject_res_t* ca_traject_res)
{
    return mavlink_msg_ca_traject_res_pack_chan(system_id, component_id, chan, msg, ca_traject_res->time_usec, ca_traject_res->PC_time_usec, ca_traject_res->P_d, ca_traject_res->vel_d, ca_traject_res->acc_d);
}

/**
 * @brief Send a ca_traject_res message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param PC_time_usec Timestamp (micros since boot or Unix epoch)
 * @param P_d desire position
 * @param vel_d desire vel
 * @param acc_d desire position
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ca_traject_res_send(mavlink_channel_t chan, uint64_t time_usec, uint64_t PC_time_usec, const float *P_d, const float *vel_d, const float *acc_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CA_TRAJECT_RES_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, PC_time_usec);
    _mav_put_float_array(buf, 16, P_d, 4);
    _mav_put_float_array(buf, 32, vel_d, 4);
    _mav_put_float_array(buf, 48, acc_d, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CA_TRAJECT_RES, buf, MAVLINK_MSG_ID_CA_TRAJECT_RES_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECT_RES_LEN, MAVLINK_MSG_ID_CA_TRAJECT_RES_CRC);
#else
    mavlink_ca_traject_res_t packet;
    packet.time_usec = time_usec;
    packet.PC_time_usec = PC_time_usec;
    mav_array_memcpy(packet.P_d, P_d, sizeof(float)*4);
    mav_array_memcpy(packet.vel_d, vel_d, sizeof(float)*4);
    mav_array_memcpy(packet.acc_d, acc_d, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CA_TRAJECT_RES, (const char *)&packet, MAVLINK_MSG_ID_CA_TRAJECT_RES_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECT_RES_LEN, MAVLINK_MSG_ID_CA_TRAJECT_RES_CRC);
#endif
}

/**
 * @brief Send a ca_traject_res message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ca_traject_res_send_struct(mavlink_channel_t chan, const mavlink_ca_traject_res_t* ca_traject_res)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ca_traject_res_send(chan, ca_traject_res->time_usec, ca_traject_res->PC_time_usec, ca_traject_res->P_d, ca_traject_res->vel_d, ca_traject_res->acc_d);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CA_TRAJECT_RES, (const char *)ca_traject_res, MAVLINK_MSG_ID_CA_TRAJECT_RES_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECT_RES_LEN, MAVLINK_MSG_ID_CA_TRAJECT_RES_CRC);
#endif
}

#if MAVLINK_MSG_ID_CA_TRAJECT_RES_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ca_traject_res_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint64_t PC_time_usec, const float *P_d, const float *vel_d, const float *acc_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, PC_time_usec);
    _mav_put_float_array(buf, 16, P_d, 4);
    _mav_put_float_array(buf, 32, vel_d, 4);
    _mav_put_float_array(buf, 48, acc_d, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CA_TRAJECT_RES, buf, MAVLINK_MSG_ID_CA_TRAJECT_RES_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECT_RES_LEN, MAVLINK_MSG_ID_CA_TRAJECT_RES_CRC);
#else
    mavlink_ca_traject_res_t *packet = (mavlink_ca_traject_res_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->PC_time_usec = PC_time_usec;
    mav_array_memcpy(packet->P_d, P_d, sizeof(float)*4);
    mav_array_memcpy(packet->vel_d, vel_d, sizeof(float)*4);
    mav_array_memcpy(packet->acc_d, acc_d, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CA_TRAJECT_RES, (const char *)packet, MAVLINK_MSG_ID_CA_TRAJECT_RES_MIN_LEN, MAVLINK_MSG_ID_CA_TRAJECT_RES_LEN, MAVLINK_MSG_ID_CA_TRAJECT_RES_CRC);
#endif
}
#endif

#endif

// MESSAGE CA_TRAJECT_RES UNPACKING


/**
 * @brief Get field time_usec from ca_traject_res message
 *
 * @return Timestamp (micros since boot or Unix epoch)
 */
static inline uint64_t mavlink_msg_ca_traject_res_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field PC_time_usec from ca_traject_res message
 *
 * @return Timestamp (micros since boot or Unix epoch)
 */
static inline uint64_t mavlink_msg_ca_traject_res_get_PC_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field P_d from ca_traject_res message
 *
 * @return desire position
 */
static inline uint16_t mavlink_msg_ca_traject_res_get_P_d(const mavlink_message_t* msg, float *P_d)
{
    return _MAV_RETURN_float_array(msg, P_d, 4,  16);
}

/**
 * @brief Get field vel_d from ca_traject_res message
 *
 * @return desire vel
 */
static inline uint16_t mavlink_msg_ca_traject_res_get_vel_d(const mavlink_message_t* msg, float *vel_d)
{
    return _MAV_RETURN_float_array(msg, vel_d, 4,  32);
}

/**
 * @brief Get field acc_d from ca_traject_res message
 *
 * @return desire position
 */
static inline uint16_t mavlink_msg_ca_traject_res_get_acc_d(const mavlink_message_t* msg, float *acc_d)
{
    return _MAV_RETURN_float_array(msg, acc_d, 4,  48);
}

/**
 * @brief Decode a ca_traject_res message into a struct
 *
 * @param msg The message to decode
 * @param ca_traject_res C-struct to decode the message contents into
 */
static inline void mavlink_msg_ca_traject_res_decode(const mavlink_message_t* msg, mavlink_ca_traject_res_t* ca_traject_res)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ca_traject_res->time_usec = mavlink_msg_ca_traject_res_get_time_usec(msg);
    ca_traject_res->PC_time_usec = mavlink_msg_ca_traject_res_get_PC_time_usec(msg);
    mavlink_msg_ca_traject_res_get_P_d(msg, ca_traject_res->P_d);
    mavlink_msg_ca_traject_res_get_vel_d(msg, ca_traject_res->vel_d);
    mavlink_msg_ca_traject_res_get_acc_d(msg, ca_traject_res->acc_d);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CA_TRAJECT_RES_LEN? msg->len : MAVLINK_MSG_ID_CA_TRAJECT_RES_LEN;
        memset(ca_traject_res, 0, MAVLINK_MSG_ID_CA_TRAJECT_RES_LEN);
    memcpy(ca_traject_res, _MAV_PAYLOAD(msg), len);
#endif
}
