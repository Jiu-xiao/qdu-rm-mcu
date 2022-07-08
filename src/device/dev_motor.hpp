#pragma once

#include <stdbool.h>

#include "FreeRTOS.h"
#include "bsp_can.h"
#include "comp_ahrs.hpp"
#include "comp_utils.hpp"
#include "dev.hpp"
#include "queue.h"

/* Motor id */
/* id     feedback id     control id */
/* 1-4    0x205 to 0x208  0x1ff */
/* 5-6    0x209 to 0x20B  0x2ff */
#define GM6020_FB_ID_BASE (0x205)
#define GM6020_FB_ID_EXTAND (0x209)
#define GM6020_CTRL_ID_BASE (0x1ff)
#define GM6020_CTRL_ID_EXTAND (0x2ff)

/* id     feedback id		  control id */
/* 1-4		0x201 to 0x204  0x200 */
/* 5-6		0x205 to 0x208  0x1ff */
#define M3508_M2006_FB_ID_BASE (0x201)
#define M3508_M2006_FB_ID_EXTAND (0x205)
#define M3508_M2006_CTRL_ID_BASE (0x200)
#define M3508_M2006_CTRL_ID_EXTAND (0x1ff)
#define M3508_M2006_ID_SETTING_ID (0x700)

/* 电机型号 */
typedef enum {
  MOTOR_NONE = 0,
  MOTOR_M2006,
  MOTOR_M3508,
  MOTOR_GM6020,
} motor_model_t;

typedef struct {
  uint32_t id_feedback;
  uint32_t id_control;
  motor_model_t model[4];
  uint8_t num;
  bsp_can_t can;
} motor_group_t;

typedef union {
  float as_array[4];

  struct {
    float m1;
    float m2;
    float m3;
    float m4;
  } as_chassis;

  struct {
    float yaw;
  } as_gimbal_yaw;

  struct {
    float pit;
  } as_gimbal_pit;

  struct {
    float fric_left;
    float fric_right;
  } as_launcher_fric;

  struct {
    float trig;
  } as_launcher_trig;
} motor_control_t;

/* 电机反馈信息 */
typedef struct {
  float rotor_abs_angle;  /* 转子绝对角度 单位：rad */
  float rotational_speed; /* 转速 单位：rpm */
  float torque_current;   /* 转矩电流 单位：A*/
  float temp;             /* 电机温度 单位：℃*/
} motor_feedback_t;

typedef union {
  motor_feedback_t as_array[4];

  struct {
    motor_feedback_t m1;
    motor_feedback_t m2;
    motor_feedback_t m3;
    motor_feedback_t m4;
  } as_chassis;

  struct {
    motor_feedback_t yaw;
  } as_gimbal_yaw;

  struct {
    motor_feedback_t pit;
  } as_gimbal_pit;

  struct {
    motor_feedback_t fric_left;
    motor_feedback_t fric_right;
  } as_launcher_fric;

  struct {
    motor_feedback_t trig;
  } as_launcher_trig;
} motor_feedback_group_t;

typedef enum {
  MOTOR_GROUP_ID_CHASSIS = 0,
  MOTOR_GROUP_ID_GIMBAL_YAW,
  MOTOR_GROUP_ID_GIMBAL_PIT,
  MOTOR_GROUP_ID_LAUNCHER_FRIC,
  MOTOR_GROUP_ID_LAUNCHER_TRIG,
  MOTOR_GROUP_ID_NUM,
} motor_group_id_t;

typedef struct {
  motor_feedback_group_t feedback[MOTOR_GROUP_ID_NUM];
  QueueHandle_t msgq[MOTOR_GROUP_ID_NUM];
  uint32_t mailbox;
  const motor_group_t *group_cfg;
} motor_t;

int8_t motor_init(motor_t *motor, const motor_group_t *group_cfg);
int8_t motor_update(motor_t *motor);
int8_t motor_pack_data(motor_t *motor, motor_group_id_t group,
                      motor_control_t *output);
int8_t motor_control(motor_t *motor);
int8_t motor_handle_offline(motor_t *motor);