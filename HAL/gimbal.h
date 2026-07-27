#ifndef HAL_GIMBAL_H
#define HAL_GIMBAL_H
#include <stdint.h>

/* Axis identifiers (match F32C motor IDs) */
#define GIMBAL_AXIS_X  0x01
#define GIMBAL_AXIS_Y  0x02
#define GIMBAL_BOOKMARK_MAX  8

/* Forward declaration */
struct __UART_HandleTypeDef;
typedef struct Gimbal Gimbal;

struct Gimbal {
    /* === Lifecycle === */
    void      (*init)(void *self, struct __UART_HandleTypeDef *huart);
    void      (*poll)(void *self);
    void      (*enable)(void *self);
    void      (*sync)(void *self);
    uint8_t   (*is_init_done)(void *self);

    /* === Angle === */
    void      (*set_angle)(void *self, uint8_t axis, int32_t pos);
    void      (*move_delta)(void *self, uint8_t axis, int32_t delta);
    int32_t   (*get_current)(void *self, uint8_t axis);
    int32_t   (*get_target)(void *self, uint8_t axis);

    /* === Speed === */
    void      (*set_speed)(void *self, int16_t rpm);
    int16_t   (*get_speed)(void *self);

    /* === Bookmark === */
    int       (*save_bookmark)(void *self, uint8_t slot);
    int       (*go_bookmark)(void *self, uint8_t slot);
    int       (*get_bookmark)(void *self, uint8_t slot, int32_t *x, int32_t *y);

    /* === Soft Limits === */
    void      (*set_limit)(void *self, uint8_t axis, int32_t min, int32_t max);
    void      (*disable_limit)(void *self, uint8_t axis);
};

#endif
