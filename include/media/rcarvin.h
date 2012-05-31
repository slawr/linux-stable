#ifndef __ASM_RCAR_VIN_H__
#define __ASM_RCAR_VIN_H__

#include <media/soc_camera.h>

#define RCAR_VIN_FLAG_USE_8BIT_BUS	(1 << 0) /* use  8bit bus width */
#define RCAR_VIN_FLAG_USE_16BIT_BUS	(1 << 1) /* use 16bit bus width */
#define RCAR_VIN_FLAG_HSYNC_LOW		(1 << 2) /* default High if possible */
#define RCAR_VIN_FLAG_VSYNC_LOW		(1 << 3) /* default High if possible */

struct rcar_vin_info {
	unsigned long flags;
};

#endif /* __ASM_RCAR_VIN_H__ */
