#ifndef __ASM_RCAR_VIN_H__
#define __ASM_RCAR_VIN_H__

#include <media/soc_camera.h>

#define RCAR_VIN_FLAG_HSYNC_LOW		(1 << 0) /* default High if possible */
#define RCAR_VIN_FLAG_VSYNC_LOW		(1 << 1) /* default High if possible */

struct rcar_vin_info {
	unsigned long flags;
	enum v4l2_mbus_pixelcode format;
};

#endif /* __ASM_RCAR_VIN_H__ */
