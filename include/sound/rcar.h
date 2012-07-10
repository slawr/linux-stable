/*
 * include/sound/rcar.h
 *
 * Copyright (C) 2011-2012 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __SOUND_RCAR_H__
#define __SOUND_RCAR_H__

/************************************************************************

	define

************************************************************************/
/* direction */
#define	PLAY			0
#define	CAPT			1

/* mode */
#define SSI_MODE_MASTER		0	/* master mode	*/
#define SSI_MODE_SLAVE		1	/* slave mode	*/
#define CODEC_MODE_MASTER	0	/* master mode	*/
#define CODEC_MODE_SLAVE	1	/* slave mode	*/

/* channel */
#define	STEREO			2
#define	MONO			1

/************************************************************************

	structure

************************************************************************/
struct rcar_ssi_ctrl {
	unsigned char	m_s;	/* master/slave */
};
#define rcar_ssi_ctrl_t		struct rcar_ssi_ctrl

struct rcar_codec_ctrl {
	unsigned char	m_s;	/* master/slave */
};
#define rcar_codec_ctrl_t	struct rcar_codec_ctrl

struct rcar_pcm_ctrl {
	rcar_ssi_ctrl_t		ssi0;		/* not use */
	rcar_ssi_ctrl_t		ssi1;		/* not use */
	rcar_ssi_ctrl_t		ssi2;		/* not use */
	rcar_ssi_ctrl_t		ssi3;
	rcar_ssi_ctrl_t		ssi4;
	rcar_ssi_ctrl_t		ssi5;
	rcar_ssi_ctrl_t		ssi6;
	rcar_ssi_ctrl_t		ssi7;
	rcar_ssi_ctrl_t		ssi8;
	rcar_codec_ctrl_t	codec1;		/* AK4643 */
	rcar_codec_ctrl_t	codec2;		/* AK4554 */
	rcar_codec_ctrl_t	codec3;		/* AK4554 */
};
#define rcar_pcm_ctrl_t	struct rcar_pcm_ctrl

#define SNDRV_RCAR_IOCTL_SET_PCM _IOW('H', 0x00, struct rcar_pcm_ctrl)
#define SNDRV_RCAR_IOCTL_GET_PCM _IOR('H', 0x01, struct rcar_pcm_ctrl)

#endif	/* __SOUND_RCAR_H__ */
