/*
 * arch/arm/mach-shmobile/include/mach/io.h
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __ASM_ARCH_IO_H
#define __ASM_ARCH_IO_H

#define IO_SPACE_LIMIT		0xffffffff

#define __io(a)  	__typesafe_io(a)

#endif
