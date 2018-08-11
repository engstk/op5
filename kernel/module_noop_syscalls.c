/* SPDX-License-Identifier: GPL-2.0 */

#include <linux/syscalls.h>

SYSCALL_DEFINE3(init_module, void __user *, umod,
		unsigned long, len, const char __user *, uargs)
{
	return 0;
}

SYSCALL_DEFINE3(finit_module, int, fd, const char __user *, uargs, int, flags)
{
	return 0;
}
