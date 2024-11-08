#define USE_STDIO 0  // 设为 0 禁用标准 I/O

#if USE_STDIO
#include <stdio.h>
#endif

// 定义 `_sys_exit` 函数，不返回
__attribute__((noreturn)) void _sys_exit(int x) {
    while (1) {}
}




