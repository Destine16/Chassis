#include "common_assert.h"

#include "main.h"

void common_assert_failed(const char *expr, const char *file, uint32_t line)
{
    /* 第一版仅保留停机入口，表达式和文件行号留给后续调试扩展。 */
    (void)expr;
    (void)file;
    (void)line;

    Error_Handler();
}
