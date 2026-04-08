#include "drv_uart_idle_dma.h"

#include "stm32f4xx_hal_dma_ex.h"

/* 清空本 DMA 流相关标志，避免复用时残留上一次状态。 */
static void drv_uart_idle_dma_clear_flags(DMA_HandleTypeDef *hdma)
{
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_HT_FLAG_INDEX(hdma));
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TE_FLAG_INDEX(hdma));
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_DME_FLAG_INDEX(hdma));
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_FE_FLAG_INDEX(hdma));
}

static HAL_StatusTypeDef drv_uart_idle_dma_arm(drv_uart_idle_dma_t *ctx)
{
    HAL_StatusTypeDef status;

    /*
     * 第一步：检查配置是否合法。
     * 这个函数后面会直接访问 UART 和 DMA 的硬件寄存器，
     * 所以只要句柄、缓冲区或长度有问题，就必须立即返回错误。
     */
    if ((ctx == NULL) || (ctx->huart == NULL) || (ctx->hdma == NULL) || (ctx->buffer0 == NULL) || (ctx->buffer_size == 0U))
        return HAL_ERROR;

    /*
     * 双缓冲模式必须同时提供两块缓冲区。
     * 如果只给了一块缓冲区，却声明成双缓冲模式，后续 DMA 会没有合法的第二目标地址。
     */
    if ((ctx->mode == DRV_UART_IDLE_DMA_MODE_DOUBLE) && (ctx->buffer1 == NULL))
        return HAL_ERROR;

    /*
     * 第二步：如果 DMA 之前还在工作，先中止掉旧传输。
     * 这里的 arm 既会用于第一次启动，也会用于单缓冲模式下“处理完一帧后的重装”。
     */
    if (ctx->hdma->State == HAL_DMA_STATE_BUSY)
        (void)HAL_DMA_Abort(ctx->hdma);

    /*
     * 第三步：彻底关掉旧的 UART->DMA 接收链路。
     * 顺序上先关 UART 的 DMA 请求，再关 DMA 流本身，最后等待硬件确认关闭完成。
     * 这样后面改 DMA 配置时，不会和旧状态发生冲突。
     */
    ATOMIC_CLEAR_BIT(ctx->huart->Instance->CR3, USART_CR3_DMAR);
    __HAL_DMA_DISABLE(ctx->hdma);
    while ((ctx->hdma->Instance->CR & DMA_SxCR_EN) != 0U)
    {
    }

    /*
     * 清掉 DMA 的双缓冲模式位和各种状态标志。
     * 无论这次最终走单缓冲还是双缓冲，都先回到一个干净的初始状态再重新配置。
     */
    ctx->hdma->Instance->CR &= ~DMA_SxCR_DBM;
    drv_uart_idle_dma_clear_flags(ctx->hdma);

    /*
     * 第四步：同步更新 HAL 侧的 UART 接收状态。
     * 这里不仅要配置硬件寄存器，也要把 HAL 内部状态机字段改对，
     * 否则后续 HAL 相关逻辑可能认为 UART 仍处于 READY 或错误状态。
     */
    ctx->huart->ErrorCode = HAL_UART_ERROR_NONE;
    ctx->huart->RxState = HAL_UART_STATE_BUSY_RX;
    ctx->huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
    ctx->huart->pRxBuffPtr = ctx->buffer0;
    ctx->huart->RxXferSize = ctx->buffer_size;
    ctx->huart->RxXferCount = ctx->buffer_size;

    /*
     * 第五步：先清 UART 接收错误，再打开接收相关错误中断。
     * 这里暂时还没有打开 IDLE 中断，IDLE 会在 DMA 成功启动后再打开。
     *
     * - ORE: 清除可能残留的溢出错误
     * - PEIE: 若使用奇偶校验，则打开奇偶校验错误中断
     * - EIE: 打开帧错误、噪声、溢出等接收错误中断
     */
    __HAL_UART_CLEAR_OREFLAG(ctx->huart);
    if (ctx->huart->Init.Parity != UART_PARITY_NONE)
        ATOMIC_SET_BIT(ctx->huart->Instance->CR1, USART_CR1_PEIE);
    ATOMIC_SET_BIT(ctx->huart->Instance->CR3, USART_CR3_EIE);

    if (ctx->mode == DRV_UART_IDLE_DMA_MODE_DOUBLE)
    {
        /*
         * 第六步-A：双缓冲模式启动 DMA。
         * DMA 会在 buffer0 和 buffer1 之间自动切换，
         * 适合 DBUS 这种固定长度、连续不断到来的数据流。
         *
         * 源地址固定为 UART 数据寄存器 DR，
         * 目标地址为两块用户提供的缓冲区。
         */
        status = HAL_DMAEx_MultiBufferStart(ctx->hdma,
                                            (uint32_t)&ctx->huart->Instance->DR,
                                            (uint32_t)ctx->buffer0,
                                            (uint32_t)ctx->buffer1,
                                            ctx->buffer_size);
    }
    else
    {
        /*
         * 第六步-B：单缓冲模式启动 DMA。
         * 第一版导航协议先只用一块缓冲区：
         * 收到一帧后由 IDLE 中断停下 DMA，任务取走数据后再重新调用 arm 重装。
         */
        status = HAL_DMA_Start(ctx->hdma,
                               (uint32_t)&ctx->huart->Instance->DR,
                               (uint32_t)ctx->buffer0,
                               ctx->buffer_size);
    }

    if (status != HAL_OK)
    {
        /*
         * DMA 启动失败时，要把 HAL 侧 UART 状态恢复为 READY，
         * 避免上层以为 UART 还处于 BUSY_RX。
         */
        ctx->huart->RxState = HAL_UART_STATE_READY;
        return status;
    }

    /*
     * 第七步：在 DMA 已经成功装填后，打开 IDLE 中断和 UART 的 DMA 接收请求。
     *
     * - UART_IT_IDLE: 用于检测“总线空闲”，也就是一帧数据已经收完
     * - USART_CR3_DMAR: 允许 UART 把接收到的字节送给 DMA
     *
     * 到这里，这一路 UART + DMA + IDLE 接收链路才算真正进入工作状态。
     */
    __HAL_UART_ENABLE_IT(ctx->huart, UART_IT_IDLE);
    ATOMIC_SET_BIT(ctx->huart->Instance->CR3, USART_CR3_DMAR);

    /* 只要 arm 成功，就说明当前接收链路处于运行中，不是暂停状态。 */
    ctx->paused = false;
    return HAL_OK;
}

void drv_uart_idle_dma_init(drv_uart_idle_dma_t *ctx,
                            UART_HandleTypeDef *huart,
                            DMA_HandleTypeDef *hdma,
                            drv_uart_idle_dma_mode_t mode,
                            uint8_t *buffer0,
                            uint8_t *buffer1,
                            uint16_t buffer_size,
                            TaskHandle_t notify_task,
                            uint32_t notify_value)
{
    /* 这里只保存配置，不访问硬件寄存器。 */
    ctx->huart = huart;
    ctx->hdma = hdma;
    ctx->mode = mode;
    ctx->buffer0 = buffer0;
    ctx->buffer1 = buffer1;
    ctx->buffer_size = buffer_size;
    ctx->notify_task = notify_task;
    ctx->notify_value = notify_value;
    ctx->latest_buffer_index = 0U;
    ctx->latest_length = 0U;
    ctx->latest_sequence = 0U;
    ctx->frame_ready = false;
    ctx->paused = false;
}

HAL_StatusTypeDef drv_uart_idle_dma_start(drv_uart_idle_dma_t *ctx)
{
    return drv_uart_idle_dma_arm(ctx);
}

HAL_StatusTypeDef drv_uart_idle_dma_restart(drv_uart_idle_dma_t *ctx)
{
    return drv_uart_idle_dma_arm(ctx);
}

void drv_uart_idle_dma_irq_handler(drv_uart_idle_dma_t *ctx)
{
    BaseType_t higher_priority_woken = pdFALSE;
    uint8_t completed_index = 0U; /* 本次确认收完的是哪块缓冲区。 */
    uint16_t remaining = 0U;      /* DMA 还剩多少字节没有搬完。 */
    uint16_t received = 0U;       /* 本次这一帧的有效长度。 */
    uint32_t sr = 0U;

    /* 第一步：基础指针保护。没有上下文或硬件句柄时，中断不做任何事。 */
    if ((ctx == NULL) || (ctx->huart == NULL) || (ctx->hdma == NULL))
        return;

    sr = ctx->huart->Instance->SR;

    /*
     * 第二步：确认这次进入函数真的是因为 UART IDLE 中断。
     * 如果只是普通的 USART IRQ 入口被调用，但当前既没有 IDLE 标志、也没有开 IDLE 中断，
     * 那就说明这次中断不归这个驱动处理。
     *
     * 这一路 DBUS 接收不再走 HAL_UART_IRQHandler()，否则 HAL 在 ORE/FE 等错误下会
     * abort DMA，导致后续只能看到 1 字节短帧。非 IDLE 的错误中断在这里清掉即可。
     */
    if ((__HAL_UART_GET_FLAG(ctx->huart, UART_FLAG_IDLE) == RESET) ||
        (__HAL_UART_GET_IT_SOURCE(ctx->huart, UART_IT_IDLE) == RESET))
    {
        if ((sr & (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE)) != 0U)
        {
            __HAL_UART_CLEAR_PEFLAG(ctx->huart);
            ctx->huart->ErrorCode = HAL_UART_ERROR_NONE;
        }
        return;
    }

    /*
     * 第三步：清除 IDLE 标志，并读取 DMA 当前剩余计数。
     *
     * 对 UART + DMA 接收来说：
     * - DMA 一开始被设置为接收 buffer_size 个字节
     * - 每收到一个字节，DMA 计数器就减 1
     * - 所以“已经收到多少字节” = buffer_size - remaining
     *
     * HAL 的 CLEAR_IDLE 宏内部会按 UART 的要求完成清标志动作。
     */
    __HAL_UART_CLEAR_IDLEFLAG(ctx->huart);
    remaining = (uint16_t)__HAL_DMA_GET_COUNTER(ctx->hdma);

    if (ctx->mode == DRV_UART_IDLE_DMA_MODE_DOUBLE)
    {
        uint8_t current_target = ((ctx->hdma->Instance->CR & DMA_SxCR_CT) != 0U) ? 1U : 0U;

        /*
         * 第四步-A：双缓冲模式下判断“哪一块缓冲区刚刚形成完整帧”。
         *
         * current_target 表示 DMA 此刻正在写哪一块缓冲区：
         * - 0 表示当前写 buffer0
         * - 1 表示当前写 buffer1
         *
         * 这里有两种情况：
         *
         * 1. remaining == 0 或 remaining == buffer_size
         *    说明某一块缓冲区刚好被完整填满，DMA 目标已经切到另一块了。
         *    因此“刚完成的缓冲区”应该是 current_target 的另一块。
         *
         * 2. 其余情况
         *    说明这次不是收满一整块，而是因为总线空闲提前形成一帧。
         *    那么此刻正在写的 current_target 这一块，就是刚收完的那一块。
         */
        if ((remaining == 0U) || (remaining == ctx->buffer_size))
        {
            completed_index = (uint8_t)(current_target ^ 1U);
            received = ctx->buffer_size;
        }
        else
        {
            completed_index = current_target;
            received = (uint16_t)(ctx->buffer_size - remaining);
        }
    }
    else
    {
        /*
         * 第四步-B：单缓冲模式更简单。
         * 只有 buffer0，所以完成的缓冲区永远是 0。
         * 本次已接收长度同样通过“总长度 - 剩余长度”得到。
         */
        completed_index = 0U;
        received = (uint16_t)(ctx->buffer_size - remaining);

        /*
         * 单缓冲模式下一旦判定一帧结束，就先把 DMA 停住。
         * 原因是这块 buffer0 马上要交给任务解析，
         * 如果不停 DMA，后续字节可能继续写进来，破坏任务正在读取的数据。
         *
         * 后续由任务在处理完这帧后调用 restart，再重新上膛。
         */
        ATOMIC_CLEAR_BIT(ctx->huart->Instance->CR3, USART_CR3_DMAR);
        __HAL_DMA_DISABLE(ctx->hdma);
        ctx->paused = true;
    }

    /*
     * 第五步：长度合法性检查。
     * - received == 0 说明根本没有收到有效字节
     * - received > buffer_size 说明计算结果异常
     * 这两种情况都不应登记为有效帧。
     */
    if ((received == 0U) || (received > ctx->buffer_size))
        return;

    /*
     * 第六步：把“最新一帧”的元数据登记到上下文里。
     * 注意这里只记录：
     * - 帧在哪块缓冲区
     * - 长度是多少
     * - 这是第几帧
     * 不做任何协议解析。
     */
    ctx->latest_buffer_index = completed_index;
    ctx->latest_length = received;
    ctx->latest_sequence++;
    ctx->frame_ready = true;

    /*
     * 第七步：通知对应任务“有新帧到了”。
     * 这里使用任务通知位而不是消息队列，原因是：
     * - RC/NAV 只关心最新值
     * - 中断里不需要拷贝整帧到队列
     * - 开销更小
     */
    if (ctx->notify_task != NULL)
        xTaskNotifyFromISR(ctx->notify_task, ctx->notify_value, eSetBits, &higher_priority_woken);

    /*
     * 如果被唤醒的任务优先级更高，这里允许在退出 ISR 后立刻切换过去执行。
     * 这样可以尽快把刚收到的一帧交给任务处理。
     */
    portYIELD_FROM_ISR(higher_priority_woken);
}

bool drv_uart_idle_dma_take_latest(drv_uart_idle_dma_t *ctx, const uint8_t **data, uint16_t *length, uint32_t *sequence)
{
    bool ready;

    /* 用极短临界区复制元数据，避免任务侧和 ISR 同时访问。 */
    __disable_irq();
    ready = ctx->frame_ready;
    if (ready)
    {
        *data = (ctx->latest_buffer_index == 0U) ? ctx->buffer0 : ctx->buffer1;
        *length = ctx->latest_length;
        *sequence = ctx->latest_sequence;
        ctx->frame_ready = false;
    }
    __enable_irq();

    return ready;
}
