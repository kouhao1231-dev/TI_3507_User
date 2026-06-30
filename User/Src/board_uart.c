#include "board_uart.h"

#include "ti_msp_dl_config.h"

#define BOARD_UART_INST        UART2
#define BOARD_UART_BAUD        115200U
#define BOARD_UART_CLK_FREQ    CPUCLK_FREQ
#define BOARD_UART_TX_IOMUX    IOMUX_PINCM43
#define BOARD_UART_TX_PF       IOMUX_PINCM43_PF_UART2_TX
#define BOARD_UART_RX_IOMUX    IOMUX_PINCM44
#define BOARD_UART_RX_PF       IOMUX_PINCM44_PF_UART2_RX
#define BOARD_UART_RX_BUF_SIZE 64U
#define BOARD_UART_LINE_SIZE   32U
#define BOARD_UART_LINE_QUEUE  4U
#define BOARD_UART_TX_WAIT     200000U

static volatile BoardUartStatus g_uart_status = BOARD_UART_NOT_CONFIGURED;
static volatile uint32_t g_uart_tx_count;
static volatile uint32_t g_uart_rx_count;
static volatile uint8_t g_uart_last_rx;
static volatile uint8_t g_uart_rx_buf[BOARD_UART_RX_BUF_SIZE];
static volatile uint8_t g_uart_rx_head;
static volatile uint8_t g_uart_rx_tail;
static volatile uint32_t g_uart_rx_overflow_count;
static char g_uart_line_buf[BOARD_UART_LINE_SIZE];
static char g_uart_last_line[BOARD_UART_LINE_SIZE];
static char g_uart_line_queue[BOARD_UART_LINE_QUEUE][BOARD_UART_LINE_SIZE];
static volatile uint8_t g_uart_line_pos;
static volatile uint8_t g_uart_line_head;
static volatile uint8_t g_uart_line_tail;
static volatile uint32_t g_uart_line_overflow_count;
volatile uint32_t g_uart_irq_count;

static void uart_copy_text(char *out, uint8_t max_len, const char *text)
{
    uint8_t i = 0U;

    if ((out == 0) || (max_len == 0U)) {
        return;
    }
    if (text != 0) {
        while ((text[i] != '\0') && (i < (uint8_t) (max_len - 1U))) {
            out[i] = text[i];
            i++;
        }
    }
    out[i] = '\0';
}

static uint8_t uart_text_len(const char *text)
{
    uint8_t len = 0U;

    if (text == 0) {
        return 0U;
    }
    while ((text[len] != '\0') && (len < 250U)) {
        len++;
    }
    return len;
}

static uint8_t uart_append_text(char *out, uint8_t pos, uint8_t max_len, const char *text)
{
    if ((out == 0) || (max_len == 0U) || (text == 0)) {
        return pos;
    }
    while ((text[0] != '\0') && (pos < (uint8_t) (max_len - 1U))) {
        out[pos++] = *text++;
    }
    out[pos] = '\0';
    return pos;
}

static void uart_append_unsigned(char *out, uint8_t *pos, uint8_t max_len, uint32_t value, uint8_t hex)
{
    char tmp[10];
    uint8_t count = 0U;
    const char digits[] = "0123456789ABCDEF";
    uint32_t base = hex ? 16U : 10U;

    if ((out == 0) || (pos == 0) || (max_len == 0U)) {
        return;
    }

    if (value == 0U) {
        tmp[count++] = '0';
    } else {
        while ((value > 0U) && (count < sizeof(tmp))) {
            tmp[count++] = digits[value % base];
            value /= base;
        }
    }

    while ((count > 0U) && (*pos < (uint8_t) (max_len - 1U))) {
        out[(*pos)++] = tmp[--count];
    }
    out[*pos] = '\0';
}

static void uart_store_rx(uint8_t byte)
{
    uint8_t next = (uint8_t) ((g_uart_rx_head + 1U) % BOARD_UART_RX_BUF_SIZE);

    if (next == g_uart_rx_tail) {
        g_uart_rx_overflow_count++;
        return;
    }

    g_uart_rx_buf[g_uart_rx_head] = byte;
    g_uart_rx_head = next;
}

static void uart_store_line_byte(uint8_t byte)
{
    if (byte == '\r') {
        return;
    }
    if (byte == '\n') {
        uint8_t next = (uint8_t) ((g_uart_line_head + 1U) % BOARD_UART_LINE_QUEUE);

        g_uart_line_buf[g_uart_line_pos] = '\0';
        uart_copy_text(g_uart_last_line, BOARD_UART_LINE_SIZE, g_uart_line_buf);
        if (next == g_uart_line_tail) {
            g_uart_line_overflow_count++;
        } else {
            uart_copy_text(g_uart_line_queue[g_uart_line_head], BOARD_UART_LINE_SIZE, g_uart_line_buf);
            g_uart_line_head = next;
        }
        g_uart_line_pos = 0U;
        return;
    }
    if (g_uart_line_pos < (BOARD_UART_LINE_SIZE - 1U)) {
        g_uart_line_buf[g_uart_line_pos++] = (char) byte;
    }
}

static void uart_poll_rx(void)
{
    while (!DL_UART_isRXFIFOEmpty(BOARD_UART_INST)) {
        uint8_t byte = DL_UART_receiveData(BOARD_UART_INST);
        g_uart_last_rx = byte;
        g_uart_rx_count++;
        uart_store_rx(byte);
        uart_store_line_byte(byte);
    }
}

static uint8_t uart_wait_tx_ready(void)
{
    uint32_t wait = BOARD_UART_TX_WAIT;

    while ((DL_UART_isTXFIFOFull(BOARD_UART_INST) || DL_UART_isBusy(BOARD_UART_INST)) && (wait > 0U)) {
        wait--;
    }
    return (wait > 0U) ? 1U : 0U;
}

void BoardUart_Init(void)
{
    const DL_UART_ClockConfig clock_config = {
        .clockSel = DL_UART_CLOCK_BUSCLK,
        .divideRatio = DL_UART_CLOCK_DIVIDE_RATIO_1,
    };
    const DL_UART_Config uart_config = {
        .mode = DL_UART_MODE_NORMAL,
        .direction = DL_UART_DIRECTION_TX_RX,
        .flowControl = DL_UART_FLOW_CONTROL_NONE,
        .parity = DL_UART_PARITY_NONE,
        .wordLength = DL_UART_WORD_LENGTH_8_BITS,
        .stopBits = DL_UART_STOP_BITS_ONE,
    };

    DL_GPIO_initPeripheralOutputFunction(BOARD_UART_TX_IOMUX, BOARD_UART_TX_PF);
    DL_GPIO_initPeripheralInputFunction(BOARD_UART_RX_IOMUX, BOARD_UART_RX_PF);

    DL_UART_reset(BOARD_UART_INST);
    DL_UART_enablePower(BOARD_UART_INST);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_UART_setClockConfig(BOARD_UART_INST, &clock_config);
    DL_UART_init(BOARD_UART_INST, &uart_config);
    DL_UART_setOversampling(BOARD_UART_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_configBaudRate(BOARD_UART_INST, BOARD_UART_CLK_FREQ, BOARD_UART_BAUD);
    DL_UART_enableFIFOs(BOARD_UART_INST);
    DL_UART_setRXFIFOThreshold(BOARD_UART_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_setRXInterruptTimeout(BOARD_UART_INST, 1U);
    DL_UART_enableInterrupt(BOARD_UART_INST,
        DL_UART_INTERRUPT_RX | DL_UART_INTERRUPT_RX_TIMEOUT_ERROR |
        DL_UART_INTERRUPT_OVERRUN_ERROR | DL_UART_INTERRUPT_FRAMING_ERROR);
    DL_UART_enable(BOARD_UART_INST);
    NVIC_SetPriority(UART2_INT_IRQn, 2U);
    NVIC_ClearPendingIRQ(UART2_INT_IRQn);
    NVIC_EnableIRQ(UART2_INT_IRQn);

    g_uart_tx_count = 0U;
    g_uart_rx_count = 0U;
    g_uart_last_rx = 0U;
    g_uart_rx_head = 0U;
    g_uart_rx_tail = 0U;
    g_uart_rx_overflow_count = 0U;
    g_uart_irq_count = 0U;
    g_uart_line_pos = 0U;
    g_uart_line_head = 0U;
    g_uart_line_tail = 0U;
    g_uart_line_overflow_count = 0U;
    g_uart_line_buf[0] = '\0';
    g_uart_last_line[0] = '\0';
    g_uart_status = BOARD_UART_READY;
}

void BoardUart_Task(void)
{
    if (g_uart_status == BOARD_UART_READY) {
        uart_poll_rx();
    }
}

BoardUartStatus BoardUart_GetStatus(void)
{
    return g_uart_status;
}

uint8_t BoardUart_WriteByte(uint8_t byte)
{
    if (g_uart_status != BOARD_UART_READY) {
        return 0U;
    }
    if (!uart_wait_tx_ready()) {
        return 0U;
    }

    DL_UART_transmitData(BOARD_UART_INST, byte);
    g_uart_tx_count++;
    return 1U;
}

uint8_t BoardUart_Write(const uint8_t *data, uint16_t len)
{
    uint16_t sent = 0U;

    if (data == 0) {
        return 0U;
    }

    while (sent < len) {
        if (!BoardUart_WriteByte(data[sent])) {
            break;
        }
        sent++;
    }

    return (uint8_t) sent;
}

uint8_t BoardUart_SendText(const char *text)
{
    return BoardUart_Write((const uint8_t *) text, uart_text_len(text));
}

uint8_t BoardUart_SendLine(const char *text)
{
    uint8_t sent = BoardUart_SendText(text);
    static const uint8_t newline[] = "\r\n";

    sent = (uint8_t) (sent + BoardUart_Write(newline, 2U));
    return sent;
}

uint8_t BoardUart_SendNumber(const char *label, int32_t value)
{
    char buf[BOARD_UART_LINE_SIZE];
    uint8_t pos = 0U;
    uint32_t mag;

    pos = uart_append_text(buf, pos, BOARD_UART_LINE_SIZE, label);
    if (pos < (BOARD_UART_LINE_SIZE - 1U)) {
        buf[pos++] = ':';
        buf[pos] = '\0';
    }
    if (value < 0) {
        if (pos < (BOARD_UART_LINE_SIZE - 1U)) {
            buf[pos++] = '-';
        }
        mag = (uint32_t) (-value);
    } else {
        mag = (uint32_t) value;
    }
    uart_append_unsigned(buf, &pos, BOARD_UART_LINE_SIZE, mag, 0U);
    return BoardUart_SendLine(buf);
}

uint8_t BoardUart_SendHex(const char *label, uint32_t value)
{
    char buf[BOARD_UART_LINE_SIZE];
    uint8_t pos = 0U;

    pos = uart_append_text(buf, pos, BOARD_UART_LINE_SIZE, label);
    if (pos < (BOARD_UART_LINE_SIZE - 1U)) {
        buf[pos++] = ':';
    }
    if (pos < (BOARD_UART_LINE_SIZE - 1U)) {
        buf[pos++] = '0';
    }
    if (pos < (BOARD_UART_LINE_SIZE - 1U)) {
        buf[pos++] = 'x';
    }
    buf[pos] = '\0';
    uart_append_unsigned(buf, &pos, BOARD_UART_LINE_SIZE, value, 1U);
    return BoardUart_SendLine(buf);
}

uint16_t BoardUart_Available(void)
{
    if (g_uart_rx_head >= g_uart_rx_tail) {
        return (uint16_t) (g_uart_rx_head - g_uart_rx_tail);
    }
    return (uint16_t) (BOARD_UART_RX_BUF_SIZE - g_uart_rx_tail + g_uart_rx_head);
}

uint8_t BoardUart_ReadByte(uint8_t *byte)
{
    if ((byte == 0) || (g_uart_status != BOARD_UART_READY)) {
        return 0U;
    }
    uart_poll_rx();
    if (g_uart_rx_tail == g_uart_rx_head) {
        return 0U;
    }

    *byte = g_uart_rx_buf[g_uart_rx_tail];
    g_uart_rx_tail = (uint8_t) ((g_uart_rx_tail + 1U) % BOARD_UART_RX_BUF_SIZE);
    return 1U;
}

uint8_t BoardUart_ReadLine(char *out, uint8_t max_len)
{
    if ((out == 0) || (max_len == 0U) || (g_uart_status != BOARD_UART_READY)) {
        return 0U;
    }
    uart_poll_rx();
    if (g_uart_line_tail == g_uart_line_head) {
        out[0] = '\0';
        return 0U;
    }

    uart_copy_text(out, max_len, g_uart_line_queue[g_uart_line_tail]);
    g_uart_line_tail = (uint8_t) ((g_uart_line_tail + 1U) % BOARD_UART_LINE_QUEUE);
    return 1U;
}

const char *BoardUart_GetLastLine(void)
{
    return g_uart_last_line;
}

uint32_t BoardUart_GetTxCount(void)
{
    return g_uart_tx_count;
}

uint32_t BoardUart_GetRxCount(void)
{
    return g_uart_rx_count;
}

uint8_t BoardUart_GetLastRx(void)
{
    return g_uart_last_rx;
}

void UART2_IRQHandler(void)
{
    g_uart_irq_count++;
    uart_poll_rx();
    DL_UART_clearInterruptStatus(BOARD_UART_INST,
        DL_UART_INTERRUPT_RX | DL_UART_INTERRUPT_RX_TIMEOUT_ERROR |
        DL_UART_INTERRUPT_OVERRUN_ERROR | DL_UART_INTERRUPT_FRAMING_ERROR);
}
