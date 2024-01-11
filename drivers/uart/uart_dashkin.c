/*
 *  UART driver for Dashkin CPU
 */

#define DT_DRV_COMPAT dashkin_uart

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/irq.h>

//FIFO Depth
#define DASHKIN_UART_FIFO_DEPTH 8

//Memory offsets
#define DASHKIN_UART_CFG_OFFSET     0x00
#define DASHKIN_UART_RX_DATA_OFFSET 0x04
#define DASHKIN_UART_TX_DATA_OFFSET 0x08
#define DASHKIN_UART_RX_STAT_OFFSET 0x0C
#define DASHKIN_UART_TX_STAT_OFFSET 0x10

//Bit masks
#define DASHKIN_UART_CFG_PARITY 0x00000001
#define DASHKIN_UART_CFG_STOP   0x00000002
#define DASHKIN_UART_CFG_ORDER  0x00000004
#define DASHKIN_UART_CFG_BAUD   0x000000E0
#define DASHKIN_UART_CFG_RX_CNT 0x0000FF00
#define DASHKIN_UART_CFG_RX_IRQ 0x00010000
#define DASHKIN_UART_CFG_TX_CNT 0x0FF00000
#define DASHKIN_UART_CFG_TX_IRQ 0x10000000

#define DASHKIN_UART_RX_STAT_ERROR  0x00000001
#define DASHKIN_UART_RX_STAT_FULL   0x00000002
#define DASHKIN_UART_RX_STAT_EMPTY  0x00000004
#define DASHKIN_UART_RX_STAT_READY  0x00000008

#define DASHKIN_UART_TX_STAT_FULL   0x00000001
#define DASHKIN_UART_TX_STAT_EMPTY  0x00000002

#define DASHKIN_UART_RX_EMPTY 0x100
#define DASHKIN_UART_RX_DATA  0x0FF

/*
  Zephyr has ROM and RAM parts of the driver.
  In this driver everything is ROM. Nothing computed dynamically.
*/

/*
  Config
*/
struct dashkin_uart_config
{
  mem_addr_t  base;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	void (*irq_config_func)(const struct device *dev);
#endif
};

struct dashkin_uart_data
{
	struct uart_config uart_cfg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
  uart_irq_callback_user_data_t callback;
  void *callback_data;
#endif
};

#define GET_DTS_CFG(dev) \
    ((const struct dashkin_uart_config * const)(dev)->config)
#define GET_UART_BASE(dev) \
    (GET_DTS_CFG(dev))->base
/*
  Function to read one byte from RX FIFO
*/
static inline uint32_t dashkin_uart_read_data(const struct device *dev, uint32_t offset)
{
  return sys_read32(GET_UART_BASE(dev)+offset);
}

static inline void dashkin_uart_write_data(const struct device *dev, uint32_t data, uint32_t offset)
{
  sys_write32(data, GET_UART_BASE(dev)+offset);

  return;
}

/*
Standard UART Functions
*/
int dashkin_uart_err_check(const struct device *dev)
{
  return 0;
}
/*
struct uart_config
uint32_t baudrate - Baudrate setting in bps
uint8_t  parity   - Parity bit, use uart_config_parity
uint8_t  stop_bits - Stop bits, use uart_config_stop_bits
uint8_t  data_bits - Data bits, use uart_config_data_bits
uint8_t  flow_ctrl - Flow control setting, use uart_config_flow_control
*/
int dashkin_uart_configure(const struct device *dev, const struct uart_config *cfg)
{
  uint32_t dashkin_cfg = 0;

  if (cfg->data_bits != UART_CFG_DATA_BITS_8) {
    return -ENOSYS;
  }

  switch (cfg->parity) {
  case UART_CFG_PARITY_EVEN : dashkin_cfg |= 0x0; break;
  case UART_CFG_PARITY_ODD  : dashkin_cfg |= 0x1; break;
  default:
    return -ENOSYS;
  }

  switch (cfg->stop_bits) {
  case UART_CFG_STOP_BITS_1 : dashkin_cfg |= 0x0 << 1; break;
  case UART_CFG_STOP_BITS_2 : dashkin_cfg |= 0x1 << 1; break;
  default:
    return -ENOSYS;
  }

  switch (cfg->baudrate) {
  case 1200   : dashkin_cfg |= 0b000 << 5; break;
  case 2400   : dashkin_cfg |= 0b001 << 5; break;
  case 4800   : dashkin_cfg |= 0b010 << 5; break;
  case 9600   : dashkin_cfg |= 0b011 << 5; break;
  case 19200  : dashkin_cfg |= 0b100 << 5; break;
  case 38400  : dashkin_cfg |= 0b101 << 5; break;
  case 57600  : dashkin_cfg |= 0b110 << 5; break;
  case 115200 : dashkin_cfg |= 0b111 << 5; break;
  default:
    return -ENOSYS;
  }

  dashkin_cfg |= 8 << 8;
  dashkin_cfg |= 8 << 20;

  struct dashkin_uart_data *data = dev->data;
  data->uart_cfg = *cfg;
  dashkin_uart_write_data(dev, dashkin_cfg, DASHKIN_UART_CFG_OFFSET); 

  return 0; 
}

int dashkin_uart_config_get(const struct device *dev, struct uart_config *cfg)
{
  struct dashkin_uart_data *data = dev->data;
  *cfg = data->uart_cfg;

  return 0;
}

int dashkin_uart_line_ctrl_set(const struct device *dev, uint32_t ctrl, uint32_t val)
{
  return -ENOSYS;
}

int dashkin_uart_line_ctrl_get(const struct device *dev, uint32_t ctrl, uint32_t val)
{
  return -ENOSYS;
}

int dashkin_uart_drv_cmd(const struct device *dev, uint32_t cmd, uint32_t p)
{
  return -ENOSYS;
}

static int dashkin_uart_init(const struct device *dev)
{
  uint32_t dashkin_cfg = 0;

  dashkin_cfg |= 1 << 3; //enable

  dashkin_cfg |= 0x0; //even parity
  dashkin_cfg |= 0x0 << 1; //1 stop bit
  dashkin_cfg |= 0b011 << 5; //9600

  dashkin_cfg &= ~(DASHKIN_UART_CFG_RX_IRQ); //disable rx interrupt
  dashkin_cfg &= ~(DASHKIN_UART_CFG_TX_IRQ); //disable tx interrupt

  dashkin_cfg |= 1 << 8; //FIFO 8
  dashkin_cfg |= 1 << 20; //FIFO 8

  dashkin_uart_write_data(dev, dashkin_cfg, DASHKIN_UART_CFG_OFFSET); 

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
  GET_DTS_CFG(dev)->irq_config_func(dev);
#endif

  return 0;
}

/*
  POLL API
*/

/*
Read a character from the device for input.

This routine checks if the receiver has valid data. When the receiver has valid data, 
it reads a character from the device, stores to the location pointed to by p_char, and 
returns 0 to the calling thread. It returns -1, otherwise. 
This function is a non-blocking call.
*/
static int dashkin_uart_poll_in(const struct device *dev, unsigned char *p_char)
{

  uint32_t data;
  data = dashkin_uart_read_data(dev, DASHKIN_UART_RX_DATA_OFFSET);

  if ((data & DASHKIN_UART_RX_EMPTY) == 0) {
    *p_char = data & 0xFF;
    return 0;
  }

  return -1;
}

int dashkin_uart_poll_in_u16(const struct device *dev, uint16_t *p_u16)
{
  return -ENOSYS;
}

/*
Write a character to the device for output.

This routine checks if the transmitter is full. When the transmitter is not full, it writes 
a character to the data register. It waits and blocks the calling thread, otherwise. 
This function is a blocking call.

To send a character when hardware flow control is enabled, the handshake signal CTS must be asserted.
*/
static void dashkin_uart_poll_out(const struct device *dev, unsigned char out_char)
{
  uint32_t data = -1;
  while ((data & DASHKIN_UART_TX_STAT_FULL) != 0) {
    data = dashkin_uart_read_data(dev, DASHKIN_UART_TX_STAT_OFFSET);
  }

  dashkin_uart_write_data(dev, out_char, DASHKIN_UART_TX_DATA_OFFSET);

  return;
}

void dashkin_uart_poll_out_u16(const struct device *dev, uint16_t out_u16)
{
  return;
}

/*
  INTERRUPT API
*/
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
/*
Fill FIFO with data.

Returns a number of bytes sent.
*/
static int dashkin_uart_fifo_fill(const struct device *dev, const uint8_t *tx_data, int size)
{
  if (size <= 0) {
    return 0;
  }

  uint32_t full = -1;
  for (int i = 0; i < size; i++) {
    full = dashkin_uart_read_data(dev, DASHKIN_UART_TX_STAT_OFFSET) & DASHKIN_UART_TX_STAT_FULL;
    if (full == 1) {
      dashkin_uart_write_data(dev, tx_data[i], DASHKIN_UART_TX_DATA_OFFSET);
    } else {
      return i;
    }
  }

  return 0;
}

static int dashkin_uart_fifo_fill_u16(const struct device *dev, const uint8_t *tx_data, int size)
{
  return -ENOSYS;
}

/*
Read data from FIFO

Returns a number of bytes read.
*/

static int dashkin_uart_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{

  if (size <= 0) {
    return 0;
  }

  uint32_t data = dashkin_uart_read_data(dev, DASHKIN_UART_RX_DATA_OFFSET);
  int i = 0;
  while ((data & DASHKIN_UART_RX_EMPTY) == 0) {
    rx_data[i] = data & DASHKIN_UART_RX_DATA;
    if (i == size-1) {
      return size;
    }
    i++;
    data = dashkin_uart_read_data(dev, DASHKIN_UART_RX_DATA_OFFSET);
  }

  return i;
}

static int dashkin_uart_fifo_read_u16(const struct device *dev, uint8_t *rx_data, const int size)
{
  return -ENOSYS;
}

void dashkin_uart_irq_tx_enable(const struct device *dev)
{
  uint32_t cfg = dashkin_uart_read_data(dev,DASHKIN_UART_CFG_OFFSET);
  cfg = cfg | DASHKIN_UART_CFG_TX_IRQ;
  dashkin_uart_write_data(dev,cfg,DASHKIN_UART_CFG_OFFSET);

  return;
}

void dashkin_uart_irq_tx_disable(const struct device *dev)
{
  uint32_t cfg = dashkin_uart_read_data(dev,DASHKIN_UART_CFG_OFFSET);
  cfg = cfg & (~DASHKIN_UART_CFG_TX_IRQ);
  dashkin_uart_write_data(dev,cfg,DASHKIN_UART_CFG_OFFSET);

  return;
}

static int dashkin_uart_irq_tx_ready(const struct device *dev)
{
  uint32_t cfg = dashkin_uart_read_data(dev,DASHKIN_UART_CFG_OFFSET);
  uint32_t en = (cfg & DASHKIN_UART_CFG_TX_IRQ) != 0;
  uint32_t ready = (dashkin_uart_read_data(dev, DASHKIN_UART_TX_STAT_OFFSET) & 
                    DASHKIN_UART_TX_STAT_FULL) == 0;

  return ready & en;
}

void dashkin_uart_irq_rx_enable(const struct device *dev)
{
  uint32_t cfg = dashkin_uart_read_data(dev,DASHKIN_UART_CFG_OFFSET);
  cfg = cfg | DASHKIN_UART_CFG_RX_IRQ;
  dashkin_uart_write_data(dev,cfg,DASHKIN_UART_CFG_OFFSET);

  return;
}

void dashkin_uart_irq_rx_disable(const struct device *dev)
{
  uint32_t cfg = dashkin_uart_read_data(dev,DASHKIN_UART_CFG_OFFSET);
  cfg = cfg & (~DASHKIN_UART_CFG_RX_IRQ);
  dashkin_uart_write_data(dev,cfg,DASHKIN_UART_CFG_OFFSET);

  return;
}

static int dashkin_uart_irq_tx_complete(const struct device *dev)
{
  uint32_t empty = dashkin_uart_read_data(dev, DASHKIN_UART_TX_STAT_OFFSET) & 
                    DASHKIN_UART_TX_STAT_EMPTY;

  return empty;

}

static int dashkin_uart_irq_rx_ready(const struct device *dev)
{
  uint32_t ready = dashkin_uart_read_data(dev, DASHKIN_UART_RX_STAT_OFFSET) &
                    DASHKIN_UART_RX_STAT_READY;

  return ready;
}

void dashkin_uart_irq_err_enable(const struct device *dev)
{
  return;
}

void dashkin_uart_irq_err_disable(const struct device *dev)
{
  return;
}

int dashkin_uart_irq_is_pending(const struct device *dev)
{
  return -ENOSYS;
}

int dashkin_uart_irq_update(const struct device *dev)
{
  return -ENOSYS;
}

static void dashkin_uart_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb, void *user_data)
{
  struct dashkin_uart_data *data = dev->data;

  data->callback = cb;
  data->callback_data = user_data;

  return;
}

static void dashkin_uart_isr(const struct device *dev)
{
	struct dashkin_uart_data *data = dev->data;
	uart_irq_callback_user_data_t callback = data->callback;

	if (callback) {
		callback(dev, data->callback_data);
	}
}

#endif

static const struct uart_driver_api dashkin_uart_driver_api = {
	.poll_in = dashkin_uart_poll_in,
	.poll_out = dashkin_uart_poll_out,
	.err_check = dashkin_uart_err_check,
	.configure = dashkin_uart_configure,
	.config_get = dashkin_uart_config_get,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = dashkin_uart_fifo_fill,
	.fifo_read = dashkin_uart_fifo_read,
	.irq_tx_enable = dashkin_uart_irq_tx_enable,
	.irq_tx_disable = dashkin_uart_irq_tx_disable,
	.irq_tx_ready = dashkin_uart_irq_tx_ready,
	.irq_rx_enable = dashkin_uart_irq_rx_enable,
	.irq_rx_disable = dashkin_uart_irq_rx_disable,
	.irq_tx_complete = dashkin_uart_irq_tx_complete,
	.irq_rx_ready = dashkin_uart_irq_rx_ready,
	.irq_err_enable = dashkin_uart_irq_err_enable,
	.irq_err_disable = dashkin_uart_irq_err_disable,
	.irq_is_pending = dashkin_uart_irq_is_pending,
	.irq_update = dashkin_uart_irq_update,
	.irq_callback_set = dashkin_uart_irq_callback_set
#endif
};

//If interrupt driven then define irq_config_function
//else do not define anything
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define DASHKIN_UART_CONFIG_FUNC(node_id, n)	\
	static void dashkin_uart_irq_config_func_##n(const struct device *dev) \
	{								\
		IRQ_CONNECT(DT_IRQ_BY_NAME(node_id, tx, irq),		\
			    DT_IRQ_BY_NAME(node_id, tx, priority),	\
			    dashkin_uart_isr,				\
			    DEVICE_DT_GET(node_id), 0);			\
    irq_enable(DT_IRQ_BY_NAME(node_id, tx, irq)); \
									\
		IRQ_CONNECT(DT_IRQ_BY_NAME(node_id, rx, irq),		\
			    DT_IRQ_BY_NAME(node_id, rx, priority),	\
			    dashkin_uart_isr,				\
			    DEVICE_DT_GET(node_id), 0);			\
    irq_enable(DT_IRQ_BY_NAME(node_id, rx, irq)); \
	}\

#define DASHKIN_UART_CONFIG_INIT(node_id, n)				\
	.irq_config_func = dashkin_uart_irq_config_func_##n,		
#else
#define DASHKIN_UART_CONFIG_FUNC(node_id, n)
#define DASHKIN_UART_CONFIG_INIT(node_id, n)
#endif

#define DASHKIN_UART_INIT(node_id, n)					\
	DASHKIN_UART_CONFIG_FUNC(node_id, n)				\
									\
	static struct dashkin_uart_data dashkin_uart_##n##_data = {	\
		.uart_cfg = {						\
			.baudrate = DT_PROP(node_id, current_speed),	\
			.parity = DT_ENUM_IDX_OR(node_id, parity,	\
						 UART_CFG_PARITY_NONE),	\
			.stop_bits = UART_CFG_STOP_BITS_1,		\
			.data_bits = UART_CFG_DATA_BITS_8,		\
      .flow_ctrl = UART_CFG_FLOW_CTRL_NONE \
		},							\
	};								\
									\
	static const struct dashkin_uart_config dashkin_uart_##n##_config = { \
		.base = DT_REG_ADDR(node_id),				\
		DASHKIN_UART_CONFIG_INIT(node_id, n)			\
	};								\
									\
	DEVICE_DT_DEFINE(node_id, &dashkin_uart_init,			\
			 PM_DEVICE_DT_GET(node_id),			\
			 &dashkin_uart_##n##_data,			\
			 &dashkin_uart_##n##_config,			\
			 PRE_KERNEL_1,					\
			 CONFIG_SERIAL_INIT_PRIORITY,			\
			 &dashkin_uart_driver_api)

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(uart0), DT_DRV_COMPAT, okay)
DASHKIN_UART_INIT(DT_NODELABEL(uart0), 0);
#endif
