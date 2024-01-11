/*
 *  GPIO driver for Dashkin CPU
 */

#define DT_DRV_COMPAT dashkin_gpio

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/gpio/gpio_utils.h>

//Number of ports
#define DASHKIN_GPIO_NUM 8

//Memory offset
#define DASHKIN_GPIO_PORT_CFG_OFFSET 0x00
#define DASHKIN_GPIO_IR_CFG_OFFSET   0x04
#define DASHKIN_GPIO_DATA_OFFSET     0x08
#define DASHKIN_GPIO_STATUS_OFFSET   0x0C

#define DASHKIN_GPIO_DIR_MASK_ALL     0x000000FF
#define DASHKIN_GPIO_DIR_MASK(shift)  0x00000001 << shift

#define DASHKIN_GPIO_EN_MASK_ALL    0x0000FF00
#define DASHKIN_GPIO_EN_MASK(shift) 0x00000100 << shift

#define DASHKIN_GPIO_UP_MASK_ALL    0x00FF0000
#define DASHKIN_GPIO_DN_MASK_ALL    0xFF00FFFF
#define DASHKIN_GPIO_PS_MASK(shift) 0x00010000 << shift

#define DASHKIN_GPIO_IR_EN_ALL          0x00FF0000 << shift
#define DASHKIN_GPIO_IR_EN_MASK(shift)  0x00010000 << shift
#define DASHKIN_GPIO_IR_LO_TRIG(shift)  0x00000003 << shift
#define DASHKIN_GPIO_IR_PO_TRIG(shift)  0x00000002 << shift
#define DASHKIN_GPIO_IR_NE_TRIG(shift)  0x00000001 << shift
#define DASHKIN_GPIO_IR_HI_TRIG(shift)  0x00000000 << shift

struct dashkin_gpio_config
{
  struct gpio_driver_config common;
  mem_addr_t  base;
  uint32_t irq_base;
  void (*irq_config_func)(const struct device *dev);
};

struct dashkin_gpio_data
{
  struct gpio_driver_data common;
  sys_slist_t cb;
};

#define GET_DTS_DATA(dev)\
    ((struct dashkin_gpio_data * const)(dev)->data)
#define GET_DTS_CFG(dev) \
    ((const struct dashkin_gpio_config * const)(dev)->config)
#define GET_GPIO_BASE(dev) \
    (GET_DTS_CFG(dev))->base

static uint32_t dashkin_gpio_read(const struct device *dev, uint32_t offset)
{
  return sys_read32(GET_GPIO_BASE(dev)+offset);
}

static void dashkin_gpio_write(const struct device *dev, uint32_t data, uint32_t offset)
{
  sys_write32(data,GET_GPIO_BASE(dev)+offset);

  return;
}

int dashkin_gpio_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
  uint32_t cfg = dashkin_gpio_read(dev,DASHKIN_GPIO_PORT_CFG_OFFSET);

  if ((flags & GPIO_OUTPUT) && (flags & GPIO_INPUT)) {
    return -ENOTSUP;
  } else if (!(flags & (GPIO_INPUT | GPIO_OUTPUT))) {
    return -ENOTSUP;
  }

  if ((flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) != 0) {
    return -ENOTSUP;
  }

  if ((flags & (GPIO_OPEN_DRAIN | GPIO_OPEN_SOURCE)) != 0) {
    return -ENOTSUP;
  }

  cfg = cfg | (BIT(pin) << 8);
  if ((flags & GPIO_INPUT) != 0) {
    cfg = cfg | BIT(pin);
  } else if ((flags & GPIO_OUTPUT) != 0) {
    cfg = cfg & ~(BIT(pin));
  } else if ((flags & GPIO_OUTPUT_LOW) != 0) {
    cfg = cfg & ~(BIT(pin));
    cfg = cfg & (~(BIT(pin) << 16));
  } else if ((flags & GPIO_OUTPUT_HIGH) != 0) {
    cfg = cfg & ~(BIT(pin));
    cfg = cfg | ((BIT(pin)) << 16);
  } else if ((flags & GPIO_DISCONNECTED) != 0) {
    cfg = cfg & (~(BIT(pin) << 8));
  }

  dashkin_gpio_write(dev, cfg, DASHKIN_GPIO_PORT_CFG_OFFSET);

  return 0;
}

static int dashkin_gpio_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin, enum gpio_int_mode mode, enum gpio_int_trig trig)
{
  uint32_t cfg = dashkin_gpio_read(dev,DASHKIN_GPIO_IR_CFG_OFFSET);

  switch(mode){
    case GPIO_INT_MODE_DISABLED:
      cfg = cfg & (~(BIT(pin) << 16)); 
      break;
    case GPIO_INT_MODE_LEVEL:
      cfg = cfg | (BIT(pin) << 16); //enable
      if (trig == GPIO_INT_TRIG_LOW) {
        cfg = cfg & (~(0x3 << (2*pin))); //set bits to 00
      } else if (trig == GPIO_INT_TRIG_HIGH) {
        cfg = cfg | (0x3 << (2*pin)); //set bits to 11
      } else {
        return -ENOTSUP;
      }
      break;
    case GPIO_INT_MODE_EDGE:
      cfg = cfg | (BIT(pin) << 16); //enable
      if (trig == GPIO_INT_TRIG_LOW) {
        cfg = cfg & (~(0x3 << (2*pin))); //clear bits
        cfg = cfg | ((2) << (2*pin)); //set bits to 10
      } else if (trig == GPIO_INT_TRIG_HIGH) {
        cfg = cfg & (~(0x3 << (2*pin))); //clear bits
        cfg = cfg | ((1) << (2*pin)); //set bits to 01
      } else {
        return -ENOTSUP;
      }
      break;
    default:
      return -ENOTSUP;
  }

  dashkin_gpio_write(dev, cfg, DASHKIN_GPIO_IR_CFG_OFFSET);
  
  return 0;
}

static int dashkin_gpio_init(const struct device *dev)
{

  uint32_t cfg = 0x00000000; //disable all ports
  dashkin_gpio_write(dev, cfg, DASHKIN_GPIO_PORT_CFG_OFFSET);
  cfg = 0x00000000; //disable all interrupts
  dashkin_gpio_write(dev, cfg, DASHKIN_GPIO_IR_CFG_OFFSET);

  GET_DTS_CFG(dev)->irq_config_func(dev);

  return 0;
}

int dashkin_gpio_port_get_direction(const struct device *port, gpio_port_pins_t map, gpio_port_pins_t *inputs, gpio_port_pins_t *outputs)
{
  return -ENOSYS;
}

static inline int dashkin_gpio_pin_is_input(const struct device *dev, gpio_pin_t pin)
{

  uint32_t cfg = dashkin_gpio_read(dev,DASHKIN_GPIO_PORT_CFG_OFFSET);

  if ((cfg & BIT(pin)) != 0) {
    return 1;
  }

  return 0;
}

static inline int dashkin_gpio_pin_is_output(const struct device *dev, gpio_pin_t pin)
{
  return !(dashkin_gpio_pin_is_input(dev,pin));
}

#ifdef CONFIG_GPIO_GET_CONFIG
int dashkin_gpio_pin_get_config(const struct device *dev, gpio_pin_t pin, gpio_flags_t *flags)
{

  uint32_t cfg = dashkin_gpio_read(dev,DASHKIN_GPIO_PORT_CFG_OFFSET);
 
  if ((cfg & (BIT(pin) << 8)) == 0) { //disabled
    flags = flags | GPIO_DISCONNECTED;
  } else if ((cfg & BIT(pin)) != 0) { //input
    flags = flags | GPIO_INPUT; 
  } else if ((cfg & (BIT(pin) << 16)) == 0) {
    flags = flags | GPIO_OUTPUT_LEVEL_LOW; 
  } else if ((cfg & (BIT(pin) << 16)) == 1) {
    flags = flags | GPIO_OUTPUT_LEVEL_HIGH; 
  } else {
    flags = flags | GPIO_OUTPUT; 
  }
 
  return 0;
}
#endif

int dashkin_gpio_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
  
  uint32_t cfg = dashkin_gpio_read(dev,DASHKIN_GPIO_PORT_CFG_OFFSET);
  uint32_t mask = cfg & 0xFF;
  uint32_t data = dashkin_gpio_read(dev,DASHKIN_GPIO_DATA_OFFSET);
  *value = data & mask;

  return 0;
}

int dashkin_gpio_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask, gpio_port_value_t value)
{
  uint32_t data = dashkin_gpio_read(dev,DASHKIN_GPIO_DATA_OFFSET);
  data = (data & ~mask) | (value & mask);
  dashkin_gpio_write(dev, data,DASHKIN_GPIO_DATA_OFFSET);

  return 0;
}

int dashkin_gpio_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
  uint32_t data = dashkin_gpio_read(dev,DASHKIN_GPIO_DATA_OFFSET);
  dashkin_gpio_write(dev, data|pins,DASHKIN_GPIO_DATA_OFFSET);

  return 0;
}

int dashkin_gpio_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
  uint32_t data = dashkin_gpio_read(dev,DASHKIN_GPIO_DATA_OFFSET);
  dashkin_gpio_write(dev, data&(~pins),DASHKIN_GPIO_DATA_OFFSET);

  return 0;
}

int dashkin_gpio_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
  uint32_t data = dashkin_gpio_read(dev,DASHKIN_GPIO_DATA_OFFSET);
  dashkin_gpio_write(dev, data^pins,DASHKIN_GPIO_DATA_OFFSET);

  return 0;
}

static int dashkin_gpio_manage_callback(const struct device *dev, struct gpio_callback *callback, bool set)
{
	struct dashkin_gpio_data *data = GET_DTS_DATA(dev);

	return gpio_manage_callback(&data->cb, callback, set);
}

int dashkin_gpio_isr(const struct device *dev)
{
  struct dashkin_gpio_data *data = GET_DTS_DATA(dev);
  const struct dashkin_gpio_config *cfg = GET_DTS_CFG(dev);
  uint32_t pin = 1 + riscv_plic_get_irq() - (cfg->irq_base>>8);

  uint32_t ir_cfg = dashkin_gpio_read(dev,DASHKIN_GPIO_IR_CFG_OFFSET);
  uint32_t new_ir_cfg = ir_cfg & (~(BIT(pin) << 16));
  dashkin_gpio_write(dev,new_ir_cfg,DASHKIN_GPIO_IR_CFG_OFFSET);
  
  gpio_fire_callbacks(&data->cb,dev,BIT(pin));

  dashkin_gpio_write(dev,ir_cfg,DASHKIN_GPIO_IR_CFG_OFFSET);

  return 0;
}

uint32_t dashkin_gpio_get_pending_int(const struct device *dev)
{
  uint32_t status = dashkin_gpio_read(dev,DASHKIN_GPIO_STATUS_OFFSET);

  return status;
}

static const struct gpio_driver_api dashkin_gpio_driver_api = {
  .pin_configure = dashkin_gpio_pin_configure,
  .pin_interrupt_configure = dashkin_gpio_pin_interrupt_configure,
#ifdef CONFIG_GPIO_GET_DIRECTION
  .port_get_direction = dashkin_gpio_port_get_direction,
#endif
#ifdef CONFIG_GPIO_GET_CONFIG
  .pin_get_config = dashkin_gpio_pin_get_config,
#endif
  .port_get_raw = dashkin_gpio_port_get_raw,
  .port_set_masked_raw = dashkin_gpio_port_set_masked_raw,
  .port_set_bits_raw = dashkin_gpio_port_set_bits_raw,
  .port_clear_bits_raw = dashkin_gpio_port_clear_bits_raw,
  .manage_callback = dashkin_gpio_manage_callback,
  .get_pending_int = dashkin_gpio_get_pending_int,
};

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(gpio0), DT_DRV_COMPAT, okay)
static void dashkin_gpio_irq_config_func_0(const struct device *dev);
#endif

#define DASHKIN_GPIO_INIT(node_id, n)                         \
  static struct dashkin_gpio_data dashkin_gpio_##n_data = {   \
  };                                                          \
\
  static const struct dashkin_gpio_config dashkin_gpio_##n_config = {   \
    .base = DT_REG_ADDR(node_id),                                        \
    .irq_base = DT_INST_IRQN(n),\
    .irq_config_func = dashkin_gpio_irq_config_func_##n,                 \
  };                                                                    \
                                                                        \
  DEVICE_DT_INST_DEFINE(n,                                              \
      &dashkin_gpio_init,                                               \
      NULL,                                                             \
      &dashkin_gpio_##n_data,                                           \
      &dashkin_gpio_##n_config,                                         \
      PRE_KERNEL_1,                                                     \
      CONFIG_GPIO_INIT_PRIORITY,                                        \
      &dashkin_gpio_driver_api                                          \
  );                                                                    \


#define DASHKIN_GPIO_IRQ_CONNECT(n, idx) \
  IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, idx, irq), \
      DT_INST_IRQ_BY_IDX(n, idx, priority),    \
      dashkin_gpio_isr,                 \
      DEVICE_DT_INST_GET(n),                 \
      0);                                    \
  irq_enable(DT_INST_IRQ_BY_IDX(n, idx, irq));  \


#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(gpio0), DT_DRV_COMPAT, okay)
DASHKIN_GPIO_INIT(DT_NODELABEL(gpio0),0);

static void dashkin_gpio_irq_config_func_0(const struct device *dev)
{
#if DT_INST_IRQ_HAS_IDX(0,0)
	DASHKIN_GPIO_IRQ_CONNECT(0,0);
#endif
#if DT_INST_IRQ_HAS_IDX(0,1)
	DASHKIN_GPIO_IRQ_CONNECT(0,1);
#endif
#if DT_INST_IRQ_HAS_IDX(0,2)
	DASHKIN_GPIO_IRQ_CONNECT(0,2);
#endif
#if DT_INST_IRQ_HAS_IDX(0,3)
	DASHKIN_GPIO_IRQ_CONNECT(0,3);
#endif
#if DT_INST_IRQ_HAS_IDX(0,4)
	DASHKIN_GPIO_IRQ_CONNECT(0,4);
#endif
#if DT_INST_IRQ_HAS_IDX(0,5)
	DASHKIN_GPIO_IRQ_CONNECT(0,5);
#endif
#if DT_INST_IRQ_HAS_IDX(0,6)
	DASHKIN_GPIO_IRQ_CONNECT(0,6);
#endif
#if DT_INST_IRQ_HAS_IDX(0,7)
	DASHKIN_GPIO_IRQ_CONNECT(0,7);
#endif
}
#endif
