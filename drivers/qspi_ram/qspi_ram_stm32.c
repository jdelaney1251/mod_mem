#include <zephyr.h>
#include <device.h>
#include <sys/util.h>
#include <soc.h>
//#include <pinmux/pinmux_stm32.h>
#include <drivers/pinctrl.h>
#include <drivers/clock_control/stm32_clock_control.h>
#include <drivers/clock_control.h>

#include <stm32l4xx_hal_qspi.h>
#include <drivers/qspi_ram.h>


#include <logging/log.h>
LOG_MODULE_REGISTER(qspi_ram_stm32, CONFIG_QSPI_RAM_LOG_LEVEL);

#define DT_DRV_COMPAT st_stm32_qspi_ram

#define STM32_QSPI_FIFO_THRESHOLD         1
#define STM32_QSPI_CLOCK_PRESCALER_MAX  255

#define INS_READ						0x03
#define INS_WRITE						0x02
#define INS_ENTER_SDI					0x3B
#define INS_ENTER_SQI					0x38
#define INS_RSTDQI						0xFF
#define INS_RDMR						0x05
#define INS_WRMR						0x01					

#define MODE_REG_BYTE					0x00
#define MODE_REG_PAGE					0x80
#define MODE_REG_SEQ					0x40

#define MODE_SPI						0x00
#define MODE_SDI						0x01
#define MODE_SQI						0x02

typedef void (*irq_config_func_t)(const struct device *dev);

struct qspi_ram_stm32_config {
    QUADSPI_TypeDef *regs;
    struct stm32_pclken pclken;
	irq_config_func_t irq_config;
    uint32_t ram_size;
    uint32_t max_frequency;
	//const struct soc_gpio_pinctrl *pinctrl_list;
	//size_t pinctrl_list_size;
	const struct pinctrl_dev_config *pcfg;
	uint8_t dummy_cycles;
	uint8_t read_dummy_cycles;
};

struct qspi_ram_stm32_data {
    QSPI_HandleTypeDef hqspi;
	struct k_sem sem;
	struct k_sem sync;
	int cmd_status;
	uint8_t access_mode;
};

static void qspi_ram_stm32_isr(const struct device *dev)
{
	struct qspi_ram_stm32_data *dev_data = dev->data;

	HAL_QSPI_IRQHandler(&dev_data->hqspi);
}

__weak HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma)
{
	return HAL_OK;
}


/*
 * Transfer Error callback.
 */
void HAL_QSPI_ErrorCallback(QSPI_HandleTypeDef *hqspi)
{
	struct qspi_ram_stm32_data *dev_data =
		CONTAINER_OF(hqspi, struct qspi_ram_stm32_data, hqspi);

	LOG_DBG("Error");

	dev_data->cmd_status = -EIO;

	k_sem_give(&dev_data->sync);
}

/*
 * Command completed callback.
 */
void HAL_QSPI_CmdCpltCallback(QSPI_HandleTypeDef *hqspi)
{
	struct qspi_ram_stm32_data *dev_data =
		CONTAINER_OF(hqspi, struct qspi_ram_stm32_data, hqspi);

	LOG_DBG("Cmd complete");
	k_sem_give(&dev_data->sync);
}

/*
 * Rx Transfer completed callback.
 */
void HAL_QSPI_RxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
	struct qspi_ram_stm32_data *dev_data =
		CONTAINER_OF(hqspi, struct qspi_ram_stm32_data, hqspi);
	
	LOG_DBG("Rx complete");
	k_sem_give(&dev_data->sync);
}

/*
 * Tx Transfer completed callback.
 */
void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
	struct qspi_ram_stm32_data *dev_data =
		CONTAINER_OF(hqspi, struct qspi_ram_stm32_data, hqspi);

		LOG_DBG("Tx complete");
	k_sem_give(&dev_data->sync);
}

/*
 * Status Match callback.
 */
void HAL_QSPI_StatusMatchCallback(QSPI_HandleTypeDef *hqspi)
{
	struct qspi_ram_stm32_data *dev_data =
		CONTAINER_OF(hqspi, struct qspi_ram_stm32_data, hqspi);

	LOG_DBG("Status match cb");
	k_sem_give(&dev_data->sync);
}

/*
 * Timeout callback.
 */
void HAL_QSPI_TimeOutCallback(QSPI_HandleTypeDef *hqspi)
{
	struct qspi_ram_stm32_data *dev_data =
		CONTAINER_OF(hqspi, struct qspi_ram_stm32_data, hqspi);

	LOG_DBG("Timeout");

	dev_data->cmd_status = -EIO;

	k_sem_give(&dev_data->sync);
}

static int qspi_ram_reset(const struct device *dev)
{
	const struct qspi_ram_stm32_config *dev_cfg = dev->config;
    struct qspi_ram_stm32_data *dev_data = dev->data;
    HAL_StatusTypeDef hal_ret;

	QSPI_CommandTypeDef cmd = {
        .Instruction = INS_RSTDQI,
        .DummyCycles = 0,
        .InstructionMode = QSPI_INSTRUCTION_4_LINES,
    };

    hal_ret = HAL_QSPI_Command_IT(&dev_data->hqspi, &cmd);
	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to send QSPI instruction", hal_ret);
		return -EIO;
	}

	cmd.InstructionMode = QSPI_INSTRUCTION_2_LINES;

	hal_ret = HAL_QSPI_Command_IT(&dev_data->hqspi, &cmd);
	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to send QSPI instruction", hal_ret);
		return -EIO;
	}
}

static int qspi_write(const struct device *dev, QSPI_CommandTypeDef *cmd, uint8_t *data, size_t size)
{
    const struct qspi_ram_stm32_config *dev_cfg = dev->config;
    struct qspi_ram_stm32_data *dev_data = dev->data;
    HAL_StatusTypeDef hal_ret;

    cmd->NbData = size;

	dev_data->cmd_status = 0;

	// ensure the BUSY flag is clear
	HAL_QSPI_Abort_IT(&dev_data->hqspi);

    hal_ret = HAL_QSPI_Command_IT(&dev_data->hqspi, cmd);
	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to send QSPI instruction", hal_ret);
		return -EIO;
	}

    hal_ret = HAL_QSPI_Transmit_IT(&dev_data->hqspi, (uint8_t *)data);

    if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to read data", hal_ret);
		return -EIO;
	}
	LOG_DBG("CCR 0x%x", dev_cfg->regs->CCR);

	k_sem_take(&dev_data->sync, K_FOREVER);

	return dev_data->cmd_status;
}

static int qspi_read(const struct device *dev, QSPI_CommandTypeDef *cmd, uint8_t *data, size_t size)
{
	const struct qspi_ram_stm32_config *dev_cfg = dev->config;
    struct qspi_ram_stm32_data *dev_data = dev->data;
    HAL_StatusTypeDef hal_ret;

    cmd->NbData = size;

	dev_data->cmd_status = 0;

    hal_ret = HAL_QSPI_Command_IT(&dev_data->hqspi, cmd);
	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to send QSPI instruction", hal_ret);
		return -EIO;
	}

    hal_ret = HAL_QSPI_Receive_IT(&dev_data->hqspi, (uint8_t *)data);

    if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to read data", hal_ret);
		return -EIO;
	}
	LOG_DBG("CCR 0x%x", dev_cfg->regs->CCR);

	k_sem_take(&dev_data->sync, K_FOREVER);

	return dev_data->cmd_status;
}

/*
	Internal interface
*/
static int qspi_ram_send_cmd(const struct device *dev, uint8_t cmd_ins)
{	
	const struct qspi_ram_stm32_config *dev_cfg = dev->config;
    struct qspi_ram_stm32_data *dev_data = dev->data;
    HAL_StatusTypeDef hal_ret;

	QSPI_CommandTypeDef cmd = {
        .Instruction = cmd_ins,
        .DummyCycles = 0,
        .InstructionMode = QSPI_INSTRUCTION_1_LINE,
    };

	dev_data->cmd_status = 0;
    hal_ret = HAL_QSPI_Command_IT(&dev_data->hqspi, &cmd);
	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to send QSPI instruction", hal_ret);
		return -EIO;
	}

	LOG_DBG("CCR 0x%x", dev_cfg->regs->CCR);

	// wait on the tx complete or timeout interrupt
	k_sem_take(&dev_data->sync, K_FOREVER);

	return dev_data->cmd_status;
}

static int qspi_ram_set_mode(const struct device *dev, uint8_t mode)
{
	int ret;

	QSPI_CommandTypeDef cmd = {
        .Instruction = INS_WRMR,
        .InstructionMode = QSPI_INSTRUCTION_1_LINE,
        .DataMode = QSPI_DATA_1_LINE,
		.NbData = 1,
    };

	ret = qspi_write(dev, &cmd, &mode, 1);
	return ret;
}

static int qspi_ram_init_mm_mode(const struct device *dev)
{
	const struct qspi_ram_stm32_config *dev_cfg = dev->config;
	struct qspi_ram_stm32_data *dev_data = dev->data;
	int ret;
	QSPI_MemoryMappedTypeDef mmap;

	QSPI_CommandTypeDef cmd = {
        .Instruction = INS_READ,
        .AddressSize = QSPI_ADDRESS_24_BITS,
		.DummyCycles = dev_cfg->read_dummy_cycles,
        .InstructionMode = QSPI_INSTRUCTION_4_LINES,
        .AddressMode = QSPI_ADDRESS_4_LINES,
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
        .DataMode = QSPI_DATA_4_LINES,
		.DdrMode = QSPI_DDR_MODE_DISABLE,
		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD,
    };

	mmap.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;

	ret = HAL_QSPI_MemoryMapped(&dev_data->hqspi, &cmd, &mmap);
	if (ret != HAL_OK)
	{
		LOG_ERR("Could not init memory mapped mode");
	}
	LOG_DBG("hal memmap ret: %d", ret);
	return ret;
}

/*
	Public interface

*/
static int qspi_ram_stm32_set_access_mode(const struct device *dev, uint8_t mode)
{
	struct qspi_ram_stm32_data *dev_data = (struct qspi_ram_stm32_data *)dev->data;
	int ret;

	LOG_DBG("switching access mode");
	if ((dev_data->access_mode == QSPI_RAM_MODE_INDIRECT) && (mode == QSPI_RAM_MODE_MEM_MAPPED))
	{
		LOG_DBG("switching to mem mapped mode");
		ret = qspi_ram_init_mm_mode(dev);

	}
	else if ((dev_data->access_mode == QSPI_RAM_MODE_MEM_MAPPED) && (mode == QSPI_RAM_MODE_INDIRECT))
	{
		// clear busy bit and send any command to get out of mem mapped mode
		HAL_QSPI_Abort_IT(&dev_data->hqspi);
		ret = qspi_ram_send_cmd(dev, INS_ENTER_SQI);
	}
	else if (dev_data->access_mode == mode)
	{
		return 0;
	}
	else
	{
		return -1;
	}
	return ret;
}

static int qspi_ram_stm32_get_access_mode(const struct device *dev)
{
	struct qspi_ram_stm32_data *dev_data = (struct qspi_ram_stm32_data *)dev->data;
	return dev_data->access_mode;
}

static int qspi_ram_stm32_write_mem_addr(const struct device *dev, uint32_t addr, uint8_t data)
{
	int ret; 

	QSPI_CommandTypeDef cmd = {
        .Instruction = INS_WRITE,
        .AddressSize = QSPI_ADDRESS_24_BITS,
		.Address = addr,
        .DummyCycles = 0,
        .InstructionMode = QSPI_INSTRUCTION_4_LINES,
        .AddressMode = QSPI_ADDRESS_4_LINES,
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
        .DataMode = QSPI_DATA_4_LINES,
		.NbData = 1,
    };

	ret = qspi_write(dev, &cmd, &data, 1);
    return ret;
}

static int qspi_ram_stm32_read_mem_addr(const struct device *dev, uint32_t addr, uint8_t *data)
{
	const struct qspi_ram_stm32_config *dev_cfg = dev->config;
	int ret;
	
	QSPI_CommandTypeDef cmd = {
        .Instruction = INS_READ,
        .AddressSize = QSPI_ADDRESS_24_BITS,
		.Address = addr,
		.DummyCycles = dev_cfg->read_dummy_cycles,
        .InstructionMode = QSPI_INSTRUCTION_4_LINES,
        .AddressMode = QSPI_ADDRESS_4_LINES,
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
        .DataMode = QSPI_DATA_4_LINES,
		.NbData = 1,
    };	
	
	ret = qspi_read(dev, &cmd, data, 1);
    return ret;
}

static int qspi_ram_stm32_write_mem_range(const struct device *dev, uint32_t addr, uint8_t *data, uint32_t data_len)
{
    int ret; 

	QSPI_CommandTypeDef cmd = {
        .Instruction = INS_WRITE,
        .AddressSize = QSPI_ADDRESS_24_BITS,
		.Address = addr,
        .DummyCycles = 0,
        .InstructionMode = QSPI_INSTRUCTION_4_LINES,
        .AddressMode = QSPI_ADDRESS_4_LINES,
		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
        .DataMode = QSPI_DATA_4_LINES,
		.NbData = data_len,
    };

	ret = qspi_write(dev, &cmd, data, data_len);
    return ret;
}

static int qspi_ram_stm32_read_mem_range(const struct device *dev)
{
    return 0;
}

static const struct qspi_ram_driver_api qspi_ram_stm32_driver_api = {
	.set_access_mode = qspi_ram_stm32_set_access_mode,
	.get_access_mode = qspi_ram_stm32_get_access_mode,
    .write_mem_addr = qspi_ram_stm32_write_mem_addr,
    .read_mem_addr = qspi_ram_stm32_read_mem_addr,
    .write_mem_range = qspi_ram_stm32_write_mem_range,
    .read_mem_range = qspi_ram_stm32_read_mem_range,
};


static int qspi_ram_stm32_init(const struct device *dev)
{
    const struct qspi_ram_stm32_config *dev_cfg = dev->config;
    struct qspi_ram_stm32_data *dev_data = dev->data;
    uint32_t ahb_clock_freq;
    uint32_t prescaler = 0;
    int ret;

    //ret = stm32_dt_pinctrl_configure(dev_cfg->pinctrl_list, dev_cfg->pinctrl_list_size, (uint32_t)dev_cfg->regs);
	ret = pinctrl_apply_state(dev_cfg->pcfg, PINCTRL_STATE_DEFAULT); 
    if (ret < 0)
    {
        LOG_ERR("QSPI pinctrl setup failed (%d)", ret);
        return ret;
    }

    /* Clock configuration */
	if (clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
			     (clock_control_subsys_t) &dev_cfg->pclken) != 0) {
		LOG_DBG("Could not enable QSPI clock");
		return -EIO;
	}

	if (clock_control_get_rate(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
			(clock_control_subsys_t) &dev_cfg->pclken,
			&ahb_clock_freq) < 0) {
		LOG_DBG("Failed to get AHB clock frequency");
		return -EIO;
	}

	for (; prescaler <= STM32_QSPI_CLOCK_PRESCALER_MAX; prescaler++) {
		uint32_t clk = ahb_clock_freq / (prescaler + 1);

		if (clk <= dev_cfg->max_frequency) {
			break;
		}
	}
	__ASSERT_NO_MSG(prescaler <= STM32_QSPI_CLOCK_PRESCALER_MAX);
	/* Initialize QSPI HAL */
	dev_data->hqspi.Init.ClockPrescaler = prescaler;
	dev_data->hqspi.Init.FlashSize = find_msb_set(dev_cfg->ram_size);
	LOG_DBG("flashsize set to %d", find_msb_set(dev_cfg->ram_size));

	LOG_DBG("prescaler %d", prescaler);

	ret = HAL_QSPI_Init(&dev_data->hqspi);
	LOG_DBG("HAL QSPI init ret: %d", ret);

	k_sem_init(&dev_data->sem, 1, 1);
	k_sem_init(&dev_data->sync, 0, 1);

	dev_cfg->irq_config(dev);

	ret = qspi_ram_reset(dev);

	// put QSPI RAM in sequential address mode
	ret = qspi_ram_set_mode(dev, MODE_REG_SEQ);

	// enter QUAD data mode
	ret = qspi_ram_send_cmd(dev, INS_ENTER_SQI);

	dev_data->access_mode = QSPI_RAM_MODE_INDIRECT;

	LOG_INF("Init completed");
    return 0;
}

static void qspi_ram_stm32_irq_config_func(const struct device *dev);

// static const struct soc_gpio_pinctrl qspi_pins[] =
// 					ST_STM32_DT_PINCTRL(quadspi, 0);


#define STM32_QSPI_NODE DT_PARENT(DT_DRV_INST(0))

PINCTRL_DT_DEFINE(STM32_QSPI_NODE)

static const struct qspi_ram_stm32_config qspi_ram_stm32_cfg = {
	.regs = (QUADSPI_TypeDef *)DT_REG_ADDR(STM32_QSPI_NODE),
	.pclken = {
		.enr = DT_CLOCKS_CELL(STM32_QSPI_NODE, bits),
		.bus = DT_CLOCKS_CELL(STM32_QSPI_NODE, bus)
	},
	.irq_config = qspi_ram_stm32_irq_config_func,
	.ram_size = DT_INST_PROP(0, size) / 8U,
	.max_frequency = DT_INST_PROP(0, qspi_max_frequency),
	//.pinctrl_list = qspi_pins,
	//.pinctrl_list_size = ARRAY_SIZE(qspi_pins),
	.pcfg = PINCTRL_DT_DEV_CONFIG_GET(STM32_QSPI_NODE),
	.dummy_cycles = 0,
	.read_dummy_cycles = DT_INST_PROP(0, data_lines) / 2
};

static struct qspi_ram_stm32_data qspi_ram_stm32_dev_data = {
	.hqspi = {
		.Instance = (QUADSPI_TypeDef *)DT_REG_ADDR(STM32_QSPI_NODE),
		.Init = {
			.FifoThreshold = STM32_QSPI_FIFO_THRESHOLD,
			.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE,
			.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE,
			.ClockMode = QSPI_CLOCK_MODE_0,
			},
	},
};

DEVICE_DT_INST_DEFINE(0, &qspi_ram_stm32_init, NULL,
		      &qspi_ram_stm32_dev_data, &qspi_ram_stm32_cfg,
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		      &qspi_ram_stm32_driver_api);

static void qspi_ram_stm32_irq_config_func(const struct device *dev)
{
	IRQ_CONNECT(DT_IRQN(STM32_QSPI_NODE), DT_IRQ(STM32_QSPI_NODE, priority),
		    qspi_ram_stm32_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_IRQN(STM32_QSPI_NODE));
}

// #include <zephyr.h>
// #include <device.h>
// #include <sys/util.h>
// #include <soc.h>
// #include <pinmux/pinmux_stm32.h>
// #include <drivers/clock_control/stm32_clock_control.h>
// #include <drivers/clock_control.h>

// #include <stm32l4xx_hal_qspi.h>
// #include <drivers/qspi_ram.h>


// #include <logging/log.h>
// LOG_MODULE_REGISTER(qspi_ram_stm32, CONFIG_QSPI_RAM_LOG_LEVEL);

// #define DT_DRV_COMPAT st_stm32_qspi_ram

// #define STM32_QSPI_FIFO_THRESHOLD         1
// #define STM32_QSPI_CLOCK_PRESCALER_MAX  255

// #define INS_READ						0x03
// #define INS_WRITE						0x02
// #define INS_ENTER_SDI					0x3B
// #define INS_ENTER_SQI					0x38
// #define INS_RSTDQI						0xFF
// #define INS_RDMR						0x05
// #define INS_WRMR						0x01					

// #define MODE_REG_BYTE					0x00
// #define MODE_REG_PAGE					0x80
// #define MODE_REG_SEQ					0x40

// #define MODE_SPI						0x00
// #define MODE_SDI						0x01
// #define MODE_SQI						0x02

// typedef void (*irq_config_func_t)(const struct device *dev);

// struct qspi_ram_stm32_config {
//     QUADSPI_TypeDef *regs;
//     struct stm32_pclken pclken;
// 	irq_config_func_t irq_config;
//     uint32_t ram_size;
//     uint32_t max_frequency;
// 	const struct soc_gpio_pinctrl *pinctrl_list;
// 	size_t pinctrl_list_size;
// 	uint8_t dummy_cycles;
// 	uint8_t read_dummy_cycles;
// };

// struct qspi_ram_stm32_data {
//     QSPI_HandleTypeDef hqspi;
// 	struct k_sem sem;
// 	struct k_sem sync;
// 	int cmd_status;
// 	uint8_t access_mode;
// };

// static void qspi_ram_stm32_isr(const struct device *dev)
// {
// 	struct qspi_ram_stm32_data *dev_data = dev->data;

// 	HAL_QSPI_IRQHandler(&dev_data->hqspi);
// }

// __weak HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma)
// {
// 	return HAL_OK;
// }


// /*
//  * Transfer Error callback.
//  */
// void HAL_QSPI_ErrorCallback(QSPI_HandleTypeDef *hqspi)
// {
// 	struct qspi_ram_stm32_data *dev_data =
// 		CONTAINER_OF(hqspi, struct qspi_ram_stm32_data, hqspi);

// 	LOG_DBG("Error");

// 	dev_data->cmd_status = -EIO;

// 	k_sem_give(&dev_data->sync);
// }

// /*
//  * Command completed callback.
//  */
// void HAL_QSPI_CmdCpltCallback(QSPI_HandleTypeDef *hqspi)
// {
// 	struct qspi_ram_stm32_data *dev_data =
// 		CONTAINER_OF(hqspi, struct qspi_ram_stm32_data, hqspi);

// 	LOG_DBG("Cmd complete");
// 	k_sem_give(&dev_data->sync);
// }

// /*
//  * Rx Transfer completed callback.
//  */
// void HAL_QSPI_RxCpltCallback(QSPI_HandleTypeDef *hqspi)
// {
// 	struct qspi_ram_stm32_data *dev_data =
// 		CONTAINER_OF(hqspi, struct qspi_ram_stm32_data, hqspi);
	
// 	LOG_DBG("Rx complete");
// 	k_sem_give(&dev_data->sync);
// }

// /*
//  * Tx Transfer completed callback.
//  */
// void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi)
// {
// 	struct qspi_ram_stm32_data *dev_data =
// 		CONTAINER_OF(hqspi, struct qspi_ram_stm32_data, hqspi);

// 		LOG_DBG("Tx complete");
// 	k_sem_give(&dev_data->sync);
// }

// /*
//  * Status Match callback.
//  */
// void HAL_QSPI_StatusMatchCallback(QSPI_HandleTypeDef *hqspi)
// {
// 	struct qspi_ram_stm32_data *dev_data =
// 		CONTAINER_OF(hqspi, struct qspi_ram_stm32_data, hqspi);

// 	LOG_DBG("Status match cb");
// 	k_sem_give(&dev_data->sync);
// }

// /*
//  * Timeout callback.
//  */
// void HAL_QSPI_TimeOutCallback(QSPI_HandleTypeDef *hqspi)
// {
// 	struct qspi_ram_stm32_data *dev_data =
// 		CONTAINER_OF(hqspi, struct qspi_ram_stm32_data, hqspi);

// 	LOG_DBG("Timeout");

// 	dev_data->cmd_status = -EIO;

// 	k_sem_give(&dev_data->sync);
// }

// static int qspi_ram_reset(const struct device *dev)
// {
// 	const struct qspi_ram_stm32_config *dev_cfg = dev->config;
//     struct qspi_ram_stm32_data *dev_data = dev->data;
//     HAL_StatusTypeDef hal_ret;

// 	QSPI_CommandTypeDef cmd = {
//         .Instruction = INS_RSTDQI,
//         .DummyCycles = 0,
//         .InstructionMode = QSPI_INSTRUCTION_4_LINES,
//     };

//     hal_ret = HAL_QSPI_Command_IT(&dev_data->hqspi, &cmd);
// 	if (hal_ret != HAL_OK) {
// 		LOG_ERR("%d: Failed to send QSPI instruction", hal_ret);
// 		return -EIO;
// 	}

// 	cmd.InstructionMode = QSPI_INSTRUCTION_2_LINES;

// 	hal_ret = HAL_QSPI_Command_IT(&dev_data->hqspi, &cmd);
// 	if (hal_ret != HAL_OK) {
// 		LOG_ERR("%d: Failed to send QSPI instruction", hal_ret);
// 		return -EIO;
// 	}
// }

// static int qspi_write(const struct device *dev, QSPI_CommandTypeDef *cmd, uint8_t *data, size_t size)
// {
//     const struct qspi_ram_stm32_config *dev_cfg = dev->config;
//     struct qspi_ram_stm32_data *dev_data = dev->data;
//     HAL_StatusTypeDef hal_ret;

//     cmd->NbData = size;

// 	dev_data->cmd_status = 0;

//     hal_ret = HAL_QSPI_Command_IT(&dev_data->hqspi, cmd);
// 	if (hal_ret != HAL_OK) {
// 		LOG_ERR("%d: Failed to send QSPI instruction", hal_ret);
// 		return -EIO;
// 	}

//     hal_ret = HAL_QSPI_Transmit_IT(&dev_data->hqspi, (uint8_t *)data);

//     if (hal_ret != HAL_OK) {
// 		LOG_ERR("%d: Failed to read data", hal_ret);
// 		return -EIO;
// 	}
// 	LOG_DBG("CCR 0x%x", dev_cfg->regs->CCR);

// 	k_sem_take(&dev_data->sync, K_FOREVER);

// 	return dev_data->cmd_status;
// }

// static int qspi_read(const struct device *dev, QSPI_CommandTypeDef *cmd, uint8_t *data, size_t size)
// {
// 	const struct qspi_ram_stm32_config *dev_cfg = dev->config;
//     struct qspi_ram_stm32_data *dev_data = dev->data;
//     HAL_StatusTypeDef hal_ret;

//     cmd->NbData = size;

// 	dev_data->cmd_status = 0;

//     hal_ret = HAL_QSPI_Command_IT(&dev_data->hqspi, cmd);
// 	if (hal_ret != HAL_OK) {
// 		LOG_ERR("%d: Failed to send QSPI instruction", hal_ret);
// 		return -EIO;
// 	}

//     hal_ret = HAL_QSPI_Receive_IT(&dev_data->hqspi, (uint8_t *)data);

//     if (hal_ret != HAL_OK) {
// 		LOG_ERR("%d: Failed to read data", hal_ret);
// 		return -EIO;
// 	}
// 	LOG_DBG("CCR 0x%x", dev_cfg->regs->CCR);

// 	k_sem_take(&dev_data->sync, K_FOREVER);

// 	return dev_data->cmd_status;
// }

// /*
// 	Internal interface
// */
// static int qspi_ram_send_cmd(const struct device *dev, uint8_t cmd_ins)
// {	
// 	const struct qspi_ram_stm32_config *dev_cfg = dev->config;
//     struct qspi_ram_stm32_data *dev_data = dev->data;
//     HAL_StatusTypeDef hal_ret;

// 	QSPI_CommandTypeDef cmd = {
//         .Instruction = cmd_ins,
//         .DummyCycles = 0,
//         .InstructionMode = QSPI_INSTRUCTION_1_LINE,
//     };

// 	dev_data->cmd_status = 0;
//     hal_ret = HAL_QSPI_Command_IT(&dev_data->hqspi, &cmd);
// 	if (hal_ret != HAL_OK) {
// 		LOG_ERR("%d: Failed to send QSPI instruction", hal_ret);
// 		return -EIO;
// 	}

// 	LOG_DBG("CCR 0x%x", dev_cfg->regs->CCR);

// 	// wait on the tx complete or timeout interrupt
// 	k_sem_take(&dev_data->sync, K_FOREVER);

// 	return dev_data->cmd_status;
// }

// static int qspi_ram_set_mode(const struct device *dev, uint8_t mode)
// {
// 	int ret;

// 	QSPI_CommandTypeDef cmd = {
//         .Instruction = INS_WRMR,
//         .InstructionMode = QSPI_INSTRUCTION_1_LINE,
//         .DataMode = QSPI_DATA_1_LINE,
// 		.NbData = 1,
//     };

// 	ret = qspi_write(dev, &cmd, &mode, 1);
// 	return ret;
// }

// static int qspi_ram_init_mm_mode(const struct device *dev)
// {
// 	const struct qspi_ram_stm32_config *dev_cfg = dev->config;
// 	struct qspi_ram_stm32_data *dev_data = dev->data;
// 	int ret;
// 	QSPI_MemoryMappedTypeDef mmap;

// 	QSPI_CommandTypeDef cmd = {
//         .Instruction = INS_READ,
//         .AddressSize = QSPI_ADDRESS_24_BITS,
// 		.DummyCycles = dev_cfg->read_dummy_cycles,
//         .InstructionMode = QSPI_INSTRUCTION_4_LINES,
//         .AddressMode = QSPI_ADDRESS_4_LINES,
// 		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
//         .DataMode = QSPI_DATA_4_LINES,
// 		.DdrMode = QSPI_DDR_MODE_DISABLE,
// 		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD,
//     };

// 	mmap.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;

// 	ret = HAL_QSPI_MemoryMapped(&dev_data->hqspi, &cmd, &mmap);
// 	if (ret != HAL_OK)
// 	{
// 		LOG_ERR("Could not init memory mapped mode");
// 	}
// 	LOG_DBG("hal memmap ret: %d", ret);
// 	return ret;
// }

// /*
// 	Public interface

// */
// static int qspi_ram_stm32_set_access_mode(const struct device *dev, uint8_t mode)
// {
// 	struct qspi_ram_stm32_data *dev_data = dev->data;
// 	int ret;

// 	LOG_DBG("switching access mode");
// 	if ((dev_data->access_mode == QSPI_RAM_MODE_INDIRECT) && (mode == QSPI_RAM_MODE_MEM_MAPPED))
// 	{
// 		LOG_DBG("switching to mem mapped mode");
// 		ret = qspi_ram_init_mm_mode(dev);

// 	}
// 	else if ((dev_data->access_mode == QSPI_RAM_MODE_MEM_MAPPED) && (mode == QSPI_RAM_MODE_INDIRECT))
// 	{
// 		// clear busy bit and send any command to get out of mem mapped mode
// 		HAL_QSPI_Abort_IT(&dev_data->hqspi);
// 		ret = qspi_ram_send_cmd(dev, INS_ENTER_SQI);
// 	}
// 	else
// 	{
// 		return -1;
// 	}
// 	return ret;
// }

// static int qspi_ram_stm32_write_mem_addr(const struct device *dev, uint32_t addr, uint8_t data)
// {
// 	int ret; 

// 	QSPI_CommandTypeDef cmd = {
//         .Instruction = INS_WRITE,
//         .AddressSize = QSPI_ADDRESS_24_BITS,
// 		.Address = addr,
//         .DummyCycles = 0,
//         .InstructionMode = QSPI_INSTRUCTION_4_LINES,
//         .AddressMode = QSPI_ADDRESS_4_LINES,
// 		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
//         .DataMode = QSPI_DATA_4_LINES,
// 		.NbData = 1,
//     };

// 	ret = qspi_write(dev, &cmd, &data, 1);
//     return ret;
// }

// static int qspi_ram_stm32_read_mem_addr(const struct device *dev, uint32_t addr, uint8_t *data)
// {
// 	const struct qspi_ram_stm32_config *dev_cfg = dev->config;
// 	int ret;
	
// 	QSPI_CommandTypeDef cmd = {
//         .Instruction = INS_READ,
//         .AddressSize = QSPI_ADDRESS_24_BITS,
// 		.Address = addr,
// 		.DummyCycles = dev_cfg->read_dummy_cycles,
//         .InstructionMode = QSPI_INSTRUCTION_4_LINES,
//         .AddressMode = QSPI_ADDRESS_4_LINES,
// 		.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
//         .DataMode = QSPI_DATA_4_LINES,
// 		.NbData = 1,
//     };	
	
// 	ret = qspi_read(dev, &cmd, data, 1);
//     return ret;
// }

// static int qspi_ram_stm32_write_mem_range(const struct device *dev)
// {
//     return 0;
// }

// static int qspi_ram_stm32_read_mem_range(const struct device *dev)
// {
//     return 0;
// }

// static const struct qspi_ram_driver_api qspi_ram_stm32_driver_api = {
// 	.set_access_mode = qspi_ram_stm32_set_access_mode,
//     .write_mem_addr = qspi_ram_stm32_write_mem_addr,
//     .read_mem_addr = qspi_ram_stm32_read_mem_addr,
//     .write_mem_range = qspi_ram_stm32_write_mem_range,
//     .read_mem_range = qspi_ram_stm32_read_mem_range,
// };


// static int qspi_ram_stm32_init(const struct device *dev)
// {
//     const struct qspi_ram_stm32_config *dev_cfg = dev->config;
//     struct qspi_ram_stm32_data *dev_data = dev->data;
//     uint32_t ahb_clock_freq;
//     uint32_t prescaler = 0;
//     int ret;

//     ret = stm32_dt_pinctrl_configure(dev_cfg->pinctrl_list, dev_cfg->pinctrl_list_size, (uint32_t)dev_cfg->regs);
//     if (ret < 0)
//     {
//         LOG_ERR("QSPI pinctrl setup failed (%d)", ret);
//         return ret;
//     }

//     /* Clock configuration */
// 	if (clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
// 			     (clock_control_subsys_t) &dev_cfg->pclken) != 0) {
// 		LOG_DBG("Could not enable QSPI clock");
// 		return -EIO;
// 	}

// 	if (clock_control_get_rate(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
// 			(clock_control_subsys_t) &dev_cfg->pclken,
// 			&ahb_clock_freq) < 0) {
// 		LOG_DBG("Failed to get AHB clock frequency");
// 		return -EIO;
// 	}

// 	for (; prescaler <= STM32_QSPI_CLOCK_PRESCALER_MAX; prescaler++) {
// 		uint32_t clk = ahb_clock_freq / (prescaler + 1);

// 		if (clk <= dev_cfg->max_frequency) {
// 			break;
// 		}
// 	}
// 	__ASSERT_NO_MSG(prescaler <= STM32_QSPI_CLOCK_PRESCALER_MAX);
// 	/* Initialize QSPI HAL */
// 	dev_data->hqspi.Init.ClockPrescaler = prescaler;
// 	dev_data->hqspi.Init.FlashSize = find_lsb_set(dev_cfg->ram_size);

// 	LOG_DBG("prescaler %d", prescaler);

// 	ret = HAL_QSPI_Init(&dev_data->hqspi);
// 	LOG_DBG("HAL QSPI init ret: %d", ret);

// 	k_sem_init(&dev_data->sem, 1, 1);
// 	k_sem_init(&dev_data->sync, 0, 1);

// 	dev_cfg->irq_config(dev);

// 	ret = qspi_ram_reset(dev);

// 	// put QSPI RAM in sequential address mode
// 	ret = qspi_ram_set_mode(dev, MODE_REG_SEQ);

// 	// enter QUAD data mode
// 	ret = qspi_ram_send_cmd(dev, INS_ENTER_SQI);

// 	dev_data->access_mode = QSPI_RAM_MODE_INDIRECT;

// 	LOG_INF("Init completed");
//     return 0;
// }

// static void qspi_ram_stm32_irq_config_func(const struct device *dev);

// static const struct soc_gpio_pinctrl qspi_pins[] =
// 					ST_STM32_DT_PINCTRL(quadspi, 0);

// #define STM32_QSPI_NODE DT_PARENT(DT_DRV_INST(0))

// static const struct qspi_ram_stm32_config qspi_ram_stm32_cfg = {
// 	.regs = (QUADSPI_TypeDef *)DT_REG_ADDR(STM32_QSPI_NODE),
// 	.pclken = {
// 		.enr = DT_CLOCKS_CELL(STM32_QSPI_NODE, bits),
// 		.bus = DT_CLOCKS_CELL(STM32_QSPI_NODE, bus)
// 	},
// 	.irq_config = qspi_ram_stm32_irq_config_func,
// 	.ram_size = DT_INST_PROP(0, size) / 8U,
// 	.max_frequency = DT_INST_PROP(0, qspi_max_frequency),
// 	.pinctrl_list = qspi_pins,
// 	.pinctrl_list_size = ARRAY_SIZE(qspi_pins),
// 	.dummy_cycles = 0,
// 	.read_dummy_cycles = DT_INST_PROP(0, data_lines) / 2
// };

// static struct qspi_ram_stm32_data qspi_ram_stm32_dev_data = {
// 	.hqspi = {
// 		.Instance = (QUADSPI_TypeDef *)DT_REG_ADDR(STM32_QSPI_NODE),
// 		.Init = {
// 			.FifoThreshold = STM32_QSPI_FIFO_THRESHOLD,
// 			.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE,
// 			.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE,
// 			.ClockMode = QSPI_CLOCK_MODE_0,
// 			},
// 	},
// };

// DEVICE_DT_INST_DEFINE(0, &qspi_ram_stm32_init, NULL,
// 		      &qspi_ram_stm32_dev_data, &qspi_ram_stm32_cfg,
// 		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
// 		      &qspi_ram_stm32_driver_api);

// static void qspi_ram_stm32_irq_config_func(const struct device *dev)
// {
// 	IRQ_CONNECT(DT_IRQN(STM32_QSPI_NODE), DT_IRQ(STM32_QSPI_NODE, priority),
// 		    qspi_ram_stm32_isr, DEVICE_DT_INST_GET(0), 0);
// 	irq_enable(DT_IRQN(STM32_QSPI_NODE));
// }