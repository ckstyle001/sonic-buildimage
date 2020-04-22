#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c/pca954x.h>
#include <linux/i2c-mux.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon.h>
#include <linux/err.h>
#include <linux/ipmi.h>
#include <linux/ipmi_smi.h>
#include <linux/ipmi_msgdefs.h>

#define SYSTEM_CPLD_ADDR    0x30
#define PORT_CPLD0_ADDR     0x32
#define PORT_CPLD1_ADDR     0x32
#define DEFAULT_CPLD        0

#define DEF_DEV_NUM      1

#define PORT_SFP_DEV_NUM                2
#define PORT_SFP_BASE_NUM               41
#define PORT_QSFP_DEV_NUM               32
#define PORT_QSFP_BASE_NUM              51
#define PORT_QSFP_GROUP1_BASE_NUM       51
#define PORT_QSFP_GROUP2_BASE_NUM       59
#define PORT_QSFP_GROUP3_BASE_NUM       67
#define PORT_QSFP_GROUP4_BASE_NUM       75
#define PORT_QSFP_GROUP1_DEV_NUM        8
#define PORT_QSFP_GROUP2_DEV_NUM        8
#define PORT_QSFP_GROUP3_DEV_NUM        8
#define PORT_QSFP_GROUP4_DEV_NUM        8
#define PORT_QSFP_GROUP1_MUX_SEL_REG    0x0c
#define PORT_QSFP_GROUP2_MUX_SEL_REG    0x0d
#define PORT_QSFP_GROUP3_MUX_SEL_REG    0x0c
#define PORT_QSFP_GROUP4_MUX_SEL_REG    0x0d

/* BMC IMPI CMD */
#define IPMI_MAX_INTF (4)

#define BMC_I2C_BUS0    ((0x00 << 1) | 0x01)
#define BMC_I2C_BUS1    ((0x01 << 1) | 0x01)
#define BMC_I2C_BUS2    ((0x02 << 1) | 0x01)
#define BMC_I2C_BUS3    ((0x03 << 1) | 0x01)
#define BMC_I2C_BUS4    ((0x04 << 1) | 0x01)
#define BMC_I2C_BUS5    ((0x05 << 1) | 0x01)
#define BMC_I2C_BUS6    ((0x06 << 1) | 0x01)
#define BMC_I2C_BUS7    ((0x07 << 1) | 0x01)

#define PCA9548_0_MUX_ADDR  (0x71 << 1)
#define PCA9546_0_MUX_ADDR  (0x72 << 1)

#define PCA9548_0_MUX_REG   0x00
#define PCA9546_0_MUX_REG   0x00

#define PCA954X_MUX_OFF     0x00
#define PCA954X_MUX_CH0     (0x01 << 0)
#define PCA954X_MUX_CH1     (0x01 << 1)
#define PCA954X_MUX_CH2     (0x01 << 2)
#define PCA954X_MUX_CH3     (0x01 << 3)
#define PCA954X_MUX_CH4     (0x01 << 4)
#define PCA954X_MUX_CH5     (0x01 << 5)
#define PCA954X_MUX_CH6     (0x01 << 6)
#define PCA954X_MUX_CH7     (0x01 << 7)

/* Check cpld read results */
#define VALIDATED_READ(_buf, _rv, _read, _invert)   \
    do {                                            \
        _rv = _read;                                \
        if (_rv < 0) {                              \
            return sprintf(_buf, "READ ERROR\n");   \
        }                                           \
        if (_invert) {                              \
            _rv = ~_rv;                             \
        }                                           \
        _rv &= 0xFF;                                \
    } while(0)                                      \

enum bus_index {
    BUS0 = 0,
};

struct mutex dni_lock;

extern int dni_ipmi_create_user(void);
extern int dni_ipmi_i2c_read_byte(uint8_t dev_bus, uint8_t dev_addr, uint8_t dev_reg);
extern int dni_ipmi_i2c_write_byte(uint8_t dev_bus, uint8_t dev_addr, uint8_t dev_reg, uint8_t write_data);
extern unsigned char dni_log2(unsigned char num);
extern void device_release(struct device *dev);
extern void msg_handler(struct ipmi_recv_msg *recv_msg,void* handler_data);
extern void dummy_smi_free(struct ipmi_smi_msg *msg);
extern void dummy_recv_free(struct ipmi_recv_msg *msg);

static atomic_t dummy_count = ATOMIC_INIT(0);
static ipmi_user_t ipmi_mh_user = NULL;
static struct ipmi_user_hndl ipmi_hndlrs = {
    .ipmi_recv_hndl = msg_handler
};

static struct ipmi_smi_msg halt_smi_msg = {
    .done = dummy_smi_free
};

static struct ipmi_recv_msg halt_recv_msg = {
    .done = dummy_recv_free
};

void device_release(struct device *dev)
{
    return;
}
EXPORT_SYMBOL(device_release);

void msg_handler(struct ipmi_recv_msg *recv_msg, void* handler_data)
{
    struct completion *comp = recv_msg->user_msg_data;
    if (comp)
         complete(comp);
    else
        ipmi_free_recv_msg(recv_msg);
    return;
}
EXPORT_SYMBOL(msg_handler);

void dummy_smi_free(struct ipmi_smi_msg *msg)
{
    atomic_dec(&dummy_count);
}
EXPORT_SYMBOL(dummy_smi_free);

void dummy_recv_free(struct ipmi_recv_msg *msg)
{
    atomic_dec(&dummy_count);
}
EXPORT_SYMBOL(dummy_recv_free);

unsigned char dni_log2 (unsigned char num)
{
    unsigned char num_log2 = 0;
    while(num > 0){
        num = num >> 1;
        num_log2 += 1;
    }
    return num_log2 -1;
}
EXPORT_SYMBOL(dni_log2);


/* ---------------- IPMI - start ------------- */
int dni_ipmi_create_user(void)
{
    int rv, i;

    for (i = 0, rv = 1; i < IPMI_MAX_INTF && rv; i++)
    {
        rv = ipmi_create_user(i, &ipmi_hndlrs, NULL, &ipmi_mh_user);
    }
    if (rv == 0)
        printk(KERN_INFO "Enable IPMI protocol.\n");

    return rv;
}
EXPORT_SYMBOL(dni_ipmi_create_user);

int dni_ipmi_i2c_read_byte(uint8_t dev_bus, uint8_t dev_addr, uint8_t dev_reg)
{
    int rv;
    struct ipmi_system_interface_addr addr;
    struct kernel_ipmi_msg msg;
    struct completion comp;
    uint8_t cmd_data[4] = {0};

    cmd_data[0] = dev_bus;
    cmd_data[1] = dev_addr;
    cmd_data[2] = 0x00;
    cmd_data[3] = dev_reg;

    addr.addr_type = IPMI_SYSTEM_INTERFACE_ADDR_TYPE;
    addr.channel = IPMI_BMC_CHANNEL;
    addr.lun = 0;

    msg.netfn = IPMI_NETFN_APP_REQUEST;
    msg.cmd = 0x52;
    msg.data_len = sizeof(cmd_data);;
    msg.data = cmd_data;

    init_completion(&comp);
    rv = ipmi_request_supply_msgs(ipmi_mh_user, (struct ipmi_addr*)&addr, 0, &msg, &comp, &halt_smi_msg, &halt_recv_msg, 0);
    wait_for_completion(&comp);
    if(rv)
    {
        printk(KERN_ERR "IPMI i2c read byte command Error!\n");
        ipmi_free_recv_msg(&halt_recv_msg);
        return rv;
    }

    return halt_recv_msg.msg.data[1];
}
EXPORT_SYMBOL(dni_ipmi_i2c_read_byte);

int dni_ipmi_i2c_write_byte(uint8_t dev_bus, uint8_t dev_addr, uint8_t dev_reg, uint8_t write_data)
{
    int rv;
    struct ipmi_system_interface_addr addr;
    struct kernel_ipmi_msg msg;
    struct completion comp;
    uint8_t cmd_data[5] = {0};

    cmd_data[0] = dev_bus;
    cmd_data[1] = dev_addr;
    cmd_data[2] = 0x00;
    cmd_data[3] = dev_reg;
    cmd_data[4] = write_data;

    addr.addr_type = IPMI_SYSTEM_INTERFACE_ADDR_TYPE;
    addr.channel = IPMI_BMC_CHANNEL;
    addr.lun = 0;

    msg.netfn = IPMI_NETFN_APP_REQUEST;
    msg.cmd = 0x52;
    msg.data_len = sizeof(cmd_data);;
    msg.data = cmd_data;

    init_completion(&comp);
    rv = ipmi_request_supply_msgs(ipmi_mh_user, (struct ipmi_addr*)&addr, 0, &msg, &comp, &halt_smi_msg, &halt_recv_msg, 0);
    wait_for_completion(&comp);
    if(rv)
    {
        printk(KERN_ERR "IPMI i2c write byte command Error!\n");
        ipmi_free_recv_msg(&halt_recv_msg);
        return rv;
    }

    return rv;
}
EXPORT_SYMBOL(dni_ipmi_i2c_write_byte);

/* ---------------- IPMI - stop ------------- */

/* ---------------- I2C device - start ------------- */
struct i2c_device_platform_data {
    int parent;
    struct i2c_board_info info;
    struct i2c_client *client;
};

static struct i2c_device_platform_data evsa_32q56_i2c_device_platform_data[] = {
    {
        // id eeprom (0x56)
        .parent = 0,
        .info = { I2C_BOARD_INFO("24c128-delta", 0x56) },
        .client = NULL,
    },
    {
        // sfp 1 (0x50)
        .parent = 41,
        .info = { .type = "optoe2", .addr = 0x50 },
        .client = NULL,
    },
    {
        // sfp 2 (0x50)
        .parent = 42,
        .info = { .type = "optoe2", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 1 (0x50)
        .parent = 51,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 2 (0x50)
        .parent = 52,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 3 (0x50)
        .parent = 53,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 4 (0x50)
        .parent = 54,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 5 (0x50)
        .parent = 55,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 6 (0x50)
        .parent = 56,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 7 (0x50)
        .parent = 57,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 8 (0x50)
        .parent = 58,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 9 (0x50)
        .parent = 59,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 10 (0x50)
        .parent = 60,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 11 (0x50)
        .parent = 61,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 12 (0x50)
        .parent = 62,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 13 (0x50)
        .parent = 63,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 14 (0x50)
        .parent = 64,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 15 (0x50)
        .parent = 65,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 16 (0x50)
        .parent = 66,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 17 (0x50)
        .parent = 67,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 18 (0x50)
        .parent = 68,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 19 (0x50)
        .parent = 69,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 20 (0x50)
        .parent = 70,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 21 (0x50)
        .parent = 71,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 22 (0x50)
        .parent = 72,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 23 (0x50)
        .parent = 73,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 24 (0x50)
        .parent = 74,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 25 (0x50)
        .parent = 75,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 26 (0x50)
        .parent = 76,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 27 (0x50)
        .parent = 77,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 28 (0x50)
        .parent = 78,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 29 (0x50)
        .parent = 79,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 30 (0x50)
        .parent = 80,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 31 (0x50)
        .parent = 81,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
    {
        // qsfp 32 (0x50)
        .parent = 82,
        .info = { .type = "optoe1", .addr = 0x50 },
        .client = NULL,
    },
};

#define evsa_32q56_i2c_device_num(NUM){    \
    .name = "delta-evsa-32q56-i2c-device", \
    .id   = NUM,                           \
    .dev  = {                              \
        .platform_data = &evsa_32q56_i2c_device_platform_data[NUM], \
        .release       = device_release,   \
    },                                     \
}

static struct platform_device evsa_32q56_i2c_device[] = {
    evsa_32q56_i2c_device_num(0),
    evsa_32q56_i2c_device_num(1),
    evsa_32q56_i2c_device_num(2),
    evsa_32q56_i2c_device_num(3),
    evsa_32q56_i2c_device_num(4),
    evsa_32q56_i2c_device_num(5),
    evsa_32q56_i2c_device_num(6),
    evsa_32q56_i2c_device_num(7),
    evsa_32q56_i2c_device_num(8),
    evsa_32q56_i2c_device_num(9),
    evsa_32q56_i2c_device_num(10),
    evsa_32q56_i2c_device_num(11),
    evsa_32q56_i2c_device_num(12),
    evsa_32q56_i2c_device_num(13),
    evsa_32q56_i2c_device_num(14),
    evsa_32q56_i2c_device_num(15),
    evsa_32q56_i2c_device_num(16),
    evsa_32q56_i2c_device_num(17),
    evsa_32q56_i2c_device_num(18),
    evsa_32q56_i2c_device_num(19),
    evsa_32q56_i2c_device_num(20),
    evsa_32q56_i2c_device_num(21),
    evsa_32q56_i2c_device_num(22),
    evsa_32q56_i2c_device_num(23),
    evsa_32q56_i2c_device_num(24),
    evsa_32q56_i2c_device_num(25),
    evsa_32q56_i2c_device_num(26),
    evsa_32q56_i2c_device_num(27),
    evsa_32q56_i2c_device_num(28),
    evsa_32q56_i2c_device_num(29),
    evsa_32q56_i2c_device_num(30),
    evsa_32q56_i2c_device_num(31),
    evsa_32q56_i2c_device_num(32),
    evsa_32q56_i2c_device_num(33),
    evsa_32q56_i2c_device_num(34),
};
/* ---------------- I2C device - end ------------- */

/* ---------------- I2C driver - start ------------- */
static int __init i2c_device_probe(struct platform_device *pdev)
{
    struct i2c_device_platform_data *pdata;
    struct i2c_adapter *parent;

    pdata = pdev->dev.platform_data;
    if (!pdata) {
        dev_err(&pdev->dev, "Missing platform data\n");
        return -ENODEV;
    }

    parent = i2c_get_adapter(pdata->parent);
    if (!parent) {
        dev_err(&pdev->dev, "Parent adapter (%d) not found\n",
            pdata->parent);
        return -ENODEV;
    }

    pdata->client = i2c_new_device(parent, &pdata->info);
    if (!pdata->client) {
        dev_err(&pdev->dev, "Failed to create i2c client %s at %d\n",
            pdata->info.type, pdata->parent);
        return -ENODEV;
    }

    return 0;
}

static int __exit i2c_deivce_remove(struct platform_device *pdev)
{
    struct i2c_adapter *parent;
    struct i2c_device_platform_data *pdata;

    pdata = pdev->dev.platform_data;
    if (!pdata) {
        dev_err(&pdev->dev, "Missing platform data\n");
        return -ENODEV;
    }

    if (pdata->client) {
        parent = (pdata->client)->adapter;
        i2c_unregister_device(pdata->client);
        i2c_put_adapter(parent);
    }

    return 0;
}
static struct platform_driver i2c_device_driver = {
    .probe  = i2c_device_probe,
    .remove = __exit_p(i2c_deivce_remove),
    .driver = {
        .owner = THIS_MODULE,
        .name  = "delta-evsa-32q56-i2c-device",
    }
};
/* ---------------- I2C driver - end ------------- */

/* ---------------- CPLD - start ------------- */

/* ---------------- CPLD - end ------------- */

/* ---------------- MUX - start ------------- */
struct virtual_mux_platform_data {
    int                 parent;
    int                 base_nr;
    int                 reg_addr;
    struct i2c_client   *cpld;
};

struct virtual_mux {
    struct i2c_adapter                  *parent;
    struct i2c_adapter                  **child;
    struct virtual_mux_platform_data    data;
};

static struct virtual_mux_platform_data evsa_32q56_port_mux_platform_data[] = {
    {
        .parent         = BUS0,
        .base_nr        = PORT_SFP_BASE_NUM,
        .reg_addr       = 0,
        .cpld           = NULL,
    },
    {
        .parent         = BUS0,
        .base_nr        = PORT_QSFP_GROUP1_BASE_NUM,
        .reg_addr       = PORT_QSFP_GROUP1_MUX_SEL_REG,
        .cpld           = NULL,
    },
    {
        .parent         = BUS0,
        .base_nr        = PORT_QSFP_GROUP2_BASE_NUM,
        .reg_addr       = PORT_QSFP_GROUP2_MUX_SEL_REG,
        .cpld           = NULL,
    },
    {
        .parent         = BUS0,
        .base_nr        = PORT_QSFP_GROUP3_BASE_NUM,
        .reg_addr       = PORT_QSFP_GROUP3_MUX_SEL_REG,
        .cpld           = NULL,
    },
    {
        .parent         = BUS0,
        .base_nr        = PORT_QSFP_GROUP4_BASE_NUM,
        .reg_addr       = PORT_QSFP_GROUP4_MUX_SEL_REG,
        .cpld           = NULL,
    },
};

static struct platform_device port_mux_device[] =
{
    {
        .name = "delta-evsa-32q56-port-mux",
        .id   = 0,
        .dev  = {
            .platform_data  = &evsa_32q56_port_mux_platform_data[0],
            .release        = device_release,
        },
    },
    {
        .name = "delta-evsa-32q56-port-mux",
        .id   = 1,
        .dev  = {
            .platform_data  = &evsa_32q56_port_mux_platform_data[1],
            .release        = device_release,
        },
    },
    {
        .name = "delta-evsa-32q56-port-mux",
        .id   = 2,
        .dev  = {
            .platform_data  = &evsa_32q56_port_mux_platform_data[2],
            .release        = device_release,
        },
    },
    {
        .name = "delta-evsa-32q56-port-mux",
        .id   = 3,
        .dev  = {
            .platform_data  = &evsa_32q56_port_mux_platform_data[3],
            .release        = device_release,
        },
    },
    {
        .name = "delta-evsa-32q56-port-mux",
        .id   = 4,
        .dev  = {
            .platform_data  = &evsa_32q56_port_mux_platform_data[4],
            .release        = device_release,
        },
    },
};

static int port_mux_select(struct i2c_mux_core *muxc, u32 chan)
{
    int ret;
    struct virtual_mux  *mux;

    mux = i2c_mux_priv(muxc);
    if (mux->data.base_nr == PORT_SFP_BASE_NUM)
    {
        dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, PCA9548_0_MUX_ADDR, PCA9548_0_MUX_REG, PCA954X_MUX_OFF);
        dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, PCA9546_0_MUX_ADDR, PCA9546_0_MUX_REG, (chan + 1));
    }
    else if (mux->data.base_nr == PORT_QSFP_GROUP1_BASE_NUM)
    {
        dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, PCA9548_0_MUX_ADDR, PCA9548_0_MUX_REG, PCA954X_MUX_CH0);
        dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, (PORT_CPLD0_ADDR << 1), PORT_QSFP_GROUP1_MUX_SEL_REG, ~(0x01 << chan));
        if (chan / 4)
        {
            dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, PCA9548_0_MUX_ADDR, PCA9548_0_MUX_REG, PCA954X_MUX_CH1);
        }
        else
        {
            dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, PCA9548_0_MUX_ADDR, PCA9548_0_MUX_REG, PCA954X_MUX_CH0);
        }
    }
    else if (mux->data.base_nr == PORT_QSFP_GROUP2_BASE_NUM)
    {
        dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, PCA9548_0_MUX_ADDR, PCA9548_0_MUX_REG, PCA954X_MUX_CH0);
        dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, (PORT_CPLD0_ADDR << 1), PORT_QSFP_GROUP2_MUX_SEL_REG, ~(0x01 << chan));
        if (chan / 4)
        {
            dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, PCA9548_0_MUX_ADDR, PCA9548_0_MUX_REG, PCA954X_MUX_CH3);
        }
        else
        {
            dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, PCA9548_0_MUX_ADDR, PCA9548_0_MUX_REG, PCA954X_MUX_CH2);
        }
    }
    else if (mux->data.base_nr == PORT_QSFP_GROUP3_BASE_NUM)
    {
        dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, PCA9548_0_MUX_ADDR, PCA9548_0_MUX_REG, PCA954X_MUX_CH4);
        dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, (PORT_CPLD1_ADDR << 1), PORT_QSFP_GROUP3_MUX_SEL_REG, ~(0x01 << chan));
        if (chan / 4)
        {
            dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, PCA9548_0_MUX_ADDR, PCA9548_0_MUX_REG, PCA954X_MUX_CH5);
        }
        else
        {
            dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, PCA9548_0_MUX_ADDR, PCA9548_0_MUX_REG, PCA954X_MUX_CH4);
        }
    }
    else if (mux->data.base_nr == PORT_QSFP_GROUP4_BASE_NUM)
    {
        dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, PCA9548_0_MUX_ADDR, PCA9548_0_MUX_REG, PCA954X_MUX_CH4);
        dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, (PORT_CPLD1_ADDR << 1), PORT_QSFP_GROUP4_MUX_SEL_REG, ~(0x01 << chan));
        if (chan / 4)
        {
            dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, PCA9548_0_MUX_ADDR, PCA9548_0_MUX_REG, PCA954X_MUX_CH7);
        }
        else
        {
            dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, PCA9548_0_MUX_ADDR, PCA9548_0_MUX_REG, PCA954X_MUX_CH6);
        }
    }
    return ret;
}

static int port_mux_deselect(struct i2c_mux_core *muxc, u32 chan)
{
    int ret;
    struct virtual_mux  *mux;

    mux = i2c_mux_priv(muxc);
    if (mux->data.base_nr == PORT_SFP_BASE_NUM)
    {
        dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, PCA9546_0_MUX_ADDR, PCA9546_0_MUX_REG, PCA954X_MUX_OFF);

    }
    else if (mux->data.base_nr == PORT_QSFP_GROUP1_BASE_NUM)
    {
        dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, PCA9548_0_MUX_ADDR, PCA9548_0_MUX_REG, PCA954X_MUX_CH0);
        dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, (PORT_CPLD0_ADDR << 1), PORT_QSFP_GROUP1_MUX_SEL_REG, 0xff);
    }
    else if (mux->data.base_nr == PORT_QSFP_GROUP2_BASE_NUM)
    {
        dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, PCA9548_0_MUX_ADDR, PCA9548_0_MUX_REG, PCA954X_MUX_CH0);
        dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, (PORT_CPLD0_ADDR << 1), PORT_QSFP_GROUP2_MUX_SEL_REG, 0xff);
    }
    else if (mux->data.base_nr == PORT_QSFP_GROUP3_BASE_NUM)
    {
        dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, PCA9548_0_MUX_ADDR, PCA9548_0_MUX_REG, PCA954X_MUX_CH4);
        dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, (PORT_CPLD1_ADDR << 1), PORT_QSFP_GROUP3_MUX_SEL_REG, 0xff);
    }
    else if (mux->data.base_nr == PORT_QSFP_GROUP4_BASE_NUM)
    {
        dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, PCA9548_0_MUX_ADDR, PCA9548_0_MUX_REG, PCA954X_MUX_CH4);
        dni_ipmi_i2c_write_byte(BMC_I2C_BUS3, (PORT_CPLD1_ADDR << 1), PORT_QSFP_GROUP4_MUX_SEL_REG, 0xff);
    }
    return ret;
}

static int __init port_mux_probe(struct platform_device *pdev)
{
    struct i2c_mux_core *muxc;
    struct virtual_mux *mux;
    struct virtual_mux_platform_data *pdata;
    struct i2c_adapter *parent;
    int i, ret, dev_num;

    pdata = pdev->dev.platform_data;
    if (!pdata) {
        dev_err(&pdev->dev, "Port MUX platform data not found\n");
        return -ENODEV;
    }
    mux = kzalloc(sizeof(*mux), GFP_KERNEL);
    if (!mux) {
        printk(KERN_ERR "Failed to allocate memory for mux\n");
        return -ENOMEM;
    }
    mux->data = *pdata;
    parent = i2c_get_adapter(pdata->parent);
    if (!parent) {
        kfree(mux);
        dev_err(&pdev->dev, "Parent adapter (%d) not found\n", pdata->parent);
        return -ENODEV;
    }
    /* Judge bus number to decide how many devices*/
    switch (pdata->base_nr) {
        case PORT_SFP_BASE_NUM:
            dev_num = PORT_SFP_DEV_NUM;
            break;
        case PORT_QSFP_GROUP1_BASE_NUM:
            dev_num = PORT_QSFP_GROUP1_DEV_NUM;
            break;
        case PORT_QSFP_GROUP2_BASE_NUM:
            dev_num = PORT_QSFP_GROUP2_DEV_NUM;
            break;
        case PORT_QSFP_GROUP3_BASE_NUM:
            dev_num = PORT_QSFP_GROUP3_DEV_NUM;
            break;
        case PORT_QSFP_GROUP4_BASE_NUM:
            dev_num = PORT_QSFP_GROUP4_DEV_NUM;
            break;
        default :
            dev_num = DEF_DEV_NUM;
            break;
    }

    muxc = i2c_mux_alloc(parent, &pdev->dev, dev_num, 0, 0, port_mux_select, port_mux_deselect);
    if (!muxc) {
        ret = -ENOMEM;
        goto alloc_failed;
    }
    muxc->priv = mux;
    platform_set_drvdata(pdev, muxc);
    for (i = 0; i < dev_num; i++)
    {
        int nr = pdata->base_nr + i;
        unsigned int class = 0;
        ret = i2c_mux_add_adapter(muxc, nr, i, class);
        if (ret) {
            dev_err(&pdev->dev, "Failed to add adapter %d\n", i);
            goto add_adapter_failed;
        }
    }
    dev_info(&pdev->dev, "%d port mux on %s adapter\n", dev_num, parent->name);
    return 0;

add_adapter_failed:
    i2c_mux_del_adapters(muxc);
alloc_failed:
    kfree(mux);
    i2c_put_adapter(parent);

    return ret;
}

static int __exit port_mux_remove(struct platform_device *pdev)
{
    struct i2c_mux_core *muxc  = platform_get_drvdata(pdev);
    struct i2c_adapter *parent = muxc->parent;

    i2c_mux_del_adapters(muxc);
    i2c_put_adapter(parent);

    return 0;
}

static struct platform_driver port_mux_driver = {
    .probe  = port_mux_probe,
    .remove = __exit_p(port_mux_remove), /* TODO */
    .driver = {
        .owner = THIS_MODULE,
        .name  = "delta-evsa-32q56-port-mux",
    },
};
/* ---------------- MUX - end ------------- */

/* ---------------- module initialization ------------- */
static int __init delta_evsa_32q56_platform_init(void)
{
    struct virtual_mux_platform_data *cpld_mux_pdata;
    struct cpld_platform_data        *syspld_pdata;
    struct virtual_mux_platform_data *swpld3_mux_pdata;
    struct cpld_platform_data        *swpld3_pdata;
    struct virtual_mux_platform_data *port_mux_pdata;
    struct cpld_platform_data        *port_cpld_pdata;
    int ret,i = 0;

    mutex_init(&dni_lock);
    printk(KERN_INFO "delta-evsa-32q56-platform module initialization\n");

    ret = dni_ipmi_create_user();
    if (ret != 0) {
        printk(KERN_WARNING "Fail to create IPMI user\n");
    }

    // register the mux prob which call the port cpld by ipmi
    ret = platform_driver_register(&port_mux_driver);
    if (ret) {
        printk(KERN_WARNING "Fail to register port mux driver\n");
        goto error_port_mux_driver;
    }

    // register the i2c devices
    ret = platform_driver_register(&i2c_device_driver);
    if (ret) {
        printk(KERN_WARNING "Fail to register i2c device driver\n");
        goto error_i2c_device_driver;
    }

    // link the port cpld and the Mux
    // port_cpld_pdata = evsa_32q56_port_cpld_platform_data;
    for (i = 0; i < ARRAY_SIZE(port_mux_device); i++)
    {
        // port_mux_pdata = port_mux_device[i].dev.platform_data;
        // port_mux_pdata->cpld = port_cpld_pdata[port_cpld_0].client;
        ret = platform_device_register(&port_mux_device[i]);
        if (ret) {
            printk(KERN_WARNING "Fail to create port mux %d\n", i);
            goto error_evsa_32q56_port_mux;
        }
    }

    for (i = 0; i < ARRAY_SIZE(evsa_32q56_i2c_device); i++)
    {
        ret = platform_device_register(&evsa_32q56_i2c_device[i]);
        if (ret)
        {
            printk(KERN_WARNING "Fail to create i2c device %d\n", i);
            goto error_evsa_32q56_i2c_device;
        }
    }

    printk("Initial delta_evsa_32q56_platform driver......[Success]\n");

    return 0;

error_evsa_32q56_i2c_device:
    i--;
    for (; i >= 0; i--) {
        platform_device_unregister(&evsa_32q56_i2c_device[i]);
    }
    i = ARRAY_SIZE(port_mux_device);
error_evsa_32q56_port_mux:
    i--;
    for (; i >= 0; i--) {
        platform_device_unregister(&port_mux_device[i]);
    }
    platform_driver_unregister(&i2c_device_driver);
error_i2c_device_driver:
    platform_driver_unregister(&port_mux_driver);
error_port_mux_driver:
    printk("Initial delta_evsa_32q56_platform driver......[Fail]\n");
    return ret;
}

static void __exit delta_evsa_32q56_platform_exit(void)
{
    int i = 0;

    for (i = 0; i < ARRAY_SIZE(evsa_32q56_i2c_device); i++) {
        platform_device_unregister(&evsa_32q56_i2c_device[i]);
    }

    for (i = 0; i < ARRAY_SIZE(port_mux_device); i++) {
        platform_device_unregister(&port_mux_device[i]);
    }

    platform_driver_unregister(&i2c_device_driver);
    platform_driver_unregister(&port_mux_driver);
    
    printk("Remove delta_evsa_32q56_platform driver......[Success]\n");
}

module_init(delta_evsa_32q56_platform_init);
module_exit(delta_evsa_32q56_platform_exit);

MODULE_DESCRIPTION("DELTA EVS-A-32Q56 Platform Support");
MODULE_AUTHOR("Jim Lee <jim.lee@deltaww.com>");
MODULE_LICENSE("GPL");