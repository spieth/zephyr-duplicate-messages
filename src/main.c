#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>
#include <stdlib.h>
#include <zephyr.h>
#include <stdio.h>

#define MESSAGE_INTERVAL_IN_SEC 30
#define TEST_PAYLOAD_LENGTH    10

#define MODEL_ID_TEST_DATA_CLI 0x0001

#define GROUP_ADDR 0xc000

#define BT_MESH_MODEL_OP_VENDOR_TEST_DATA BT_MESH_MODEL_OP_3(0x01, BT_COMP_ID_LF)

static const u8_t net_key[16] = {
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
};
static const u8_t dev_key[16] = {
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
};
static const u8_t app_key[16] = {
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
};

static const u16_t net_idx;
static const u16_t app_idx;
static const u32_t iv_index;
static u16_t addr = 1;
static u16_t message_id = 0;
static u8_t flags;

static const u8_t dev_uuid[16] = {
    0xef, 0xcd, 0xab, 0x89, 0x67, 0x45, 0x23, 0x01,
    0xef, 0xcd, 0xab, 0x89, 0x67, 0x45, 0x23, 0x02,
};

static const struct bt_mesh_prov prov = {
    .uuid = dev_uuid,
};
static struct bt_mesh_cfg_cli cfg_cli = {};
static struct bt_mesh_cfg_srv cfg_srv = {
    .relay = BT_MESH_RELAY_DISABLED,
    .beacon = BT_MESH_BEACON_DISABLED,
    .frnd = BT_MESH_FRIEND_NOT_SUPPORTED,
    .gatt_proxy = BT_MESH_GATT_PROXY_NOT_SUPPORTED,
    .default_ttl = 7,
    .net_transmit = BT_MESH_TRANSMIT(0, 20),
};

// health server
BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);
static struct bt_mesh_health_srv health_srv = {};

static void vnd_data_get(struct bt_mesh_model *model,
                                    struct bt_mesh_msg_ctx *ctx,
                                    struct net_buf_simple *buf)
{
    u8_t i;

    if (ctx->addr == addr) {
        return;
    }

    u16_t message_id = net_buf_simple_pull_be16(buf);
    printk("%d: got message %d from node %d (",
        k_uptime_get_32(), message_id, ctx->addr);

    for (i = 0; i < TEST_PAYLOAD_LENGTH; i++) {
        printk("%d ", net_buf_simple_pull_u8(buf));
    }
    printk(")\n");
}

static const struct bt_mesh_model_op vnd_test_data_cli_op[] = {
    {BT_MESH_MODEL_OP_VENDOR_TEST_DATA, 0, vnd_data_get},
    BT_MESH_MODEL_OP_END,
};

static struct bt_mesh_model root_models[] = {
	BT_MESH_MODEL_CFG_SRV(&cfg_srv),
	BT_MESH_MODEL_CFG_CLI(&cfg_cli),
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
};
static struct bt_mesh_model vnd_models[] = {
    BT_MESH_MODEL_VND(BT_COMP_ID_LF, MODEL_ID_TEST_DATA_CLI,
                        vnd_test_data_cli_op, NULL, NULL),
};
static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, root_models, vnd_models),
};
static const struct bt_mesh_comp comp = {
	.cid = BT_COMP_ID_LF,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

static void self_configuring()
{
    printk("self configuring\n");

    /* Add Application Key */
	bt_mesh_cfg_app_key_add(net_idx, addr, net_idx, app_idx, app_key, NULL);

	/* Bind to vendor model */
    bt_mesh_cfg_mod_app_bind_vnd(net_idx, addr, addr, app_idx,
                    MODEL_ID_TEST_DATA_CLI, BT_COMP_ID_LF, NULL);

	/* Bind to Health model */
	bt_mesh_cfg_mod_app_bind(net_idx, addr, addr, app_idx,
				    BT_MESH_MODEL_ID_HEALTH_SRV, NULL);

	/* Add model subscription */
    bt_mesh_cfg_mod_sub_add_vnd(net_idx, addr, addr, GROUP_ADDR,
                    MODEL_ID_TEST_DATA_CLI, BT_COMP_ID_LF, NULL);

    printk("self configuring complete\n");
}

void send_data(void)
{
    u8_t i;
    /**
     * 3 bytes for vendor opcode
     * payload: - X bytes dummy data
     *          - 2 bytes for message id
     * 4 bytes for TransMIC
     */
    NET_BUF_SIMPLE_DEFINE(msg, 3 + (TEST_PAYLOAD_LENGTH + 2) + 4);
    struct bt_mesh_msg_ctx ctx = {
        .net_idx = net_idx,
        .app_idx = app_idx,
        .addr = GROUP_ADDR,
        .send_ttl = 2,
    };

    bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_VENDOR_TEST_DATA);

    net_buf_simple_add_be16(&msg, message_id);

    for (i = 0; i < TEST_PAYLOAD_LENGTH; i++) {
        net_buf_simple_add_u8(&msg, i);
    }

    printk("%d: sending %d bytes test data with id %d\n", k_uptime_get_32(),
        msg.len, message_id);

    if (bt_mesh_model_send(&vnd_models[0], &ctx, &msg, NULL, NULL)) {
        printk("Unable to send test data\n");
        return;
    }
    printk("finish sending\n");
    message_id++;
}

void beacon_entry_point(void *unused1, void *unused2, void *unused3) {
    ARG_UNUSED(unused1);
    ARG_UNUSED(unused2);
    ARG_UNUSED(unused3);

    printk("Inside beacon thread\n");
    k_sleep(K_SECONDS(5));

    while (1) {
        send_data();
        k_sleep(K_SECONDS(MESSAGE_INTERVAL_IN_SEC));
    }
}

#define BEACON_STACK_SIZE 1024
K_THREAD_STACK_DEFINE(beacon_stack_area, BEACON_STACK_SIZE);
struct k_thread beacon_thread_data;

static void bt_ready(int err)
{
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");
    err = bt_mesh_init(&prov, &comp);
	if (err) {
		printk("Initializing mesh failed (err %d)\n", err);
		return;
	}

    printk("Mesh initialization complete\n");
    printk("self provisioning\n");

    err = bt_mesh_provision(net_key, net_idx, flags, iv_index, addr, dev_key);
    if (err == -EALREADY) {
        printk("Using stored settings\n");
    } else if (err) {
        printk("Provisioning failed (err %d)\n", err);
        return;
    } else {
        printk("Provisioning completed\n");
        self_configuring();
    }

    printk("Bluetooth and mesh ready\n");

    k_tid_t beacon_thread_id = k_thread_create(&beacon_thread_data,
                                    beacon_stack_area,
                                    K_THREAD_STACK_SIZEOF(beacon_stack_area),
                                    beacon_entry_point,
                                    NULL, NULL, NULL, -5, 0, K_NO_WAIT);
    printk("Created beacon thread with id %p\n", beacon_thread_id);
}

void main(void)
{
    int err = bt_enable(bt_ready);
    if (err) {
        printk("Enable bluetooth failed (err %d)\n", err);
    }
}
