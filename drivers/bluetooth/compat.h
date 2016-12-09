#define hci_skb_pkt_type(skb) bt_cb((skb))->pkt_type
#define bt_dev_info(hdev, fmt, ...)                             \
	BT_INFO("%s: " fmt, (hdev)->name, ##__VA_ARGS__)
#define bt_dev_warn(hdev, fmt, ...)                             \
	BT_WARN("%s: " fmt, (hdev)->name, ##__VA_ARGS__)
#define bt_dev_err(hdev, fmt, ...)                              \
	BT_ERR("%s: " fmt, (hdev)->name, ##__VA_ARGS__)
#define bt_dev_dbg(hdev, fmt, ...)                              \
	BT_DBG("%s: " fmt, (hdev)->name, ##__VA_ARGS__)

