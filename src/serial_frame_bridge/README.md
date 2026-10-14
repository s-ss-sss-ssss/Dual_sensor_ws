# serial_frame_bridge

ROS Noetic raw v2 serial bridge. The non-ROS C++14 core opens a serial port (default `/dev/maggrad`, 115200 8N1), reads/writes asynchronously with Boost.Asio, and retries connection after 0.5, 1, 2, then 5 seconds (5 seconds thereafter). It clears any partial RX frame and queued TX commands on disconnect. No sensor records are decoded.

`~raw_frames` (`serial_frame_bridge/RawFrame`) contains `header` and `data`: the **entire** validated wire frame including sync, header, payload and CRC. `header.stamp` is the ROS receive time (when a complete frame is validated); `header.frame_id` is empty. A bounded 512-frame FIFO drops the oldest frame when full; a ROS wall timer publishes frames, never the I/O callback. `~tx_bytes` (`std_msgs/UInt8MultiArray`) sends opaque bytes as supplied, without wrapping, CRC generation, or protocol interpretation. Outbound writes and pending submissions are bounded (128 messages / 256 KiB each); commands are dropped while disconnected and queued commands are discarded on disconnect. `~diagnostics` (`diagnostic_msgs/DiagnosticArray`) reports connection and parser/queue counters once per second.

**Firmware downlink is not yet implemented.** Sending `~tx_bytes` only writes bytes to the serial port; the present firmware has no command parser and will not respond to them.

Launch with `roslaunch serial_frame_bridge serial_frame_bridge.launch port:=/dev/maggrad`. Private parameters: `~port`, `~baud_rate`, `~frame_queue_size` (default 512, minimum 1), `~publish_batch_size` (default 512, minimum 1). The node advertises a publisher queue depth of 5. Diagnostics include parser CRC/header errors and resynchronization bytes, serial read/write errors and reconnect attempts, and dropped RX/TX counts. The launch also exposes `raw_frame_topic`, `diagnostics_topic`, `tx_bytes_topic`, `frame_queue_size`, and `publish_batch_size`; root-level defaults preserve the existing topics and queue settings. `maggrad_protocol/joint_debug.launch` starts this bridge with the decoder and statistics node, remapping bridge diagnostics to `/diagnostics` for aggregation. No device is needed to run parser tests; Linux PTY tests exercise live asynchronous I/O.

## Multiple instances and topics

The launch inherits its namespace from an enclosing `<group ns="...">` or
`ROS_NAMESPACE=board_a`. Each device needs a distinct namespace and serial port.
Topic arguments are explicit remaps of the node's private interfaces:

| Argument | Direction / type | Default (relative to instance namespace) |
| --- | --- | --- |
| `raw_frame_topic` | Output / `serial_frame_bridge/RawFrame` | `serial_frame_bridge/raw_frames` |
| `tx_bytes_topic` | Input / `std_msgs/UInt8MultiArray` | `serial_frame_bridge/tx_bytes` |
| `diagnostics_topic` | Output / `diagnostic_msgs/DiagnosticArray` | `serial_frame_bridge/diagnostics` |

Relative defaults isolate instances without changing single-device root topic
names. Absolute overrides intentionally bypass the namespace. TX configuration
is a launch argument/remap, not a new private ROS parameter; direct `rosrun`
users can remap `~tx_bytes` as before. Diagnostics now use the full node name
plus `: serial bridge` (e.g. `/board_a/serial_frame_bridge: serial bridge`),
not the fixed `serial_frame_bridge` name, so merged diagnostics do not collide.
The updated MagGrad statistics node recognizes both formats.

```bash
ROS_NAMESPACE=board_a roslaunch serial_frame_bridge serial_frame_bridge.launch \
  port:=/dev/ttyACM0 tx_bytes_topic:=commands
# Input: /board_a/commands; output: /board_a/serial_frame_bridge/raw_frames
```

Launch still sends no control commands automatically; the firmware downlink
limitation above is unchanged.
