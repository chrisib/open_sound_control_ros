# Open Sound Control ROS

This repository contains several packages for ROS 2 Jazzy intended to interact with
[Open Sound Control](https://opensoundcontrol.stanford.edu).

## What is Open Sound Control?

Open Sound Control (OSC) is an open-source data transport specification for
realtime communication between applications and hardware. Similar to MIDI, OSC
is indended for use with electronic musical instruments primarily, but can have
other applications.

OSC, by default, uses UDP packets to transmit encoded data from one application
to another. Each packet consists of an address (similar in principle to a ROS
topic) and one or more variables of specific types.

For details on the OSC 1.0 and 1.1 specifications, see the official documentation
at the links below:
- [1.0 specification](https://opensoundcontrol.stanford.edu/spec-1_0.html)
- [1.1 specification](https://opensoundcontrol.stanford.edu/spec-1_1.html)

## OSC to/from ROS

The `open_sound_control_msgs` package contains the ROS message definitions needed
for handling OSC packets in ROS.

The `open_sound_control_bridge` package contains a ROS node that will accept OSC
messages and translate them into the corresponding ROS messages, or accept ROS
messages and translate them into their corresponding OSC messages.

The `open_sound_control_bridge` node publishes all received OSC messages as
`open_sound_control_msgs/msg/OscMessage` messages on its `osc_raw` topic.
Additionally, the brige node will publish each individual variable inside the
OSC payload on an individual ROS topic whose name corresponds to the OSC
address followed by the type and a numbered suffix.

For example, this OSC message
```
  2f (/)  66 (f)  6f (o)  6f (o)

  0 ()    0 ()    0 ()    0 ()

  2c (,)  69 (i)  69 (i)  73 (s)

  66 (f)  66 (f)  0 ()    0 ()

  0 ()    0 ()    3 ()    e8 (è)

  ff (ÿ)  ff (ÿ)  ff (ÿ)  ff (ÿ)

  68 (h)  65 (e)  6c (l)  6c (l)

  6f (o)  0 ()    0 ()    0 ()

  3f (?)  9d ()   f3 (ó)  b6 (¶)

  40 (@)  b5 (µ)  b2 (”)  2d (-)
```
containing the address `/foo`, the integer `-1`, the string `hello`, the
float `1.234`, and the float `5.678` would be republished as the following
topics:
- `osc_raw`: `open_sound_control_msgs/msg/OscMessage` -- the raw OSC data re-encoded
  as a ROS message
- `/foo/int_0`: `std_msgs/msg/Integer` containing the value `-1`
- `/foo/str_0`: `std_msgs/msg/String` containing the value `hello`
- `/foo/float_0`: `std_msgs/msg/Float32` containing the value `1.234`
- `/foo/float_1`: `std_msgs/msg/Float32` containing the value `5.678`

The following table shows the OSC types, the ROS topic name, and ROS message type:

| OSC type (character) | Topic name | ROS type                              |
|----------------------|------------|---------------------------------------|
| blob (`b`)           | `blob_N`   | `open_sound_control_msgs/msg/OscBlob` |
| boolean (`T`, `F`)   | `bool_N`   | `std_msgs/msg/Bool`                   |
| char (`c`)           | `char_N`   | `std_msgs/msg/String`                 |
| double (`d`)         | `double_N` | `std_msgs/msg/Float64`                |
| float (`f`)          | `float_N`  | `std_msgs/msg/Float32`                |
| integer (`i`)        | `int_N`    | `std_msgs/msg/Int32`                  |
| impulse (`I`)        | `trig_N`   | `std_msgs/msg/Empty`                  |
| midi (`m`)           | `midi_N`   | `std_msgs/msg/ByteMultiArray`         |
| null (`N`)           | `null_N`   | `std_msgs/msg/Empty`                  |
| rgba (`r`)           | `rgba_N`   | `std_msgs/msg/ColorRGBA`              |
| string (`s`)         | `string_N` | `std_msgs/msg/String`                 |
| symbols (`S`)        | `symbol_N` | `std_msgs/msg/String`                 |
| timetag (`t`)        | `time_N`   | `std_msgs/msg/Header`                 |

where `_N` is a 0-based index indicating the relative order of variables of that type.

To configure ROS -> OSC conversions, the bridge node must be properly configured.
See [bridge configuration](#bridge-configuration), below.

## Bridge configuration

The `open_sound_control_bridge` node uses a YAML file to configure ROS topic subscribers
and OSC destinations.

```yaml
listeners:
  - topic: /my_ros/topic
    type: my_ros_msgs/msg/Type
    osc_address: /my_osc/topic
    host: <ip address of destination device>
    port: <UDP port destination accepts messages on>
```

The ROS types in the table above + `osc_control_messages/msg/OscMessage` are allowed
types; all other types are rejected.

For example, the following would be a valid configuration to control a
[EuroPi](https://github.com/allen-synthesis/europi) eurorack module:
```yaml
listeners:
  # each CV is controlled via a float in the 0-1 range
  # each of these can be set with a single Float32 value
  - topic: /europi/cv1
    type: std_msgs/msg/Float32
    osc_address: /europi/cv1
    host: 192.168.4.1
    port: 9000
  - topic: /europi/cv2
    type: std_msgs/msg/Float32
    osc_address: /europi/cv2
    host: 192.168.4.1
    port: 9000
  - topic: /europi/cv3
    type: std_msgs/msg/Float32
    osc_address: /europi/cv3
    host: 192.168.4.1
    port: 9000
  - topic: /europi/cv4
    type: std_msgs/msg/Float32
    osc_address: /europi/cv4
    host: 192.168.4.1
    port: 9000
  - topic: /europi/cv5
    type: std_msgs/msg/Float32
    osc_address: /europi/cv5
    host: 192.168.4.1
    port: 9000
  - topic: /europi/cv6
    type: std_msgs/msg/Float32
    osc_address: /europi/cv6
    host: 192.168.4.1
    port: 9000

  # EuroPi also accepts a six-field message to set all CVs at once
  # this must be represented as a raw OSC message, as it contains multiple
  # values in the payload
  - topic: /europi/cvs
    type: open_sound_control_msgs/msg/OscMessage
    osc_address: /europi/cvs
    host: 192.168.4.1
    port: 9000
```

Note that some ROS types are re-used for multiple OSC message types. To specify the desired
OSC type for outgoing OSC packets, add the `osc_type` parameter to the listener:

```yaml
listeners:
  # 3 types use std_msgs/String subscribers
  - topic: /osc/my_string
    type: std_msgs/msg/String
    osc_address: /osc_string
    osc_type: s
    host: 10.0.0.101
    port: 8000
  - topic: /osc/my_symbols
    type: std_msgs/msg/String
    osc_address: /osc_symbol
    osc_type: S
    host: 10.0.0.101
    port: 8000
  - topic: /osc/my_char
    type: std_msgs/msg/String
    osc_address: /osc_char
    osc_type: c
    host: 10.0.0.101
    port: 8000

  # 2 types use std_msgs/Empty subscribers
  - topic: /osc/my_trigger
    type: std_msgs/msg/Empty
    osc_address: /osc_trigger
    osc_type: I
    host: 10.0.0.101
    port: 8000
  - topic: /osc/my_null
    type: std_msgs/msg/Empty
    osc_address: /osc_null
    osc_type: N
    host: 10.0.0.101
    port: 8000
```

If unspecified, any subscriber using `std_msgs/msg/String` will output `s` (OSC string)
packets and any subscriber using `std_msgs/msg/Empty` will output `N` (OSC null)
packets.
