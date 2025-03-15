# Required OSC 1.1 types:
string FLOAT = "f"    # IEEE float32
string INTEGER = "i"  # signed 32-bit integer (most significant byte first)
string BLOB = "b"     # byte array of length N, where N is represented as a 32-bit integer in the first 4 bytes
string STRING = "s"   # null-terminated string of 8-bit chars
string TIMETAG = "t"  # 64-bit OSC timetag in NTP format
string IMPULSE = "I"  # impulse/trigger; no bytes allocated in payload
string NIL = "N"      # null; no bytes allocated in payload
string B_TRUE = "T"   # boolean True; no bytes allocated in payload
string B_FALSE = "F"  # boolean False; no bytes allocated in payload


# Optional additional types
string CHAR = "c"     # a single 8-bit character (contained in the least-significant byte of a 32-bit integer)
string DOUBLE = "d"   # IEEE 754 float64
string MIDI = "m"     # 4-byte MIDI message
string RGBA = "r"     # 32-bit RGBA colour
string SYMBOLS = "S"  # alternative data represented as a string; treated interchangeably with "s"

# Optional data arrays
# Array data is not supported in open_sound_control_ros; OSC messages containing
# these characters will be ignored
string ARR_START = "["
string ARR_STOP = "]"


# The OSC address of the message
string address

# The OSC v1.1 compatible list of types in the payload
# Each character in the string indicates a specific type
# See type definitions above
string[] types

# The payload bytes
# Each variable in the payload is word-aligned, meaning there
# may be padding bytes between items
uint8[] payload
