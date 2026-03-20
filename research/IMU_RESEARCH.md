# Nintendo Switch 2 Pro Controller — IMU Sensor Data (BLE)

Reverse-engineered from the 63-byte BLE input reports on GATT handle
0x000e of the Nintendo Switch 2 Pro Controller (PID 0x2069).

## Equipment

- Pro Controller 2 (Nintendo, PID 0x2069) connected via BLE
- Host: Linux 6.18, BlueZ 5.87 with custom gatt-uhid bridge plugin
- Driver: hid-switch2 (DKMS, BLE transport)
- Analysis: Python 3 (numpy, matplotlib), btmon snoop captures

## Enabling IMU Data

The IMU is enabled via two FEATSEL commands written to the command GATT
characteristic (handle 0x0014):

1. **FEATSEL_SET_MASK** (cmd 0x0C, subcmd 0x02): declares which features
   the host supports. The mask byte must include `NS2_FEATURE_IMU` (bit 2).
2. **FEATSEL_ENABLE** (cmd 0x0C, subcmd 0x04): enables the declared features.
   The mask byte must also include bit 2 — omitting it here results in
   IMU bytes remaining zero despite the feature being declared.

IMU enable was confirmed by sending the FEATSEL_ENABLE command with
bit 2 set via a live bluetoothctl GATT write — IMU bytes immediately
populated in subsequent reports.

An additional feature bit (bit 7) enables magnetometer data. See the
Magnetometer section below.

## Input Report Structure

```
Byte  Content
───────────────────────────────────────────────
0     Sequence counter (0x00–0xFF, wrapping)
1     Constant 0x20 in all our captures (may contain power/battery info)
2     Buttons R: B(0) A(1) Y(2) X(3) R(4) ZR(5) +(6) RS(7)
3     Buttons L: Dn(0) Rt(1) Lt(2) Up(3) L(4) ZL(5) -(6) LS(7)
4     Buttons 3: Home(0) Capture(1) GR(2) GL(3) C(4) SR(6) SL(7)
5–7   Left stick  (12-bit X + 12-bit Y, standard Switch packing)
8–10  Right stick (12-bit X + 12-bit Y, standard Switch packing)
11    0x38 with rumble enabled, 0x30 without
12    NFC state (0x00 = idle)
13    Headset audio state (0x00 = no headset)
14    Block length including this byte (see below)
15–43 IMU data (when block length ≥ 30)
44–53 Magnetometer data (when block length = 40)
54–62 Reserved
```

### Byte 14 — Block Length

Byte 14 is the length of the sensor data block **including byte 14
itself**. The block spans from byte 14 to byte (14 + value − 1).

| Value | Hex  | Block span | Content | Frequency |
|-------|------|------------|---------|-----------|
| 0     | 0x00 | (none)     | No sensor data | 100% (before enable) |
| 30    | 0x1e | bytes 14–43 | IMU only | ~73% (after IMU enable) |
| 40    | 0x28 | bytes 14–53 | IMU + magnetometer | ~27% (after magneto enable) |

When the block length is 30, bytes 15–43 contain IMU data and bytes
44–53 are zero. When the length is 40, bytes 44–53 additionally
contain magnetometer data.

## IMU Data (bytes 15–43)

### Preamble (bytes 15–19)

```
15    Incrementing counter (~6 counts per packet, wrapping)
16    Combined: low nibble = slow counter, high nibble = sub-index (4/5/6)
17    Constant 0x00
18    Constant 0x0C
19    Constant 0x00
```

### Sensor Data Layout (bytes 20–43)

The sensor data occupies 24 bytes arranged in 6 groups of 4 bytes each,
at stride 4 starting from byte 20. The data contains three accelerometer
axes (16-bit signed) and two gyroscope axes (12-bit signed).

#### Accelerometers — 16-bit signed integers

Three accelerometer axes use int16 values with non-adjacent byte pairing.
The high and low bytes occupy different positions within each 4-byte group.

| Axis | High byte | Low byte | Extraction | Idle value |
|------|-----------|----------|------------|------------|
| Accel A (horizontal) | byte 24 | byte 26 | `b26 \| (b24 << 8)` | varies with orientation |
| Accel B (horizontal) | byte 31 | byte 30 | `b30 \| (b31 << 8)` | varies with orientation |
| Accel V (vertical)   | byte 42 | byte 41 | `b41 \| (b42 << 8)` | ~4205 (= 1g) |

**Verification:** The gravity magnitude √(A² + B² + V²) remains constant
at ~4500 counts across recordings at five different orientations (CV = 0.034),
confirming these three axes form a consistent 3-axis accelerometer with
approximately 4500 LSB/g.

The vertical accelerometer's behaviour during isolated rotation tests:
- Idle: ~4205 (constant 1g from gravity)
- Pitch rotation: traces one sinusoidal cycle as the gravity projection
  onto the vertical axis sweeps +1g → 0 → −1g → 0 → +1g. The shape
  is the same for CW and CCW pitch
- Yaw rotation on desk: flat (vertical axis unaffected)
- Roll: responds (gravity projection changes when tilting sideways)

#### Gyroscopes — 12-bit signed integers

Two gyroscope axes use 12-bit signed values extracted as:

```
value = (byte_n >> 4) | (byte_n+1 << 4)
if value >= 2048: value -= 4096
```

The high nibble of `byte_n` provides the low 4 bits; all 8 bits of
`byte_n+1` provide the high 8 bits. The low nibble of `byte_n`
(`byte_n & 0x0F`) is **not** part of the value — it has a uniform
distribution (0–15) uncorrelated with any physical motion.

| Axis | Bytes | Extraction | Idle value |
|------|-------|------------|------------|
| Roll gyroscope  | [33, 34] | `(b33 >> 4) \| (b34 << 4)`, signed | ~0 |
| Pitch gyroscope | [37, 38] | `(b37 >> 4) \| (b38 << 4)`, signed | ~−45 |

**Roll gyroscope [33, 34]:**
- Near zero at rest
- Responds only to roll rotation; flat during pitch and yaw
- Spike direction reverses between roll-left and roll-right

**Pitch gyroscope [37, 38]:**
- Near zero at rest (small bias of ~−45)
- Responds only to pitch rotation; flat during roll and yaw
- Spike direction reverses between pitch-up and pitch-down

**Evidence for 12-bit encoding (exhaustive nibble analysis):**
The low nibble of byte 37 was tested during idle and all rotation types.
It has a perfectly uniform distribution (all 16 values equally likely),
identical variance in all conditions (std ≈ 4.6), and zero correlation
with pitch motion (r = −0.09). The 12-bit extraction from the upper
bits correlates at r = 1.000000 with the full int16 interpretation,
confirming the signal is entirely in the upper 12 bits.

#### Yaw Gyroscope — Absent

An exhaustive search was conducted across all bytes 15–47 using:
1. All 12-bit extractions for every byte pair
2. All uint8 and int8 single-byte values
3. All individual nibbles
4. Cross-boundary pairings with every stable byte
5. A targeted search for any signal matching the confirmed gyro
   characteristics (near-zero idle, low variance, axis-specific response)

**Result:** No byte, nibble, or byte pair shows yaw-specific activation
(responds to yaw but not pitch/roll). Every unattributed position is
either uniform noise, a constant, or part of an already-identified channel.

The Pro Controller 2 does not report a yaw gyroscope in these input
reports. The magnetometer (see below) provides absolute heading
information which may serve as a substitute.

### Byte Map

```
Byte  Content                              Notes
────────────────────────────────────────────────────────
15    Packet counter                        ~6 counts/pkt
16    Slow counter + sub-index
17    Constant 0x00
18    Constant 0x0C
19    Constant 0x00
20    Unknown (high variance)               Group 0
21    Unknown (high variance)
22    Unknown (~100, responds to rotation)
23    Constant 0x02
24    Accel A high byte                     Group 1
25    Unknown (high variance)
26    Accel A low byte
27    Constant 0x01
28    Unknown (high variance)               Group 2
29    Unknown (high variance)
30    Accel B low byte
31    Accel B high byte
32    Unknown (high variance)               Group 3
33    Roll gyro: hi nibble = low 4 bits     lo nibble = noise
34    Roll gyro: high 8 bits
35    Unknown (high variance)
36    Unknown (high variance)               Group 4
37    Pitch gyro: hi nibble = low 4 bits    lo nibble = noise
38    Pitch gyro: high 8 bits
39    Unknown
40    Unknown (high variance)               Group 5
41    Accel V low byte
42    Accel V high byte
43    Near-constant (~4)
```

76 of 192 bits (bytes 20–43) have been identified. The remaining 116
bits include uniform-noise bytes that may carry LSBs of values whose
MSBs are not yet identified, or may be unused padding.

## Magnetometer Data (bytes 44–53)

Enabled by setting `NS2_FEATURE_MAGNETO` (bit 7) in both FEATSEL_SET_MASK
and FEATSEL_ENABLE. When enabled, approximately 27% of reports arrive as
sub-type 0x28 with 10 bytes of magnetometer data in bytes 44–53.

**Key findings:**
- Bytes 44–47 are zero in 0x1e reports but populated in 0x28 reports,
  confirming they belong to the magnetometer, not the IMU.
- The magnetometer region is therefore **10 bytes** and 0x1e and 0x28
  are most likely block lengths, not report types.
- Byte 52 acts as a tilt-sensitive MSB: its bit 7 functions as a sign
  bit, and the raw byte traces a clean pitch arc that wraps at the
  uint8 boundary during full rotation.
- Byte 51's high nibble carries a heading-correlated signal that
  responds to yaw rotation.
- Bytes 48–49, when interpreted as int16 LE, show clear tilt response
  but overflow during large movements, indicating additional MSBs
  exist elsewhere in the 10-byte block.

**The complete encoding of the magnetometer data is under active
investigation.** The values are not simple consecutive int16 LE pairs.
The encoding uses a nibble-based packing scheme where MSBs and LSBs
are distributed across non-adjacent bytes, similar to the IMU's
non-adjacent byte pairing for accelerometers.

## Open Questions

1. **Report type:** The BLE payload on handle 0x000E does not include a
   report type byte. The byte offsets match the report 0x09 layout
   documented for Pro Controller 2.

2. **Pre-integration:** The IMU data format has been described as
   "curious" and "possibly pre-integrated" by other researchers working
   on the Switch 2 driver. This could affect interpretation of the
   sensor values.

3. **Yaw gyroscope:** The Pro Controller 2 may lack a yaw gyro in
   hardware, or it may be reported in a different format. The Joy-Con 2,
   which has additional sensors per Nintendo's specifications, may
   populate the noise bytes with a yaw gyro.

4. **Unidentified bytes:** 116 bits in the IMU region (bytes 20–43) are
   unidentified. These include both uniform-noise bytes (potential LSBs)
   and bytes with structure that doesn't match any confirmed sensor pattern.

5. **Magnetometer encoding:** The 10-byte magneto block uses a non-trivial
   packing scheme. Partial results suggest 12-bit or 16-bit values
   assembled from non-contiguous nibbles, but the complete layout is
   not yet determined.
