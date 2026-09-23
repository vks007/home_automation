# ESP-NOW Sensor OTA Update: Final Implementation Approach

## 1. Purpose

This document defines the final OTA update architecture for battery-powered ESP8266 sensors, with future ESP32 compatibility.

The sensors normally communicate with an ESP-based gateway through ESP-NOW and may be asleep or powered off for long periods. The gateway is connected to normal Wi-Fi and MQTT/Home Assistant. Firmware binaries are hosted on an always-on server accessible over the local Wi-Fi network.

This document is intended to be sufficiently detailed for an AI coding agent to implement the solution without needing the original design discussion.

---

## 2. Final Design Decision

Use two separate communication planes:

- **ESP-NOW is the control plane.**
  - The sensor announces that it is awake.
  - The gateway tells the sensor whether an OTA update is pending.
- **Wi-Fi plus HTTP/HTTPS is the data plane.**
  - When instructed, the sensor reboots into OTA mode.
  - It connects to the normal Wi-Fi network.
  - It downloads the firmware binary from the local firmware server.
  - It validates and installs the image, then restarts into normal sensor mode.

Do **not** continuously transmit OTA commands from the gateway while a sensor is asleep.

Do **not** send firmware images over ESP-NOW.

Do **not** require a physical button to enter OTA mode.

The gateway stores an OTA request as persistent desired state. A sleeping sensor discovers the request on its next normal wake cycle.

---

## 3. High-Level Architecture

```text
                         Home Assistant host/server
                         +-------------------------+
                         | Firmware repository     |
                         | manifest and .bin files |
                         | HTTP/HTTPS server       |
                         +------------+------------+
                                      |
                                      | Wi-Fi HTTP/HTTPS download
                                      |
+----------------+          +---------+---------+          +----------------+
| Sleeping       | ESP-NOW | ESP gateway       | Wi-Fi    | MQTT / Home    |
| ESP8266/ESP32  +-------->+ persistent OTA DB +--------->+ Assistant       |
| sensor         | <--------+ and web/API UI     |          | automation      |
+----------------+ command  +-------------------+          +----------------+
```

The gateway records, for each sensor:

- Device identifier or MAC address
- Device type/hardware model
- Currently reported firmware version
- Desired firmware version
- OTA transaction ID
- Firmware artifact identifier
- OTA status
- Last-seen time
- Last OTA error, if any

---

## 4. Core OTA Flow

### 4.1 Scheduling the update

A user or Home Assistant schedules an update for a device. This operation must only update persistent desired state. It must not assume the sensor is currently awake.

Example gateway record:

```text
device_id: front-door-sensor
mac: AA:BB:CC:DD:EE:FF
hardware_model: door-v1
current_version: 1.7.0
desired_version: 1.8.0
ota_id: 12345
artifact_id: door-v1-1.8.0
ota_status: PENDING
```

The record must survive a gateway restart.

### 4.2 Sensor wake sequence

The sensor sends a generic `HELLO` packet immediately after ESP-NOW initialization. It then continues normal initialization and sensor work instead of blocking while waiting for the response.

```text
Wake
  -> initialize retained/runtime state
  -> initialize ESP-NOW
  -> send HELLO
  -> initialize and read sensors
  -> allow receive callback to capture a gateway command
  -> send normal sensor data
  -> process any validated OTA command
  -> otherwise enter sleep
```

This overlaps the OTA response window with useful sensor work and minimizes additional awake time.

### 4.3 Gateway response

On receiving `HELLO`, the gateway:

1. Identifies the sensor from the authenticated/registered peer and packet device ID.
2. Updates last-seen time and the reported current firmware version.
3. Checks persistent desired state.
4. Replies immediately with either:
   - `COMMAND_NONE`, or
   - `COMMAND_OTA` containing the OTA transaction and artifact information.
5. If the reported version already equals the desired version, clears the pending update as successful.

### 4.4 Entering OTA mode

After the sensor receives and validates `COMMAND_OTA`, it must:

1. Ensure the command targets its own hardware model.
2. Reject an older or equal firmware version unless a deliberate force/downgrade flag is supported.
3. Persist the OTA transaction details.
4. Send an optional `OTA_ACCEPTED` status to the gateway.
5. Allow a short bounded interval for the outgoing packet to be transmitted.
6. Restart.

On the next boot, persistent boot state causes the device to enter OTA mode instead of normal sensor mode.

### 4.5 Firmware download and installation

In OTA mode, the sensor:

1. Connects to the configured Wi-Fi network.
2. Builds or retrieves the firmware URL from the artifact identifier.
3. Downloads the firmware using the platform-specific HTTP update library.
4. Verifies all available integrity and authenticity controls before activating it.
5. Restarts after a successful installation.
6. Boots normally and reports the new firmware version in its next `HELLO`.
7. The gateway observes the desired version and marks the transaction successful.

---

## 5. Sensor Wake Timing

### 5.1 Non-blocking response handling

Do not add a fixed `delay(50)` or `delay(80)` immediately after transmitting `HELLO`.

Instead:

- Send `HELLO` as early as possible.
- Continue sensor initialization and measurement.
- Let the ESP-NOW receive callback capture a response.
- Check the captured command before sleeping.

The callback must do minimal work. It should validate the basic frame length, copy the response into a protected/static buffer, set a flag, and return. It must not write flash, connect to Wi-Fi, restart the device, or perform lengthy logging from callback context.

Conceptual flow:

```cpp
initializeEspNow();
sendHello();

initializeSensors();
readSensors();
prepareSensorPacket();

sendSensorPacket();

if (validatedOtaCommandAvailable()) {
    persistOtaRequest();
    sendOtaAcceptedBestEffort();
    restartIntoOtaMode();
}

enterDeepSleep();
```

### 5.2 Minimum response window

Some sensors, such as door sensors, may complete all useful work in less than 5 ms. For these devices, enforce a short bounded command window before sleeping.

Recommended starting values, to be measured on the real network:

- Slow sensors: no extra wait if measurement already provides enough time.
- Fast sensors: total command opportunity of approximately 15 to 30 ms after `HELLO` transmission.
- Never wait indefinitely.

The exact value must be configurable per hardware profile and determined from measured gateway response latency, including a safety margin.

### 5.3 Missing command response

A missing gateway response is not an error that should keep the device awake. The device should complete its normal sensor transmission and sleep. The pending gateway state ensures the OTA command will be offered again on the next wake.

---

## 6. Protocol Design

## 6.1 General requirements

The management protocol must be:

- Versioned
- Idempotent
- Bounded in size
- Safe against duplicate frames
- Safe against delayed frames
- Extensible for future remote configuration commands
- Explicit about hardware compatibility
- Independent of C/C++ compiler struct padding

Do not transmit raw compiler structs unless their layout is explicitly packed and every field's byte order and size are fixed. Prefer explicit serialization/deserialization.

All multi-byte integer fields must use a documented byte order, preferably network byte order.

Every packet should include:

- Protocol version
- Message type
- Device ID or sender identity
- Boot/session ID
- Sequence or correlation ID
- Payload length
- Integrity field such as CRC for accidental corruption, if not already sufficiently handled by the chosen serialization/protocol

ESP-NOW link-layer delivery success only indicates delivery at the radio/MAC layer. It does not mean that the gateway accepted or processed the application command.

## 6.2 HELLO packet

The `HELLO` packet should be generic and reusable for future device management.

Suggested logical fields:

```text
protocol_version
message_type = HELLO
device_id
boot_id
sequence_number
hardware_model
hardware_revision
firmware_version
config_version
capabilities
battery_mv, optional
```

Notes:

- `boot_id` should change at every boot. A random value or monotonic retained counter may be used.
- `sequence_number` allows response correlation and duplicate detection.
- `hardware_model` prevents distributing an incompatible image.
- `capabilities` can indicate HTTP OTA, HTTPS OTA, signed-image support, and other future features.

## 6.3 Gateway command response

Suggested logical fields:

```text
protocol_version
message_type = COMMAND_RESPONSE
device_id
boot_id
request_sequence
command
command_id
payload_length
payload
```

Commands initially required:

```text
COMMAND_NONE
COMMAND_OTA
```

Future commands may include:

```text
COMMAND_SET_CONFIG
COMMAND_CALIBRATE
COMMAND_RESET_CONFIG
COMMAND_FACTORY_RESET
COMMAND_CHANGE_WAKE_INTERVAL
```

The gateway response must echo `boot_id` and `request_sequence`. The sensor must reject a response that does not match the current boot/session and the most recent `HELLO` request.

## 6.4 OTA payload

Suggested OTA payload:

```text
ota_id
firmware_version
hardware_model
artifact_id
firmware_size
firmware_digest
digest_algorithm
security/signature metadata
flags
```

Avoid placing a long URL in the ESP-NOW packet. ESP-NOW payload size is constrained and platform/framework limits may differ. Prefer a compact `artifact_id` and a firmware-server base URL stored in device configuration.

Example URL construction:

```text
base_url:    http://192.168.1.20/firmware
artifact_id: door-v1-1.8.0
result:      http://192.168.1.20/firmware/door-v1-1.8.0.bin
```

Alternatively, OTA mode can download a compact manifest using the artifact ID and obtain the final file metadata from it.

## 6.5 Idempotency

`ota_id` uniquely identifies one scheduled OTA transaction.

If the sensor receives the same `ota_id` multiple times:

- It must not create multiple independent operations.
- If already pending, it should retain the existing OTA state.
- If already successfully installed, it should report its current version and ignore the duplicate.
- If a previous attempt failed and retry policy allows it, it may retry the same transaction with backoff.

The gateway must continue offering an OTA transaction while status is pending or retryable. It clears the request only after the device reports the desired firmware version, or after an explicit cancellation.

---

## 7. Persistent State

## 7.1 Sensor boot state

Use a structured, versioned record rather than a single EEPROM flag.

Suggested logical record:

```cpp
enum class BootMode : uint8_t {
    NORMAL = 0,
    OTA_REQUESTED = 1,
    OTA_IN_PROGRESS = 2,
    OTA_FAILED = 3
};

struct PersistentBootState {
    uint32_t magic;
    uint16_t schema_version;
    BootMode boot_mode;
    uint32_t ota_id;
    char target_version[VERSION_MAX];
    char hardware_model[MODEL_MAX];
    char artifact_id[ARTIFACT_MAX];
    uint32_t firmware_size;
    uint8_t expected_digest[DIGEST_MAX];
    uint8_t attempt_count;
    uint32_t crc;
};
```

The actual implementation should use the platform's appropriate persistence layer:

- ESP8266: EEPROM emulation, LittleFS, or another carefully managed flash record.
- ESP32: NVS/Preferences or the project-standard persistence layer.

Requirements:

- Validate magic, schema version, bounds, and CRC before using the record.
- Use atomic or dual-slot persistence where practical, so power loss cannot leave the only state record corrupted.
- Minimize flash writes.
- Clear secrets and obsolete OTA metadata after success.
- Treat an invalid record as `NORMAL`, while reporting a diagnostic if possible.

## 7.2 Gateway OTA state

The gateway OTA registry must also be persistent.

Suggested statuses:

```text
IDLE
PENDING
OFFERED
ACCEPTED
DOWNLOADING
SUCCESS
RETRYABLE_FAILURE
PERMANENT_FAILURE
CANCELLED
```

Not every intermediate state will be directly observable. At minimum, persist `PENDING`, `SUCCESS`, failure metadata, and cancellation.

The gateway must not mark an update successful merely because it transmitted `COMMAND_OTA` or received `OTA_ACCEPTED`. Success requires a subsequent `HELLO` reporting the desired firmware version.

---

## 8. Firmware Repository on the Home Assistant Server

Host the firmware repository on the always-on Home Assistant machine or another local server. A simple static web server or reverse-proxy container is sufficient.

Suggested directory layout:

```text
firmware/
  manifest.json
  door-v1/
    1.8.0/
      firmware.bin
      metadata.json
      firmware.sig
  temperature-v1/
    2.3.0/
      firmware.bin
      metadata.json
      firmware.sig
```

Example manifest structure:

```json
{
  "schema_version": 1,
  "artifacts": {
    "door-v1-1.8.0": {
      "hardware_model": "door-v1",
      "version": "1.8.0",
      "path": "/firmware/door-v1/1.8.0/firmware.bin",
      "size": 487312,
      "digest_algorithm": "sha256",
      "digest": "REPLACE_WITH_ACTUAL_DIGEST",
      "signature_path": "/firmware/door-v1/1.8.0/firmware.sig",
      "minimum_bootloader": null
    }
  }
}
```

The build/deployment pipeline should generate metadata rather than relying on manual entry.

At publish time, it should:

1. Build the correct target and flash layout.
2. Calculate exact file size.
3. Calculate the configured digest.
4. Sign the image or metadata if signing is implemented.
5. Publish the binary and metadata atomically.
6. Update the manifest only after all referenced files are available.

Never replace a binary in place while retaining the same artifact ID and digest metadata. Artifact identifiers should be immutable.

---

## 9. OTA Mode Behavior

## 9.1 OTA boot entry

At boot, perform only the minimum initialization needed to read and validate persistent boot state.

```text
if boot_mode is OTA_REQUESTED or OTA_IN_PROGRESS:
    run OTA workflow
else:
    run normal sensor workflow
```

Set `OTA_IN_PROGRESS` before starting the network download. This allows the next boot to distinguish a fresh request from a reset during an attempted update.

## 9.2 Wi-Fi connection

OTA mode connects to the normal Wi-Fi network. It does not need to maintain ESP-NOW during the firmware download.

Requirements:

- Bounded Wi-Fi connection timeout
- Bounded DHCP timeout
- Optional static network configuration only if there is a strong operational need
- Clear logging of connection failure reasons
- Watchdog-safe retry loops
- No infinite retry loop

A fixed IP address is not required. The sensor initiates an outbound HTTP/HTTPS request, so DHCP is normally sufficient.

## 9.3 URL selection

Preferred approach:

- Store a firmware server base URL in device configuration.
- Receive only a compact immutable artifact ID over ESP-NOW.
- Construct a safe URL or query the manifest in OTA mode.

Do not blindly accept arbitrary URLs from an unauthenticated ESP-NOW command.

## 9.4 Download validation

Before activation, validate as much as the selected framework supports:

- HTTP success status
- Content length, if supplied
- Maximum allowed image size
- Exact expected artifact size
- Image/header compatibility with the platform
- Hardware model compatibility
- Cryptographic digest
- Digital signature, when implemented

Important implementation note: framework support differs between ESP8266 and ESP32 and between core versions. The coding agent must verify the exact update API and available validation hooks in the project dependencies. Do not assume that a requested SHA-256 value is automatically checked by a high-level update call. If the framework only checks another digest or provides no signature validation, implement the missing validation safely or document the limitation explicitly.

## 9.5 Success

After a successful update:

- The platform updater normally restarts the device.
- The new firmware must boot in normal mode.
- The new firmware must report its version in `HELLO`.
- The gateway marks the OTA transaction successful only when the desired version is observed.
- The sensor clears stale OTA state at a safe point that does not cause a boot loop.

The new application should understand the existing persistence schema or include a migration path.

## 9.6 Failure and timeout

OTA mode must have an overall deadline. Suggested initial value: 60 to 120 seconds, adjusted for firmware size and network performance.

On failure:

1. Record a compact failure code and increment the attempt counter.
2. Clear `OTA_IN_PROGRESS` or change it to `OTA_FAILED` according to retry policy.
3. Return to normal sensor mode so the device remains operational.
4. Report the current firmware and last OTA failure on a later `HELLO` or status packet.
5. Let the gateway decide whether to retry, cancel, or schedule another artifact.

Do not reboot endlessly into OTA mode.

Recommended retry policy:

- Limit immediate attempts during one OTA boot.
- Return to normal operation after failure.
- Retry on a later wake only if the gateway still offers the transaction.
- Apply an attempt limit or increasing backoff for persistent failures.
- Allow a gateway-side cancellation to clear the pending transaction.

---

## 10. Rollback and Recovery

Do not claim rollback support unless it is actually implemented and tested for the target flash layout and framework.

ESP32 commonly supports safer dual-partition OTA patterns when partitioning and boot validation are configured appropriately. ESP8266 constraints and available rollback behavior may differ significantly.

Minimum recovery behavior for all targets:

- Reject incompatible or oversized images before activation.
- Avoid endless OTA boot loops.
- Return to the previous running application when download/install fails before activation.
- Keep the device's normal sensing function available after a retryable failure.
- Retain a physical serial recovery path for development and severe failures.

If true rollback is required, define and test it as a separate feature, including:

- Partition layout
- First-boot health confirmation
- Watchdog/reset behavior before confirmation
- Automatic revert conditions
- Persistence migration compatibility

---

## 11. Security Requirements

### 11.1 Device and command authorization

- Maintain a gateway allowlist of sensor MAC addresses/device IDs.
- Use ESP-NOW peer encryption where supported and practical.
- Bind commands to the current `boot_id`, request sequence, and target device.
- Reject commands for a different hardware model.
- Reject malformed lengths, unknown protocol versions, and unsupported commands.

### 11.2 Firmware authenticity

A hash protects against accidental corruption only when the expected hash itself is trusted. A digital signature establishes that the artifact was published by an authorized signer.

Recommended progression:

1. **Initial LAN proof of concept:** local HTTP server with strict artifact size/version/model checks and an independently trusted digest.
2. **Production-quality home deployment:** HTTPS where reliable on the target, or signed firmware/metadata with an embedded public key.
3. **Any remotely accessible repository:** signed artifacts are strongly preferred. Do not rely only on network location.

Never embed a private signing key in the firmware, gateway, or Home Assistant configuration. Only the public verification key belongs on devices.

### 11.3 Credentials

- Do not transmit Wi-Fi credentials over normal ESP-NOW OTA commands.
- Provision credentials through the existing secure device provisioning process.
- Avoid printing secrets in logs.
- Define how Wi-Fi credential changes are recovered for inaccessible sensors.

---

## 12. Versioning Rules

Use a structured version representation, not floating-point numbers such as `1.8`.

Recommended options:

- Semantic version string: `1.8.0`
- Integer tuple: major, minor, patch, build
- Monotonic build number plus display version

Comparison rules must be deterministic.

The normal rule is:

```text
update only when desired_version > current_version
```

Downgrade or reinstall of the same version must require an explicit force flag and should be disabled by default.

Hardware model and revision compatibility take priority over version comparison.

---

## 13. Gateway Responsibilities

The gateway implementation must provide:

1. Registered sensor inventory.
2. Persistent desired firmware state.
3. Current firmware information learned from `HELLO`.
4. Immediate response to a sensor wake packet.
5. Compact command serialization.
6. Duplicate-safe OTA transaction IDs.
7. Update scheduling, cancellation, and status APIs/UI.
8. MQTT/Home Assistant status publication if desired.
9. Diagnostic event logging.
10. Automatic success detection based on the subsequent reported version.

Suggested per-device data model:

```text
device_id
espnow_mac
hardware_model
hardware_revision
current_version
desired_version
artifact_id
ota_id
ota_status
ota_attempts
last_seen
last_command_offered
last_error
```

Gateway processing must remain asynchronous. The receive callback should enqueue or copy the packet for fast processing. A worker/main loop should perform persistence, web/API work, and response construction, provided it can still meet the required wake response latency.

If queueing introduces too much latency, maintain an in-memory index of pending commands so the response can be built immediately, while persistence and UI processing remain outside callback context.

---

## 14. Home Assistant Integration

Home Assistant integration is optional for the first implementation. The gateway remains the authority for sensor OTA desired state.

A future Home Assistant integration may expose:

- Current firmware version
- Desired firmware version
- Update available state
- OTA state
- Last update attempt
- Last error
- Schedule update action
- Cancel update action

Possible flow:

```text
Home Assistant action
  -> gateway API or MQTT command
  -> gateway persists OTA_PENDING
  -> sensor wakes and sends HELLO
  -> gateway sends COMMAND_OTA
  -> sensor downloads from firmware server
  -> sensor reports new version
  -> gateway publishes SUCCESS to Home Assistant
```

Home Assistant should not assume that scheduling means immediate installation. Sleeping nodes update on their next successful wake and command exchange.

---

## 15. Observability and Diagnostics

Use compact numeric error codes on constrained devices and translate them into readable messages at the gateway.

Suggested events:

```text
OTA_SCHEDULED
OTA_OFFERED
OTA_ACCEPTED
OTA_WIFI_CONNECT_FAILED
OTA_DNS_FAILED
OTA_HTTP_FAILED
OTA_TIMEOUT
OTA_SIZE_MISMATCH
OTA_DIGEST_MISMATCH
OTA_SIGNATURE_INVALID
OTA_INCOMPATIBLE_IMAGE
OTA_FLASH_FAILED
OTA_REBOOTED
OTA_VERSION_CONFIRMED
OTA_CANCELLED
```

Useful sensor report fields:

```text
last_ota_id
last_ota_result
last_ota_error
ota_attempt_count
reset_reason
```

Avoid excessive serial output in battery mode. Make detailed logging compile-time configurable.

---

## 16. Suggested Code Organization

Keep platform-independent protocol and state logic separate from ESP8266/ESP32 adapters.

```text
src/
  protocol/
    ManagementProtocol.h
    ManagementProtocol.cpp
    PacketCodec.h
    PacketCodec.cpp
  ota/
    OtaStateMachine.h
    OtaStateMachine.cpp
    OtaPersistence.h
    FirmwareVerifier.h
  platform/
    esp8266/
      Esp8266OtaUpdater.cpp
      Esp8266Persistence.cpp
      Esp8266WiFiAdapter.cpp
    esp32/
      Esp32OtaUpdater.cpp
      Esp32Persistence.cpp
      Esp32WiFiAdapter.cpp
  sensor/
    SensorWakeCoordinator.cpp
  gateway/
    DeviceRegistry.cpp
    OtaRegistry.cpp
    CommandDispatcher.cpp
```

Suggested interfaces:

```cpp
class IOtaUpdater {
public:
    virtual OtaResult install(const OtaArtifact& artifact) = 0;
};

class IOtaPersistence {
public:
    virtual bool load(PersistentBootState& state) = 0;
    virtual bool save(const PersistentBootState& state) = 0;
    virtual bool clear() = 0;
};

class IManagementTransport {
public:
    virtual bool sendHello(const HelloMessage& message) = 0;
    virtual bool takePendingCommand(CommandMessage& command) = 0;
};
```

The AI coding agent should adapt naming and structure to the existing repository rather than creating duplicate abstractions unnecessarily.

---

## 17. Implementation Phases

### Phase 1: Protocol and gateway pending state

- Add firmware version, hardware model, boot ID, and sequence number to sensor `HELLO` or normal status packet.
- Add persistent per-device `desired_version`, `ota_id`, and `artifact_id` on the gateway.
- Add `COMMAND_NONE` and `COMMAND_OTA` responses.
- Replace continuous 100 ms transmission with response-on-wake behavior.
- Add duplicate and stale-response rejection.

### Phase 2: Sensor OTA boot state

- Implement versioned persistent boot state.
- Implement `NORMAL`, `OTA_REQUESTED`, `OTA_IN_PROGRESS`, and failure handling.
- Ensure a failed OTA attempt returns to normal sensor mode.
- Add bounded retry behavior and diagnostics.

### Phase 3: Local HTTP firmware proof of concept

- Host one known test image on the Home Assistant server.
- Have the sensor enter OTA mode, connect to Wi-Fi, download, install, restart, and report the new version.
- Initially test on a bench-accessible device with serial recovery available.

### Phase 4: Artifact manifest and validation

- Add immutable artifact IDs.
- Add size, hardware model, version, and digest metadata.
- Generate metadata during the firmware build/publish process.
- Confirm what validation is actually performed by the selected ESP8266 and ESP32 update libraries.

### Phase 5: Security hardening

- Add signed firmware or signed metadata.
- Add embedded public-key verification.
- Enable ESP-NOW peer security where suitable.
- Harden command parsing and authorization.

### Phase 6: Home Assistant/UI integration

- Show current and desired firmware versions.
- Add schedule and cancel actions.
- Show sleeping/pending semantics clearly.
- Publish success/failure and last-seen state through MQTT or the gateway API.

### Phase 7: Fleet and recovery testing

- Test many sensors waking concurrently.
- Test gateway restart with pending updates.
- Test firmware server outage.
- Test power loss at every OTA state transition.
- Test corrupted, truncated, oversized, wrong-model, and old firmware images.

---

## 18. Acceptance Tests

The implementation is complete only when the following tests pass.

### Normal operation

- A sensor with no pending command wakes, sends data, and sleeps with minimal added awake time.
- Missing command responses never leave the sensor awake indefinitely.
- Normal sensing continues after a failed OTA attempt.

### Scheduling behavior

- An OTA can be scheduled while the sensor is asleep or powered off.
- The request survives a gateway reboot.
- The sensor receives the update command on a later wake without gateway command bombardment.
- Duplicate OTA commands do not create duplicate transactions.

### Protocol correctness

- A response with the wrong device ID, boot ID, sequence, protocol version, or hardware model is rejected.
- Malformed and oversized frames are rejected safely.
- A delayed command from a previous boot is ignored.

### OTA success

- The sensor persists OTA state before rebooting.
- It connects to Wi-Fi and obtains the exact scheduled artifact.
- It validates size, model, version, and available integrity/authenticity controls.
- It installs and reboots into the new firmware.
- The new firmware reports its version.
- The gateway then marks the transaction successful and stops offering it.

### OTA failure

- Wi-Fi unavailable: timeout and return to normal mode.
- Firmware server unavailable: timeout and return to normal mode.
- Wrong model: reject without flashing.
- Oversized image: reject without flashing.
- Corrupt image/digest mismatch: reject without flashing.
- Invalid signature: reject without flashing.
- Power loss before download: recover without an endless loop.
- Power loss during download: recover according to verified platform behavior.
- Repeated failure reaches the configured retry limit and remains diagnosable.

### Version rules

- Older firmware is rejected by default.
- Same-version reinstall is rejected by default.
- A deliberate forced reinstall/downgrade works only when explicitly enabled.

### Concurrency

- Multiple sensors can have independent pending OTA transactions.
- A sensor can never receive another sensor's artifact.
- Simultaneous wakes do not corrupt the gateway registry or response queue.

---

## 19. Decisions to Keep Configurable

The following values should be configuration constants or hardware-profile settings rather than hard-coded throughout the code:

```text
protocol version
firmware server base URL
Wi-Fi connection timeout
HTTP operation timeout
overall OTA deadline
fast-sensor command window
maximum firmware size
maximum OTA attempts
retry/backoff policy
whether HTTPS is required
whether signature verification is required
whether forced reinstall/downgrade is allowed
```

---

## 20. Explicit Non-Goals for the First Version

To keep the first implementation reliable and manageable, exclude these unless already required:

- Firmware transfer over ESP-NOW
- Gateway proxying of the firmware binary
- Continuous OTA command broadcasting
- Mandatory live update while the sensor is sleeping
- Multiple simultaneous firmware downloads by one sensor
- Remote Internet exposure of the firmware repository
- Claimed automatic rollback without verified platform support
- Full Home Assistant UI before the core OTA proof of concept works

---

## 21. AI Agent Implementation Instructions

When implementing this design, the coding agent must first inspect the existing project and determine:

1. Arduino core, PlatformIO, ESP-IDF, or other framework and exact versions.
2. Current ESP-NOW initialization, callbacks, peer registration, send queue, and channel management.
3. Existing sensor wake/deep-sleep lifecycle.
4. Existing packet serialization and maximum payload assumptions.
5. Existing persistence mechanism and flash layout.
6. Existing version representation and device identity.
7. Existing gateway database/configuration storage.
8. Available ESP8266/ESP32 OTA APIs and their actual digest/signature behavior.
9. Watchdog requirements and callback-context restrictions.
10. Whether the firmware binary size fits the configured OTA flash layout.

The agent must not:

- Perform flash writes or a restart inside the ESP-NOW receive callback.
- Add long blocking delays to the normal wake path.
- Treat ESP-NOW send success as application-level acceptance.
- Clear gateway pending state merely because a command was sent.
- Trust an unvalidated artifact URL or hardware model.
- Assume rollback, SHA-256 verification, or signature checking is provided automatically by the framework.
- Use floating-point firmware versions.
- Introduce unbounded retry loops.
- Break the existing ESP-NOW/Wi-Fi channel arrangement in normal gateway operation.

The agent should implement feature flags so the new protocol and HTTP OTA can be bench-tested incrementally.

---

## 22. Final End-to-End Sequence

```text
User/Home Assistant
    -> schedules desired firmware on gateway

Gateway
    -> persists PENDING transaction

Sensor later wakes
    -> initializes ESP-NOW
    -> sends HELLO with current version, hardware model, boot ID, and sequence
    -> continues useful sensor work

Gateway
    -> finds pending transaction
    -> sends correlated COMMAND_OTA with OTA ID and artifact metadata

Sensor callback
    -> validates basic frame
    -> copies command and sets a flag

Sensor main flow
    -> fully validates command
    -> persists OTA_REQUESTED
    -> optionally sends OTA_ACCEPTED
    -> restarts

Sensor OTA boot
    -> marks OTA_IN_PROGRESS
    -> connects to normal Wi-Fi
    -> downloads immutable artifact from Home Assistant firmware server
    -> validates model, version, size, digest, and signature as implemented
    -> installs image
    -> restarts

New firmware
    -> boots normally
    -> sends HELLO reporting new version

Gateway
    -> compares current version with desired version
    -> marks OTA SUCCESS
    -> clears pending command
    -> publishes status to UI/MQTT/Home Assistant
```

This is the finalized approach: **persistent desired state at the gateway, discovery on sensor wake, ESP-NOW for control, and Wi-Fi HTTP/HTTPS for firmware distribution.**
