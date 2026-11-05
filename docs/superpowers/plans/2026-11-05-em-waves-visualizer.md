# EM Waves Visualizer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** An ESP32-S3 USB sensor plus a WebXR web app that renders one Wi-Fi router's measured signal as a false-color volumetric glow in AR.

**Architecture:** Arduino-ESP32 firmware streams line-delimited JSON (protocol v1) over native USB CDC. A Vite/TypeScript static site reads it via Web Serial (desktop) or WebUSB + `web-serial-polyfill` (Android), smooths RSSI, models free-space falloff, and draws an analytic line-of-sight glow with a three.js full-screen shader in a WebXR `immersive-ar` session. Pure logic is isolated in small modules tested with Vitest; DOM flows are tested with Playwright against a fake sensor.

**Tech Stack:** C++ / Arduino-ESP32 / PlatformIO (Unity tests, `native` env); TypeScript (strict), Vite 6, three.js 0.170, Vitest 3, Playwright 1.49+, `web-serial-polyfill`, ESP Web Tools 10; GitHub Actions + GitHub Pages.

**Spec:** `docs/superpowers/specs/2026-11-05-em-waves-visualizer-design.md`

## Global Constraints

- Protocol v1, one JSON object per line, `\n`-terminated, both directions text. `PROTOCOL_VERSION = 1`, firmware version `"1.0.0"`.
- Commands: `HELLO`, `SCAN`, `TRACK <bssid> <channel>`, `STOP`. BSSID format `aa:bb:cc:dd:ee:ff` (lowercase hex in output, case-insensitive in input). Channels 1–13 only.
- RSSI stream: 50 ms windows (~20 Hz); empty windows sent as `{"type":"rssi","rssi":null,"n":0,"t":…}`.
- Firmware rescans for the tracked BSSID after 3000 ms without packets.
- `f = 2407 + 5·ch` MHz; `c = 299792458` m/s; channel 6 → 2437 MHz → 0.1230 m.
- `P(d) = P1m − 20·log10(max(d, 0.2))`; default `P1m = −40` dBm; `k = 10^((measured − expected)/10)` clamped to `[0.05, 2]`.
- Hue: `(ch − 1) / 12 · 270` degrees (ch 1 → 0° red, ch 13 → 270° violet).
- Smoother: EMA time constant 300 ms; "no signal" after 3000 ms without a non-null sample.
- Glow: ray segment `t ∈ [0, 10]` m; `E = (atan(t1/d) − atan(t0/d)) / d`, `d` clamped ≥ 0.01 m; tone map `1 − exp(−x)`.
- WebXR: `immersive-ar`, required feature `hit-test`; optional `anchors`, `dom-overlay`.
- Site base path `/em-waves-visualizer/` (GitHub Pages, HTTPS). Pages: `index.html`, `sensor-test.html`, `install.html`.
- Dev/test fake sensor: `?fake=<scenario>` with scenarios `1`/`ok`, `badversion`, `nosignal`, `disconnect`, `channelchange`, `malformed`.
- Node ≥ 20. All web commands run from `web/`; all firmware commands from `firmware/`.

## Review Focus

1. **SSIDs containing quotes, backslashes, control chars or UTF-8** — firmware must emit valid JSON; app shows the name verbatim. Test: Task 1 `test_format_ap_escapes`.
2. **Serial chunks that split a line mid-JSON or carry several lines** — no message lost or mangled. Test: Task 7 `LineFramer` tests.
3. **Phone at (or inside) the anchor, d → 0** — no NaN/Infinity in the model or shader. Tests: Task 5 `expectedRssi clamps`, Task 6 `finite at d=0`.
4. **Two APs with the same SSID (mesh/dual routers) and hidden SSIDs** — listed separately, distinguishable by BSSID. Test: Task 8 `dedupes by bssid, keeps same-ssid APs`; Task 9 e2e picker test.
5. **Scan finds zero networks / sensor never answers HELLO** — user gets a message, not a spinner forever. Tests: Task 8 `hello timeout`, `empty scan`.

---

## Part A — Firmware

### Task 1: Firmware core library (pure C++) with native tests

**Files:**
- Create: `firmware/platformio.ini`
- Create: `firmware/lib/core/Protocol.h`, `firmware/lib/core/Protocol.cpp`
- Create: `firmware/lib/core/RssiWindow.h`, `firmware/lib/core/RssiWindow.cpp`
- Test: `firmware/test/test_core/test_main.cpp`
- Create: `.gitignore` (root: `.pio/`, `node_modules/`, `dist/`, `test-results/`, `playwright-report/`, `.claude/`)

**Interfaces:**
- Produces:
  - `enum class CmdKind { Hello, Scan, Track, Stop, Invalid };`
  - `struct Command { CmdKind kind; uint8_t bssid[6]; int channel; };`
  - `Command parseCommand(const std::string& line);` — trims whitespace/CR; `TRACK` requires a valid BSSID and channel 1–13, else `Invalid`.
  - `std::string formatHello(const char* fw, int proto);`
  - `std::string formatAp(const std::string& ssid, const uint8_t bssid[6], int channel, int rssi);`
  - `std::string formatScanDone(int count);`, `formatTracking(const uint8_t bssid[6], int channel)`, `formatRssi(bool has, int mean, int n, uint32_t t)`, `formatChannel(int channel)`, `formatStopped()`, `formatError(const std::string& msg)` — all return one line **without** trailing newline.
  - `class RssiWindow { void add(int rssi); struct Result { bool has; int mean; int n; }; Result flush(); };` — `flush()` returns rounded mean and resets.

- [ ] **Step 1: Create `platformio.ini`** with envs `esp32s3` (`platform = espressif32`, `board = esp32-s3-devkitc-1`, `framework = arduino`, `build_flags = -DARDUINO_USB_CDC_ON_BOOT=1 -DARDUINO_USB_MODE=1`, `monitor_speed = 115200`) and `native` (`platform = native`, `test_build_src = no`, `build_flags = -std=c++17`).

- [ ] **Step 2: Write failing tests** in `test/test_core/test_main.cpp` (Unity):

```cpp
test_parse_hello_scan_stop:   parseCommand("HELLO\r").kind == Hello; "SCAN" → Scan; "  STOP " → Stop
test_parse_track_valid:       parseCommand("TRACK AA:bb:cc:dd:ee:0F 6") → Track, bssid {0xaa,0xbb,0xcc,0xdd,0xee,0x0f}, channel 6
test_parse_track_invalid:     "TRACK aa:bb 6", "TRACK aa:bb:cc:dd:ee:ff 14", "TRACK aa:bb:cc:dd:ee:ff 0", "TRACK aa:bb:cc:dd:ee:ff", "FOO", "" → Invalid
test_format_hello:            formatHello("1.0.0",1) == R"({"type":"hello","fw":"1.0.0","proto":1})"
test_format_ap:               formatAp("School",{aa..ff},6,-52) == R"({"type":"ap","ssid":"School","bssid":"aa:bb:cc:dd:ee:ff","channel":6,"rssi":-52})"
test_format_ap_escapes:       ssid "a\"b\\c\x01é" → ssid field is "a\"b\\c\u0001é" (UTF-8 bytes ≥0x80 passed through, <0x20 as \u00XX)
test_format_rssi:             formatRssi(true,-48,7,123456) == R"({"type":"rssi","rssi":-48,"n":7,"t":123456})"; formatRssi(false,0,0,5) == R"({"type":"rssi","rssi":null,"n":0,"t":5})"
test_format_misc:             formatScanDone(3), formatTracking, formatChannel(11) == R"({"type":"channel","channel":11})", formatStopped() == R"({"type":"stopped"})", formatError("bad command") == R"({"type":"error","msg":"bad command"})"
test_window_mean_and_reset:   add(-50), add(-47) → flush() == {true,-49,2} (−48.5 rounds away from zero to −49); next flush() == {false,0,0}
```

- [ ] **Step 3: Run** `pio test -e native` — expected: FAIL (undefined references).

- [ ] **Step 4: Implement** `Protocol.cpp` and `RssiWindow.cpp`. Use only `<string>`, `<cstdint>`, `<cstdio>`, `<cmath>` so the library builds on both envs; JSON is hand-formatted with an `escapeJson()` helper.

- [ ] **Step 5: Run** `pio test -e native` — expected: all tests PASS.

- [ ] **Step 6: Commit** `feat(firmware): protocol formatting, command parsing, rssi window`

### Task 2: Firmware radio and serial loop

**Files:**
- Create: `firmware/src/main.cpp`

**Interfaces:**
- Consumes: everything in Task 1.
- Produces: the v1 protocol on USB CDC (`Serial`), as specified in Global Constraints.

- [ ] **Step 1: Implement `main.cpp`:**
  - `setup()`: `Serial.begin(115200)`; `WiFi.mode(WIFI_STA)`; `WiFi.disconnect()`.
  - Line reader: accumulate `Serial` bytes to `\n` (cap 128 chars, discard overflow), `parseCommand`, dispatch; `Invalid` → `formatError("bad command")`.
  - `HELLO` → `formatHello("1.0.0", 1)`.
  - `SCAN`: stop tracking if active; `WiFi.scanNetworks(false, true)`; print one `formatAp` per result with channel 1–13; then `formatScanDone(count printed)`.
  - `TRACK`: store target BSSID; `esp_wifi_set_promiscuous(false)`; `esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE)`; filter `WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA`; register callback; `esp_wifi_set_promiscuous(true)`; print `formatTracking`.
  - Promiscuous callback: compare `addr2` (payload bytes 10–15) with target; on match `window.add(pkt->rx_ctrl.rssi)` and record `lastPacketMs`, under a `portMUX_TYPE` critical section.
  - Every 50 ms while tracking: `flush()` under the mutex and print `formatRssi(has, mean, n, millis())`.
  - If `millis() − lastPacketMs > 3000`: disable promiscuous, `WiFi.scanNetworks(false, true)`, find target BSSID; if found on a different channel, switch and print `formatChannel`; re-enable promiscuous; reset `lastPacketMs = millis()` (whether found or not, so rescans happen at most every 3 s).
  - `STOP` → disable promiscuous, print `formatStopped()`.

- [ ] **Step 2: Build** `pio run -e esp32s3` — expected: `SUCCESS`.

- [ ] **Step 3: Manual verify on hardware** (`pio run -e esp32s3 -t upload`, then `pio device monitor`): `HELLO` → hello line; `SCAN` → ≥1 `ap` line then `scan_done`; `TRACK <your router bssid> <ch>` → `tracking`, then `rssi` lines at ~20/s with `n > 0`; `FOO` → `error`; `STOP` → `stopped`. Record results in the commit message.

- [ ] **Step 4: Commit** `feat(firmware): scan, promiscuous tracking, channel follow`

---

## Part B — Web app

### Task 3: Web scaffold + protocol module

**Files:**
- Create: `web/package.json`, `web/tsconfig.json`, `web/vite.config.ts`, `web/vitest.config.ts`
- Create: `web/index.html`, `web/sensor-test.html`, `web/install.html` (empty shells with `<main id="app">`)
- Create: `web/src/protocol.ts`
- Test: `web/test/protocol.test.ts`

**Interfaces:**
- Produces (`protocol.ts`):

```ts
export const PROTOCOL_VERSION = 1;
export type AccessPoint = { ssid: string; bssid: string; channel: number; rssi: number };
export type SensorMessage =
  | { type: 'hello'; fw: string; proto: number }
  | ({ type: 'ap' } & AccessPoint)
  | { type: 'scan_done'; count: number }
  | { type: 'tracking'; bssid: string; channel: number }
  | { type: 'rssi'; rssi: number | null; n: number; t: number }
  | { type: 'channel'; channel: number }
  | { type: 'stopped' }
  | { type: 'error'; msg: string };
export type Command =
  | { kind: 'HELLO' } | { kind: 'SCAN' } | { kind: 'STOP' }
  | { kind: 'TRACK'; bssid: string; channel: number };
export function parseLine(line: string): SensorMessage | null; // null on invalid JSON, unknown type, or wrong field types
export function buildCommand(cmd: Command): string;            // includes trailing "\n"
```

- [ ] **Step 1: Scaffold.** `package.json` scripts: `dev` (`vite`), `build` (`tsc --noEmit && vite build`), `test` (`vitest run`), `e2e` (`playwright test`). `vite.config.ts`: `base: '/em-waves-visualizer/'`, multi-page `build.rollupOptions.input` for the three HTML files. `tsconfig` strict. Install `three`, `web-serial-polyfill`; dev: `typescript`, `vite`, `vitest`, `@types/three`, `@playwright/test`, `@types/w3c-web-serial`, `@types/w3c-web-usb`, `@types/webxr`.

- [ ] **Step 2: Write failing tests** `test/protocol.test.ts`:
  - `parses each message type` — one line per spec example (e.g. `{"type":"rssi","rssi":null,"n":0,"t":5}` → `{type:'rssi', rssi:null, n:0, t:5}`).
  - `returns null for malformed input` — `'{"type":"rssi"'`, `'hello'`, `'{"type":"wat"}'`, `'{"type":"ap","ssid":1}'`, `''` all → `null`.
  - `keeps unicode and escaped ssid` — `{"type":"ap","ssid":"a\"b\\cé",…}` → `ssid === 'a"b\\cé'`.
  - `builds commands` — `buildCommand({kind:'TRACK', bssid:'aa:bb:cc:dd:ee:ff', channel:6}) === 'TRACK aa:bb:cc:dd:ee:ff 6\n'`; `HELLO\n`, `SCAN\n`, `STOP\n`.
  - `buildCommand rejects bad TRACK` — channel 0 / 14 / non-integer, or bssid not matching `/^([0-9a-f]{2}:){5}[0-9a-f]{2}$/i` → throws `RangeError`.

- [ ] **Step 3: Run** `npx vitest run test/protocol.test.ts` — expected FAIL (module not found).
- [ ] **Step 4: Implement** `protocol.ts`.
- [ ] **Step 5: Run** the same — expected PASS.
- [ ] **Step 6: Commit** `feat(web): scaffold and sensor protocol`

### Task 4: RSSI smoother

**Files:** Create `web/src/smoother.ts`; Test `web/test/smoother.test.ts`

**Interfaces:**
- Produces:

```ts
export class RssiSmoother {
  constructor(opts?: { tauMs?: number /* 300 */; noSignalMs?: number /* 3000 */ });
  push(rssi: number | null, nowMs: number): void;
  value(): number | null;            // null until first non-null sample
  hasSignal(nowMs: number): boolean; // false if no non-null sample within noSignalMs
  reset(): void;
}
```

- [ ] **Step 1: Failing tests:**
  - `first sample sets value` — push(−50, 0) → value() === −50.
  - `ema step` — push(−50,0), push(−40,300) → value ≈ −50 + 10·(1 − e^−1) = −43.68 (±0.01). Alpha is `1 − exp(−dt/tau)`.
  - `ignores null` — push(−50,0), push(null,50) → value −50.
  - `no signal after 3 s` — push(−50,0); hasSignal(2999) true; push(null, 2000); hasSignal(3001) false.
  - `reset` — value() null, hasSignal(any) false.
  - `converges` — 100 alternating pushes of −45/−55 every 50 ms → value within ±2 of −50.
- [ ] **Step 2: Run** `npx vitest run test/smoother.test.ts` — FAIL.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** — PASS.
- [ ] **Step 5: Commit** `feat(web): rssi smoother`

### Task 5: Wave model

**Files:** Create `web/src/waveModel.ts`; Test `web/test/waveModel.test.ts`

**Interfaces:**
- Produces:

```ts
export const SPEED_OF_LIGHT = 299_792_458;
export const DEFAULT_P1M = -40;
export function channelToFrequencyMHz(channel: number): number;
export function wavelengthMeters(freqMHz: number): number;
export function expectedRssi(p1m: number, distanceM: number): number;
export function calibrateP1m(measured: number, distanceM: number): number;
export function brightnessFactor(measured: number | null, expected: number): number; // null → 0
export function channelHueDeg(channel: number): number;
export function absorptionMessage(measured: number | null, expected: number): string | null;
```

`absorptionMessage` returns `` `${diff} dB: something is absorbing the signal` `` (diff rounded to integer, with sign, e.g. `"−12 dB…"` using U+2212) when `measured − expected ≤ −6`, otherwise `null`.

- [ ] **Step 1: Failing tests:**
  - `channel 6 → 2437 MHz → 0.1230 m` (wavelength to 4 decimals); `channel 1 → 2412`, `13 → 2472`.
  - `expectedRssi` — (−40, 1) → −40; (−40, 10) → −60; (−40, 2) ≈ −46.02.
  - `expectedRssi clamps` — (−40, 0) and (−40, 0.1) both equal (−40, 0.2) ≈ −26.02; never NaN/Infinity.
  - `calibrateP1m` — (−46.02, 2) ≈ −40; round-trip with `expectedRssi`.
  - `brightnessFactor` — equal → 1; −10 dB → 0.1; −20 dB → 0.05 (clamped); +10 dB → 2 (clamped); null → 0.
  - `channelHueDeg` — 1 → 0, 7 → 135, 13 → 270.
  - `absorptionMessage` — (−62, −50) → `"−12 dB: something is absorbing the signal"`; (−55, −50) → null; (null, −50) → null.
- [ ] **Step 2: Run** `npx vitest run test/waveModel.test.ts` — FAIL.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** — PASS.
- [ ] **Step 5: Commit** `feat(web): wave model`

### Task 6: Glow formula (TypeScript twin of the shader)

**Files:** Create `web/src/glow.ts`; Test `web/test/glow.test.ts`

**Interfaces:**
- Produces:

```ts
export type Vec3 = [number, number, number];
export const GLOW_MAX_DIST = 10;
export function lineOfSightEnergy(origin: Vec3, dir: Vec3 /* unit */, source: Vec3, maxDist?: number): number;
export function toneMap(x: number): number; // 1 - exp(-x)
```

Algorithm (the shader in Task 10 must match line for line):
```
tc = dot(source − origin, dir)            // closest-approach parameter
d  = max(|source − (origin + tc·dir)|, 0.01)
t0 = 0 − tc;  t1 = maxDist − tc
E  = (atan(t1/d) − atan(t0/d)) / d
```

- [ ] **Step 1: Failing tests:**
  - `matches numerical integration` — 200 seeded-random rays (simple LCG, seed 42), origins in a 4 m cube, sources ≥ 0.05 m from the ray; compare to midpoint-rule ∫₀¹⁰ 1/(d² + (t−tc)²) dt with 20 000 steps; relative error < 1e-3.
  - `finite at d=0` — source exactly on the ray → finite, > 0.
  - `behind camera is small` — source 5 m behind origin → E < 0.05 × E(source 5 m ahead, 0.5 m off-axis).
  - `toneMap` — 0 → 0; large → < 1 and → 1; monotonic on [0, 10].
- [ ] **Step 2: Run** `npx vitest run test/glow.test.ts` — FAIL.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** — PASS.
- [ ] **Step 5: Commit** `feat(web): analytic glow integral`

### Task 7: Sensor link (serial + fake) with integration tests

**Files:**
- Create: `web/src/sensorLink.ts`, `web/src/fakeSensor.ts`
- Test: `web/test/sensorLink.test.ts`, `web/test/helpers/fakeSerialPort.ts`

**Interfaces:**
- Consumes: `parseLine`, `buildCommand`, `Command`, `SensorMessage` (Task 3).
- Produces:

```ts
export type LinkStatus = 'connected' | 'disconnected';
export interface SensorSource {
  connect(): Promise<void>;                 // may prompt for a device
  send(cmd: Command): Promise<void>;
  onMessage(cb: (m: SensorMessage) => void): () => void;
  onStatus(cb: (s: LinkStatus) => void): () => void;
  readonly malformedCount: number;
  close(): Promise<void>;
}
export class LineFramer { push(chunk: string): string[] } // returns complete lines, buffers the rest; drops lines > 4096 chars
export interface SerialPortLike {
  open(o: { baudRate: number }): Promise<void>;
  readonly readable: ReadableStream<Uint8Array> | null;
  readonly writable: WritableStream<Uint8Array> | null;
  close(): Promise<void>;
}
export interface SerialEnv {
  requestPort(): Promise<SerialPortLike>;
  onPortConnected(cb: (p: SerialPortLike) => void): void; // fires when a previously granted port is replugged
}
export class SerialSensorSource implements SensorSource { constructor(env: SerialEnv) }
export function browserSerialEnv(): SerialEnv | null; // navigator.serial on desktop; web-serial-polyfill over navigator.usb on Android; null if neither
// fakeSensor.ts
export type FakeScenario = 'ok' | 'badversion' | 'nosignal' | 'disconnect' | 'channelchange' | 'malformed';
export function parseFakeScenario(search: string): FakeScenario | null; // "?fake=1" → 'ok'
export class FakeSensorSource implements SensorSource {
  constructor(scenario: FakeScenario, opts?: { intervalMs?: number /*50*/ });
  readonly sentCommands: Command[]; // for tests
}
export const FAKE_APS: AccessPoint[];
```

`browserSerialEnv` on Android: `onPortConnected` uses `navigator.usb` `connect` events and the polyfill's `getPorts()`. Exact polyfill wiring is the implementer's choice; it is verified manually in Task 12.

`FakeSensorSource` behavior (timings relative to `TRACK`):
- `HELLO` → `{hello, fw:'1.0.0', proto:1}` (`badversion`: `proto: 99`).
- `SCAN` → `FAKE_APS` = `School` `aa:aa:aa:aa:aa:01` ch 6 −45; `School` `aa:aa:aa:aa:aa:02` ch 11 −60; hidden `""` `aa:aa:aa:aa:aa:03` ch 1 −70; then `scan_done`.
- `TRACK` → `tracking`, then `rssi` every `intervalMs`, value −50 ± 3 (deterministic sine wobble).
- `nosignal`: after 2 s only `rssi: null, n: 0`.
- `disconnect`: after 2 s status `disconnected` and no messages; after another 2 s status `connected` (consumer must re-send HELLO/TRACK).
- `channelchange`: after 2 s emits `{type:'channel', channel: 11}`.
- `malformed`: interleaves a garbage line every 5th message (goes through `parseLine`, increments `malformedCount`).

- [ ] **Step 1: Write `test/helpers/fakeSerialPort.ts`** — a `SerialPortLike` backed by in-memory streams with `emit(text: string)` (enqueues raw chunks), `written(): string` (decoded writes), and `unplug()` (errors the readable stream).

- [ ] **Step 2: Failing tests** `test/sensorLink.test.ts`:
  - `LineFramer splits and buffers` — push(`'{"a":1}\n{"b"'`) → `['{"a":1}']`; push(`':2}\n\n'`) → `['{"b":2}']` (empty lines skipped); `\r\n` handled.
  - `LineFramer drops overlong line` — 5000 chars without newline, then `'x\n'` → returns `[]` then resumes correctly on the next line.
  - `serial: delivers messages across chunk boundaries` — emit a `hello` line split in 3 chunks → one `hello` message.
  - `serial: counts malformed` — emit `'garbage\n'` → `malformedCount === 1`, no message.
  - `serial: send writes command` — `send({kind:'SCAN'})` → `written() === 'SCAN\n'`.
  - `serial: unplug → disconnected; replug → connected` — `unplug()` fires `'disconnected'`; invoking the env's `onPortConnected` callback with a new fake port fires `'connected'` and messages from the new port are delivered.
  - `fake: parseFakeScenario` — `'?fake=1'` → `'ok'`, `'?fake=nosignal'` → `'nosignal'`, `''` / `'?fake=zzz'` → `null`.
  - `fake: ok scenario streams rssi` (Vitest fake timers) — HELLO, TRACK, advance 500 ms → ≥ 9 `rssi` messages with numeric `rssi`.
- [ ] **Step 3: Run** `npx vitest run test/sensorLink.test.ts` — FAIL.
- [ ] **Step 4: Implement** `sensorLink.ts`, `fakeSensor.ts`.
- [ ] **Step 5: Run** — PASS.
- [ ] **Step 6: Commit** `feat(web): serial and fake sensor sources`

### Task 8: Sensor session (connect → hello → scan → track, recovery)

**Files:** Create `web/src/session.ts`; Test `web/test/session.test.ts`

(Spec lists this logic under `networkPicker.ts`; it is split so the DOM in Task 9 stays thin and the flow is testable.)

**Interfaces:**
- Consumes: `SensorSource`, `FakeSensorSource` (Task 7); `RssiSmoother` (Task 4); `AccessPoint`, `PROTOCOL_VERSION` (Task 3).
- Produces:

```ts
export type SessionError = 'nosensor' | 'nofirmware' | 'badversion' | 'noaps';
export type SessionState = {
  phase: 'idle' | 'connecting' | 'scanning' | 'picking' | 'tracking';
  error: SessionError | null;
  connected: boolean;
  aps: AccessPoint[];           // sorted by rssi desc, unique by bssid
  selected: AccessPoint | null; // channel kept up to date by 'channel' messages
  rssi: number | null;          // smoothed
  hasSignal: boolean;
  malformedCount: number;
};
export class SensorSession {
  constructor(source: SensorSource, opts?: { now?: () => number; helloTimeoutMs?: number /*2000*/; scanTimeoutMs?: number /*10000*/ });
  readonly state: SessionState;
  subscribe(cb: (s: SessionState) => void): () => void; // called immediately and on every change (rssi updates throttled to 10 Hz)
  connect(): Promise<void>;   // connect + HELLO; sets error 'nosensor' | 'nofirmware' | 'badversion'
  scan(): Promise<void>;      // phase 'scanning' → 'picking'; empty → error 'noaps'
  track(ap: AccessPoint): Promise<void>; // phase 'tracking'
  stop(): Promise<void>;
}
```

On status `disconnected`: `connected = false`, `hasSignal = false`, phase unchanged. On `connected` while phase is `tracking`: re-send HELLO, then `TRACK` for `selected` automatically.

- [ ] **Step 1: Failing tests** (use `FakeSensorSource` with fake timers, plus a scripted `SensorSource` stub where noted):
  - `happy path` — connect → scan → `aps.map(a=>a.bssid)` equals `['aa:aa:aa:aa:aa:01','aa:aa:aa:aa:aa:02','aa:aa:aa:aa:aa:03']`; track(aps[0]) → phase `tracking`, `hasSignal` true and `rssi` within −50 ± 3 after 1 s.
  - `dedupes by bssid, keeps same-ssid APs` — stub emits the same bssid twice (−70, then −40) plus another `School` → 2 entries, the duplicate keeps −40, both `School` present.
  - `badversion` — `badversion` scenario → `error === 'badversion'`, phase `idle`.
  - `hello timeout` — stub that never answers → after 2000 ms `error === 'nofirmware'`.
  - `connect rejected` — stub whose `connect()` rejects → `error === 'nosensor'`.
  - `empty scan` — stub returns `scan_done` with count 0 → `error === 'noaps'`.
  - `nosignal` — after 2 s + 3 s → `hasSignal === false`.
  - `disconnect recovers` — `disconnect` scenario: at 2.5 s `connected === false`; at 5 s `connected === true`, `hasSignal === true`, and the stub/fake received a second `TRACK`.
  - `channelchange` — after 2.1 s `selected.channel === 11`.
  - `malformed` — session keeps tracking, `malformedCount > 0`.
- [ ] **Step 2: Run** `npx vitest run test/session.test.ts` — FAIL.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `npx vitest run` (all) — PASS.
- [ ] **Step 5: Commit** `feat(web): sensor session flow and recovery`

### Task 9: Pages, capability check, picker, HUD, sensor-test — with Playwright

**Files:**
- Create: `web/src/capabilities.ts`, `web/src/networkPicker.ts`, `web/src/hud.ts`, `web/src/app.ts`, `web/src/sensorTestPage.ts`, `web/src/installPage.ts`, `web/src/styles.css`
- Modify: `web/index.html`, `web/sensor-test.html`, `web/install.html`
- Create: `web/playwright.config.ts`
- Test: `web/e2e/app.spec.ts`, `web/e2e/sensorTest.spec.ts`, `web/e2e/install.spec.ts`

**Interfaces:**
- Consumes: `SensorSession` (Task 8); `browserSerialEnv`, `SerialSensorSource`, `FakeSensorSource`, `parseFakeScenario` (Task 7); `waveModel` (Task 5).
- Produces:

```ts
// capabilities.ts
export type Capabilities = { ar: boolean; sensor: boolean };
export async function detectCapabilities(nav?: Navigator): Promise<Capabilities>; // ar = await navigator.xr?.isSessionSupported('immersive-ar'); sensor = !!navigator.serial || !!navigator.usb
// hud.ts
export type HudView = {
  ssid: string; bssid: string; channel: number;
  measured: number | null; expected: number | null; distanceM: number | null;
  calibrated: boolean;
  banner: 'none' | 'nosignal' | 'disconnected' | 'trackinglost';
};
export function createHud(root: HTMLElement, handlers: { onCalibrate(): void; onRescan(): void; onReplace(): void }): { update(v: HudView): void; element: HTMLElement };
// networkPicker.ts
export function mountNetworkPicker(root: HTMLElement, session: SensorSession, onStartAr: () => void): void;
// app.ts — entry for index.html: builds the source (fake if ?fake=…, else serial), session, picker, HUD; Task 10 adds AR.
```

UI copy and test hooks (Playwright selects by role/text and these `data-testid`s):
- No sensor API (and no `?fake`): heading **"Unsupported browser"**, text "Needs Chrome on an ARCore-supported Android phone. On desktop Chrome, use the sensor test page." with link to `sensor-test.html`.
- No AR but sensor present: picker works; "Start AR" is replaced by text "AR not available on this device" and a link to the sensor test page.
- Picker: button **"Connect sensor"** with hint "Use a USB-C OTG adapter; your browser will ask you to choose the device."; list `data-testid="ap-list"` of buttons, each showing SSID (or `Hidden network`) + BSSID + channel + rssi dBm; button **"Start AR"** enabled after `track()`.
- Errors: `badversion`/`nofirmware` → "Sensor firmware missing or outdated" + link **"Install firmware"** (`install.html`); `noaps` → "No 2.4 GHz networks found" + **"Rescan"**.
- HUD (`data-testid="hud"`, visible from tracking onward, later reused as the WebXR DOM overlay root): `data-testid="hud-network"` (`School · ch 6`), `hud-wavelength` (`λ 12.3 cm`), `hud-measured` (`Measured −50 dBm`), `hud-expected`, `hud-diff` (`absorptionMessage` text or empty), `hud-distance`, hue legend with the text "Like your eyes turn light frequency into color, this turns Wi-Fi frequency into color."
  Banners (`data-testid="banner"`): `nosignal` → "No signal from School" + **"Rescan"**; `disconnected` → "Sensor disconnected – replug"; `trackinglost` → "Move slowly / point at textured surfaces"; uncalibrated → "Stand ~1 m from router and tap Calibrate".
- Sensor test page: same connect/pick flow, then a `<canvas data-testid="rssi-graph">` scrolling the last 30 s and `data-testid="rssi-value"` text.
- Install page: `<esp-web-install-button manifest="firmware/manifest.json">` (ESP Web Tools 10 via `https://unpkg.com/esp-web-tools@10/dist/web/install-button.js?module`), plus wiring instructions.

- [ ] **Step 1: `playwright.config.ts`** — `webServer: { command: 'npm run dev -- --port 5173', url: 'http://localhost:5173/em-waves-visualizer/' }`, `baseURL` same; projects `desktop-chromium` (Desktop Chrome) and `pixel-7` (devices['Pixel 7']). Shared helper `mockXr(page, supported: boolean)` via `addInitScript` defining `navigator.xr.isSessionSupported`.

- [ ] **Step 2: Failing e2e tests** `e2e/app.spec.ts`:
  - `unsupported browser` — `addInitScript` deletes `navigator.serial`, `navigator.usb`, `navigator.xr`; goto `/` → heading "Unsupported browser" visible.
  - `picker flow` — `mockXr(true)`, goto `?fake=1`; click "Connect sensor"; `ap-list` has 3 items in order School (−45), School (−60), Hidden network; both School items show distinct BSSIDs; click first → "Start AR" enabled; `hud-network` = `School · ch 6`; `hud-wavelength` = `λ 12.3 cm`; `hud-measured` matches `/Measured −(4[7-9]|5[0-3]) dBm/`.
  - `no AR` — `mockXr(false)`, `?fake=1` → after tracking, text "AR not available on this device".
  - `badversion` — `?fake=badversion` → "Sensor firmware missing or outdated" and link to `install.html`.
  - `nosignal` — `?fake=nosignal`, track → within 7 s banner "No signal from School" and "Rescan" button.
  - `disconnect` — `?fake=disconnect`, track → banner "Sensor disconnected – replug" appears, then disappears within 6 s and `hud-measured` shows a number again.
  - `channelchange` — `?fake=channelchange`, track → `hud-network` becomes `School · ch 11`, `hud-wavelength` `λ 12.2 cm`.
  - `malformed` — `?fake=malformed`, track for 3 s → no page errors (`page.on('pageerror')` count 0), `hud-measured` numeric.
  `e2e/sensorTest.spec.ts`: `graph updates` — `sensor-test.html?fake=1`, connect, pick first → `rssi-value` text changes within 2 s; canvas non-empty (`toDataURL` differs from a blank canvas).
  `e2e/install.spec.ts`: `install button present` — `install.html` has `esp-web-install-button` with `manifest="firmware/manifest.json"`; link back to app.

- [ ] **Step 3: Run** `npx playwright install chromium` then `npx playwright test` — FAIL.
- [ ] **Step 4: Implement** modules and pages. HUD formatting uses `waveModel` (wavelength rounded to 0.1 cm, dBm integers with U+2212 minus). In non-AR mode `distanceM`/`expected` show "—".
- [ ] **Step 5: Run** `npx playwright test` and `npx vitest run` — all PASS in both projects.
- [ ] **Step 6: Commit** `feat(web): pages, picker, hud, sensor test, e2e tests`

### Task 10: AR session and glow renderer

**Files:**
- Create: `web/src/arSession.ts`, `web/src/waveRenderer.ts`, `web/src/glow.glsl.ts`
- Modify: `web/src/app.ts` (wire "Start AR")

**Interfaces:**
- Consumes: `lineOfSightEnergy` algorithm (Task 6), `toneMap`; `waveModel` (Task 5); `createHud` (Task 9); `SensorSession` (Task 8).
- Produces:

```ts
// arSession.ts
export type Pose = { position: [number, number, number] };
export interface ArController {
  start(overlayRoot: HTMLElement): Promise<void>; // requestSession('immersive-ar', { requiredFeatures:['hit-test'], optionalFeatures:['anchors','dom-overlay'], domOverlay:{ root } })
  routerPose(): Pose | null;
  viewerPose(): Pose | null;
  trackingLost(): boolean;
  replace(): void;                                // next tap re-anchors
  onFrame(cb: (t: number, frame: XRFrame) => void): void;
  end(): Promise<void>;
}
export function createArController(renderer: THREE.WebGLRenderer): ArController;
// waveRenderer.ts
export function createWaveRenderer(renderer: THREE.WebGLRenderer): {
  render(camera: THREE.Camera, source: [number, number, number] | null, params: { hueDeg: number; k: number; gain: number; greyed: boolean }): void;
};
```

- [ ] **Step 1: Implement `glow.glsl.ts`** — exports the fragment shader string. It reconstructs the per-pixel world ray from `inverse(projectionMatrix·viewMatrix)` uniforms, then computes `E` exactly as in Task 6 (same constants `0.01`, `10.0`). Color `hsv(hueDeg, 0.9, 1)`; output `rgb = color · toneMap(E · k · gain)`, `alpha = toneMap(…)·0.85`; additive blending; `greyed` → saturation 0 and ×0.4.
- [ ] **Step 2: Implement `waveRenderer.ts`** — one full-screen triangle with `ShaderMaterial` (`depthTest:false`, `transparent:true`, `blending: AdditiveBlending`), skipped when `source` is null. `gain` default 1.5.
- [ ] **Step 3: Implement `arSession.ts`** — hit-test source from viewer space; `select` event → create anchor at hit (via `anchors` if granted, else store the pose); no hit → pose 1 m along viewer forward. `trackingLost()` = no viewer pose or `emulatedPosition` true.
- [ ] **Step 4: Wire in `app.ts`** per frame: `distance = |viewer − router|`; `expected = expectedRssi(p1m, distance)`; `k = brightnessFactor(session.rssi if hasSignal else null, expected)`; HUD updated at ≤ 10 Hz with banners (`disconnected` > `trackinglost` > `nosignal` > uncalibrated hint); Calibrate → `p1m = calibrateP1m(rssi, distance)`; `greyed = !connected`; glow hidden while `trackingLost`.
- [ ] **Step 5: Verify** `npm run build` (tsc clean) and `npx vitest run && npx playwright test` — PASS (no regressions).
- [ ] **Step 6: Manual on phone** via `npm run dev -- --host` + Chrome remote port forwarding (`chrome://inspect`, forward 5173 so the page is `localhost`, which counts as secure) with `?fake=1`: Start AR → tap surface → glow appears around tap point, fades with distance, HUD shows distance changing as you walk, Calibrate updates expected.
- [ ] **Step 7: Commit** `feat(web): webxr session and volumetric glow`

### Task 11: Firmware binary, manifest, CI and GitHub Pages deploy

**Files:**
- Create: `web/public/firmware/manifest.json`
- Create: `.github/workflows/ci.yml`
- Create: `README.md` (what it is, hardware list, flashing, usage, dev commands, `?fake=` scenarios)

**Interfaces:**
- Consumes: firmware env `esp32s3` (Tasks 1–2), web build (Tasks 3–10).
- Produces: site at `https://darthvanger.github.io/em-waves-visualizer/` serving `firmware/em-sensor.bin`.

- [ ] **Step 1: `manifest.json`** — `{"name":"EM Waves Sensor","version":"1.0.0","new_install_prompt_erase":true,"builds":[{"chipFamily":"ESP32-S3","parts":[{"path":"em-sensor.bin","offset":0}]}]}`.
- [ ] **Step 2: `ci.yml`** — on push/PR to `main`:
  - job `firmware`: setup Python, `pip install platformio`, `pio test -e native`, `pio run -e esp32s3`, then `esptool.py --chip esp32s3 merge_bin -o em-sensor.bin` with bootloader 0x0, partitions 0x8000, boot_app0 0xe000, app 0x10000 (paths from `.pio/build/esp32s3/` and the framework package); upload artifact.
  - job `web` (needs firmware): Node 20, `npm ci`, download artifact into `web/public/firmware/`, `npx vitest run`, `npx playwright install --with-deps chromium`, `npx playwright test`, `npm run build`, upload Pages artifact `web/dist`.
  - job `deploy` (on `main` only, needs web): `actions/deploy-pages`.
- [ ] **Step 3: Enable Pages** — `gh api -X POST repos/DarthVanger/em-waves-visualizer/pages -f build_type=workflow`.
- [ ] **Step 4: Commit** `ci: tests, firmware build, pages deploy`.
- [ ] **Step 5: Push and verify** — `git push`; `gh run watch` → all jobs green; `curl -sI https://darthvanger.github.io/em-waves-visualizer/firmware/em-sensor.bin` → `200`.

### Task 12: End-to-end hardware check

**Files:** Modify `README.md` (add "Verified on" section with phone model, Android/Chrome versions).

- [ ] **Step 1: Flash** from desktop Chrome via the deployed `install.html`; serial-monitor check as in Task 2 Step 3.
- [ ] **Step 2: Phone + ESP32-S3** on the deployed site (no `?fake`): Connect (device prompt) → picker lists real networks → pick → Start AR → tap router.
- [ ] **Step 3: Spec manual checks:** walk toward/away (measured roughly follows expected after Calibrate); body-block (glow dims, `hud-diff` message ≤ −6 dB); unplug/replug (banner, glow greyed, recovers and re-tracks); cover camera (tracking-lost banner, glow hidden). Note any failures as issues (`gh issue create`).
- [ ] **Step 4: Commit** `docs: record hardware verification`
