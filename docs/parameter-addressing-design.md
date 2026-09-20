# Parameter addressing: moving firmware self-description off the board

Status: design, not yet implemented.

Supersedes the earlier draft of this file, which was truncated on disk and is
not recoverable. `wf_manifest.py` in this directory is truncated the same way
and has to be rewritten as part of step 1.

---

## 1. The problem

The firmware carries a description of itself that only off-board consumers ever
read. Measured on `src/main/cli/settings.c`:

| contents | bytes |
| --- | --- |
| 530 setting-name strings (avg 18.2 chars) | 9,639 |
| 530 × `clivalue_t` descriptors (14 B, packed) | 7,420 |
| 46 lookup arrays: 302 label strings + pointers | 3,405 |
| **raw total** | **~20.5 KB** |

After linker dedup the earlier measurement put the shipped cost at 17.9 KB.
None of it is read by flight code. It exists so that the configurator can turn
`gyro_lpf1_dyn_min_hz` into `(PG_GYRO_CONFIG, offsetof(gyroConfig_t,
gyro_lpf1_dyn_min_hz), uint16)` — on a machine with gigabytes of memory.

On top of that sits the code that consumes it — `cli.c` is 7,593 lines — and a
second, independently hand-written description of the same structs in
`msp/msp.c`: `mspProcessInCommand` is 1,658 lines across 90 opcodes, almost all
of it per-field `sbufReadU16()` marshalling.

Flash is the presenting symptom. The F4/F7/G4 targets are 512 KB and the H750
has 128 KB of internal flash.

## 2. The actual goal

Flash is a consequence. The goal is:

> **One source of truth.** A parameter is described once, in the C declaration
> that defines it. The manifest, the configurator's accessors, the CLI name and
> the wire addressability all follow from the build.

Today, adding one setting touches: the PG struct, its reset, `settings.c`,
`cli.c` (if it needs a bespoke command), `msp.c` twice (out and in),
`MSPCodes.js`, `MSPHelper.js` twice, `fc.svelte.js`, and the tab. Nine places,
four of which are hand-maintained parallel copies of a struct layout. That is
the maintenance cost this design exists to remove. The 17.9 KB falls out of it.

A second, smaller prize: `src/js/CliAutoComplete.js` currently *scrapes*
`help` / `dump` / `get` output to discover setting names and their accepted
values (see its `parse-dump` state machine). A manifest deletes that hack
outright.

## 3. What must not change

These are user-visible contracts and the design is constrained by them.

- **Setting names.** `gyro_lpf1_dyn_min_hz` keeps that spelling forever — not
  because anyone types it into a serial terminal (in practice nobody does;
  everyone uses the configurator), but because names are baked into two file
  formats:
  - **Backups.** `src/js/cli_backup.js` captures `dump all` / `diff all` as a
    text file on disk and replays it. Users have these files.
  - **Presets.** `src/js/presets/` ships CLI-text snippets
    (`presetInstance.cliStringsArr`) from a GitHub repo, authored against
    setting names.
- **The `dump` / `diff` text format**, for the same reason. It is a file
  format, not just console output.
- **The CLI as an experience.** Same commands, same output. It moves from the
  firmware into the configurator's CLI tab.
- **Third-party ESC configuration tools.** BLHeliSuite32, ESC Configurator and
  the AM32 configurator talk to the ESCs through the FC over stock MSP. They
  are not ours to update, so the opcodes they use are frozen — see §8.1.

Names being a permanent contract is exactly why they cannot be *derived* from
field paths alone, and why a name table must exist somewhere. This design puts
it in the build artifact instead of in flash.

## 4. Architecture

```
  C declaration  ──build──┬─→  manifest.json         (names, ranges, labels,
  (struct + annotation)   │                           pgn/offset/type)
                          │
                          ├─→  .wf_meta section      (annotations; non-loaded)
                          │
                          └─→  firmware              (.pg_registry only;
                                                      no names, no descriptors)

  configurator  ── manifest.json ──→  CLI, tabs, presets, backup/restore
                ── MSP2 addressed ──→  board
```

The firmware keeps `.pg_registry`, which it already needs for config load/save.
That is its whole self-description: 111 `PG_REGISTER*` sites giving
`(pgn, version, size, length, address)`. Everything symbolic is derived at build
time and never linked into the image.

## 5. The manifest

**Generation.** `wf_manifest.py` reads `.pg_registry` from the ELF plus DWARF
type information, and merges the `.wf_meta` annotation records. `PG_REGISTER`
names each group's storage `<name>_System` / `<name>_SystemArray`, so the
registry's address field maps straight back to a DWARF variable.

**Binding.** The manifest is hashed; the hash is compiled in as `WF_BUILD_ID`
and reported over MSP. The configurator matches board → manifest exactly, never
heuristically. PG versions in the registry remain the per-group compatibility
check they already are.

**Defaults are not static.** This is the one thing that cannot come out of the
ELF: of 111 PGs, **53 compute their defaults in code** (30
`PG_REGISTER_WITH_RESET_FN` + 23 `PG_REGISTER_ARRAY_WITH_RESET_FN`) rather than
from a `.pg_resetdata` template. So the manifest cannot carry defaults, and
client-side `diff` cannot be computed from it alone.

The firmware already has the exact primitive: `pgResetCopy(void *copy, pgn_t)`
in [pg.c:66](../src/main/pg/pg.c#L66) resets any group into any
buffer, template- or function-sourced. Expose it. That is what the staging
buffer is for: one buffer of `max(pgSize)` replaces the permanently-resident
`_Copy` / `_CopyArray` shadow of every group — whose only consumers today are
`cli.c`, `pg.c` and `config_eeprom.c`, so it goes when `cli.c` goes.

## 6. Manifest resolution — how the configurator knows what to load

Once the board stops describing itself, the configurator must obtain the
manifest for the *exact* build in front of it. This is the single biggest risk
in the design, because the failure is not graceful: a manifest from a different
build has plausible-looking but wrong offsets, so a write lands in the wrong
field. That could silently reverse a servo or move a failsafe threshold. The
design therefore has to make a wrong manifest **impossible to use**, not merely
unlikely.

Four parts: an identifier, a resolution order, a verification step, and a
defined failure mode.

### 6.1 The identifier

`WF_BUILD_ID` is **the hash of the manifest itself** — the first 8 bytes of the
SHA-256 of the canonical manifest bytes — computed at build time and compiled
into the firmware as a constant.

There is no chicken-and-egg problem here, which is worth stating because it
looks like there should be one. The manifest is extracted from a *separate*
non-LTO `DEBUG=INFO` build (§12), never from the shipping binary, so the
shipping build simply consumes the resulting `wf_build_id.h` as an input.
`make TARGET=<t> manifest` produces both; an ordinary `make` picks the header
up if it is there and reports `CAP_BUILD_ID_VALID` clear if it is not. No
two-pass link, and no second full-tree compile forced on every developer for
every build.

Hashing the output rather than the inputs is what makes this work: any candidate
manifest can be checked by hashing it and comparing. There is no question of
"did I account for every input that affects layout" (target, feature flags,
compiler version, packing), because the manifest *is* the layout. If it hashes
to what the board reports, it is correct by construction. 64 bits is ample —
this is a collision guard, not a security boundary.

The hash covers the described layout **only**. The manifest's `build` block
(target, version, git revision, date, time) is excluded, because hashing the
timestamp would give every rebuild a new ID even when not one struct moved —
churning the configurator's cache for nothing and forcing a fresh manifest to
be published per build rather than per layout change. Two builds that describe
the same layout are *meant* to collide here: the manifest is interchangeable
between them, which is the whole point of keying on layout.

The `BUILD_ID` reply carries, alongside the hash:

| field | purpose |
| --- | --- |
| manifest hash (8 B) | the key, and the verifier |
| target name | human-readable, and the release-asset lookup |
| firmware version + git sha | human-readable, release lookup |
| capability flags | notably: is a manifest embedded? |
| embedded manifest size | so `MANIFEST_READ` can be paged |

### 6.2 Resolution order

Cheapest and most certain first. Every source ends in the same verification
step (§6.3), so the order is a performance and offline-friendliness decision
rather than a trust one.

1. **Session cache** — already resolved this build ID this session.
2. **Local persistent cache** — keyed by build ID, stored the way
   `FirmwareCache` already stores hex files. This is the steady-state path: you
   connect to the same board every day and it is a dictionary lookup.
3. **Bundled with the configurator** — manifests for the releases this
   configurator build shipped knowing about. Makes a fresh offline install work
   against current firmware.
4. **The board itself** — `MANIFEST_READ`, if the capability flag says there is
   an embedded blob. Authoritative, offline, and needs no release to exist.
   A few KB over MSP is roughly a second on a UART and far less over USB VCP.
5. **Release asset** — download from the GitHub release matching the reported
   version, then verify by hash. Populates the cache for next time.
6. **Load from file** — user picks a `manifest.json`. For a board of unknown
   provenance, or a local build whose manifest is sitting next to the hex.

Note the cache key is the **build ID**, not the version string. Two builds of
the same version with different options have different layouts, and
`FirmwareCache` today keys on the release descriptor — that is not sufficient
here.

### 6.3 Verification — two independent checks

**Hash.** Whatever the source, hash the candidate manifest and compare to the
reported build ID. Mismatch means reject, not warn. This alone is sufficient.

**Registry cross-check.** `PG_LIST` returns `(pgn, version, size, length)` for
every registered group; the manifest contains the same. Compare them. This is
free — `PG_LIST` is in the opcode set anyway — and it is *independent* of the
hash, so it catches a hand-edited manifest, a stale build ID, or a bug in the
generator. It also fails informatively: "PG 47 is 112 bytes on the board, 108 in
the manifest" points straight at the problem, where a hash mismatch only says
"no".

Run both. The hash is the gate; the registry check is what makes a failure
diagnosable.

### 6.4 Failure mode: no manifest, no config access

The configurator must never guess. On an unresolved board:

- Config tabs and the CLI are unavailable, showing the reported build ID,
  target and version, and a "load manifest from file" action.
- **What still works without a manifest**, deliberately:
  - the §8.1 frozen subset — so third-party ESC tools are unaffected;
  - live data and identity;
  - **the firmware flasher.**

That last one is what makes an unresolved board recoverable rather than bricked
in practice: you can always reflash to a known release and get a manifest. It
needs to be an explicit requirement on the flasher, not an accident of which
tabs happen not to need config.

One hazard to handle explicitly: `cli_backup.js`'s *backup before flashing*
flow depends on `dump all`, which needs a manifest. On an unresolved board that
safety net is gone — the user has to be told **before** they flash, not after.

A raw hex PG editor (address, length, bytes, no names) is a plausible ~50-line
recovery escape hatch on top of `PARAM_READ`/`PARAM_WRITE`. Optional; only
worth it if unresolved boards turn out to be common, which they should not.

### 6.5 Where manifests come from

Firmware is distributed as GitHub releases (`ReleaseChecker`, `FirmwareCache`);
there is no cloud build API. So:

**Release builds.** CI publishes `manifest.json` as a release asset next to
every hex, for every target. This must include nightly and dev builds — if a
hex is published, its manifest is published, or that build is unusable. Worth an
explicit CI gate.

**Local builds: `USE_EMBEDDED_MANIFEST`.** A compile option linking a *gzipped*
manifest blob into flash, served by `MANIFEST_READ`. The firmware never parses
or decompresses it — inert data behind a `memcpy`. With prefix-heavy names
compressing well, expect roughly 4–6 KB against the 20.5 KB it replaces, so even
switched on this still banks most of the win.

Default **off** for release builds (full saving; the manifest comes from the
release) and **on** for local and developer builds, where no published manifest
exists and the alternative is a board nothing can talk to. The local build also
writes `manifest.json` next to the hex, for path 6.

Defaulting it on for local builds is also what keeps path 4 alive and exercised.
A fallback that only runs in rare production circumstances is a fallback that
does not work when it is needed.

### 6.6 Consequence for the connect flow

Manifest resolution has to complete before the configurator reads any config,
so it becomes part of the connect handshake — and paths 4 and 5 are slow or
networked. Connect therefore needs a visible resolving state, a timeout, and a
sensible offline outcome (fall through to §6.4 rather than hanging). This is a
real change to connect UX and should be designed, not bolted on.

## 7. Wire protocol

MSPv2 is already supported in both framings — native and v2-over-v1
([msp_serial.c:118-172](../src/main/msp/msp_serial.c#L118-L172))
— which also means it reaches through `USE_MSP_OVER_TELEMETRY`. No new framing
is needed or wanted; `msp_serial.c`, passthrough, and the telemetry gateway stay
untouched. What changes is the *catalogue*, not the transport.

The fork already has a contiguous Wingflight block: `MSP2_WING_*` occupies
`0x5F00`–`0x5F14` in [msp_protocol.h:308-331](../src/main/msp/msp_protocol.h#L308-L331)
and is still growing one opcode at a time. The addressing opcodes go in the
same block, far enough up to leave room for that growth — `0x5F20` onwards, or
the earlier draft's `0x5FF0`. Either is fine; pick one and document it.

| opcode | purpose |
| --- | --- |
| `PARAM_READ` | `(pgn, offset, length)` → bytes |
| `PARAM_WRITE` | `(pgn, offset, length, bytes)` |
| `PG_DEFAULT` | `(pgn)` → `pgResetCopy` output |
| `PG_LIST` | registry contents (pgn, version, size, length) — also the §6.3 cross-check |
| `BUILD_ID` | manifest hash, target, version, capability flags, blob size (§6.1) |
| `MANIFEST_READ` | byte-range read of the embedded blob, when built in |

`BUILD_ID` and `PG_LIST` must be answerable **before** a manifest is resolved —
they are what resolves it. So they are hand-written replies with a fixed wire
shape, like the §8.1 subset, not something derived from the manifest.

All bounds-checked against `.pg_registry`, which is what makes a generic address
safe: a write is accepted only if `offset + length <= pgSize(reg)`. Repeated
structs are addressed as `index * stride + field offset`, the same arithmetic
the bounds check performs.

This is ~200 lines of firmware, and it replaces the *possibility* of all 90
config opcodes in `mspProcessInCommand`.

## 8. What the firmware keeps

### 8.1 The frozen MSP compatibility subset

**Third-party ESC configuration tools must keep working.** BLHeliSuite32, ESC
Configurator and the AM32 configurator all reach the ESCs *through* the flight
controller, and they speak stock MSP to get there. They are not ours to update
and they will not be ported to addressed access.

So a small, explicitly enumerated set of legacy MSP opcodes is **frozen**:
retained verbatim, byte-for-byte, past the deletion in step 5. It is not part
of the catalogue being removed and it is not migrated to `PARAM_READ`.

| opcode | why a tool needs it |
| --- | --- |
| `MSP_API_VERSION` (1) | handshake |
| `MSP_FC_VARIANT` (2) | handshake |
| `MSP_FC_VERSION` (3) | handshake |
| `MSP_BOARD_INFO` (4) | handshake |
| `MSP_BUILD_INFO` (5) | handshake |
| `MSP_UID` (160) | board identity |
| `MSP_STATUS` (101) | "disarm before flashing" check |
| `MSP_MOTOR_CONFIG` (131) | motor count and protocol |
| `MSP_MOTOR` (104) / `MSP_SET_MOTOR` (214) | motor test |
| `MSP_SET_PASSTHROUGH` (245) | **the door** — enters 4-way |
| `MSP_REBOOT` (68) | exit |
| `MSP2_SEND_DSHOT_COMMAND` (`0x3003`) | DShot ESC programming |

Two things make this carve-out cheap:

- **The door is already structurally separate.** `MSP_SET_PASSTHROUGH` is
  dispatched in `mspFcProcessCommand` ([msp.c:4456](../src/main/msp/msp.c#L4456)),
  outside the big catalogue switch, and hands the port to `esc4wayProcess` as a
  post-process function ([msp.c:311-320](../src/main/msp/msp.c#L311-L320)).
  Once 4-way starts, the port is no longer speaking MSP at all — `serial_4way.c`
  and friends are untouched by any of this.
- **The subset is small.** Identity opcodes are a few `sbufWriteData` each;
  `MSP_MOTOR_CONFIG` is a handful of `sbufWriteU16`. Call it 150–250 lines out
  of `msp.c`'s 4,484 — a rounding error against the 17.9 KB.

**Also retained for now:** `MSP_ESC_SENSOR_CONFIG` (123) /
`MSP_SET_ESC_SENSOR_CONFIG` (216) and `MSP2_WING_ESC_SENSOR_TRIAL` (`0x5F14`).
These read as plain PG config for our own configurator rather than something a
third-party tool issues, so in principle they migrate to addressed access like
everything else — but the cost of being wrong is that an ESC tool stops working,
and the cost of keeping them is a few dozen lines. They stay until the packet
capture in §14 proves nothing external uses them. Retained, not frozen: they can
migrate later without breaking the §8.1 contract.

**Verify before relying on this:** the FC already reports variant `"WGFL"`
([version.h:26](../src/main/build/version.h#L26)) and API
version `22.2` ([msp_protocol.h:57-58](../src/main/msp/msp_protocol.h#L57-L58)),
neither of which any Betaflight-era tool was written to expect. Whatever
tolerance makes ESC tools work today is therefore *already* independent of the
version numbers — what they actually depend on is the shape of the handshake
and passthrough opcodes. That is a useful thing to know (it means §13's API
version bump is a non-issue) but it should be confirmed against real tools, not
inferred.

### 8.2 Non-derivable CLI commands

Not everything in `cli.c` is derivable from PGs. These read runtime state or
target hardware tables and have no manifest representation:

- `resource`, `timer`, `dma` — need the runtime IO-owner registry and the
  target's `timerHardware[]`. Roughly 790 lines across `printResourceOwner`
  (166), `cliDmaopt` (151), `cliTimer` (122), `printTimer` (110), `cliResource`
  (100), and the two `print*DmaoptDetails` helpers.
- `status` (161 lines), `tasks` (62).
- `serialpassthrough`, `escprog`, `gpspassthrough`, `msc`, `dfu`/`bl`,
  `flash_*`, `bind_rx`, `save`, `defaults`.

The move for the first two groups is the same as everywhere else: the firmware
**emits the data, the configurator does the formatting.** `resource show`
becomes a structured list of `(owner, index, ioTag)`; `tasks` becomes an array
of counters. The data already exists; it is the `printf`-ing that costs flash.

The rest are actions, not descriptions, and stay as they are.

## 9. Configurator side

The CLI tab gains a real CLI rather than a serial pipe:

- `set` / `get` / `dump` / `diff` / `defaults` implemented against the manifest,
  emitting and accepting the existing text format.
- `diff` compares live values (`PARAM_READ`) against `PG_DEFAULT`.
- `CliAutoComplete` reads the manifest instead of scraping `dump`.

Because it is client-side it can be better than the on-device one was —
validation before send, units, per-field help, completion that knows enum
labels, `diff` against an arbitrary saved backup rather than only against
defaults. That is the user-facing upside of the move and worth building for
deliberately rather than just reimplementing the old behaviour.

Backup/restore and presets keep working unchanged, because they operate on the
same text format and the same names.

## 10. Authoring metadata

Most of what `settings.c` carries should live at the declaration:

- **Labels** → type the field as `enum foo : uint8_t` instead of `uint8_t`.
  DWARF then yields the enumerator names for free and a whole annotation
  category disappears. This is better C regardless — it is type safety the
  codebase currently forgoes for packing — it is a per-field incremental change,
  and it should become the house rule for new code immediately.
- **Min/max, legacy name override, units** → an annotation macro emitting a
  record into `.wf_meta`, an `(INFO)` output section in the linker scripts:
  present in the ELF, readable by the build, never loaded into flash.

The build fails if a registered field has neither a derivable name nor an
annotation. That assertion is what keeps the single-source-of-truth property
true over time instead of aspirational.

## 11. Migration

The earlier plan had an ordering bug: step 2 removed the metadata the on-device
CLI depends on, but the replacement CLI did not arrive until step 4, leaving a
window with no working CLI at all. Revised:

1. **Manifest + CI gate.** *(done, except release publishing.)*
   `src/utils/wf_manifest.py` extracts the manifest; `make manifest` builds the
   non-LTO ELF and emits both `manifest.json` and `wf_build_id.h`;
   `make manifest_check` diffs it against `valueTable`. While both descriptions
   exist this is a free regression test that *proves* the manifest before
   anything relies on it. Still to do: publish the manifest as a release asset,
   and the LTO-consistency gate below.
2. **Addressed opcodes, additive.** Add the `0x50xx` block. MSP catalogue and
   `cli.c` untouched. Nothing user-visible changes.
3. **Configurator on the manifest.** Build the client-side CLI and migrate tabs
   to addressed access behind a feature flag. Prove against real boards. Still
   nothing removed from the firmware.
4. **`.wf_meta`.** Convert `settings.c` to annotations. This is the step that
   banks the 17.9 KB, and it lands only once (3) has shipped — so it lands
   together with the configurator release that replaces what it removes.
5. **Delete.** `cli.c`, `settings.c`, the MSP config catalogue *except the
   frozen subset in §8.1*, `msp_box.c`, the `_Copy` buffers in `pg.h`. Add the
   staging buffer. Before this lands, the frozen subset should be moved into
   its own translation unit (`msp/msp_compat.c`) so that "the catalogue" and
   "the compatibility shim" are separable by file rather than by `#if` — the
   deletion then cannot take a frozen opcode with it by accident.
Steps 1–3 are purely additive and reversible. Step 5 is the first irreversible
one and it happens after the configurator has been running on the new path for a
release.

There is no step 6. An earlier draft had one — `wfLive_t` plus a subscription
model, deleting `mspProcessOutCommand` — and it has been dropped. See §11.1.

### 11.1 Live data is out of scope

Sensor and status data does not move to addressed access, and the MSP opcodes
that serve it are not being deleted. This is a decision, not an omission.

The mechanism does not reach it. Everything here is built on `.pg_registry`,
and live values are not in it: `acc`, `attitude` and `rcData` are plain globals
with no parameter group and therefore no pgn for `PARAM_READ` to address. That
follows from what a parameter group is — the things saved to EEPROM and reset
to defaults. A gyro reading is neither.

Three further reasons it should not be forced to fit:

- **Much of it is computed, not stored.** `MSP_RAW_IMU` rescales by
  `acc.dev.acc_1G` as it builds the reply; there is no byte range that *is*
  the answer.
- **The access pattern is the opposite.** Config is read once at connect and
  written rarely; live data is polled every frame for a handful of values, so
  one opcode returning a packed frame beats a request per field.
- **There are no stable offsets to describe.** Config struct layout is fixed by
  the ABI and checked by CI (§12). Live globals move freely between builds with
  nothing pinning them.

And the case for doing it was weak. Of the 79 opcodes in
`mspProcessOutCommand`, 46 are live or status and 33 are config-shaped; only
that config third is in scope for deletion. The live half is not where the
flash is, its replacement would still need hand-written marshalling for the
computed values, and `wfLive_t` was unmeasured RAM added to a target already
carrying 118 KB of bss. It was also the part with the widest blast radius —
radio telemetry and third-party ground stations poll these opcodes.

If live-data flash ever does become a problem, the incremental option is this
same trick applied narrowly: a small `(id → address, size)` table for the
values that really are plain globals, with bespoke cases kept for the computed
ones. That is additive and needs no new protocol.

## 12. CI gates

Without these the design decays back into hand-maintained tables.

- **Manifest ⟷ `valueTable` diff** — `make TARGET=<t> manifest_check`, during
  steps 1–4 while both descriptions exist. `valueTable` is read out of the ELF
  rather than parsed out of `settings.c`, because the C is thick with
  preprocessor conditionals and the linked table is what actually ships.

  On STM32F411 this currently matches 461 of 464 settings. The three that do
  not are a real defect in `settings.c`, listed as known disagreements in
  `wf_manifest_check.py` so the gate can be adopted now rather than after the
  backlog is cleared: `align_board_roll`/`pitch`/`yaw` are declared `VAR_INT16`
  against `int32_t` struct fields, and `cliSetVar()` writes through the
  declared type, so a negative value — the declared range is `-180..360` —
  writes only the low half and leaves the high half stale. `get` then reads
  that low half back and reports the value the user asked for while the
  firmware uses something else entirely. Inherited from upstream, not
  introduced here. Fixing it means either a new `VAR_INT32` in
  `cliValueFlag_e` (bits 0–2 have room) or narrowing the struct, which moves
  the group layout and needs a PG version bump.

  This gate found three bugs in the generator itself on its first run
  (multi-dimensional arrays truncated to their last dimension, anonymous union
  members leaking into field paths), which is the argument for having it.
- **LTO consistency.** LTO collapses per-CU DWARF: on an LTO build the
  extractor resolves 0 of 77 groups, on a non-LTO one 77 of 77. So manifest
  generation runs as a separate non-LTO `DEBUG=INFO` build, and
  `manifest_check --shipping-elf` then proves the two agree rather than
  assuming it. Currently 77 of 77 groups match on STM32F411.

  Note the design draft said to compare `.pg_registry` *byte for byte*; that
  cannot work, because the records hold link-time addresses which differ
  between any two builds. What is compared is every group's number, version,
  element count and size — all of the registry that is not an address. Field
  offsets *inside* a group cannot be checked against the shipping build at
  all, since it has no DWARF to read them from; they are fixed by the ABI
  rather than by the optimiser, and a struct that changed layout while keeping
  its exact total size is not a realistic divergence.
- **No regrowth.** Fail the build if a `settings.c`-shaped hand-maintained
  descriptor table reappears.
- **Annotation completeness.** Fail if a registered field has no resolvable
  name.
- **Registry vs. debug info.** The generator already refuses to emit a manifest
  whose fields do not fit the group sizes the registry reports (`validate()` in
  `wf_manifest.py`). These are two independent descriptions of the same memory
  and nothing forces them to agree; a disagreement hands the configurator
  offsets that write past the end of a group.
- **Staging buffer covers the largest group.** Assert
  `max(pgSize) <= MSP_PARAM_STAGING_SIZE`. Measured on STM32F411: 988 bytes
  (`servoCurves`, pgn 1017), against a 1024-byte buffer. A group that outgrows
  it silently loses `PG_DEFAULT`, and with it client-side `diff`.
- **Frozen subset golden test.** Record the exact reply bytes for every opcode
  in §8.1 against a known config, and fail on any diff. These opcodes exist
  solely for tools that cannot be updated, so "still compiles" is not the bar —
  the wire shape must not move. Run it on SITL so it costs nothing.
- **Every published hex has a published manifest.** Fail the release job if any
  target's `manifest.json` asset is missing — including nightlies. A hex without
  a manifest is a board the configurator cannot configure (§6.5).
- **Build ID round-trip.** Assert that hashing the generated manifest reproduces
  the `WF_BUILD_ID` compiled into the firmware. Cheap, and it catches the
  generator and the build drifting apart.
- **Flash budget** per target, so the saving does not quietly get spent.

## 13. Consequences of the fork

This has been decided deliberately: MSP config compatibility with upstream
Betaflight/Rotorflight tooling ends — **with the §8.1 subset carved out**, so
that third-party ESC tools keep working. Recording what the rest means
concretely, so it is a known cost rather than a surprise:

- Stock Betaflight/Rotorflight configurators can no longer read or write
  Wingflight config.
- `USE_MSP_OVER_TELEMETRY` keeps working as a *transport* — CRSF, ELRS and
  SmartPort still carry MSP frames, and MSPv2-over-v1 means the new opcodes
  reach through it. But radio-side tools that speak Betaflight *config* opcodes
  (ELRS Lua scripts in particular) break when step 5 lands. A
  Wingflight-specific Lua script can do everything the old one did, via
  `PARAM_READ`/`PARAM_WRITE`, but it has to be written.
- The MSP API version needs no bump on account of this work. It is already
  `22.2` against a Betaflight-era `1.4x`, and the variant is already `"WGFL"`,
  so identity has been forked for a long time; nothing a tool checks changes.
  What matters for compatibility is the wire shape of §8.1, which is why that
  gets a golden test rather than a version number.

**Decided:** the fork does *not* touch the live-data opcodes (`MSP_ATTITUDE`,
`MSP_BATTERY_STATE`, `MSP_STATUS`, `MSP_RAW_IMU` …). They are what radio-side
telemetry and third-party ground stations poll, they are cheap to keep, and
keeping them is orthogonal to the config fork. See §11.1.

So the fork's scope is exactly: the hand-written MSP *config* catalogue, minus
the frozen subset in §8.1. Anything that reports state rather than settings
stays.

## 14. Remaining open items

- **`MSP_MULTIPLE_MSP`** — not config, not live data. Confirm it does not reach
  into the catalogue being deleted. (`MSP_PASSTHROUGH_*` and 4-way ESC are no
  longer open: they are frozen by §8.1.)
- **Blackbox headers** carry their own field-name strings in flash for exactly
  the reason `settings.c` did. Same treatment applies and it is probably a
  cleaner win than it looks, since the log already carries a header the decoder
  parses. Out of scope here; worth a follow-up.
- **Opcode block number** — `0x5F20` onwards vs the draft's `0x5FF0`, both
  inside the existing `MSP2_WING_*` block. Pick one.
- **Confirm the §8.1 list against real tools.** It is derived from what the
  handshake and passthrough paths need, not from packet captures. Before step 5,
  run BLHeliSuite32, ESC Configurator and the AM32 configurator against a board
  and log which opcodes they actually issue — then freeze exactly that set.

---

## Appendix A — how the numbers were measured

Toolchain was `gcc 13.2.1` rather than the pinned 9.3.1, via
`GCC_REQUIRED_VERSION := 13.2.1` in `make/local.mk` and `EXTRA_FLAGS=-Wno-error`
(13.x is stricter and flags pre-existing enum/int mismatches). Absolute sizes
shift a little on the pinned compiler; the ratios do not.

```sh
# baseline
make TARGET=STM32F411

# per-module attribution (LTO defeats it, so build without)
#   temporarily drop '-flto -fuse-linker-plugin' from Makefile:159
arm-none-eabi-size obj/main/STM32F411/{cli/cli,cli/settings,msp/msp,msp/msp_box}.o

# what the CLI costs, by deletion
#   comment out '#define USE_CLI' in src/main/target/common_pre.h
make TARGET=STM32F411

# debug info is free
make TARGET=STM32F411            && cp obj/*.hex a.hex
make TARGET=STM32F411 DEBUG=INFO && cp obj/*.hex b.hex
cmp a.hex b.hex                   # identical

# registry, tables, shadow buffers
arm-none-eabi-nm -S --radix=d obj/main/wingflight_STM32F411.elf | grep __pg_registry
arm-none-eabi-nm -S --radix=d obj/main/STM32F411/cli/settings.o | sort -k2 -rn | head
arm-none-eabi-nm -S --radix=d obj/main/wingflight_STM32F411.elf \
    | grep -E '_Copy(Array)?$' | awk '{s+=$2} END {print s}'

# the manifest
make TARGET=STM32F411 DEBUG=INFO     # non-LTO
python3 src/utils/wf_manifest.py obj/main/wingflight_STM32F411.elf manifest.json
```

The `settings.c` breakdown in §1 was counted from source rather than from the
ELF, so it is the pre-dedup figure; the 17.9 KB is the linked cost.
