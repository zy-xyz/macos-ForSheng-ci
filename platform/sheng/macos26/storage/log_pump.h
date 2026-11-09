/* SPDX-License-Identifier: MIT */
/* Log pump: copies the RAM console ring (SHNLOG01) into the last 32 MiB of userdata every 2 s.
 *
 * The ring itself is already populated: patch 0013 hooks sheng_log_putc() into both
 * uart_putc_options() and _cnputc(), so every kprintf character lands in sheng_log_buffer.
 * Nothing consumed it because BootLog.c (the board's writer) is not part of the emit build.
 * This is the minimal consumer: snapshot -> native_ufs_log_write() -> SYNCHRONIZE CACHE.
 *
 * Deliberately NOT BootLog.c: no board page, no GPU rows, no export region, no autoreset.
 * Uses the LUN 0 userdata tail (NativeUfs.cpp:1130 native_ufs_log_open), which is the same
 * window the board writer used, so tools/read_shenglog.py and scripts/decode_native_log.py
 * keep working unchanged.
 */
#ifndef SHENG_LOG_PUMP_H
#define SHENG_LOG_PUMP_H

#include <IOKit/IOLib.h>
#include <kern/thread.h>
#include <string.h>

/* ArmCpu.cpp (C++) reaches panic() the same way: pexpert.h is the C++-safe door to it.
 * Declaring panic() by hand collides with the kernel's own macro and fails to parse. */
#include <pexpert/pexpert.h>

/* Signatures copied verbatim from their definitions, not from any design note:
 *   sheng_log_snapshot_copy  platform/sheng/macos26/log/retained_log.h:119
 *   native_ufs_log_open      UniXNU/drivers/NativeUfs/NativeUfs.cpp:1130
 *   native_ufs_log_write     UniXNU/drivers/NativeUfs/NativeUfs.cpp:1159
 *   native_ufs_log_sync      UniXNU/drivers/NativeUfs/NativeUfs.cpp:1189
 */
/* The pane and the retained ring are written by these two, not by IOLog: the visible
 * "SHENG:" lines come from sheng_log_stage(), which also appends to the ring. IOLog reaches
 * neither, which is why every diagnostic so far was invisible. */
extern "C" void sheng_log_stage (const char *message);
extern "C" unsigned sheng_log_snapshot_copy (void *destination, unsigned bytes);
extern "C" int native_ufs_log_open (uint64_t *firstLba, uint64_t *blocks, const char **why);
extern "C" int native_ufs_log_write (uint64_t rel, uint32_t blocks, const void *src);
extern "C" int native_ufs_log_sync (void);

namespace {

/* retained_log.h:8-9 + the 128-byte header: 128 + 16384 + 65536 = 82048.
 * retained_log.h cannot be included here (it defines globals), so the size is spelled out and
 * verified at run time: sheng_log_snapshot_copy() returns 0 unless bytes == sizeof(*r).
 */
constexpr unsigned kLogPumpSnapshotBytes = 128u + 16384u + 65536u;
constexpr unsigned kLogPumpBlock = 4096u;
/* Ceiling, because the snapshot is not a whole number of blocks (82048 = 20*4096 + 128). */
constexpr unsigned kLogPumpBlocks = (kLogPumpSnapshotBytes + kLogPumpBlock - 1) / kLogPumpBlock;  // 21
constexpr unsigned kLogPumpPeriodMs = 2000;   /* BootLog.c:135 LogPeriodMs */

uint8_t *gLogPumpStage;
volatile uint32_t gLogPumpStarted;
volatile uint64_t gLogPumpRounds, gLogPumpWriteErrors, gLogPumpSnapshotSkips;

void logPumpThread (void *, wait_result_t)
{
  uint64_t first = 0, blocks = 0;
  const char *why = "";
  unsigned tries = 0;
  /* Retry instead of giving up: the same call path fills the root layout, and repeating the
   * refusal keeps the reason at the bottom of the panel console. A single line printed here
   * would be scrolled away long before the failure that matters. */
  while (!native_ufs_log_open (&first, &blocks, &why)) {
    ++tries;
    sheng_log_stage ("logpump: refused");
    sheng_log_stage (why ? why : "why unknown");
    /* The console keeps only the last few lines, so a printed refusal is unreadable by
     * the time it matters. Panic instead: the reason then lands in PANIC DETAIL, the one
     * pane that survives. 15 tries is 30 s, far past the root layout. */
    if (tries == 15)
      panic ("logpump: refused %s", why ? why : "?");
    IOSleep (kLogPumpPeriodMs);
  }
  if (blocks < kLogPumpBlocks) {
    sheng_log_stage ("logpump: window too small");
    return;
  }
  sheng_log_stage ("logpump: armed");

  /* One immediate attempt with a report after every step. mount-phase-1 fails about five
   * seconds in, so the 2 s loop never gets far enough to say which step breaks. */
  {
    const unsigned copied = sheng_log_snapshot_copy (gLogPumpStage, kLogPumpSnapshotBytes);
    if (copied != kLogPumpSnapshotBytes) {
      sheng_log_stage ("logpump: snapshot short");
      if (copied == 0) sheng_log_stage ("logpump: copied zero");
      else sheng_log_stage ("logpump: copied partial");
    } else {
      sheng_log_stage ("logpump: snapshot ok");
      if (copied % kLogPumpBlock)
        memset (gLogPumpStage + copied, 0, kLogPumpBlocks * kLogPumpBlock - copied);
      if (native_ufs_log_write (0, kLogPumpBlocks, gLogPumpStage)) {
        sheng_log_stage ("logpump: write ok");
        if (native_ufs_log_sync ()) sheng_log_stage ("logpump: sync ok");
        else sheng_log_stage ("logpump: sync failed");
      } else {
        sheng_log_stage ("logpump: write failed");
      }
    }
  }

  for (;;) {
    const unsigned copied = sheng_log_snapshot_copy (gLogPumpStage, kLogPumpSnapshotBytes);
    if (copied != kLogPumpSnapshotBytes) {
      /* Either the writer held the ring, or the size constant disagrees with sizeof(*r). */
      gLogPumpSnapshotSkips = gLogPumpSnapshotSkips + 1;
      if (gLogPumpSnapshotSkips == 1)
        sheng_log_stage ("logpump: snapshot size mismatch");
      /* 10 s of an unchanged mismatch means the ring layout and kLogPumpSnapshotBytes
       * disagree, which nothing at runtime can fix. Same visibility reasoning as above. */
      if (gLogPumpSnapshotSkips == 5)
        panic ("logpump: snapshot %u != %u", copied, kLogPumpSnapshotBytes);
    } else {
      /* rel 0 is the window's first block = the first 4096 bytes of userdata's last 32 MiB,
       * so the SHNLOG01 magic is exactly at that offset. Pad the tail of the last block. */
      if (copied % kLogPumpBlock)
        memset (gLogPumpStage + copied, 0, kLogPumpBlocks * kLogPumpBlock - copied);
      if (native_ufs_log_write (0, kLogPumpBlocks, gLogPumpStage) && native_ufs_log_sync ()) {
        gLogPumpRounds = gLogPumpRounds + 1;
      } else {
        gLogPumpWriteErrors = gLogPumpWriteErrors + 1;
        if (gLogPumpWriteErrors == 1)
          sheng_log_stage ("logpump: write failed");
      }
    }
    IOSleep (kLogPumpPeriodMs);
  }
}

}  // namespace

/* Called once from NativeUfs::startRoot() after gStoreGeometry is filled, hostBringUp() succeeded
 * and IOService::start() returned true -- before ufsPrepareRootWindows(), whose validate gate
 * belongs to the LUN 4 macoslog path and must not be involved here.
 */
static void logPumpStart (void)
{
  if (__atomic_exchange_n (&gLogPumpStarted, 1, __ATOMIC_ACQ_REL)) return;
  gLogPumpStage = static_cast<uint8_t *> (IOMallocAligned (kLogPumpBlocks * kLogPumpBlock, kLogPumpBlock));
  if (!gLogPumpStage) {
    sheng_log_stage ("logpump: stage allocation failed");
    return;
  }
  bzero (gLogPumpStage, kLogPumpBlocks * kLogPumpBlock);
  thread_t thread = nullptr;
  if (kernel_thread_start (logPumpThread, nullptr, &thread) != KERN_SUCCESS) {
    sheng_log_stage ("logpump: thread start failed");
    IOFreeAligned (gLogPumpStage, kLogPumpBlocks * kLogPumpBlock);
    gLogPumpStage = nullptr;
    __atomic_store_n (&gLogPumpStarted, 0, __ATOMIC_RELEASE);
    return;
  }
  thread_deallocate (thread);
}

#endif  /* SHENG_LOG_PUMP_H */
