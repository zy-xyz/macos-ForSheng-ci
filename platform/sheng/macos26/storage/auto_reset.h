/* SPDX-License-Identifier: MIT */
/* Unattended-run reset: write the PMIC reboot-reason cell to "bootloader", then PSCI SYSTEM_RESET.
 *
 * Deliberately self-contained instead of reusing UniXNU-sheng/package/BootReset.c: that file
 * unmaps the SPMI windows with ml_io_unmap, and on this kernel that path faults inside
 * pmap_remove_options_internal ("Invalid address range") before it can print why. Here the four
 * windows are mapped and never unmapped -- a one-shot reset leaks a few pages of kernel VA, which
 * costs nothing and keeps the reset path free of pmap calls entirely.
 */
#ifndef SHENG_AUTO_RESET_H
#define SHENG_AUTO_RESET_H

#include <IOKit/IOLib.h>
#include <kern/thread.h>
#include <string.h>

extern "C" void sheng_log_stage (const char *message);

/* Permanent, never-freed IO mappings. flags 7 is VM_WIMG_IO. */
extern "C" vm_offset_t ml_io_map_unmappable (vm_offset_t phys, vm_size_t size, uint32_t flags);

namespace {

enum {
    AR_SPMI_CORE = 0xc400000, AR_SPMI_CORE_SIZE = 0x3000,
    AR_SPMI_OBSRVR = 0xc440000,
    AR_SPMI_CHNLS = 0xc500000,
    AR_SPMI_CNFG = 0xc42d000, AR_SPMI_CNFG_SIZE = 0x4000,
    /* The channel windows are one 4 KiB channel per APID plus 0x8000 per execution environment. */
    AR_OBS_SIZE = 0x8000 * 8, AR_CHN_SIZE = 0x1000 * 1024,
    AR_VERSION_V7_MIN = 0x70000000u, AR_VERSION_V8_MIN = 0x80000000u,
    AR_FEATURES = 0x04, AR_FEATURES_APIDS_MASK = 0x7ffu,
    AR_APID_MAP = 0x2000, AR_CNFG_OWNER = 0x00,
    AR_OBS_EE_STRIDE = 0x8000, AR_OBS_APID_STRIDE = 0x20,
    AR_CHNL_APID_STRIDE = 0x1000,
    AR_CMD = 0x00, AR_STATUS = 0x08, AR_WDATA0 = 0x10, AR_RDATA0 = 0x18,
    AR_OP_EXT_WRITEL = 0, AR_OP_EXT_READL = 1,
    AR_STATUS_DONE = 1u << 0, AR_STATUS_ERRORS = 0xeu,
    AR_SDAM_PPID = 0x071, AR_REBOOT_CELL = 0x48,
    AR_REBOOT_MASK = 0xfe, AR_REBOOT_SHIFT = 1, AR_REBOOT_BOOTLOADER = 2,
    AR_POLLS = 100000,
    AR_PSCI_SYSTEM_RESET = 0x84000009u
};

struct ArArbiter { volatile uint32_t *core, *cnfg; uint8_t ee; };
struct ArFound { uint32_t version, apids; int apid, owner; };

static inline uint32_t arCmd (uint8_t op, uint8_t byteAddr, uint8_t bc)
{
    return ((uint32_t)op << 27) | ((uint32_t)byteAddr << 4) | (bc & 7u);
}

static inline int arFind (const ArArbiter *a, ArFound *f)
{
    f->version = a->core[0];
    f->apids = a->core[AR_FEATURES / 4] & AR_FEATURES_APIDS_MASK;
    f->apid = -1; f->owner = -1;
    if (f->version < AR_VERSION_V7_MIN || f->version >= AR_VERSION_V8_MIN) return 1;
    if (!f->apids || f->apids > 1024) return 2;
    for (uint32_t i = 0; i < f->apids; ++i) {
        uint32_t map = a->core[(AR_APID_MAP + 4 * i) / 4];
        if (!map || ((map >> 8) & 0xfff) != AR_SDAM_PPID) continue;
        int owner = (int)(a->cnfg[(AR_CNFG_OWNER + 4 * i) / 4] & 7u);
        if (f->apid < 0 || owner == a->ee) { f->apid = (int)i; f->owner = owner; }
    }
    return f->apid < 0 ? 3 : 0;
}

static inline int arWait (volatile uint32_t *base)
{
    for (unsigned n = 0; n < AR_POLLS; ++n) {
        uint32_t s = base[AR_STATUS / 4];
        if (s & AR_STATUS_DONE) return (s & AR_STATUS_ERRORS) ? 2 : 0;
    }
    return 1;
}

static inline int arRead8 (volatile uint32_t *obs, const ArArbiter *a, int apid,
                           uint8_t byteAddr, uint8_t *value)
{
    volatile uint32_t *ch = obs + (AR_OBS_EE_STRIDE * a->ee + AR_OBS_APID_STRIDE * (uint32_t)apid) / 4;
    ch[AR_CMD / 4] = arCmd(AR_OP_EXT_READL, byteAddr, 0);
    int rc = arWait(ch);
    if (rc) return rc;
    *value = (uint8_t)ch[AR_RDATA0 / 4];
    return 0;
}

static inline int arWrite8 (volatile uint32_t *chn, int apid, uint8_t byteAddr, uint8_t value)
{
    volatile uint32_t *ch = chn + (AR_CHNL_APID_STRIDE * (uint32_t)apid) / 4;
    ch[AR_WDATA0 / 4] = value;
    ch[AR_CMD / 4] = arCmd(AR_OP_EXT_WRITEL, byteAddr, 0);
    return arWait(ch);
}

static inline void arPsciReset (void)
{
    register uint64_t x0 __asm__("x0") = AR_PSCI_SYSTEM_RESET;
    register uint64_t x1 __asm__("x1") = 0;
    register uint64_t x2 __asm__("x2") = 0;
    register uint64_t x3 __asm__("x3") = 0;
    __asm__ volatile ("smc #0" : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3) :: "memory");
}

/* Marks the reboot reason, then resets. Never returns unless the mark fails. */
static int arRebootToBootloader (char *text, unsigned cap)
{
    ArArbiter a = {};
    a.core = (volatile uint32_t *)ml_io_map_unmappable((vm_offset_t)AR_SPMI_CORE, AR_SPMI_CORE_SIZE, 7);
    a.cnfg = (volatile uint32_t *)ml_io_map_unmappable((vm_offset_t)AR_SPMI_CNFG, AR_SPMI_CNFG_SIZE, 7);
    a.ee = 0;
    if (!a.core || !a.cnfg) { snprintf(text, cap, "reset: map failed"); return 1; }

    volatile uint32_t *obs = (volatile uint32_t *)ml_io_map_unmappable((vm_offset_t)AR_SPMI_OBSRVR, AR_OBS_SIZE, 7);
    volatile uint32_t *chn = (volatile uint32_t *)ml_io_map_unmappable((vm_offset_t)AR_SPMI_CHNLS, AR_CHN_SIZE, 7);
    if (!obs || !chn) { snprintf(text, cap, "reset: channel map failed"); return 2; }

    ArFound f = {};
    int rc = arFind(&a, &f);
    if (rc) { snprintf(text, cap, "reset: arbiter find rc=%d", rc); return 3 + rc; }

    uint8_t before = 0, after = 0;
    rc = arRead8(obs, &a, f.apid, AR_REBOOT_CELL, &before);
    if (rc) { snprintf(text, cap, "reset: read rc=%d", rc); return 20 + rc; }
    uint8_t next = (uint8_t)((before & ~AR_REBOOT_MASK) | (AR_REBOOT_BOOTLOADER << AR_REBOOT_SHIFT));
    rc = arWrite8(chn, f.apid, AR_REBOOT_CELL, next);
    if (rc) { snprintf(text, cap, "reset: write rc=%d", rc); return 30 + rc; }
    rc = arRead8(obs, &a, f.apid, AR_REBOOT_CELL, &after);
    if (rc || after != next) { snprintf(text, cap, "reset: verify rc=%d %02x->%02x", rc, before, after); return 40; }

    snprintf(text, cap, "reset: cell %02x -> %02x, resetting", before, after);
    return 0;
}

static void arThread (void *, wait_result_t)
{
    uint32_t seconds = 0;
    if (!PE_parse_boot_argn("sheng-autoreset", &seconds, sizeof seconds) || !seconds) return;
    IOSleep(seconds * 1000u);

    char text[160];
    int rc = arRebootToBootloader(text, sizeof text);
    sheng_log_stage(text);
    if (rc == 0) {
        IOSleep(200);
        arPsciReset();
    }
    /* A refused mark means the device would come back up in macOS; say so and leave it running. */
    sheng_log_stage("reset: not taken");
}

}  // namespace

static void autoResetStart (void)
{
    thread_t thread = nullptr;
    if (kernel_thread_start(arThread, nullptr, &thread) != KERN_SUCCESS) {
        sheng_log_stage("reset: thread start failed");
        return;
    }
    thread_deallocate(thread);
}

#endif  /* SHENG_AUTO_RESET_H */
