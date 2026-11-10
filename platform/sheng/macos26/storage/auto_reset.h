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
    AR_PSCI_SYSTEM_RESET = 0x84000009u,
    AR_PSCI_SYSTEM_RESET2 = 0xc4000012u,
    AR_PSCI_DENIED = -3
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

/* Returns the PSCI status word instead of discarding it: 0 is SUCCESS, -1 NOT_SUPPORTED,
 * -2 INVALID_PARAMETERS, -3 DENIED. The first attempt threw this away, so an intermittent
 * refusal looked exactly like a reset that never happened. */
static inline int64_t arPsciCall (uint64_t function, uint64_t a1, uint64_t a2, uint64_t a3)
{
    register uint64_t x0 __asm__("x0") = function;
    register uint64_t x1 __asm__("x1") = a1;
    register uint64_t x2 __asm__("x2") = a2;
    register uint64_t x3 __asm__("x3") = a3;
    __asm__ volatile ("smc #0" : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3) :: "memory");
    return (int64_t)x0;
}

/* Qualcomm's own reboot request: a Silicon-Provider SMC, not PSCI. kShim takes exactly this
 * path (vendor/kshim/src/drivers/qcom_reboot.c:591) and the vendor firmware knows it, whereas
 * PSCI SYSTEM_RESET can come back DENIED under Gunyah. Register convention is r0 = function,
 * r1 = argument count, r2..r5 = arguments; r0 returns 0 on success. */
static inline int64_t arScmReboot (void)
{
    register uint64_t x0 __asm__("x0") = 0xc3000922ULL;
    register uint64_t x1 __asm__("x1") = 0;
    register uint64_t x2 __asm__("x2") = 0;
    register uint64_t x3 __asm__("x3") = 0;
    register uint64_t x4 __asm__("x4") = 0;
    register uint64_t x5 __asm__("x5") = 0;
    register uint64_t x6 __asm__("x6") = 0;
    register uint64_t x7 __asm__("x7") = 0;
    __asm__ volatile ("smc #0"
                      : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3),
                        "+r"(x4), "+r"(x5), "+r"(x6), "+r"(x7) :: "memory");
    return (int64_t)x0;
}

static inline uint64_t arMpidr (void)
{
    uint64_t value;
    __asm__ volatile ("mrs %0, mpidr_el1" : "=r"(value));
    return value;
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

/* Marks the PMIC reboot reason without resetting. Called at the very start of the storage
 * bring-up so that even a kernel that panics a second later still sends the next boot to
 * fastboot; pressing power is then enough to recover, no key chord required. */
static int arMarkOnly (char *text, unsigned cap)
{
    ArArbiter a = {};
    a.core = (volatile uint32_t *)ml_io_map_unmappable((vm_offset_t)AR_SPMI_CORE, AR_SPMI_CORE_SIZE, 7);
    a.cnfg = (volatile uint32_t *)ml_io_map_unmappable((vm_offset_t)AR_SPMI_CNFG, AR_SPMI_CNFG_SIZE, 7);
    a.ee = 0;
    if (!a.core || !a.cnfg) { snprintf(text, cap, "reset: mark map failed"); return 1; }
    volatile uint32_t *obs = (volatile uint32_t *)ml_io_map_unmappable((vm_offset_t)AR_SPMI_OBSRVR, AR_OBS_SIZE, 7);
    volatile uint32_t *chn = (volatile uint32_t *)ml_io_map_unmappable((vm_offset_t)AR_SPMI_CHNLS, AR_CHN_SIZE, 7);
    if (!obs || !chn) { snprintf(text, cap, "reset: mark channel map failed"); return 2; }
    ArFound f = {};
    int rc = arFind(&a, &f);
    if (rc) { snprintf(text, cap, "reset: mark arbiter find rc=%d", rc); return 3 + rc; }
    uint8_t before = 0, after = 0;
    rc = arRead8(obs, &a, f.apid, AR_REBOOT_CELL, &before);
    if (rc) { snprintf(text, cap, "reset: mark read rc=%d", rc); return 20 + rc; }
    uint8_t next = (uint8_t)((before & ~AR_REBOOT_MASK) | (AR_REBOOT_BOOTLOADER << AR_REBOOT_SHIFT));
    rc = arWrite8(chn, f.apid, AR_REBOOT_CELL, next);
    if (rc) { snprintf(text, cap, "reset: mark write rc=%d", rc); return 30 + rc; }
    rc = arRead8(obs, &a, f.apid, AR_REBOOT_CELL, &after);
    if (rc || after != next) { snprintf(text, cap, "reset: mark verify rc=%d %02x->%02x", rc, before, after); return 40; }
    snprintf(text, cap, "reset: marked %02x -> %02x", before, after);
    return 0;
}

static void arThread (void *, wait_result_t)
{
    sheng_log_stage("reset: thread up");
    uint32_t seconds = 0;
    if (!PE_parse_boot_argn("sheng-autoreset", &seconds, sizeof seconds) || !seconds) {
        sheng_log_stage("reset: no sheng-autoreset boot-arg");
        return;
    }
    sheng_log_stage(seconds >= 60 ? "reset: waiting >60s" : "reset: waiting <=60s");
    IOSleep(seconds * 1000u);

    char text[160];
    int rc = arRebootToBootloader(text, sizeof text);
    sheng_log_stage(text);
    if (rc != 0) {
        /* A refused mark means the device would come back up in macOS; say so and keep running. */
        sheng_log_stage("reset: not taken");
        return;
    }
    /* Give the log pump two full periods before the reset takes the machine down. */
    IOSleep(3000);

    snprintf(text, sizeof text, "reset: cpu mpidr=%llx", (unsigned long long)arMpidr());
    sheng_log_stage(text);

    /* Some Qualcomm firmware only honours SYSTEM_RESET from the boot CPU; it answers DENIED to any
     * other core. Park this thread on the boot CPU before asking, and record which one that was. */
    {
        int bootCpu = ml_get_boot_cpu_number();
        thread_bind(current_thread(), cpu_to_processor(bootCpu));
        snprintf(text, sizeof text, "reset: bound to cpu %d, mpidr=%llx",
                 bootCpu, (unsigned long long)arMpidr());
        sheng_log_stage(text);
    }

    int64_t reset = arPsciCall(AR_PSCI_SYSTEM_RESET, 0, 0, 0);
    snprintf(text, sizeof text, "reset: SYSTEM_RESET -> %lld", reset);
    sheng_log_stage(text);
    if (reset == 0) {
        /* The call reported success yet we are still here: say so rather than loop forever. */
        sheng_log_stage("reset: SYSTEM_RESET returned success but did not reset");
    }
    if (reset != 0) {
        /* PSCI refused. Reset2 takes a type argument and is what kShim prefers when it has a
         * choice, so try it before falling back to the vendor SMC. */
        sheng_log_stage("reset: trying SYSTEM_RESET2");
        int64_t reset2 = arPsciCall(AR_PSCI_SYSTEM_RESET2, 0, 0, 0);
        snprintf(text, sizeof text, "reset: SYSTEM_RESET2 -> %lld", reset2);
        sheng_log_stage(text);
        if (reset2 != 0) {
            sheng_log_stage("reset: trying SCM_OEM_REBOOT");
            int64_t scm = arScmReboot();
            snprintf(text, sizeof text, "reset: SCM_OEM_REBOOT -> %lld", scm);
            sheng_log_stage(text);
        }
    }
    sheng_log_stage("reset: not taken");
}

}  // namespace

/* Called as early as the storage bring-up allows. The mark runs inline -- before any thread and
 * before anything that can panic -- so a kernel that dies a second later still leaves the reboot
 * reason set and the next power press lands in fastboot. */
static void autoResetStart (void)
{
    uint32_t seconds = 0;
    if (!PE_parse_boot_argn("sheng-autoreset", &seconds, sizeof seconds) || !seconds) {
        sheng_log_stage("reset: disabled (no sheng-autoreset)");
        return;
    }
    char text[160];
    int rc = arMarkOnly(text, sizeof text);
    sheng_log_stage(text);
    if (rc != 0) {
        /* Without the mark a reset would come back into macOS; keep the machine running instead. */
        sheng_log_stage("reset: mark failed, not arming the timer");
        return;
    }
    thread_t thread = nullptr;
    if (kernel_thread_start(arThread, nullptr, &thread) != KERN_SUCCESS) {
        sheng_log_stage("reset: thread start failed");
        return;
    }
    thread_deallocate(thread);
}

#endif  /* SHENG_AUTO_RESET_H */
