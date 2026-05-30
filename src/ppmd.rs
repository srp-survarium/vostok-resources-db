//! Port of the Vostok engine's custom Dmitry Shkarin PPMd (PPMII) decoder.
//!
//! Faithfully ported from:
//!   sources/vostok/core/sources/compressor_ppmd.cpp
//!   sources/vostok/core/sources/compressor_ppmd_allocator.h
//!
//! Notes that make this variant non-standard (do NOT swap for an off-the-shelf
//! PPMd7/PPMd8 implementation):
//!   * Model order is 8 (`order_model` in compressor_ppmd.cpp), restart model
//!     restoration, 32 MiB sub-allocator (`ppmd_compressor(allocator, 32)`).
//!   * Subbotin carryless range coder with TOP=1<<24, BOT=1<<15 (BOT is 1<<15,
//!     not the more common 1<<16). 4-byte decoder init.
//!   * Built for Win32 => UNIT_SIZE = 12, 4-byte in-heap pointers.
//!
//! The original works directly on raw pointers inside a single sub-allocated
//! heap. We emulate that 32-bit address space with a byte buffer plus a virtual
//! base address (`HEAP_BASE`); "pointers" are absolute virtual addresses and
//! NULL == 0.

const UNIT_SIZE: u32 = 12;
const ORDER_MODEL: i32 = 8;
const SUB_ALLOCATOR_SIZE_MB: u32 = 32;

const MAX_O: usize = 16;
const UP_FREQ: u32 = 5;
const INT_BITS: u32 = 7;
const PERIOD_BITS: u32 = 7;
const TOT_BITS: u32 = INT_BITS + PERIOD_BITS;
const INTERVAL: u32 = 1 << INT_BITS;
const BIN_SCALE: u32 = 1 << TOT_BITS;
const MAX_FREQ: u8 = 124;
const O_BOUND: i32 = 9;

const TOP: u32 = 1 << 24;
const BOT: u32 = 1 << 15;

const N1: usize = 4;
const N2: usize = 4;
const N3: usize = 4;
const N4: usize = (128 + 3 - 1 * N1 - 2 * N2 - 3 * N3) / 4;
const N_INDEXES: usize = N1 + N2 + N3 + N4;

const PPMD_SIGNATURE: u32 = 0x84AC_AF8F;

// Virtual base so that NULL (0) never aliases a real heap address.
const HEAP_BASE: u32 = 0x1_0000;

const EXP_ESCAPE: [u8; 16] = [25, 14, 9, 7, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 2];
const INIT_BIN_ESC: [u16; 8] = [0x3CDD, 0x1F3F, 0x59BF, 0x48F3, 0x64A1, 0x5ABC, 0x6632, 0x6051];

#[inline]
fn get_mean(summ: u32, shift: u32, round: u32) -> u32 {
    (summ + (1 << (shift - round))) >> shift
}

// --- PPM_CONTEXT field offsets (packed, 12 bytes) ---
// NumStats:u8@0  Flags:u8@1  SummFreq:u16@2  Stats:u32@4  Suffix:u32@8
// oneState() overlays a STATE at offset 2 (over SummFreq..Suffix).
// --- STATE (6 bytes): Symbol:u8@0  Freq:u8@1  Successor:u32@2 ---

pub struct Heap {
    mem: Vec<u8>,
}

impl Heap {
    fn new(size: u32) -> Heap {
        Heap { mem: vec![0u8; size as usize] }
    }
    #[inline]
    fn idx(&self, addr: u32) -> usize {
        (addr - HEAP_BASE) as usize
    }
    #[inline]
    fn u8(&self, a: u32) -> u8 {
        self.mem[self.idx(a)]
    }
    #[inline]
    fn set_u8(&mut self, a: u32, v: u8) {
        let i = self.idx(a);
        self.mem[i] = v;
    }
    #[inline]
    fn u16(&self, a: u32) -> u16 {
        let i = self.idx(a);
        u16::from_le_bytes(self.mem[i..i + 2].try_into().unwrap())
    }
    #[inline]
    fn set_u16(&mut self, a: u32, v: u16) {
        let i = self.idx(a);
        self.mem[i..i + 2].copy_from_slice(&v.to_le_bytes());
    }
    #[inline]
    fn u32(&self, a: u32) -> u32 {
        let i = self.idx(a);
        u32::from_le_bytes(self.mem[i..i + 4].try_into().unwrap())
    }
    #[inline]
    fn set_u32(&mut self, a: u32, v: u32) {
        let i = self.idx(a);
        self.mem[i..i + 4].copy_from_slice(&v.to_le_bytes());
    }
}

// Context accessors (ctx == address of a PPM_CONTEXT).
#[inline]
fn ctx_num_stats(h: &Heap, c: u32) -> u8 {
    h.u8(c)
}
#[inline]
fn ctx_set_num_stats(h: &mut Heap, c: u32, v: u8) {
    h.set_u8(c, v);
}
#[inline]
fn ctx_flags(h: &Heap, c: u32) -> u8 {
    h.u8(c + 1)
}
#[inline]
fn ctx_set_flags(h: &mut Heap, c: u32, v: u8) {
    h.set_u8(c + 1, v);
}
#[inline]
fn ctx_summ_freq(h: &Heap, c: u32) -> u16 {
    h.u16(c + 2)
}
#[inline]
fn ctx_set_summ_freq(h: &mut Heap, c: u32, v: u16) {
    h.set_u16(c + 2, v);
}
#[inline]
fn ctx_stats(h: &Heap, c: u32) -> u32 {
    h.u32(c + 4)
}
#[inline]
fn ctx_set_stats(h: &mut Heap, c: u32, v: u32) {
    h.set_u32(c + 4, v);
}
#[inline]
fn ctx_suffix(h: &Heap, c: u32) -> u32 {
    h.u32(c + 8)
}
#[inline]
fn ctx_set_suffix(h: &mut Heap, c: u32, v: u32) {
    h.set_u32(c + 8, v);
}
#[inline]
fn ctx_one_state(c: u32) -> u32 {
    c + 2
}

// STATE accessors (s == address of a STATE).
#[inline]
fn st_symbol(h: &Heap, s: u32) -> u8 {
    h.u8(s)
}
#[inline]
fn st_set_symbol(h: &mut Heap, s: u32, v: u8) {
    h.set_u8(s, v);
}
#[inline]
fn st_freq(h: &Heap, s: u32) -> u8 {
    h.u8(s + 1)
}
#[inline]
fn st_set_freq(h: &mut Heap, s: u32, v: u8) {
    h.set_u8(s + 1, v);
}
#[inline]
fn st_successor(h: &Heap, s: u32) -> u32 {
    h.u32(s + 2)
}
#[inline]
fn st_set_successor(h: &mut Heap, s: u32, v: u32) {
    h.set_u32(s + 2, v);
}

#[inline]
fn state_cpy(h: &mut Heap, dst: u32, src: u32) {
    let sym = st_symbol(h, src);
    let freq = st_freq(h, src);
    let suc = st_successor(h, src);
    st_set_symbol(h, dst, sym);
    st_set_freq(h, dst, freq);
    st_set_successor(h, dst, suc);
}

#[inline]
fn swap_state(h: &mut Heap, a: u32, b: u32) {
    let (as_, af, asu) = (st_symbol(h, a), st_freq(h, a), st_successor(h, a));
    let (bs, bf, bsu) = (st_symbol(h, b), st_freq(h, b), st_successor(h, b));
    st_set_symbol(h, a, bs);
    st_set_freq(h, a, bf);
    st_set_successor(h, a, bsu);
    st_set_symbol(h, b, as_);
    st_set_freq(h, b, af);
    st_set_successor(h, b, asu);
}

#[derive(Clone, Copy)]
struct SubRange {
    low: u32,
    high: u32,
    scale: u32,
}

struct See2 {
    summ: u16,
    shift: u8,
    count: u8,
}

impl See2 {
    fn init(&mut self, init_val: u32) {
        self.shift = (PERIOD_BITS - 4) as u8;
        self.summ = (init_val << self.shift) as u16;
        self.count = 7;
    }
    fn get_mean(&mut self) -> u32 {
        let ret = (self.summ >> self.shift) as u32;
        self.summ = self.summ.wrapping_sub(ret as u16);
        ret + (ret == 0) as u32
    }
    fn update(&mut self) {
        if (self.shift as u32) < PERIOD_BITS {
            self.count -= 1;
            if self.count == 0 {
                self.summ = self.summ.wrapping_add(self.summ);
                self.count = 3 << self.shift;
                self.shift += 1;
            }
        }
    }
}

// --- sub-allocator (ported from compressor_ppmd_allocator.h) ---
struct Allocator {
    h: Heap,
    sub_allocator_size: u32,
    glue_count: u32,
    heap_start: u32,
    p_text: u32,
    units_start: u32,
    lo_unit: u32,
    hi_unit: u32,
    // BList[N_INDEXES] of BLK_NODE (Stamp:u32, next:u32) kept outside the heap.
    blist_stamp: [u32; N_INDEXES],
    blist_next: [u32; N_INDEXES],
    indx2units: [u8; N_INDEXES],
    units2indx: [u8; 128],
}

#[inline]
fn u2b(nu: u32) -> u32 {
    UNIT_SIZE * nu
}

impl Allocator {
    fn new(size_mb: u32) -> Allocator {
        let size = size_mb << 20;
        let mut a = Allocator {
            h: Heap::new(size),
            sub_allocator_size: size,
            glue_count: 0,
            heap_start: HEAP_BASE,
            p_text: HEAP_BASE,
            units_start: HEAP_BASE,
            lo_unit: HEAP_BASE,
            hi_unit: HEAP_BASE,
            blist_stamp: [0; N_INDEXES],
            blist_next: [0; N_INDEXES],
            indx2units: [0; N_INDEXES],
            units2indx: [0; 128],
        };
        // index tables
        let mut k: u32 = 1;
        let mut i = 0usize;
        while i < N1 {
            a.indx2units[i] = k as u8;
            i += 1;
            k += 1;
        }
        k += 1;
        while i < N1 + N2 {
            a.indx2units[i] = k as u8;
            i += 1;
            k += 2;
        }
        k += 1;
        while i < N1 + N2 + N3 {
            a.indx2units[i] = k as u8;
            i += 1;
            k += 3;
        }
        k += 1;
        while i < N1 + N2 + N3 + N4 {
            a.indx2units[i] = k as u8;
            i += 1;
            k += 4;
        }
        let mut i2 = 0usize;
        for kk in 0..128usize {
            i2 += (a.indx2units[i2] < (kk + 1) as u8) as usize;
            a.units2indx[kk] = i2 as u8;
        }
        a
    }

    // BLK_NODE helpers. A BLK_NODE list head is BList[i]; the in-heap MEM_BLKs
    // share the (Stamp,next) layout at their own address (Stamp@0, next@4, NU@8).
    fn blk_stamp(&self, addr: u32) -> u32 {
        self.h.u32(addr)
    }
    fn set_blk_stamp(&mut self, addr: u32, v: u32) {
        self.h.set_u32(addr, v);
    }
    fn blk_next(&self, addr: u32) -> u32 {
        self.h.u32(addr + 4)
    }
    fn set_blk_next(&mut self, addr: u32, v: u32) {
        self.h.set_u32(addr + 4, v);
    }
    fn blk_nu(&self, addr: u32) -> u32 {
        self.h.u32(addr + 8)
    }
    fn set_blk_nu(&mut self, addr: u32, v: u32) {
        self.h.set_u32(addr + 8, v);
    }

    // insert(pv,NU): link pv into list head i, set its stamp=~0, NU.
    fn list_insert(&mut self, i: usize, pv: u32, nu: u32) {
        // p->next = head.next; head.next = p;
        let head_next = self.blist_next[i];
        self.set_blk_next(pv, head_next);
        self.blist_next[i] = pv;
        self.set_blk_stamp(pv, !0u32);
        self.set_blk_nu(pv, nu);
        self.blist_stamp[i] += 1;
    }
    fn list_avail(&self, i: usize) -> bool {
        self.blist_next[i] != 0
    }
    fn list_remove(&mut self, i: usize) -> u32 {
        let p = self.blist_next[i];
        // unlink: head.next = p->next
        let pn = self.blk_next(p);
        self.blist_next[i] = pn;
        self.blist_stamp[i] -= 1;
        p
    }

    fn split_block(&mut self, pv: u32, old_indx: usize, new_indx: usize) {
        let mut u_diff = (self.indx2units[old_indx] - self.indx2units[new_indx]) as u32;
        let mut p = pv + u2b(self.indx2units[new_indx] as u32);
        let mut i = self.units2indx[(u_diff - 1) as usize] as usize;
        if self.indx2units[i] as u32 != u_diff {
            i -= 1;
            let k = self.indx2units[i] as u32;
            self.list_insert(i, p, k);
            p += u2b(k);
            u_diff -= k;
        }
        let j = self.units2indx[(u_diff - 1) as usize] as usize;
        self.list_insert(j, p, u_diff);
    }

    fn get_used_memory(&self) -> u32 {
        let mut ret = self
            .sub_allocator_size
            .wrapping_sub(self.hi_unit - self.lo_unit)
            .wrapping_sub(self.units_start - self.p_text);
        for i in 0..N_INDEXES {
            ret = ret.wrapping_sub(UNIT_SIZE * self.indx2units[i] as u32 * self.blist_stamp[i]);
        }
        ret
    }

    fn init_sub_allocator(&mut self) {
        for i in 0..N_INDEXES {
            self.blist_stamp[i] = 0;
            self.blist_next[i] = 0;
        }
        self.p_text = self.heap_start;
        self.hi_unit = self.heap_start + self.sub_allocator_size;
        let diff = UNIT_SIZE * (self.sub_allocator_size / 8 / UNIT_SIZE * 7);
        self.lo_unit = self.hi_unit - diff;
        self.units_start = self.lo_unit;
        self.glue_count = 0;
    }

    fn glue_free_blocks(&mut self) {
        // s0 is a temporary BLK_NODE head kept outside heap.
        let mut s0_next: u32 = 0;
        let mut s0_stamp: u32 = 0;
        if self.lo_unit != self.hi_unit {
            self.h.set_u8(self.lo_unit, 0);
        }
        // p0 starts as &s0
        // We collect into a singly-linked list via s0.
        // First pass: walk each BList, merge adjacent free blocks.
        // Build chain: p0->link(p) means p->next = p0->next ; p0->next = p ;
        // but here they iteratively set p0=p, linking forward. We emulate the
        // original "p0->link(p); p0=p;" which prepends — net effect builds a
        // list in s0.
        let mut p0_is_s0 = true;
        let mut p0_addr: u32 = 0; // valid only when !p0_is_s0
        for i in 0..N_INDEXES {
            while self.list_avail(i) {
                let p = self.list_remove(i);
                if self.blk_nu(p) == 0 {
                    continue;
                }
                // merge adjacent
                loop {
                    let p1 = p + self.blk_nu(p) * UNIT_SIZE;
                    if self.blk_stamp(p1) != !0u32 {
                        break;
                    }
                    let add = self.blk_nu(p1);
                    let cur = self.blk_nu(p);
                    self.set_blk_nu(p, cur + add);
                    self.set_blk_nu(p1, 0);
                }
                // p0->link(p): p->next = p0->next; p0->next = p
                if p0_is_s0 {
                    self.set_blk_next(p, s0_next);
                    s0_next = p;
                    s0_stamp += 1;
                    p0_is_s0 = false;
                    p0_addr = p;
                } else {
                    let p0_next = self.blk_next(p0_addr);
                    self.set_blk_next(p, p0_next);
                    self.set_blk_next(p0_addr, p);
                    p0_addr = p;
                }
            }
        }
        let _ = s0_stamp;
        // Second pass: redistribute.
        while s0_next != 0 {
            // remove from s0
            let p = s0_next;
            s0_next = self.blk_next(p);
            let mut sz = self.blk_nu(p);
            let mut pp = p;
            if sz == 0 {
                continue;
            }
            while sz > 128 {
                self.list_insert(N_INDEXES - 1, pp, 128);
                sz -= 128;
                pp += 128 * UNIT_SIZE; // p += 128 (MEM_BLK units)
            }
            let mut i = self.units2indx[(sz - 1) as usize] as usize;
            if self.indx2units[i] as u32 != sz {
                let k = sz - self.indx2units[i - 1] as u32;
                i -= 1;
                self.list_insert((k - 1) as usize, pp + (sz - k) * UNIT_SIZE, k);
                i += 1; // restore i to original (we used i-1 only for indx2units)
                i -= 1;
                let units = self.indx2units[i] as u32;
                self.list_insert(i, pp, units);
                continue;
            }
            let units = self.indx2units[i] as u32;
            self.list_insert(i, pp, units);
        }
        self.glue_count = 1 << 13;
    }

    fn alloc_units_rare(&mut self, indx: usize) -> u32 {
        if self.glue_count == 0 {
            self.glue_free_blocks();
            if self.list_avail(indx) {
                return self.list_remove(indx);
            }
        }
        let mut i = indx;
        loop {
            i += 1;
            if i == N_INDEXES {
                self.glue_count -= 1;
                let bytes = u2b(self.indx2units[indx] as u32);
                if self.units_start - self.p_text > bytes {
                    self.units_start -= bytes;
                    return self.units_start;
                }
                return 0;
            }
            if self.list_avail(i) {
                break;
            }
        }
        let ret = self.list_remove(i);
        self.split_block(ret, i, indx);
        ret
    }

    fn alloc_units(&mut self, nu: u32) -> u32 {
        let indx = self.units2indx[(nu - 1) as usize] as usize;
        if self.list_avail(indx) {
            return self.list_remove(indx);
        }
        let ret = self.lo_unit;
        self.lo_unit += u2b(self.indx2units[indx] as u32);
        if self.lo_unit <= self.hi_unit {
            return ret;
        }
        self.lo_unit -= u2b(self.indx2units[indx] as u32);
        self.alloc_units_rare(indx)
    }

    fn alloc_context(&mut self) -> u32 {
        if self.hi_unit != self.lo_unit {
            self.hi_unit -= UNIT_SIZE;
            return self.hi_unit;
        }
        if self.list_avail(0) {
            return self.list_remove(0);
        }
        self.alloc_units_rare(0)
    }

    fn units_cpy(&mut self, dst: u32, src: u32, nu: u32) {
        let n = (nu * UNIT_SIZE) as usize;
        let di = self.h.idx(dst);
        let si = self.h.idx(src);
        if di == si {
            return;
        }
        // copy with potential overlap handled like memcpy per-unit (forward)
        let mut tmp = vec![0u8; n];
        tmp.copy_from_slice(&self.h.mem[si..si + n]);
        self.h.mem[di..di + n].copy_from_slice(&tmp);
    }

    fn expand_units(&mut self, old_ptr: u32, old_nu: u32) -> u32 {
        let i0 = self.units2indx[(old_nu - 1) as usize] as usize;
        let i1 = self.units2indx[old_nu as usize] as usize; // (OldNU-1+1)
        if i0 == i1 {
            return old_ptr;
        }
        let ptr = self.alloc_units(old_nu + 1);
        if ptr != 0 {
            self.units_cpy(ptr, old_ptr, old_nu);
            self.list_insert(i0, old_ptr, old_nu);
        }
        ptr
    }

    fn shrink_units(&mut self, old_ptr: u32, old_nu: u32, new_nu: u32) -> u32 {
        let i0 = self.units2indx[(old_nu - 1) as usize] as usize;
        let i1 = self.units2indx[(new_nu - 1) as usize] as usize;
        if i0 == i1 {
            return old_ptr;
        }
        if self.list_avail(i1) {
            let ptr = self.list_remove(i1);
            self.units_cpy(ptr, old_ptr, new_nu);
            let u0 = self.indx2units[i0] as u32;
            self.list_insert(i0, old_ptr, u0);
            ptr
        } else {
            self.split_block(old_ptr, i0, i1);
            old_ptr
        }
    }

    fn free_units(&mut self, ptr: u32, nu: u32) {
        let indx = self.units2indx[(nu - 1) as usize] as usize;
        let u = self.indx2units[indx] as u32;
        self.list_insert(indx, ptr, u);
    }

    fn special_free_unit(&mut self, ptr: u32) {
        if ptr != self.units_start {
            self.list_insert(0, ptr, 1);
        } else {
            self.h.set_u32(ptr, !0u32);
            self.units_start += UNIT_SIZE;
        }
    }

    fn move_units_up(&mut self, old_ptr: u32, nu: u32) -> u32 {
        let indx = self.units2indx[(nu - 1) as usize] as usize;
        if old_ptr > self.units_start + 16 * 1024 || self.blist_next[indx] == 0 || old_ptr > self.blist_next[indx] {
            return old_ptr;
        }
        let ptr = self.list_remove(indx);
        self.units_cpy(ptr, old_ptr, nu);
        let units = self.indx2units[indx] as u32;
        if old_ptr != self.units_start {
            self.list_insert(indx, old_ptr, units);
        } else {
            self.units_start += u2b(units);
        }
        ptr
    }

    fn expand_text_area(&mut self) {
        let mut count = [0u32; N_INDEXES];
        loop {
            let p = self.units_start;
            if self.blk_stamp(p) != !0u32 {
                break;
            }
            let nu = self.blk_nu(p);
            self.units_start = p + nu * UNIT_SIZE;
            count[self.units2indx[(nu - 1) as usize] as usize] += 1;
            self.set_blk_stamp(p, 0);
        }
        for i in 0..N_INDEXES {
            if count[i] == 0 {
                continue;
            }
            // walk list i, unlinking blocks with Stamp==0
            // p starts at BList+i (head); we operate on p->next.
            let mut prev_is_head = true;
            let mut prev_addr: u32 = 0;
            loop {
                if count[i] == 0 {
                    break;
                }
                let next = if prev_is_head {
                    self.blist_next[i]
                } else {
                    self.blk_next(prev_addr)
                };
                if next == 0 {
                    break;
                }
                if self.blk_stamp(next) == 0 {
                    // unlink next
                    let nn = self.blk_next(next);
                    if prev_is_head {
                        self.blist_next[i] = nn;
                    } else {
                        self.set_blk_next(prev_addr, nn);
                    }
                    self.blist_stamp[i] -= 1;
                    count[i] -= 1;
                    // prev stays the same
                } else {
                    prev_is_head = false;
                    prev_addr = next;
                }
            }
        }
    }
}

struct Model {
    a: Allocator,
    // decoder/model state
    see2: Vec<See2>, // [24*32], indexed see2[row*32+col]
    dummy_see2: See2,
    max_context: u32,
    ns2bs_indx: [u8; 256],
    qtable: [u8; 260],
    found_state: u32, // address of a STATE, 0 == NULL
    init_esc: i32,
    order_fall: i32,
    run_length: i32,
    init_rl: i32,
    max_order: i32,
    char_mask: [u8; 256],
    num_masked: u8,
    prev_success: u8,
    esc_count: u8,
    print_count: u8,
    bin_summ: [[u16; 64]; 25],
    sub_range: SubRange,
    low: u32,
    code: u32,
    range: u32,
    start_first_time: bool,
    start_context: u32,
}

// input/output streams
struct InStream<'a> {
    data: &'a [u8],
    pos: usize,
}
impl<'a> InStream<'a> {
    fn get(&mut self) -> i32 {
        if self.pos < self.data.len() {
            let b = self.data[self.pos] as i32;
            self.pos += 1;
            b
        } else {
            // EOF sentinel; original returns EOF (-1) but range decoder keeps
            // shifting in bytes past end as 0xFF via (i32)(-1) & 0xff.
            -1
        }
    }
}
struct OutStream<'a> {
    data: &'a mut [u8],
    pos: usize,
}
impl<'a> OutStream<'a> {
    fn put(&mut self, b: u8) {
        if self.pos < self.data.len() {
            self.data[self.pos] = b;
        }
        self.pos += 1;
    }
}

impl Model {
    fn new() -> Model {
        let mut see2 = Vec::with_capacity(24 * 32);
        for _ in 0..(24 * 32) {
            see2.push(See2 { summ: 0, shift: 0, count: 0 });
        }
        let mut m = Model {
            a: Allocator::new(SUB_ALLOCATOR_SIZE_MB),
            see2,
            dummy_see2: See2 { summ: 0, shift: 0, count: 0 },
            max_context: 0,
            ns2bs_indx: [0; 256],
            qtable: [0; 260],
            found_state: 0,
            init_esc: 0,
            order_fall: 0,
            run_length: 0,
            init_rl: 0,
            max_order: 0,
            char_mask: [0; 256],
            num_masked: 0,
            prev_success: 0,
            esc_count: 0,
            print_count: 0,
            bin_summ: [[0u16; 64]; 25],
            sub_range: SubRange { low: 0, high: 0, scale: 0 },
            low: 0,
            code: 0,
            range: 0,
            start_first_time: true,
            start_context: 0,
        };
        // NS2BSIndx
        m.ns2bs_indx[0] = 0;
        m.ns2bs_indx[1] = 2;
        for i in 2..11 {
            m.ns2bs_indx[i] = 4;
        }
        for i in 11..256 {
            m.ns2bs_indx[i] = 6;
        }
        // QTable
        for i in 0..(UP_FREQ as usize) {
            m.qtable[i] = i as u8;
        }
        let mut mm = UP_FREQ;
        let mut k = 1u32;
        let mut step = 1u32;
        let mut i = UP_FREQ as usize;
        while i < 260 {
            m.qtable[i] = mm as u8;
            k -= 1;
            if k == 0 {
                step += 1;
                k = step;
                mm += 1;
            }
            i += 1;
        }
        // DummySEE2Cont gets PPMdSignature written over its bytes; only matters
        // for the trained-model path which is not used here.
        m.dummy_see2.summ = (PPMD_SIGNATURE & 0xffff) as u16;
        m.dummy_see2.shift = ((PPMD_SIGNATURE >> 16) & 0xff) as u8;
        m.dummy_see2.count = ((PPMD_SIGNATURE >> 24) & 0xff) as u8;
        m
    }

    // --- range coder ---
    fn rc_init_decoder(&mut self, inp: &mut InStream) {
        self.low = 0;
        self.code = 0;
        self.range = 0xFFFF_FFFF;
        for _ in 0..4 {
            self.code = (self.code << 8) | (inp.get() as u32 & 0xff);
        }
    }
    fn rc_dec_normalize(&mut self, inp: &mut InStream) {
        while (self.low ^ self.low.wrapping_add(self.range)) < TOP
            || (self.range < BOT && {
                self.range = self.low.wrapping_neg() & (BOT - 1);
                true
            })
        {
            self.code = (self.code << 8) | (inp.get() as u32 & 0xff);
            self.range <<= 8;
            self.low <<= 8;
        }
    }
    fn rc_get_current_count(&mut self) -> u32 {
        self.range /= self.sub_range.scale;
        (self.code - self.low) / self.range
    }
    fn rc_remove_subrange(&mut self) {
        self.low = self.low.wrapping_add(self.range.wrapping_mul(self.sub_range.low));
        self.range = self.range.wrapping_mul(self.sub_range.high - self.sub_range.low);
    }
    fn rc_bin_start(&mut self, f0: u32, shift: u32) -> u32 {
        self.range >>= shift;
        f0 * self.range
    }
    fn rc_bin_decode(&self, tmp: u32) -> u32 {
        (self.code - self.low >= tmp) as u32
    }
    fn rc_bin_correct0(&mut self, tmp: u32) {
        self.range = tmp;
    }
    fn rc_bin_correct1(&mut self, tmp: u32, f1: u32) {
        self.low = self.low.wrapping_add(tmp);
        self.range = self.range.wrapping_mul(f1);
    }
}

#[derive(Debug)]
pub enum PpmdError {
    Corrupt(&'static str),
}
impl std::fmt::Display for PpmdError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PpmdError::Corrupt(s) => write!(f, "corrupt stream: {s}"),
        }
    }
}
impl std::error::Error for PpmdError {}

/// Decompress a PPMd stream (`src`) into `dst` (must be sized to the exact
/// uncompressed length).
pub fn decompress(src: &[u8], dst: &mut [u8]) -> Result<(), PpmdError> {
    let mut m = Model::new();
    let mut inp = InStream { data: src, pos: 0 };
    let mut out = OutStream { data: dst, pos: 0 };
    m.decode_file(&mut inp, &mut out)?;
    Ok(())
}

include!("ppmd_model.rs");

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decoder_runs_without_panic() {
        // We have no real compressed sample in-repo, so this only checks that
        // the heap-address model and control flow don't panic while producing
        // a small number of bytes from arbitrary input.
        let src = vec![0xABu8; 64];
        let mut dst = vec![0u8; 32];
        let _ = decompress(&src, &mut dst);
    }
}
