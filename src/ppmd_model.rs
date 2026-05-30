// PPMd model + decoder logic, included into ppmd.rs (shares the `Model` type).
// Ported from compressor_ppmd.cpp. Only the decode path is implemented.

impl Model {
    fn h(&self) -> &Heap {
        &self.a.h
    }

    fn start_model_rare(&mut self, max_order: i32) {
        if self.start_first_time {
            for v in self.char_mask.iter_mut() {
                *v = 0;
            }
            self.esc_count = 1;
            self.print_count = 1;
            if max_order < 2 {
                // solid mode — not used here
                self.order_fall = self.max_order;
                let mut pc = self.max_context;
                while pc != 0 && ctx_suffix(self.h(), pc) != 0 {
                    self.order_fall -= 1;
                    pc = ctx_suffix(self.h(), pc);
                }
                return;
            }
            self.order_fall = max_order;
            self.max_order = max_order;

            self.a.init_sub_allocator();
            self.init_rl = -(if max_order < 12 { max_order } else { 12 }) - 1;
            self.run_length = self.init_rl;

            // BinSumm init
            let mut i = 0usize;
            for mm in 0..25usize {
                while self.qtable[i] as usize == mm {
                    i += 1;
                }
                for k in 0..8usize {
                    self.bin_summ[mm][k] = (BIN_SCALE - INIT_BIN_ESC[k] as u32 / (i as u32 + 1)) as u16;
                }
                let mut k = 8usize;
                while k < 64 {
                    let src = self.bin_summ[mm][0..8].to_vec();
                    self.bin_summ[mm][k..k + 8].copy_from_slice(&src);
                    k += 8;
                }
            }
            // SEE2 init
            let mut i = 0usize;
            for mm in 0..24usize {
                while self.qtable[i + 3] as usize == mm + 3 {
                    i += 1;
                }
                let init_val = 2 * i as u32 + 5;
                self.see2[mm * 32].init(init_val);
                let (s, sh, c) = {
                    let r = &self.see2[mm * 32];
                    (r.summ, r.shift, r.count)
                };
                for k in 1..32usize {
                    let d = &mut self.see2[mm * 32 + k];
                    d.summ = s;
                    d.shift = sh;
                    d.count = c;
                }
            }

            self.max_context = self.a.alloc_context();
            ctx_set_suffix(&mut self.a.h, self.max_context, 0);

            // No trained model: init 256 equal symbols.
            let mc = self.max_context;
            ctx_set_num_stats(&mut self.a.h, mc, 255);
            ctx_set_summ_freq(&mut self.a.h, mc, 255 + 2);
            let stats = self.a.alloc_units(256 / 2);
            ctx_set_stats(&mut self.a.h, mc, stats);
            self.prev_success = 0;
            for i in 0..256u32 {
                let s = stats + i * 6;
                st_set_symbol(&mut self.a.h, s, i as u8);
                st_set_freq(&mut self.a.h, s, 1);
                st_set_successor(&mut self.a.h, s, 0);
            }
            self.start_context = self.max_context;
        } else {
            for v in self.char_mask.iter_mut() {
                *v = 0;
            }
            self.esc_count = 1;
            self.print_count = 1;
            self.order_fall = max_order;
            self.max_order = max_order;
            self.init_rl = -(if max_order < 12 { max_order } else { 12 }) - 1;
            self.run_length = self.init_rl;
            self.max_context = self.start_context;
            self.found_state = 0;
        }
    }

    // --- model context methods ---

    fn refresh(&mut self, ctx: u32, old_nu: u32, scale: u32) {
        let mut i = ctx_num_stats(self.h(), ctx) as i32;
        let mut p = self.a.shrink_units(ctx_stats(self.h(), ctx), old_nu, (i as u32 + 2) >> 1);
        ctx_set_stats(&mut self.a.h, ctx, p);
        let flags = ctx_flags(self.h(), ctx);
        let psym = st_symbol(self.h(), p);
        ctx_set_flags(
            &mut self.a.h,
            ctx,
            (flags & (0x10 + 0x04 * scale as u8)) + 0x08 * (psym >= 0x40) as u8,
        );
        let pfreq = st_freq(self.h(), p) as u32;
        let mut esc_freq = ctx_summ_freq(self.h(), ctx) as i32 - pfreq as i32;
        let nf = (pfreq + scale) >> scale;
        st_set_freq(&mut self.a.h, p, nf as u8);
        let mut summ = nf;
        loop {
            p += 6;
            let pf = st_freq(self.h(), p) as u32;
            esc_freq -= pf as i32;
            let nf = (pf + scale) >> scale;
            st_set_freq(&mut self.a.h, p, nf as u8);
            summ += nf;
            let fl = ctx_flags(self.h(), ctx);
            let psym = st_symbol(self.h(), p);
            ctx_set_flags(&mut self.a.h, ctx, fl | 0x08 * (psym >= 0x40) as u8);
            i -= 1;
            if i == 0 {
                break;
            }
        }
        let ef = ((esc_freq as u32) + scale) >> scale;
        summ += ef;
        ctx_set_summ_freq(&mut self.a.h, ctx, summ as u16);
    }

    fn rescale(&mut self, ctx: u32) {
        let num_stats = ctx_num_stats(self.h(), ctx);
        let stats = ctx_stats(self.h(), ctx);
        let mut i = num_stats as u32;
        // move FoundState to front (bubble up)
        let mut p = self.found_state;
        while p != stats {
            swap_state(&mut self.a.h, p, p - 6);
            p -= 6;
        }
        // p == stats now
        let mut summ_freq = ctx_summ_freq(self.h(), ctx) as i32;
        let pf = st_freq(self.h(), p) as i32 + 4;
        st_set_freq(&mut self.a.h, p, pf as u8);
        summ_freq += 4;
        let mut esc_freq = summ_freq - pf;
        let adder = (self.order_fall != 0) as i32; // MRMethod==restart, so > freeze is false
        let new_pf = (pf + adder) >> 1;
        st_set_freq(&mut self.a.h, p, new_pf as u8);
        summ_freq = new_pf;
        loop {
            p += 6;
            esc_freq -= st_freq(self.h(), p) as i32;
            let nf = (st_freq(self.h(), p) as i32 + adder) >> 1;
            st_set_freq(&mut self.a.h, p, nf as u8);
            summ_freq += nf;
            // keep sorted by freq descending
            if st_freq(self.h(), p) > st_freq(self.h(), p - 6) {
                // tmp = *p; shift down while tmp.Freq > prev[-1].Freq
                let tmp_sym = st_symbol(self.h(), p);
                let tmp_freq = st_freq(self.h(), p);
                let tmp_suc = st_successor(self.h(), p);
                let mut p1 = p;
                loop {
                    state_cpy(&mut self.a.h, p1, p1 - 6);
                    p1 -= 6;
                    if !(tmp_freq > st_freq(self.h(), p1 - 6)) {
                        break;
                    }
                }
                st_set_symbol(&mut self.a.h, p1, tmp_sym);
                st_set_freq(&mut self.a.h, p1, tmp_freq);
                st_set_successor(&mut self.a.h, p1, tmp_suc);
            }
            i -= 1;
            if i == 0 {
                break;
            }
        }
        if st_freq(self.h(), p) == 0 {
            let mut cnt = 0u32;
            loop {
                cnt += 1;
                p -= 6;
                if st_freq(self.h(), p) != 0 {
                    break;
                }
            }
            esc_freq += cnt as i32;
            let old_nu = (num_stats as u32 + 2) >> 1;
            let new_num = num_stats as i32 - cnt as i32;
            ctx_set_num_stats(&mut self.a.h, ctx, new_num as u8);
            if new_num == 0 {
                // collapse to one state
                let tmp_sym = st_symbol(self.h(), stats);
                let mut tmp_freq = st_freq(self.h(), stats) as i32;
                let tmp_suc = st_successor(self.h(), stats);
                tmp_freq = (2 * tmp_freq + esc_freq - 1) / esc_freq;
                if tmp_freq > (MAX_FREQ as i32) / 3 {
                    tmp_freq = (MAX_FREQ as i32) / 3;
                }
                self.a.free_units(stats, old_nu);
                let one = ctx_one_state(ctx);
                st_set_symbol(&mut self.a.h, one, tmp_sym);
                st_set_freq(&mut self.a.h, one, tmp_freq as u8);
                st_set_successor(&mut self.a.h, one, tmp_suc);
                let flags = ctx_flags(self.h(), ctx);
                ctx_set_flags(&mut self.a.h, ctx, (flags & 0x10) + 0x08 * (tmp_sym >= 0x40) as u8);
                self.found_state = ctx_one_state(ctx);
                return;
            }
            let new_stats = self.a.shrink_units(stats, old_nu, (new_num as u32 + 2) >> 1);
            ctx_set_stats(&mut self.a.h, ctx, new_stats);
            let mut flags = ctx_flags(self.h(), ctx) & !0x08;
            let mut i2 = new_num as u32;
            let mut p2 = new_stats;
            flags |= 0x08 * (st_symbol(self.h(), p2) >= 0x40) as u8;
            loop {
                p2 += 6;
                flags |= 0x08 * (st_symbol(self.h(), p2) >= 0x40) as u8;
                i2 -= 1;
                if i2 == 0 {
                    break;
                }
            }
            ctx_set_flags(&mut self.a.h, ctx, flags);
            // p now points to last (with-freq-0 region trimmed); recompute below
        }
        esc_freq -= esc_freq >> 1;
        summ_freq += esc_freq;
        ctx_set_summ_freq(&mut self.a.h, ctx, summ_freq as u16);
        let flags = ctx_flags(self.h(), ctx) | 0x04;
        ctx_set_flags(&mut self.a.h, ctx, flags);
        self.found_state = ctx_stats(self.h(), ctx);
    }

    fn create_successors(&mut self, skip: bool, p_in: u32, pc_in: u32) -> u32 {
        let up_branch = st_successor(self.h(), self.found_state);
        let mut ps: [u32; MAX_O] = [0; MAX_O];
        let mut pps = 0usize;
        let sym = st_symbol(self.h(), self.found_state);
        let mut pc = pc_in;
        let mut p = p_in;

        if !skip {
            ps[pps] = self.found_state;
            pps += 1;
            if ctx_suffix(self.h(), pc) == 0 {
                // goto NO_LOOP
                return self.cs_no_loop(&ps, pps, up_branch, pc);
            }
        }
        if p != 0 {
            pc = ctx_suffix(self.h(), pc);
            // goto LOOP_ENTRY
        } else {
            loop {
                pc = ctx_suffix(self.h(), pc);
                if ctx_num_stats(self.h(), pc) != 0 {
                    p = ctx_stats(self.h(), pc);
                    if st_symbol(self.h(), p) != sym {
                        loop {
                            let t = st_symbol(self.h(), p + 6);
                            p += 6;
                            if t == sym {
                                break;
                            }
                        }
                    }
                    let tmp = (st_freq(self.h(), p) < MAX_FREQ - 9) as u8;
                    { let _v = st_freq(self.h(), p) + tmp; st_set_freq(&mut self.a.h, p, _v); }
                    let sf = ctx_summ_freq(self.h(), pc) + tmp as u16;
                    ctx_set_summ_freq(&mut self.a.h, pc, sf);
                } else {
                    p = ctx_one_state(pc);
                    let suffix_num = ctx_num_stats(self.h(), ctx_suffix(self.h(), pc));
                    let inc = ((suffix_num == 0) as u8) & ((st_freq(self.h(), p) < 24) as u8);
                    { let _v = st_freq(self.h(), p) + inc; st_set_freq(&mut self.a.h, p, _v); }
                }
                // LOOP_ENTRY
                if st_successor(self.h(), p) != up_branch {
                    pc = st_successor(self.h(), p);
                    break;
                }
                ps[pps] = p;
                pps += 1;
                if ctx_suffix(self.h(), pc) == 0 {
                    break;
                }
            }
            return self.cs_finish(&ps, pps, up_branch, pc, sym);
        }
        // LOOP_ENTRY path when p != 0 initially:
        loop {
            // (entered via goto LOOP_ENTRY: skip the per-iteration freq bump)
            if st_successor(self.h(), p) != up_branch {
                pc = st_successor(self.h(), p);
                break;
            }
            ps[pps] = p;
            pps += 1;
            if ctx_suffix(self.h(), pc) == 0 {
                break;
            }
            pc = ctx_suffix(self.h(), pc);
            if ctx_num_stats(self.h(), pc) != 0 {
                p = ctx_stats(self.h(), pc);
                if st_symbol(self.h(), p) != sym {
                    loop {
                        let t = st_symbol(self.h(), p + 6);
                        p += 6;
                        if t == sym {
                            break;
                        }
                    }
                }
                let tmp = (st_freq(self.h(), p) < MAX_FREQ - 9) as u8;
                { let _v = st_freq(self.h(), p) + tmp; st_set_freq(&mut self.a.h, p, _v); }
                let sf = ctx_summ_freq(self.h(), pc) + tmp as u16;
                ctx_set_summ_freq(&mut self.a.h, pc, sf);
            } else {
                p = ctx_one_state(pc);
                let suffix_num = ctx_num_stats(self.h(), ctx_suffix(self.h(), pc));
                let inc = ((suffix_num == 0) as u8) & ((st_freq(self.h(), p) < 24) as u8);
                { let _v = st_freq(self.h(), p) + inc; st_set_freq(&mut self.a.h, p, _v); }
            }
        }
        self.cs_finish(&ps, pps, up_branch, pc, sym)
    }

    fn cs_no_loop(&mut self, ps: &[u32; MAX_O], pps: usize, up_branch: u32, pc: u32) -> u32 {
        self.cs_finish(ps, pps, up_branch, pc, st_symbol(self.h(), self.found_state))
    }

    fn cs_finish(&mut self, ps: &[u32; MAX_O], mut pps: usize, up_branch: u32, mut pc: u32, sym_in: u8) -> u32 {
        if pps == 0 {
            return pc;
        }
        // Build template context ct (kept in locals).
        let mut sym = sym_in;
        let mut ct_num_stats: u8 = 0;
        let mut ct_flags: u8 = 0x10 * (sym >= 0x40) as u8;
        // ct.oneState().Symbol = sym = *(BYTE*)UpBranch
        sym = self.h().u8(up_branch);
        let ct_one_symbol = sym;
        let ct_one_successor = up_branch + 1;
        ct_flags |= 0x08 * (sym >= 0x40) as u8;
        let ct_one_freq: u8;
        if ctx_num_stats(self.h(), pc) != 0 {
            let mut p = ctx_stats(self.h(), pc);
            if st_symbol(self.h(), p) != sym {
                loop {
                    let t = st_symbol(self.h(), p + 6);
                    p += 6;
                    if t == sym {
                        break;
                    }
                }
            }
            let cf = st_freq(self.h(), p) as i32 - 1;
            let s0 = ctx_summ_freq(self.h(), pc) as i32 - ctx_num_stats(self.h(), pc) as i32 - cf;
            let val = if 2 * cf <= s0 {
                (5 * cf > s0) as i32
            } else {
                (cf + 2 * s0 - 3) / s0
            };
            ct_one_freq = (1 + val) as u8;
        } else {
            ct_one_freq = st_freq(self.h(), ctx_one_state(pc));
        }
        let _ = ct_num_stats;
        // Allocate contexts walking ps[] backwards.
        loop {
            let pc1 = self.a.alloc_context();
            if pc1 == 0 {
                return 0;
            }
            ct_num_stats = 0;
            ctx_set_num_stats(&mut self.a.h, pc1, ct_num_stats);
            ctx_set_flags(&mut self.a.h, pc1, ct_flags);
            // ct.SummFreq overlaps oneState; we set one_state fields directly.
            let one = ctx_one_state(pc1);
            st_set_symbol(&mut self.a.h, one, ct_one_symbol);
            st_set_freq(&mut self.a.h, one, ct_one_freq);
            st_set_successor(&mut self.a.h, one, ct_one_successor);
            ctx_set_suffix(&mut self.a.h, pc1, pc);
            pps -= 1;
            let p = ps[pps];
            st_set_successor(&mut self.a.h, p, pc1);
            pc = pc1;
            if pps == 0 {
                break;
            }
        }
        pc
    }

    fn reduce_order(&mut self, p_in: u32, pc_in: u32) -> u32 {
        let mut ps: [u32; MAX_O] = [0; MAX_O];
        let mut pps = 0usize;
        let pc1 = pc_in;
        let up_branch = self.a.p_text;
        let sym = st_symbol(self.h(), self.found_state);
        let mut p = p_in;
        let mut pc = pc_in;

        ps[pps] = self.found_state;
        pps += 1;
        st_set_successor(&mut self.a.h, self.found_state, up_branch);
        self.order_fall += 1;

        if p != 0 {
            pc = ctx_suffix(self.h(), pc);
            // goto LOOP_ENTRY
            loop {
                if st_successor(self.h(), p) != 0 {
                    break;
                }
                ps[pps] = p;
                pps += 1;
                st_set_successor(&mut self.a.h, p, up_branch);
                self.order_fall += 1;
                // continue main loop body
                pc = ctx_suffix(self.h(), pc);
                if pc == 0 {
                    // !pc->Suffix handled at top in C; here Suffix(pc)==0 means
                    // we reached root: replicate the "!pc->Suffix" branch.
                    return p; // MRMethod==restart: returns pc, but loop structure
                }
                if ctx_num_stats(self.h(), pc) != 0 {
                    p = ctx_stats(self.h(), pc);
                    if st_symbol(self.h(), p) != sym {
                        loop {
                            let t = st_symbol(self.h(), p + 6);
                            p += 6;
                            if t == sym {
                                break;
                            }
                        }
                    }
                    let tmp = 2 * (st_freq(self.h(), p) < MAX_FREQ - 9) as u8;
                    { let _v = st_freq(self.h(), p) + tmp; st_set_freq(&mut self.a.h, p, _v); }
                    let sf = ctx_summ_freq(self.h(), pc) + tmp as u16;
                    ctx_set_summ_freq(&mut self.a.h, pc, sf);
                } else {
                    p = ctx_one_state(pc);
                    let inc = (st_freq(self.h(), p) < 32) as u8;
                    { let _v = st_freq(self.h(), p) + inc; st_set_freq(&mut self.a.h, p, _v); }
                }
            }
        } else {
            loop {
                if ctx_suffix(self.h(), pc) == 0 {
                    return pc;
                }
                pc = ctx_suffix(self.h(), pc);
                if ctx_num_stats(self.h(), pc) != 0 {
                    p = ctx_stats(self.h(), pc);
                    if st_symbol(self.h(), p) != sym {
                        loop {
                            let t = st_symbol(self.h(), p + 6);
                            p += 6;
                            if t == sym {
                                break;
                            }
                        }
                    }
                    let tmp = 2 * (st_freq(self.h(), p) < MAX_FREQ - 9) as u8;
                    { let _v = st_freq(self.h(), p) + tmp; st_set_freq(&mut self.a.h, p, _v); }
                    let sf = ctx_summ_freq(self.h(), pc) + tmp as u16;
                    ctx_set_summ_freq(&mut self.a.h, pc, sf);
                } else {
                    p = ctx_one_state(pc);
                    let inc = (st_freq(self.h(), p) < 32) as u8;
                    { let _v = st_freq(self.h(), p) + inc; st_set_freq(&mut self.a.h, p, _v); }
                }
                // LOOP_ENTRY
                if st_successor(self.h(), p) != 0 {
                    break;
                }
                ps[pps] = p;
                pps += 1;
                st_set_successor(&mut self.a.h, p, up_branch);
                self.order_fall += 1;
            }
        }

        // MRMethod==restart, so NOT > freeze. p->Successor <= UpBranch check:
        if st_successor(self.h(), p) <= up_branch {
            let p1 = self.found_state;
            self.found_state = p;
            let new_suc = self.create_successors(false, 0, pc);
            st_set_successor(&mut self.a.h, p, new_suc);
            self.found_state = p1;
        }
        if self.order_fall == 1 && pc1 == self.max_context {
            let suc = st_successor(self.h(), p);
            st_set_successor(&mut self.a.h, self.found_state, suc);
            self.a.p_text -= 1;
        }
        st_successor(self.h(), p)
    }

    fn update_model(&mut self, min_context: u32) {
        let mut p: u32 = 0;
        let mut f_successor = st_successor(self.h(), self.found_state);
        let pc = ctx_suffix(self.h(), min_context);
        let mut pc1 = self.max_context;
        let f_freq = st_freq(self.h(), self.found_state) as u32;
        let f_symbol = st_symbol(self.h(), self.found_state);

        if f_freq < (MAX_FREQ as u32) / 4 && pc != 0 {
            if ctx_num_stats(self.h(), pc) != 0 {
                p = ctx_stats(self.h(), pc);
                if st_symbol(self.h(), p) != f_symbol {
                    loop {
                        let s = st_symbol(self.h(), p + 6);
                        p += 6;
                        if s == f_symbol {
                            break;
                        }
                    }
                    if st_freq(self.h(), p) >= st_freq(self.h(), p - 6) {
                        swap_state(&mut self.a.h, p, p - 6);
                        p -= 6;
                    }
                }
                let cf = 2 * (st_freq(self.h(), p) < MAX_FREQ - 9) as u8;
                { let _v = st_freq(self.h(), p) + cf; st_set_freq(&mut self.a.h, p, _v); }
                let sf = ctx_summ_freq(self.h(), pc) + cf as u16;
                ctx_set_summ_freq(&mut self.a.h, pc, sf);
            } else {
                p = ctx_one_state(pc);
                let inc = (st_freq(self.h(), p) < 32) as u8;
                { let _v = st_freq(self.h(), p) + inc; st_set_freq(&mut self.a.h, p, _v); }
            }
        }

        if self.order_fall == 0 && f_successor != 0 {
            let ns = self.create_successors(true, p, min_context);
            if ns == 0 {
                self.restart_model(pc1, min_context, f_successor);
                return;
            }
            st_set_successor(&mut self.a.h, self.found_state, ns);
            self.max_context = ns;
            return;
        }

        self.a.h.set_u8(self.a.p_text, f_symbol);
        self.a.p_text += 1;
        let mut successor = self.a.p_text;

        if self.a.p_text >= self.a.units_start {
            self.restart_model(pc1, min_context, f_successor);
            return;
        }

        if f_successor != 0 {
            if f_successor < self.a.units_start {
                f_successor = self.create_successors(false, p, min_context);
                if f_successor == 0 {
                    self.restart_model(pc1, min_context, f_successor);
                    return;
                }
            }
        } else {
            f_successor = self.reduce_order(p, min_context);
            if f_successor == 0 {
                self.restart_model(pc1, min_context, f_successor);
                return;
            }
        }

        self.order_fall -= 1;
        if self.order_fall == 0 {
            successor = f_successor;
            if self.max_context != min_context {
                self.a.p_text -= 1;
            }
        }
        // (MRMethod==restart, so the >freeze branch is skipped)

        let ns = ctx_num_stats(self.h(), min_context) as u32;
        let s0 = ctx_summ_freq(self.h(), min_context) as i32 - ns as i32 - f_freq as i32;
        let flag = 0x08 * (f_symbol >= 0x40) as u8;
        while pc1 != min_context {
            let ns1 = ctx_num_stats(self.h(), pc1) as u32;
            if ns1 != 0 {
                if (ns1 & 1) != 0 {
                    let np = self.a.expand_units(ctx_stats(self.h(), pc1), (ns1 + 1) >> 1);
                    if np == 0 {
                        self.restart_model(pc1, min_context, f_successor);
                        return;
                    }
                    ctx_set_stats(&mut self.a.h, pc1, np);
                }
                let add = (3 * ns1 + 1 < ns) as u16;
                let sf = ctx_summ_freq(self.h(), pc1) + add;
                ctx_set_summ_freq(&mut self.a.h, pc1, sf);
            } else {
                let np = self.a.alloc_units(1);
                if np == 0 {
                    self.restart_model(pc1, min_context, f_successor);
                    return;
                }
                state_cpy(&mut self.a.h, np, ctx_one_state(pc1));
                ctx_set_stats(&mut self.a.h, pc1, np);
                let f = st_freq(self.h(), np) as u32;
                if f < (MAX_FREQ as u32) / 4 - 1 {
                    st_set_freq(&mut self.a.h, np, (f + f) as u8);
                } else {
                    st_set_freq(&mut self.a.h, np, MAX_FREQ - 4);
                }
                let sf = st_freq(self.h(), np) as i32 + self.init_esc + (ns > 2) as i32;
                ctx_set_summ_freq(&mut self.a.h, pc1, sf as u16);
            }

            let cf_full = 2 * f_freq as i32 * (ctx_summ_freq(self.h(), pc1) as i32 + 6);
            let sf2 = s0 + ctx_summ_freq(self.h(), pc1) as i32;
            let cf;
            if cf_full < 6 * sf2 {
                cf = 1 + (cf_full > sf2) as i32 + (cf_full >= 4 * sf2) as i32;
                let nsf = ctx_summ_freq(self.h(), pc1) + 4;
                ctx_set_summ_freq(&mut self.a.h, pc1, nsf);
            } else {
                cf = 4 + (cf_full > 9 * sf2) as i32 + (cf_full > 12 * sf2) as i32 + (cf_full > 15 * sf2) as i32;
                let nsf = ctx_summ_freq(self.h(), pc1) + cf as u16;
                ctx_set_summ_freq(&mut self.a.h, pc1, nsf);
            }
            let new_ns = ctx_num_stats(self.h(), pc1) + 1;
            ctx_set_num_stats(&mut self.a.h, pc1, new_ns);
            let np = ctx_stats(self.h(), pc1) + new_ns as u32 * 6;
            st_set_successor(&mut self.a.h, np, successor);
            st_set_symbol(&mut self.a.h, np, f_symbol);
            st_set_freq(&mut self.a.h, np, cf as u8);
            let fl = ctx_flags(self.h(), pc1) | flag;
            ctx_set_flags(&mut self.a.h, pc1, fl);

            pc1 = ctx_suffix(self.h(), pc1);
        }
        self.max_context = f_successor;
    }

    fn restart_model(&mut self, _pc1: u32, _min: u32, _fs: u32) {
        // MRMethod == model_restoration_restart: restart from scratch.
        // (RestoreModelRare's restart branch ultimately calls StartModelRare.)
        self.esc_count = 0;
        self.print_count = 0xFF;
        // Re-init via StartModelRare (first_time path stays true in this build,
        // since StartModelRare_first_time is never set to false — see C++).
        let mo = self.max_order;
        self.start_model_rare(mo);
    }

    // --- SEE / escape helpers used by decodeSymbol2 ---
    fn make_esc_freq2(&mut self, ctx: u32) -> i32 {
        let num_stats = ctx_num_stats(self.h(), ctx);
        if num_stats != 0xFF {
            let suffix_ns = ctx_num_stats(self.h(), ctx_suffix(self.h(), ctx)) as u32;
            let row = self.qtable[num_stats as usize + 2] as usize - 3;
            let mut idx = row * 32;
            idx += (ctx_summ_freq(self.h(), ctx) as u32 > 11 * (num_stats as u32 + 1)) as usize;
            idx += 2 * ((2 * num_stats as u32) < (suffix_ns + self.num_masked as u32)) as usize
                + ctx_flags(self.h(), ctx) as usize;
            let mean = self.see2[idx].get_mean();
            self.sub_range.scale = mean;
            idx as i32
        } else {
            self.sub_range.scale = 1;
            -1
        }
    }

    fn update1(&mut self, ctx: u32, mut p: u32) {
        self.found_state = p;
        let nf = st_freq(self.h(), p) + 4;
        st_set_freq(&mut self.a.h, p, nf);
        let sf = ctx_summ_freq(self.h(), ctx) + 4;
        ctx_set_summ_freq(&mut self.a.h, ctx, sf);
        if st_freq(self.h(), p) > st_freq(self.h(), p - 6) {
            swap_state(&mut self.a.h, p, p - 6);
            p -= 6;
            self.found_state = p;
            if st_freq(self.h(), p) > MAX_FREQ {
                self.rescale(ctx);
            }
        }
    }

    fn update2(&mut self, ctx: u32, p: u32) {
        self.found_state = p;
        let nf = st_freq(self.h(), p) + 4;
        st_set_freq(&mut self.a.h, p, nf);
        let sf = ctx_summ_freq(self.h(), ctx) + 4;
        ctx_set_summ_freq(&mut self.a.h, ctx, sf);
        if st_freq(self.h(), p) > MAX_FREQ {
            self.rescale(ctx);
        }
        self.esc_count += 1;
        self.run_length = self.init_rl;
    }

    fn decode_bin_symbol(&mut self, ctx: u32) {
        let suffix = ctx_suffix(self.h(), ctx);
        let indx = self.ns2bs_indx[ctx_num_stats(self.h(), suffix) as usize] as usize
            + self.prev_success as usize
            + ctx_flags(self.h(), ctx) as usize;
        let rs = ctx_one_state(ctx);
        let qi = self.qtable[(st_freq(self.h(), rs) - 1) as usize] as usize;
        let col = indx + (((self.run_length >> 26) & 0x20) as usize);
        let bs = self.bin_summ[qi][col] as u32;
        let tmp = self.rc_bin_start(bs, TOT_BITS);
        if self.rc_bin_decode(tmp) == 0 {
            self.found_state = rs;
            let f = st_freq(self.h(), rs);
            st_set_freq(&mut self.a.h, rs, f + (f < 196) as u8);
            self.rc_bin_correct0(tmp);
            self.bin_summ[qi][col] = (bs + INTERVAL - get_mean(bs, PERIOD_BITS, 2)) as u16;
            self.prev_success = 1;
            self.run_length += 1;
        } else {
            self.rc_bin_correct1(tmp, BIN_SCALE - bs);
            let nb = bs - get_mean(bs, PERIOD_BITS, 2);
            self.bin_summ[qi][col] = nb as u16;
            self.init_esc = EXP_ESCAPE[(nb >> 10) as usize] as i32;
            let sym = st_symbol(self.h(), rs);
            self.char_mask[sym as usize] = self.esc_count;
            self.num_masked = 0;
            self.prev_success = 0;
            self.found_state = 0;
        }
    }

    fn decode_symbol1(&mut self, ctx: u32) {
        self.sub_range.scale = ctx_summ_freq(self.h(), ctx) as u32;
        let stats = ctx_stats(self.h(), ctx);
        let mut p = stats;
        let mut hi_cnt = st_freq(self.h(), p) as u32;
        let count = self.rc_get_current_count();
        if count < hi_cnt {
            self.sub_range.high = hi_cnt;
            self.prev_success = (2 * hi_cnt >= self.sub_range.scale) as u8;
            self.found_state = p;
            hi_cnt += 4;
            st_set_freq(&mut self.a.h, p, hi_cnt as u8);
            let sf = ctx_summ_freq(self.h(), ctx) + 4;
            ctx_set_summ_freq(&mut self.a.h, ctx, sf);
            self.run_length += self.prev_success as i32;
            if hi_cnt > MAX_FREQ as u32 {
                self.rescale(ctx);
            }
            self.sub_range.low = 0;
            return;
        }
        self.prev_success = 0;
        let mut i = ctx_num_stats(self.h(), ctx) as i32;
        loop {
            p += 6;
            hi_cnt += st_freq(self.h(), p) as u32;
            if hi_cnt > count {
                break;
            }
            i -= 1;
            if i == 0 {
                self.sub_range.low = hi_cnt;
                let sym = st_symbol(self.h(), p);
                self.char_mask[sym as usize] = self.esc_count;
                self.num_masked = ctx_num_stats(self.h(), ctx);
                i = self.num_masked as i32;
                self.found_state = 0;
                let mut pp = p;
                loop {
                    pp -= 6;
                    let s = st_symbol(self.h(), pp);
                    self.char_mask[s as usize] = self.esc_count;
                    i -= 1;
                    if i == 0 {
                        break;
                    }
                }
                self.sub_range.high = self.sub_range.scale;
                return;
            }
        }
        self.sub_range.high = hi_cnt;
        self.sub_range.low = hi_cnt - st_freq(self.h(), p) as u32;
        self.update1(ctx, p);
    }

    fn decode_symbol2(&mut self, ctx: u32) {
        let see_idx = self.make_esc_freq2(ctx);
        let mut hi_cnt = 0u32;
        let num_stats = ctx_num_stats(self.h(), ctx);
        let mut i = num_stats as i32 - self.num_masked as i32;
        let mut ps: [u32; 256] = [0; 256];
        let mut pps = 0usize;
        let mut p = ctx_stats(self.h(), ctx).wrapping_sub(6);
        loop {
            loop {
                let sym = st_symbol(self.h(), p + 6);
                p += 6;
                if self.char_mask[sym as usize] != self.esc_count {
                    break;
                }
            }
            hi_cnt += st_freq(self.h(), p) as u32;
            ps[pps] = p;
            pps += 1;
            i -= 1;
            if i == 0 {
                break;
            }
        }
        self.sub_range.scale += hi_cnt;
        let count = self.rc_get_current_count();
        let mut pidx = 0usize;
        let mut pp = ps[pidx];
        if count < hi_cnt {
            hi_cnt = 0;
            loop {
                hi_cnt += st_freq(self.h(), pp) as u32;
                if hi_cnt > count {
                    break;
                }
                pidx += 1;
                pp = ps[pidx];
            }
            self.sub_range.high = hi_cnt;
            self.sub_range.low = hi_cnt - st_freq(self.h(), pp) as u32;
            self.see_update(see_idx);
            self.update2(ctx, pp);
        } else {
            self.sub_range.low = hi_cnt;
            self.sub_range.high = self.sub_range.scale;
            let mut j = num_stats as i32 - self.num_masked as i32;
            self.num_masked = num_stats;
            let mut k = 0usize;
            loop {
                let sym = st_symbol(self.h(), ps[k]);
                self.char_mask[sym as usize] = self.esc_count;
                k += 1;
                j -= 1;
                if j == 0 {
                    break;
                }
            }
            let scale = self.sub_range.scale;
            self.see_correct_add(see_idx, scale);
        }
    }

    fn see_update(&mut self, idx: i32) {
        if idx >= 0 {
            self.see2[idx as usize].update();
        }
    }
    fn see_correct_add(&mut self, idx: i32, scale: u32) {
        if idx >= 0 {
            let s = &mut self.see2[idx as usize];
            s.summ = (s.summ as u32).wrapping_add(scale) as u16;
        } else {
            let s = &mut self.dummy_see2;
            s.summ = (s.summ as u32).wrapping_add(scale) as u16;
        }
    }

    fn decode_file(&mut self, inp: &mut InStream, out: &mut OutStream) -> Result<(), PpmdError> {
        self.rc_init_decoder(inp);
        self.start_model_rare(ORDER_MODEL);

        let mut min_context = self.max_context;
        let mut ns = ctx_num_stats(self.h(), min_context);
        loop {
            if ns != 0 {
                self.decode_symbol1(min_context);
                self.rc_remove_subrange();
            } else {
                self.decode_bin_symbol(min_context);
            }

            while self.found_state == 0 {
                self.rc_dec_normalize(inp);
                loop {
                    self.order_fall += 1;
                    min_context = ctx_suffix(self.h(), min_context);
                    if min_context == 0 {
                        return Ok(()); // STOP_DECODING
                    }
                    if ctx_num_stats(self.h(), min_context) != self.num_masked {
                        break;
                    }
                }
                self.decode_symbol2(min_context);
                self.rc_remove_subrange();
            }

            let sym = st_symbol(self.h(), self.found_state);
            out.put(sym);
            if out.pos >= out.data.len() {
                // produced all requested bytes
                return Ok(());
            }

            if self.order_fall == 0 && st_successor(self.h(), self.found_state) >= self.a.units_start {
                self.max_context = st_successor(self.h(), self.found_state);
            } else {
                self.update_model(min_context);
                if self.esc_count == 0 {
                    self.clear_mask();
                }
            }
            min_context = self.max_context;
            ns = ctx_num_stats(self.h(), min_context);
            self.rc_dec_normalize(inp);
        }
    }

    fn clear_mask(&mut self) {
        self.esc_count = 1;
        for v in self.char_mask.iter_mut() {
            *v = 0;
        }
        self.print_count = self.print_count.wrapping_add(1);
    }
}
