// Inject a synthetic GPS L1 C/A signal of known PRN, code phase and Doppler into
// the emitted tracking channel and sweep the replica's code phase.
//
// Two things this gets right that the first attempt did not:
//
//  * the sample stream is strictly continuous -- the pointer only ever advances
//    by one per strobe. Jumping it mid-signal breaks the code phase and caps the
//    correlation.
//  * exactly 8000 samples (two whole code periods, since 1023 chips / 1.023 MHz
//    is 4000 samples at 4 MHz) are fed per swept phase. A whole number of periods
//    means the injected signal presents the *same* code phase at every restart,
//    so the peak stays at one replica phase across the sweep instead of drifting.
//
// The measurement is the last dump of each window, which is always a full-period
// integration: the first dump after a restart at chip p closes after
// (1023-p) chips, and another whole period then fits inside the remaining
// samples.
//
// The signal is noiseless and full scale, so the expected numbers are exact:
//   per-sample product  = 127 (carrier LUT peak) * 1061 (signal amplitude) = 134747
//   aligned prompt       = 134747 * 4000 = 5.39e8
//   misaligned prompt    = 134747 * 3.91 * {-1, 63, -65}, the Gold-code off-peak
//                          autocorrelation, i.e. ~5e5 to ~3.4e7
// So an aligned peak must stand ~1-3 orders of magnitude above the floor. No
// statistics: either it despreads or it does not.
`timescale 1ns/1ps

module tb_inject;
  localparam integer FRAC       = 24;
  localparam integer CODE_LEN   = 1023;
  integer CODE_STEP, CARRIER_FW_S, SPACING, EXPECT_PHASE;
  localparam integer CLK_PER_SA = 8;
  localparam integer NSAMP      = 128000;           // 32 code periods
  localparam integer PERIOD_SA  = 4000;
  localparam integer WINDOW     = 2 * PERIOD_SA;    // samples fed per swept phase

  reg sys_clk = 0, sys_rst = 0;
  always #4 sys_clk = ~sys_clk;

  reg signed [15:0] i_sample_i = 0, i_sample_q = 0;
  reg i_sample_stb = 0, i_carrier_set = 0, i_restart = 0;
  reg [63:0] i_sample_count = 0;
  reg [31:0] i_carrier_fw = 0, i_carrier_phase_in = 0;
  reg [23:0] i_code_step = 0, i_spacing = 0, i_code_phase_frac = 0;
  reg [9:0] i_code_phase_chip = 0;
  reg i_load_we = 0, i_load_dat = 0;
  reg [9:0] i_load_adr = 0;

  wire o_dump_stb, o_dump_saturated, o_saturated;
  wire [31:0] o_integrated_samples;
  wire [63:0] o_sample_index;
  wire [23:0] o_dump_code_phase;
  wire signed [31:0] o_acc_ie, o_acc_qe, o_acc_ip, o_acc_qp, o_acc_il, o_acc_ql;

  ch_wrap dut (.*);

  reg [15:0] smp_i [0:NSAMP-1];
  reg [15:0] smp_q [0:NSAMP-1];
  reg        chips [0:CODE_LEN-1];

  // Last dump of the current window.
  reg signed [31:0] last_ip, last_qp, last_ie, last_il;
  reg [31:0] last_n;
  reg        last_sat;
  integer    ndumps, win_dumps;

  always @(posedge sys_clk) begin
    if (o_dump_stb) begin
      last_ip <= o_acc_ip; last_qp <= o_acc_qp;
      last_ie <= o_acc_ie; last_il <= o_acc_il;
      last_n  <= o_integrated_samples;
      last_sat <= o_dump_saturated;
      ndumps <= ndumps + 1;
      win_dumps <= win_dumps + 1;
    end
  end

  integer pos;    // strictly monotonic sample pointer

  task load_chip(input integer addr, input bit dat);
    begin
      i_load_adr <= addr[9:0]; i_load_dat <= dat; i_load_we <= 1'b1;
      @(posedge sys_clk);
      i_load_we <= 1'b0;
      @(posedge sys_clk);
    end
  endtask

  // Feed n samples, continuing the stream. Never jumps.
  task feed(input integer n);
    integer s, c;
    begin
      for (s = 0; s < n; s = s + 1) begin
        i_sample_i     <= $signed(smp_i[pos % NSAMP]);
        i_sample_q     <= $signed(smp_q[pos % NSAMP]);
        i_sample_count <= pos;
        i_sample_stb   <= 1'b1;
        @(posedge sys_clk);
        i_sample_stb   <= 1'b0;
        for (c = 1; c < CLK_PER_SA; c = c + 1) @(posedge sys_clk);
        pos = pos + 1;
      end
    end
  endtask

  integer phase, k, fd, rc, best_phase;
  real    power, best_power, sum_power;
  real    prof [0:CODE_LEN-1];
  real    ip_at [0:CODE_LEN-1];

  initial begin
    // code_step, carrier frequency word (signed), expected peak phase.
    fd = $fopen("params.txt", "r");
    if (fd == 0) begin $display("FATAL: no params.txt"); $finish; end
    rc = $fscanf(fd, "%d %d %d\n", CODE_STEP, CARRIER_FW_S, EXPECT_PHASE);
    $fclose(fd);
    SPACING = 2 * CODE_STEP;
    $display("params: code_step=%0d carrier_fw=%0d expect_phase=%0d",
             CODE_STEP, CARRIER_FW_S, EXPECT_PHASE);

    fd = $fopen("samples.txt", "r");
    if (fd == 0) begin $display("FATAL: no samples.txt"); $finish; end
    for (k = 0; k < NSAMP; k = k + 1) begin
      rc = $fscanf(fd, "%h %h\n", smp_i[k], smp_q[k]);
      if (rc != 2) begin $display("FATAL: short samples.txt"); $finish; end
    end
    $fclose(fd);
    fd = $fopen("chips.txt", "r");
    for (k = 0; k < CODE_LEN; k = k + 1) rc = $fscanf(fd, "%b\n", chips[k]);
    $fclose(fd);
    $display("loaded %0d samples, %0d chips", NSAMP, CODE_LEN);

    ndumps = 0; pos = 0;
    sys_rst = 1; repeat (4) @(posedge sys_clk); sys_rst = 0;
    repeat (4) @(posedge sys_clk);

    for (k = 0; k < CODE_LEN; k = k + 1) load_chip(k, chips[k]);
    $display("code loaded (PRN 1, from GNSSSignals.jl)");

    // Injected signal has zero Doppler: park the carrier at DC so this measures
    // the code correlator alone.
    i_carrier_fw <= CARRIER_FW_S[31:0]; i_carrier_phase_in <= 32'd0;
    i_code_step  <= CODE_STEP[23:0]; i_spacing <= SPACING[23:0];

    best_power = 0.0; best_phase = 0; sum_power = 0.0;

    for (phase = 0; phase < CODE_LEN; phase = phase + 1) begin
      i_code_phase_chip <= phase[9:0];
      i_code_phase_frac <= 24'd0;
      i_carrier_set     <= 1'b1;
      i_restart         <= 1'b1;
      @(posedge sys_clk);
      i_carrier_set     <= 1'b0;
      i_restart         <= 1'b0;
      @(posedge sys_clk);

      win_dumps = 0;
      feed(WINDOW);

      power = $itor(last_ip) * $itor(last_ip) + $itor(last_qp) * $itor(last_qp);
      prof[phase]  = power;
      ip_at[phase] = $itor(last_ip);
      sum_power = sum_power + power;
      if (power > best_power) begin best_power = power; best_phase = phase; end
      if (phase % 256 == 0)
        $display("  phase %4d: ip=%0d qp=%0d n=%0d dumps=%0d sat=%b",
                 phase, last_ip, last_qp, last_n, win_dumps, last_sat);
    end

    $display("\n=== RESULT ===");
    $display("total dumps        : %0d", ndumps);
    $display("best replica phase : %0d chips", best_phase);
    $display("prompt at peak     : ip = %0.0f", ip_at[best_phase]);
    $display("expected if aligned: ip = %0d   (127 * 1061 * 4000)", 127*1061*4000);
    $display("mean power         : %0.3e", sum_power / CODE_LEN);
    $display("peak power         : %0.3e", best_power);
    $display("peak / mean        : %0.0fx", best_power / (sum_power / CODE_LEN));
    $display("expected peak phase: %0d chips", EXPECT_PHASE);
    if (best_phase == EXPECT_PHASE)
        $display("VERDICT: PEAK AT THE INJECTED CODE PHASE");
    else
        $display("VERDICT: peak at %0d, expected %0d", best_phase, EXPECT_PHASE);
    $display("\nprofile +-5 chips around the peak (ip):");
    for (k = -5; k <= 5; k = k + 1) begin
      phase = (best_phase + k + CODE_LEN) % CODE_LEN;
      $display("  phase %4d : ip = %14.0f", phase, ip_at[phase]);
    end
    $finish;
  end
endmodule
