// iici_rotate.v - the Portrait Display turned a quarter turn for a monitor on its side: the
// 640 x 870 picture written into a DDR frame buffer as 870 x 640 for the scaler (MISTER_FB),
// 8 bpp through a grey palette.  Eight lines collect in block RAM, then leave as 640 64-bit
// words with the busy handshake; three buffers (two with FB_LL), as sys/arcade_video.v's.

`timescale 1ns/1ps

module iici_rotate #(
  parameter W        = 640,              // pixels a line coming in
  parameter H        = 870,              // lines a frame coming in
  parameter STRIDE_W = 110,              // 64-bit words an out row (880 bytes, a multiple of 16)
  parameter [6:0] MEM_BASE = 7'b0010010  // DDR byte address 0x24000000, three 8 MB slots
) (
  input             clk,                 // CLK_VIDEO
  input             ce,                  // CE_PIXEL
  input             de,                  // active picture
  input             vblank,
  input       [7:0] grey,

  input             enable_in,           // the Portrait Display with rotation chosen (the OSD's: any clock)
  input             cw_in,               // 1 clockwise, 0 anticlockwise (taken at each frame's start)

  output            FB_EN,
  output      [4:0] FB_FORMAT,
  output     [11:0] FB_WIDTH,
  output     [11:0] FB_HEIGHT,
  output     [31:0] FB_BASE,
  output     [13:0] FB_STRIDE,
  input             FB_VBL,
  input             FB_LL,

  output            FB_PAL_CLK,
  output      [7:0] FB_PAL_ADDR,
  output     [23:0] FB_PAL_DOUT,
  output            FB_PAL_WR,

  output            DDRAM_CLK,
  input             DDRAM_BUSY,
  output      [7:0] DDRAM_BURSTCNT,
  output     [28:0] DDRAM_ADDR,
  output     [63:0] DDRAM_DIN,
  output      [7:0] DDRAM_BE,
  output            DDRAM_WE,
  output            DDRAM_RD,

  output reg        overrun              // sticky: a strip ended before the last one had left
);

  localparam [2:0] PHASE = (8 - (H % 8)) % 8;   // clockwise: lines missing from the first strip
  localparam       TOPW  = (H - 1 + PHASE) / 8; // clockwise: the word column of the first strip

  // the OSD's settings, two flip-flops into clk
  reg  [1:0] en_s = 0, cw_s = 0;
  always @(posedge clk) begin en_s <= {en_s[0], enable_in}; cw_s <= {cw_s[0], cw_in}; end
  wire       enable = en_s[1];
  wire       cw     = cw_s[1];

  // ------------------------------------------------------- the framework's side
  assign FB_FORMAT = 5'b00011;           // 8 bpp through the palette
  assign FB_WIDTH  = H;
  assign FB_HEIGHT = W;
  assign FB_STRIDE = STRIDE_W * 8;

  // the palette: a grey ramp, written round and round (256 clocks a pass)
  reg [7:0] pal_a = 0;
  always @(posedge clk) pal_a <= pal_a + 1'd1;
  assign FB_PAL_CLK  = clk;
  assign FB_PAL_ADDR = pal_a;
  assign FB_PAL_DOUT = {pal_a, pal_a, pal_a};
  assign FB_PAL_WR   = 1'b1;

  // ------------------------------------------------------- frames and buffers
  function [1:0] buf_next;
    input [1:0] a, b;
    begin
      buf_next = 1;
      if ((a == 0 && b == 1) || (a == 1 && b == 0)) buf_next = 2;
      if ((a == 1 && b == 2) || (a == 2 && b == 1)) buf_next = 0;
    end
  endfunction

  reg  [1:0] i_fb = 0, o_fb = 1, done_fb = 2;   // always three different buffers
  reg        f_ok = 0, done_ok = 0, o_ok = 0;   // the frame being written / done_fb / o_fb whole, rotation on throughout
  reg        old_vbl = 0, old_vb = 0, old_de = 0;
  reg        f_cw = 1;                   // this frame's direction
  wire       vb_rise = ce &  vblank & ~old_vb;
  wire       vb_fall = ce & ~vblank &  old_vb;

  assign FB_EN   = o_ok & enable;
  assign FB_BASE = {MEM_BASE, o_fb, 23'd0};

  always @(posedge clk) begin
    old_vbl <= FB_VBL;
    if (ce) old_vb <= vblank;
    if (!enable) f_ok <= 0;
    if (vb_fall) begin                   // a frame is finished: write the buffer neither shown nor just finished
      done_fb <= i_fb;
      done_ok <= f_ok & enable;
      f_ok    <= enable;
      i_fb    <= FB_LL ? {1'b0, ~i_fb[0]} : buf_next(i_fb, o_fb);
    end
    // the scaler's frame: always the newest finished one
    if (~old_vbl & FB_VBL) begin
      o_fb <= FB_LL ? {1'b0, ~i_fb[0]} : vb_fall ? i_fb : done_fb;
      o_ok <= (vb_fall & ~FB_LL) ? f_ok & enable : done_ok;
    end
    if (!enable) begin done_ok <= 0; o_ok <= 0; end
  end

  // ------------------------------------------------------- the strips coming in
  // eight byte-wide RAMs (one a byte lane of the out word), two strips deep
  reg  [9:0] x = 0;                      // pixel in the line
  reg  [2:0] r = 0;                      // line in the strip
  reg        dirty = 0;                  // the strip holds a line
  reg        bank = 0;                   // the half being filled
  reg  [7:0] k = 0;                      // strip in the frame
  wire [2:0] lane = f_cw ? ~r : r;

  reg        fl_go = 0;                  // a strip to send: one clock
  reg        fl_bank = 0;
  reg [19:0] fl_col = 0;                 // its word column in an out row
  reg        fl_busy;                    // (the sender's, below)

  // the frame is set up at vblank's rise, after any part-strip has been sent
  reg        fl_cw = 1;
  always @(posedge clk) begin
    fl_go <= 0;
    if (ce) begin
      old_de <= de;
      if (de) begin
        x <= old_de ? x + 1'd1 : 10'd1;
        dirty <= 1;
      end
      // the strip is full at its eighth line, or the frame ends with a part of one
      if ((old_de & ~de & (r == 3'd7)) | (vb_rise & dirty)) begin
        if (enable) begin
          fl_go   <= 1;
          fl_bank <= bank;
          fl_cw   <= f_cw;
          fl_col  <= f_cw ? TOPW - k : k;
          if (fl_busy) overrun <= 1;
        end
        bank  <= ~bank;
        k     <= k + 1'd1;
        r     <= 0;
        dirty <= 0;
      end else if (old_de & ~de)
        r <= r + 1'd1;
      if (vb_rise) begin                 // the next frame
        f_cw  <= cw;
        r     <= cw ? PHASE : 3'd0;
        k     <= 0;
      end
    end
  end

  wire [9:0] wx = (de & ~old_de) ? 10'd0 : x;   // this pixel's column

  // ------------------------------------------------------- the strips going out
  // a word a clock while DDR takes them: the RAMs read a column ahead (q), a holding word (h)
  // catches it when BUSY stalls the bus word (o)
  reg        active = 0;                 // a strip being sent
  reg        iss_done = 0;               // every column read
  reg  [9:0] sx = 0;                     // the column read next = the out row's source x
  reg [19:0] row_w = 0;                  // that out row's first word
  reg        s_cw = 1;
  reg  [1:0] s_fb = 0;
  reg [19:0] s_col = 0;
  reg        s_bank = 0;
  wire [63:0] q;
  reg        qv = 0, hv = 0, ov = 0;
  reg [28:0] qa, ha, oa;
  reg [63:0] hd, od;

  wire take  = ov & ~DDRAM_BUSY;
  wire free  = ~ov | take;
  wire issue = active & ~iss_done & ~hv & (~qv | free);

  initial overrun = 0;
  always @(posedge clk) fl_busy <= active | fl_go;

  // one block RAM a byte lane: written at the pixel rate, read by the sender
  genvar gl;
  generate for (gl = 0; gl < 8; gl = gl + 1) begin : lanes
    reg [7:0] m [0:2047];
    reg [7:0] rq;
    always @(posedge clk) begin
      if (ce & de & (lane == gl)) m[{bank, wx}] <= grey;
      rq <= m[{s_bank, sx}];
    end
    assign q[gl*8 +: 8] = rq;
  end endgenerate

  always @(posedge clk) begin
    if (fl_go & ~active) begin           // (a strip arriving while one is sent is an overrun: dropped)
      s_bank <= fl_bank; s_col <= fl_col; s_cw <= fl_cw; s_fb <= i_fb;
      sx <= 0; iss_done <= 0; active <= 1;
      row_w <= fl_cw ? 20'd0 : (W - 1) * STRIDE_W;
    end
    if (issue) begin
      qa <= {MEM_BASE, s_fb, row_w + s_col};
      row_w <= s_cw ? row_w + STRIDE_W : row_w - STRIDE_W;
      if (sx == W - 1) iss_done <= 1; else sx <= sx + 1'd1;
    end
    qv <= issue;
    if (free) begin
      if (hv)      begin od <= hd; oa <= ha; ov <= 1; hv <= 0; end   // (hv and qv are never both set)
      else if (qv) begin od <= q;  oa <= qa; ov <= 1; end
      else ov <= 0;
    end else if (qv) begin hd <= q; ha <= qa; hv <= 1; end
    if (active & iss_done & ~qv & ~hv & free) active <= 0;
  end

  assign DDRAM_CLK      = clk;
  assign DDRAM_BURSTCNT = 8'd1;
  assign DDRAM_ADDR     = oa;
  assign DDRAM_DIN      = od;
  assign DDRAM_BE       = 8'hFF;
  assign DDRAM_WE       = ov;
  assign DDRAM_RD       = 1'b0;

endmodule
