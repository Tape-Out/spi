"""spi 的行为测试台：IO0 接回 IO1 做自环，发几个字节收回来对；开了 quad 再验帧方向。

单线模式下 MOSI 是 IO0、MISO 是 IO1，接一起就等于对面挂了个回环从机。
顺带验片选在传输期间拉低、空闲时抬起。

帧方向那一段只在 quad 开着时跑（那个寄存器本来就被 quad 门控）：`dir = 1` 是
只发，线上回来的必须丢掉。原来的实现不看这一位，收到的照收，这一步就会露馅。

认矩阵：`fifoDepth`、`csWidth`、`quad` 都从这一点的旋钮来。
"""
import json
import pathlib
import sys

out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out.mkdir(parents=True, exist_ok=True)
cfg = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
label = cfg.get("label", "")
k = cfg.get("knobs", {})
depth = int(k.get("fifoDepth", 8))
csw = int(k.get("csWidth", 1))
quad = bool(k.get("quad", False))

BYTES = [0xA5, 0x00, 0xFF, 0x3C]
# 这一台是「先全发完再全读出」，发多了接收队列会溢出丢字节
NSEND = min(len(BYTES), depth)
QUIET = 600

lst = chr(10).join(f"      {i}: return 8'h{b:02X};" for i, b in enumerate(BYTES))

if quad:
    dir_phase = f'''  // dir = 1 是只发：线上回来的必须丢掉
  rule dirSet (ph == DirSet);
    case (s)
      0: wr(rFMT, 32'h00000008);      // proto 单线、dir = 1
      1: wr(rTXDATA, 32'h000000A5);
      default: begin ph <= DirWait; end
    endcase
    if (s < 2) s <= s + 1; else s <= 0;
  endrule

  rule dirWait (ph == DirWait);
    if (s > {QUIET}) begin ph <= DirCheck; s <= 0; end
    else s <= s + 1;
  endrule

  rule dirCheck (ph == DirCheck);
    let x <- sp.regs.access(RegReq {{ addr: rRXDATA, write: False,
                                      wdata: 0, wstrb: 4'hF }});
    // 位 31 为一表示队列空——只发的那一帧不该留下任何东西
    if (x.rdata[31] == 0) begin
      $display("FAIL fmt.dir says send only but a byte was received: %02h",
               x.rdata[7:0]);
      bad <= True;
    end
    ph <= Done;
  endrule
'''
    verdict = "loopback, chip select, and a send only frame keeps nothing"
    after_recv = "DirSet"
else:
    dir_phase = '''  rule dirSet (ph == DirSet);
    ph <= Done;                      // 没开 quad，帧格式寄存器不存在
  endrule
'''
    verdict = "loopback and chip select"
    after_recv = "DirSet"

txt = f'''package Spi{label}Tb;

import RegIf::*;
import Spi::*;

// 由 tb/mkspitb.py 生成，勿手改。
// 这一点：fifoDepth={depth} csWidth={csw} quad={quad}

Integer nbytes = {NSEND};

function Bit#(8) want(Bit#(8) i);
  case (i)
{lst}
    default: return 0;
  endcase
endfunction

Bit#(8) rSCKDIV = 8'h00;
Bit#(8) rCSMODE = 8'h18;
Bit#(8) rTXMARK = 8'h50;
Bit#(8) rRXMARK = 8'h54;
Bit#(8) rIP     = 8'h74;
Bit#(8) rFMT    = 8'h40;
Bit#(8) rTXDATA = 8'h48;
Bit#(8) rRXDATA = 8'h4C;

typedef enum {{ Setup, Send, Recv, WmA, WmB, WmC, WmD, WmE,
               CsOffA, CsOffB, CsOffC, CsHoldA, CsHoldB, CsHoldC, CsHoldD, Flush,
               DirSet, DirWait, DirCheck, Done }}
  Phase deriving (Bits, Eq);

(* synthesize *)
module mkSpi{label}Tb(Empty);
  SpiIfc#(8, 32, {depth}, {csw}) sp <- mkSpi(
      SpiCfg {{ quad: {"True" if quad else "False"} }});

  Reg#(Phase)    ph   <- mkReg(Setup);
  Reg#(Bit#(16)) s    <- mkReg(0);
  Reg#(Bit#(8))  sent <- mkReg(0);
  Reg#(Bit#(8))  got  <- mkReg(0);
  Reg#(Bit#(32)) cyc  <- mkReg(0);
  Reg#(Bool)     bad  <- mkReg(False);
  Reg#(Bool)     sawCs <- mkReg(False);

  // 自环：IO0 出去的接回 IO1
  Reg#(Bool) csNow  <- mkReg(False);
  Reg#(Bool) csSaw[2] <- mkCReg(2, False);

  rule loop;
    Bit#(4) o = sp.pins.io_o;
    sp.pins.io_i({{2'b00, o[0], 1'b0}});
    if (sp.pins.cs_n != '1) sawCs <= True;
    // 片选的快照：读 cs_n 的规则不能同时写寄存器（会被判永不触发），
    // 所以在这里打一拍，检查规则只看寄存器
    csNow <= (sp.pins.cs_n != '1);
    if (sp.pins.cs_n != '1) csSaw[0] <= True;
  endrule

  rule timeout;
    cyc <= cyc + 1;
    if (cyc > 100000) begin
      $display("TIMEOUT in phase %0d", pack(ph));
      $finish(1);
    end
  endrule

  function Action wr(Bit#(8) a, Bit#(32) d) = action
    let _ <- sp.regs.access(RegReq {{ addr: a, write: True,
                                      wdata: d, wstrb: 4'hF }});
  endaction;

  rule setup (ph == Setup);
    case (s)
      0: wr(rSCKDIV, 2);
      1: wr(rCSMODE, 0);
      default: ph <= Send;
    endcase
    if (s < 2) s <= s + 1; else s <= 0;
  endrule

  rule send (ph == Send && sent < fromInteger(nbytes));
    wr(rTXDATA, zeroExtend(want(sent)));
    sent <= sent + 1;
  endrule

  rule sendDone (ph == Send && sent == fromInteger(nbytes));
    ph <= Recv;
  endrule

  rule recv (ph == Recv);
    let x <- sp.regs.access(RegReq {{ addr: rRXDATA, write: False,
                                      wdata: 0, wstrb: 4'hF }});
    if (x.rdata[31] == 0) begin
      if (x.rdata[7:0] != want(got)) begin
        $display("FAIL byte %0d: got %02h want %02h",
                 got, x.rdata[7:0], want(got));
        bad <= True;
      end
      got <= got + 1;
      if (got + 1 == fromInteger(nbytes)) begin ph <= WmA; s <= 0; end
    end
  endrule

{dir_phase}
  // 19.15：rxwm 在接收队列**严格多于** rxmark 时才抬。门限取 1：
  // 收到一个字节时不该抬（1 不大于 1），收到两个才该抬。
  // 队列只装得下一个的那一点跳过——两个字节根本放不下。
  rule wmA (ph == WmA);
    if (fromInteger(nbytes) < 2) ph <= CsOffA;
    else begin wr(rRXMARK, 1); ph <= WmB; s <= 0; end
  endrule

  rule wmB (ph == WmB);
    wr(rTXDATA, 32'h0000005A);       // 自环回来一个字节
    ph <= WmC;
    s  <= 0;
  endrule

  rule wmC (ph == WmC);
    if (s > {QUIET}) begin ph <= WmD; s <= 0; end else s <= s + 1;
  endrule

  rule wmD (ph == WmD);
    case (s)
      0: action
           let x <- sp.regs.access(RegReq {{ addr: rIP, write: False,
                                             wdata: 0, wstrb: 4'hF }});
           if (x.rdata[1] != 0) begin
             $display("FAIL one entry with rxmark=1 already raises rxwm");
             bad <= True;
           end
         endaction
      1: wr(rTXDATA, 32'h0000005A);   // 再来一个，凑到两条
      default: begin ph <= WmE; s <= 0; end
    endcase
    if (s < 2) s <= s + 1;
  endrule

  rule wmE (ph == WmE);
    if (s <= {QUIET}) s <= s + 1;
    else begin
      let x <- sp.regs.access(RegReq {{ addr: rIP, write: False,
                                        wdata: 0, wstrb: 4'hF }});
      if (x.rdata[1] != 1) begin
        $display("FAIL two entries with rxmark=1 but rxwm stays low");
        bad <= True;
      end
      ph <= CsOffA;
      s  <= 0;
    end
  endrule

  // 19.8 表 73：csmode = 3 是 OFF，硬件完全不碰片选
  rule csOffA (ph == CsOffA);
    wr(rCSMODE, 3);
    csSaw[1] <= False;
    ph <= CsOffB;
  endrule

  rule csOffB (ph == CsOffB);
    wr(rTXDATA, 32'h0000005A);
    ph <= CsOffC;
    s  <= 0;
  endrule

  rule csOffC (ph == CsOffC);
    if (s <= {QUIET}) s <= s + 1;
    else begin
      if (csSaw[1]) begin
        $display("FAIL csmode is OFF but cs was asserted");
        bad <= True;
      end
      ph <= CsHoldA;
    end
  endrule

  // csmode = 2 是 HOLD：第一帧之后片选一直按住，不随帧起落
  rule csHoldA (ph == CsHoldA);
    wr(rCSMODE, 2);
    ph <= CsHoldB;
  endrule

  rule csHoldB (ph == CsHoldB);
    wr(rTXDATA, 32'h0000005A);
    ph <= CsHoldC;
    s  <= 0;
  endrule

  rule csHoldC (ph == CsHoldC);
    if (s <= {QUIET}) s <= s + 1;
    else begin
      // 这一条只看快照、不碰总线：既读 cs_n 的快照又写 csmode 的话，
      // 「排在采样规则之前」与「之后」会同时成立，bsc 把整条规则丢掉。
      if (!csNow) begin
        $display("FAIL csmode is HOLD but cs was released after the frame");
        bad <= True;
      end
      ph <= CsHoldD;
      s  <= 0;
    end
  endrule

  rule csHoldD (ph == CsHoldD);
    wr(rCSMODE, 0);                   // 放回 AUTO
    ph <= Flush;
  endrule

  // 上面几段往接收队列里塞了字节。不读空的话，后面 quad 的 dir 检查
  // 会把它们当成「只发却收到了」——那是这一台自己造出来的假失败。
  rule flush (ph == Flush);
    let x <- sp.regs.access(RegReq {{ addr: rRXDATA, write: False,
                                      wdata: 0, wstrb: 4'hF }});
    if (x.rdata[31] == 1) ph <= {after_recv};
  endrule

  rule fin (ph == Done);
    if (!sawCs) begin
      $display("FAIL chip select never went low");
      bad <= True;
    end
    if (bad || !sawCs) $display("FAILED");
    else $display("PASS spi: {verdict}");
    $finish((bad || !sawCs) ? 1 : 0);
  endrule
endmodule

endpackage
'''

(out / f"Spi{label}Tb.bsv").write_text(txt, encoding="utf-8")
print(f"  spi 自环 {NSEND} 字节：fifoDepth={depth} csWidth={csw} quad={quad}")
