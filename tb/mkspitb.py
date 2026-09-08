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
Bit#(8) rFMT    = 8'h40;
Bit#(8) rTXDATA = 8'h48;
Bit#(8) rRXDATA = 8'h4C;

typedef enum {{ Setup, Send, Recv, DirSet, DirWait, DirCheck, Done }}
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
  rule loop;
    Bit#(4) o = sp.pins.io_o;
    sp.pins.io_i({{2'b00, o[0], 1'b0}});
    if (sp.pins.cs_n != '1) sawCs <= True;
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
      if (got + 1 == fromInteger(nbytes)) begin ph <= {after_recv}; s <= 0; end
    end
  endrule

{dir_phase}
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
