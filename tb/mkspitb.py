"""spi 的行为测试台：把 IO0 接回 IO1 做自环，发几个字节再收回来对。

单线模式下 MOSI 是 IO0、MISO 是 IO1，接一起就等于对面挂了个回环从机。
顺带验片选在传输期间拉低、空闲时抬起。
"""
import pathlib
import sys

out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out.mkdir(parents=True, exist_ok=True)

BYTES = [0xA5, 0x00, 0xFF, 0x3C]
lst = "\n".join(f"      {i}: return 8'h{b:02X};" for i, b in enumerate(BYTES))

(out / "SpiTb.bsv").write_text(f'''package SpiTb;

import RegIf::*;
import Spi::*;

// 由 tb/mkspitb.py 生成，勿手改。

Integer nbytes = {len(BYTES)};

function Bit#(8) want(Bit#(8) i);
  case (i)
{lst}
    default: return 0;
  endcase
endfunction

Bit#(8) rSCKDIV = 8'h00;
Bit#(8) rCSMODE = 8'h18;
Bit#(8) rTXDATA = 8'h48;
Bit#(8) rRXDATA = 8'h4C;

typedef enum {{ Setup, Send, Recv, Done }} Phase deriving (Bits, Eq);

(* synthesize *)
module mkSpiTb(Empty);
  SpiIfc#(8, 32, 8, 1) sp <- mkSpi(SpiCfg {{ quad: False }});

  Reg#(Phase)    ph   <- mkReg(Setup);
  Reg#(Bit#(8))  s    <- mkReg(0);
  Reg#(Bit#(8))  sent <- mkReg(0);
  Reg#(Bit#(8))  got  <- mkReg(0);
  Reg#(Bit#(32)) cyc  <- mkReg(0);
  Reg#(Bool)     bad  <- mkReg(False);
  Reg#(Bool)     sawCs <- mkReg(False);

  // 自环：IO0 出去的接回 IO1
  rule loop;
    Bit#(4) o = sp.pins.io_o;
    sp.pins.io_i({{2'b00, o[0], 1'b0}});
    if (sp.pins.cs_n == 0) sawCs <= True;
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
    s <= s + 1;
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
      if (got + 1 == fromInteger(nbytes)) ph <= Done;
    end
  endrule

  rule fin (ph == Done);
    if (!sawCs) begin
      $display("FAIL chip select never went low");
      bad <= True;
    end
    if (bad || !sawCs) $display("FAILED");
    else $display("PASS spi loopback, %0d bytes", nbytes);
    $finish((bad || !sawCs) ? 1 : 0);
  endrule
endmodule

endpackage
''', encoding="utf-8")
print(f"  spi 自环：{len(BYTES)} 个字节")
