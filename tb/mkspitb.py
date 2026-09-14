"""spi 的行为测试台：IO0 接回 IO1 做自环，发几个字节收回来对；开了 quad 再验帧方向。

单线模式下 MOSI 是 IO0、MISO 是 IO1，接一起就等于对面挂了个回环从机。
顺带验片选在传输期间拉低、空闲时抬起。

帧方向那一段只在 quad 开着时跑（那个寄存器本来就被 quad 门控）：`dir = 1` 是
只发，线上回来的必须丢掉。原来的实现不看这一位，收到的照收，这一步就会露馅。

片选与时序（19.6 至 19.9）：pha = 1 的自环；csdef 的复位值与极性；HOLD 只在值真的
变了时放开；四段延时各量一次引脚。延时量的是**差值**，比如 cssck 从 1 调到 3，
首个时钟沿要恰好晚两个周期：装帧那一两拍的固定开销在两边抵掉，判据可以写成等式。

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
# 片选的放开要走完 sckcs 才落到引脚上，等这么久足够
SETTLE = 64
# 量延时用的分频：半个周期 H 拍，一个周期 2H 拍
DIV = 4
H = DIV + 1
CSALL = (1 << csw) - 1
other = csw >= 2
# 两帧紧挨着发，队列得装得下两个
two = depth >= 2
# 只发不读时要发的帧数：比接收队列多两帧，满了还卡不卡一眼看得出来
NOVF = depth + 2

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
    ph <= LsbSet;
    s  <= 0;
  endrule

  // 低位先出（19.10 表 78）。移位方向必须跟着 endian 走：取的是第 0 位就得右移。
  // 取第 0 位却左移的话，线上只出得来真正的第 0 位，后面七位全是零——
  // 而这一档此前一次都没被走过。
  rule lsbSet (ph == LsbSet);
    case (s)
      0: wr(rFMT, 32'h00080004);      // len = 8、endian = 1（低位先出）、dir = 0
      1: wr(rTXDATA, 32'h000000A5);   // 0xA5 高低位不对称，转了看得出来
      default: begin ph <= LsbWait; end
    endcase
    if (s < 2) s <= s + 1; else s <= 0;
  endrule

  rule lsbWait (ph == LsbWait);
    if (s > {QUIET}) begin ph <= LsbCheck; s <= 0; end
    else s <= s + 1;
  endrule

  rule lsbCheck (ph == LsbCheck);
    let x <- sp.regs.access(RegReq {{ addr: rRXDATA, write: False,
                                      wdata: 0, wstrb: 4'hF }});
    if (x.rdata[31] == 1) begin
      $display("FAIL nothing came back with lsb first");
      bad <= True;
    end else if (x.rdata[7:0] != 8'hA5) begin
      $display("FAIL lsb first loops back %02h, want a5", x.rdata[7:0]);
      bad <= True;
    end
    ph <= Done;
  endrule
'''
    verdict = ("loopback both ways round and at pha=1, chip select default level and hold release, "
               "the four delays, and a send only frame keeps nothing")
else:
    dir_phase = '''  rule dirSet (ph == DirSet);
    ph <= Done;                      // 没开 quad，帧格式寄存器不存在
  endrule
'''
    verdict = "loopback at both phases, chip select default level and hold release, and the four delays"

hold_other = f'''  // 只改另一根的默认电平：选中那一根的状态没变，片选得继续按住
  rule holdH (ph == HoldH);
    wr(rCSDEF, 32'h{CSALL ^ 2:08X});
    ph <= HoldI;
    s  <= 0;
  endrule

  rule holdI (ph == HoldI);
    if (s < {SETTLE}) s <= s + 1;
    else begin
      if (csPins[0] != 0) begin
        $display("FAIL a csdef write that leaves the selected pin alone released it");
        bad <= True;
      end
      ph <= HoldJ;
      s  <= 0;
    end
  endrule
''' if other else ""

# 检查只看快照、不碰总线：既读引脚快照又写总线的规则会被判永不触发，与 csHoldC 同理
cs_phase = f'''  // 19.6 表 69：pha = 1 是前沿移位、后沿采样。此前所有判据都跑在 pha = 0 上
  rule phaA (ph == PhaA);
    case (s)
      0: wr(rSCKMODE, 1);
      1: wr(rTXDATA, 32'h000000A5);
      default: ph <= PhaB;
    endcase
    if (s < 2) s <= s + 1; else s <= 0;
  endrule

  rule phaB (ph == PhaB);
    if (s > {QUIET}) begin ph <= PhaC; s <= 0; end
    else s <= s + 1;
  endrule

  rule phaC (ph == PhaC);
    let x <- sp.regs.access(RegReq {{ addr: rRXDATA, write: False,
                                      wdata: 0, wstrb: 4'hF }});
    if (x.rdata[31] == 1) begin
      $display("FAIL nothing came back with pha=1");
      bad <= True;
    end else if (x.rdata[7:0] != 8'hA5) begin
      $display("FAIL with pha=1 the loop returns %02h, want a5", x.rdata[7:0]);
      bad <= True;
    end
    ph <= PhaD;
  endrule

  rule phaD (ph == PhaD);
    wr(rSCKMODE, 0);
    ph <= CsdA;
  endrule

  // 19.7：csdef 复位全一
  rule csdA (ph == CsdA);
    let x <- sp.regs.access(RegReq {{ addr: rCSDEF, write: False,
                                      wdata: 0, wstrb: 4'hF }});
    if (x.rdata != 32'h{CSALL:08X}) begin
      $display("FAIL csdef reads %08h after reset, want {CSALL:08x}", x.rdata);
      bad <= True;
    end
    ph <= CsdB;
  endrule

  // 非活动电平跟着 csdef 走：写零以后空闲时全低，传输时选中那一根翻高
  rule csdB (ph == CsdB);
    wr(rCSDEF, 0);
    ph <= CsdC;
    s  <= 0;
  endrule

  rule csdC (ph == CsdC);
    if (s < {SETTLE}) s <= s + 1;
    else begin
      if (csPins != 0) begin
        $display("FAIL with csdef=0 the idle chip select pins read %b, want all low", csPins);
        bad <= True;
      end
      ph <= CsdD;
    end
  endrule

  rule csdD (ph == CsdD);
    wr(rTXDATA, 32'h0000005A);
    csHi[1] <= False;
    ph <= CsdE;
    s  <= 0;
  endrule

  rule csdE (ph == CsdE);
    if (s <= {QUIET}) s <= s + 1;
    else begin
      if (!csHi[1]) begin
        $display("FAIL with csdef=0 the selected chip select never went high during a frame");
        bad <= True;
      end
      ph <= CsdF;
    end
  endrule

  rule csdF (ph == CsdF);
    wr(rCSDEF, 32'h{CSALL:08X});
    ph <= HoldA;
  endrule

  // 19.8：HOLD 只在值真的变了时放开
  rule holdA (ph == HoldA);
    wr(rCSMODE, 2);
    ph <= HoldB;
  endrule

  rule holdB (ph == HoldB);
    wr(rTXDATA, 32'h0000005A);
    ph <= HoldC;
    s  <= 0;
  endrule

  rule holdC (ph == HoldC);
    if (s <= {QUIET}) s <= s + 1;
    else begin
      if (csPins[0] != 0) begin
        $display("FAIL the chip select was not held before the same-value writes");
        bad <= True;
      end
      ph <= HoldD;
    end
  endrule

  rule holdD (ph == HoldD);
    wr(rCSMODE, 2);
    ph <= HoldE;
    s  <= 0;
  endrule

  rule holdE (ph == HoldE);
    if (s < {SETTLE}) s <= s + 1;
    else begin
      if (csPins[0] != 0) begin
        $display("FAIL writing the same value to csmode released the held chip select");
        bad <= True;
      end
      ph <= HoldF;
    end
  endrule

  rule holdF (ph == HoldF);
    wr(rCSID, 0);
    ph <= HoldG;
    s  <= 0;
  endrule

  rule holdG (ph == HoldG);
    if (s < {SETTLE}) s <= s + 1;
    else begin
      if (csPins[0] != 0) begin
        $display("FAIL writing the same value to csid released the held chip select");
        bad <= True;
      end
      ph <= {"HoldH" if other else "HoldJ"};
      s  <= 0;
    end
  endrule

{hold_other}
  // 改选中那一根的默认电平就放开。按住时引脚锁在按下那一刻，放开与否在引脚上
  // 看不出来（两边都是低），要看下一帧：放开了才会按新的极性重新按下，第 0 根翻高
  rule holdJ (ph == HoldJ);
    wr(rCSDEF, 32'h{CSALL ^ 1:08X});
    ph <= HoldJ2;
    s  <= 0;
  endrule

  rule holdJ2 (ph == HoldJ2);
    if (s < {SETTLE}) s <= s + 1;
    else ph <= HoldJ3;
  endrule

  rule holdJ3 (ph == HoldJ3);
    wr(rTXDATA, 32'h0000005A);
    csHi[1] <= False;
    ph <= HoldK;
    s  <= 0;
  endrule

  rule holdK (ph == HoldK);
    if (s <= {QUIET}) s <= s + 1;
    else begin
      if (!csHi[1]) begin
        $display("FAIL a csdef write that flips the selected pin did not release it: the next frame ran under the old chip select");
        bad <= True;
      end
      ph <= HoldL;
    end
  endrule

  rule holdL (ph == HoldL);
    wr(rCSDEF, 32'h{CSALL:08X});
    ph <= HoldM;
  endrule

  rule holdM (ph == HoldM);
    wr(rCSMODE, 0);
    ph <= Flush2;
    s  <= 0;
  endrule

  // 上面几帧又往接收队列里塞了字节，量延时之前读空
  rule flush2 (ph == Flush2);
    let x <- sp.regs.access(RegReq {{ addr: rRXDATA, write: False,
                                      wdata: 0, wstrb: 4'hF }});
    if (x.rdata[31] == 1) begin ph <= R0W; s <= 0; end
  endrule
'''

# 19.9：每一轮先写寄存器，再发一或两帧，等静下来，把监视器量到的数存进自己的槽
REG = {"sckdiv": "rSCKDIV", "sckmode": "rSCKMODE", "csmode": "rCSMODE",
       "delay0": "rDELAY0", "delay1": "rDELAY1"}
runs = [
    ("leadA", [("sckdiv", DIV), ("sckmode", 0), ("csmode", 0),
               ("delay0", 0x00010001), ("delay1", 0x00000001)], 1, "mLead"),
    ("leadB", [("sckmode", 1)], 1, "mLead"),
    ("leadC", [("sckmode", 0), ("delay0", 0x00010003)], 1, "mLead"),
    ("tailA", [("delay0", 0x00010001)], 1, "mTail"),
    ("tailB", [("sckmode", 1)], 1, "mTail"),
    ("tailC", [("sckmode", 0), ("delay0", 0x00030001)], 1, "mTail"),
]
if two:
    runs += [
        ("inactA", [("delay0", 0x00010001), ("delay1", 0x00000001)], 2, "mInact"),
        ("inactC", [("delay1", 0x00000003)], 2, "mInact"),
        # HOLD 下两帧之间的空当；interxfr 不是放开条件，改它片选照样按住
        ("gapA", [("delay1", 0x00000001), ("csmode", 2)], 2, "mGap"),
        ("gapB", [("delay1", 0x00020001)], 2, "mGap"),
        # AUTO 下 interxfr 不起作用（19.9：只用于 HOLD 与 OFF）
        ("autoA", [("csmode", 0), ("delay1", 0x00000001)], 2, "mGap"),
        ("autoB", [("delay1", 0x00020001)], 2, "mGap"),
    ]

run_rules = []
for i, (name, writes, frames, metric) in enumerate(runs):
    nxt = f"R{i + 1}W" if i + 1 < len(runs) else "TmChk"
    case = chr(10).join(f"      {j}: wr({REG[r]}, 32'h{v:08X});" for j, (r, v) in enumerate(writes))
    run_rules.append(f'''  rule r{i}w (ph == R{i}W);
    case (s)
{case}
      default: ph <= R{i}S;
    endcase
    if (s < {len(writes)}) s <= s + 1; else s <= 0;
  endrule

  rule r{i}s (ph == R{i}S);
    wr(rTXDATA, 32'h0000005A);
    if (s + 1 >= {frames}) begin ph <= R{i}Q; s <= 0; end else s <= s + 1;
  endrule

  rule r{i}q (ph == R{i}Q);
    if (s > {QUIET}) begin ph <= R{i}M; s <= 0; end else s <= s + 1;
  endrule

  rule r{i}m (ph == R{i}M);
    v{name} <= {metric};
    ph <= {nxt};
  endrule
''')

checks = [
    (f"vleadA - vleadB != {H}",
     "with pha=0 the first clock edge should trail cs by half a period more than with pha=1: %0d and %0d cycles",
     "vleadA, vleadB"),
    (f"vleadB < {2 * H}",
     "cssck=1 put only %0d cycles between cs and the first clock edge, less than a period", "vleadB"),
    (f"vleadC - vleadA != {4 * H}",
     "cssck=3 should put the first clock edge two periods later than cssck=1: %0d and %0d cycles",
     "vleadC, vleadA"),
    (f"vtailB - vtailA != {H}",
     "with pha=1 cs should be released half a period later after the last clock edge than with pha=0: %0d and %0d cycles",
     "vtailB, vtailA"),
    (f"vtailC - vtailA != {4 * H}",
     "sckcs=3 should release cs two periods later than sckcs=1: %0d and %0d cycles", "vtailC, vtailA"),
]
if two:
    checks += [
        (f"vinactA < {2 * H}",
         "intercs=1 kept cs inactive for only %0d cycles, less than a period", "vinactA"),
        (f"vinactC - vinactA != {4 * H}",
         "intercs=3 should keep cs inactive two periods longer than intercs=1: %0d and %0d cycles",
         "vinactC, vinactA"),
        (f"vgapB - vgapA != {4 * H}",
         "in HOLD, interxfr=2 should put two more periods between frames than interxfr=0: %0d and %0d cycles",
         "vgapB, vgapA"),
        ("vautoB != vautoA",
         "in AUTO the gap between frames moved with interxfr, which applies only to HOLD and OFF: %0d and %0d cycles",
         "vautoB, vautoA"),
    ]
# 并列的 if 各写一次 bad 会被判并行冲突（G0004），先攒进局部变量
chk = "    Bool wrong = False;" + chr(10) + chr(10).join(f'''    if ({c}) begin
      $display("FAIL {m}", {a});
      wrong = True;
    end''' for c, m, a in checks) + chr(10) + "    if (wrong) bad <= True;"

slots = chr(10).join(f"  Reg#(Bit#(32)) v{name} <- mkReg(0);" for name, *_ in runs)
run_names = ", ".join(f"R{i}W, R{i}S, R{i}Q, R{i}M" for i in range(len(runs)))

txt = f'''package Spi{label}Tb;

import ConfigReg::*;
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

Bit#(8) rSCKDIV  = 8'h00;
Bit#(8) rSCKMODE = 8'h04;
Bit#(8) rCSID    = 8'h10;
Bit#(8) rCSDEF   = 8'h14;
Bit#(8) rCSMODE  = 8'h18;
Bit#(8) rDELAY0  = 8'h28;
Bit#(8) rDELAY1  = 8'h2C;
Bit#(8) rTXMARK  = 8'h50;
Bit#(8) rRXMARK  = 8'h54;
Bit#(8) rIP      = 8'h74;
Bit#(8) rFMT     = 8'h40;
Bit#(8) rTXDATA  = 8'h48;
Bit#(8) rRXDATA  = 8'h4C;

typedef enum {{ Setup, Send, Recv, WmA, WmB, WmC, WmD, WmE,
               CsOffA, CsOffB, CsOffC, CsHoldA, CsHoldB, CsHoldC, CsHoldD, Flush,
               PhaA, PhaB, PhaC, PhaD, CsdA, CsdB, CsdC, CsdD, CsdE, CsdF,
               HoldA, HoldB, HoldC, HoldD, HoldE, HoldF, HoldG, HoldH, HoldI,
               HoldJ, HoldJ2, HoldJ3, HoldK, HoldL, HoldM, Flush2,
               {run_names}, TmChk, Flush3, OvfM, OvfA, OvfB, OvfC, Flush4,
               DirSet, DirWait, DirCheck, LsbSet, LsbWait, LsbCheck, Done }}
  Phase deriving (Bits, Eq);

(* synthesize *)
module mkSpi{label}Tb(Empty);
  SpiIfc#(8, 32, {depth}, {csw}) sp <- mkSpi(
      SpiCfg {{ quad: {"True" if quad else "False"} }});

  Reg#(Phase)    ph   <- mkReg(Setup);
  Reg#(Bit#(16)) s    <- mkReg(0);
  Reg#(Bit#(8))  sent <- mkReg(0);
  Reg#(Bit#(8))  got  <- mkReg(0);
  // 监视器与超时规则都读它。用普通寄存器会与各阶段规则绕成环，bsc 把超时规则
  // 整条挡掉，cyc 恒为零，量出来的时序全是零
  Reg#(Bit#(32)) cyc  <- mkConfigReg(0);
  Reg#(Bool)     bad  <- mkReg(False);
  Reg#(Bool)     sawCs <- mkReg(False);

  // 自环：IO0 出去的接回 IO1
  Reg#(Bool) csNow  <- mkReg(False);
  Reg#(Bool) csSaw[2] <- mkCReg(2, False);
  // 片选引脚的整组快照，与第 0 根是否翻高过
  Reg#(Bit#({csw})) csPins <- mkReg('1);
  Reg#(Bool) csHi[2] <- mkCReg(2, False);

  // 时序监视器：只看引脚。第 0 根片选的按下与放开、SCK 的每一次翻转
  Reg#(Bool)     onWas    <- mkReg(False);
  Reg#(Bit#(1))  sckWas   <- mkReg(0);
  Reg#(Bool)     leadPend <- mkReg(False);
  Reg#(Bit#(32)) tOn      <- mkReg(0);
  Reg#(Bit#(32)) tOff     <- mkReg(0);
  Reg#(Bit#(32)) tEdge    <- mkReg(0);
  Reg#(Bit#(32)) mLead    <- mkReg(0);
  Reg#(Bit#(32)) mTail    <- mkReg(0);
  Reg#(Bit#(32)) mInact   <- mkReg(0);
  Reg#(Bit#(32)) mGap     <- mkReg(0);
  Reg#(Bit#(32)) nOn      <- mkReg(0);
  Reg#(Bit#(32)) onMark   <- mkReg(0);
  Reg#(Bit#(8))  ovfN     <- mkReg(0);
{slots}

  rule loop;
    Bit#(4) o = sp.pins.io_o;
    sp.pins.io_i({{2'b00, o[0], 1'b0}});
    if (sp.pins.cs_n != '1) sawCs <= True;
    // 片选的快照：读 cs_n 的规则不能同时写寄存器（会被判永不触发），
    // 所以在这里打一拍，检查规则只看寄存器
    csNow <= (sp.pins.cs_n != '1);
    if (sp.pins.cs_n != '1) csSaw[0] <= True;
    csPins <= sp.pins.cs_n;
    if (sp.pins.cs_n[0] == 1) csHi[0] <= True;

    Bool on = sp.pins.cs_n[0] == 0;
    Bit#(1) k = sp.pins.sck;
    onWas  <= on;
    sckWas <= k;
    if (on && !onWas) begin
      tOn <= cyc;
      nOn <= nOn + 1;
      mInact <= cyc - tOff;
      leadPend <= k == sckWas;
      if (k != sckWas) mLead <= 0;
    end else if (leadPend && k != sckWas) begin
      mLead <= cyc - tOn;
      leadPend <= False;
    end
    if (!on && onWas) begin
      tOff <= cyc;
      mTail <= cyc - tEdge;
    end
    // 一帧之内相邻两沿恰好隔 H 拍，比它长的只能是帧与帧之间
    if (k != sckWas) begin
      tEdge <= cyc;
      if (cyc - tEdge > {H}) mGap <= cyc - tEdge;
    end
  endrule

  rule timeout;
    cyc <= cyc + 1;
    if (cyc > 400000) begin
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
    if (x.rdata[31] == 1) ph <= PhaA;
  endrule

{cs_phase}
{chr(10).join(run_rules)}
  rule tmChk (ph == TmChk);
{chk}
    ph <= Flush3;
  endrule

  rule flush3 (ph == Flush3);
    let x <- sp.regs.access(RegReq {{ addr: rRXDATA, write: False,
                                      wdata: 0, wstrb: 4'hF }});
    if (x.rdata[31] == 1) ph <= OvfM;
  endrule

  // 只发不读：接收队列满了以后帧照样得发出去。原来接收入队带守卫，队列一满
  // 整条移位规则停住，发满 fifoDepth 帧就卡死。AUTO 下一帧按一次片选，数按下的次数
  rule ovfM (ph == OvfM);
    onMark <= nOn;
    ovfN   <= 0;
    ph     <= OvfA;
  endrule

  rule ovfA (ph == OvfA);
    wr(rTXDATA, 32'h0000005A);
    ph <= OvfB;
    s  <= 0;
  endrule

  rule ovfB (ph == OvfB);
    if (s < 300) s <= s + 1;
    else begin
      s    <= 0;
      ovfN <= ovfN + 1;
      ph   <= (ovfN + 1 == {NOVF}) ? OvfC : OvfA;
    end
  endrule

  rule ovfC (ph == OvfC);
    if (nOn - onMark != {NOVF}) begin
      $display("FAIL only %0d of {NOVF} frames went out while rxdata was never read", nOn - onMark);
      bad <= True;
    end
    ph <= Flush4;
  endrule

  rule flush4 (ph == Flush4);
    let x <- sp.regs.access(RegReq {{ addr: rRXDATA, write: False,
                                      wdata: 0, wstrb: 4'hF }});
    if (x.rdata[31] == 1) begin ph <= DirSet; s <= 0; end
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
