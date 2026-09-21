package Spi;

import FIFOF::*;
import ConfigReg::*;
import RegIf::*;
import SpiRegs::*;

// 本包不认识任何总线：对外只给中立的 RegIf，接哪种总线由 wrap 或装配决定。
typedef struct {
  Bit#(0) none;
} SpiCfg;

// 一帧的节拍（19.9）。Lead、Tail、Rest、Gap 各数一段延时，单位都是一个 SCK 周期
typedef enum { Idle, Lead, Load, Xfer, Tail, Rest, Gap } Beat deriving (Bits, Eq);

// 数据线根数由 lines 定，但单线与双线用的是分开的 MOSI 与 MISO，所以至少两根：
// 单线 MOSI=IO0、MISO=IO1，四线八线是一组双向的 IO。
interface SpiPins#(numeric type csWidth, numeric type io);
  (* always_ready, result = "sck"    *) method Bit#(1) sck;
  (* always_ready, result = "cs_n"   *) method Bit#(csWidth) cs_n;
  (* always_ready, result = "io_o"   *) method Bit#(io) io_o;
  (* always_ready, result = "io_oe"  *) method Bit#(io) io_oe;
  (* always_ready, always_enabled, prefix = "" *)
  method Action io_i((* port = "io_i" *) Bit#(io) v);
endinterface

interface SpiIfc#(numeric type aw, numeric type dw,
                  numeric type fifoDepth, numeric type csWidth,
                  numeric type lines);
  interface RegIf#(aw, dw) regs;
  interface SpiPins#(csWidth, TMax#(2, lines)) pins;
  (* always_ready *) method Bool irq;
endinterface

module mkSpi#(SpiCfg cfg)(SpiIfc#(aw, dw, fifoDepth, csWidth, lines))
    provisos (Mul#(TDiv#(dw, 8), 8, dw), Add#(_a, 8, aw), Add#(_b, 12, dw),
              Add#(_c, 1, dw), Add#(_d, csWidth, dw), Add#(_e, 2, dw),
              Add#(_f, 4, dw), Add#(_g, 8, dw), Add#(_h, csWidth, 8), Add#(_i, 3, dw),
              Add#(_j, TMax#(2, lines), 8));

  SpiRegsIfc#(aw, dw, fifoDepth, csWidth, lines) r <- mkSpiRegs;

  // 去守卫按端口给：软件那一端走 always_ready 的总线方法，硬件那一端走规则。
  // 接收的入队也去守卫：带守卫时 notFull 被提到整条移位规则头上，接收队列一满
  // 时钟就停在帧中间，只发不读的软件发满 fifoDepth 帧就卡死。满了就丢这一帧，
  // 与 sifive-blocks 一致
  FIFOF#(Bit#(8)) txq <- mkGSizedFIFOF(True,  False, valueOf(fifoDepth));
  FIFOF#(Bit#(8)) rxq <- mkGSizedFIFOF(True,  True,  valueOf(fifoDepth));

  Reg#(Beat)     st    <- mkConfigReg(Idle);
  Reg#(Bit#(8))   dly   <- mkReg(0);
  Reg#(Bit#(12))  div   <- mkReg(0);
  Reg#(Bool)      half  <- mkReg(False);   // 一位两个半周期，延时也按半周期数
  Reg#(Bit#(4))   bitn  <- mkReg(0);
  Reg#(Bit#(8))   shTx  <- mkReg(0);
  Reg#(Bit#(8))   shRx  <- mkReg(0);
  Reg#(Bool)      txPend <- mkReg(False);
  Wire#(Bit#(TMax#(2, lines))) ioIn <- mkBypassWire;

  // 每拍走几位。单线是经典四线 SPI（MOSI 出、MISO 进），多线共享一组双向 IO
  Integer nl = valueOf(lines);
  Bool multi = nl > 1;

  // 片选由硬件按着没有，以及按下那一刻各根的电平。按住期间 csid、csdef 再变，
  // 引脚不跟着跳：先走完 sckcs 再放开、再数 intercs（19.8、19.9）
  Reg#(Bool)          csOn   <- mkConfigReg(False);
  Reg#(Bit#(csWidth)) csAct  <- mkConfigReg('1);
  Reg#(Bool)          csDrop[2] <- mkCReg(2, False);
  // 放开条件是「写进了不同的值」，写同一个值不算。每拍留一份上一拍的值来比，
  // 这样也不必在同一条规则里既读写脉冲又读寄存器（G0021）
  Reg#(Bit#(csWidth)) idWas   <- mkReg(0);
  Reg#(Bit#(2))       modeWas <- mkReg(0);
  Reg#(Bit#(csWidth)) defWas  <- mkReg('1);

  // 队列里有几条：FIFOF 不给计数，而入队与出队在不同规则里，共用一个计数器
  // 会抢同一个写口——各自一个自由计数器，相减即占用数。宽度够，回绕不影响差值。
  Reg#(Bit#(8)) txIn  <- mkReg(0);
  Reg#(Bit#(8)) txOut <- mkReg(0);
  Reg#(Bit#(8)) rxIn  <- mkReg(0);
  Reg#(Bit#(8)) rxOut <- mkReg(0);
  Bit#(8) txLevel = txIn - txOut;
  Bit#(8) rxLevel = rxIn - rxOut;

  // 帧方向。fmt 本来就被 lines 门控，单线时它读回零、两个判据都不成立。
  //   dir = 1：只发，线上回来的丢掉——共享线上主机正在驱动，采到的是自己
  //   dir = 0：只收，共享线上放开驱动让从机送
  Bool sendOnly = multi && r.fmt_dir == 1;
  Bool recvOnly = multi && r.fmt_dir == 0;

  // 帧长只有多线才可配，否则恒 8 位
  Bit#(4) flen = multi ? (r.fmt_len == 0 ? 8 : r.fmt_len) : 8;

  // 位序（19.10 表 78）。取哪一位与往哪边移是**同一个决定**：取第 0 位就得右移。
  // 手册还要求 len < 8 时低位先出右对齐，右移正好满足。
  Bool lsb = multi && r.fmt_endian == 1;

  // 表 73：只有 HOLD 与 OFF 帧后不放片选，保留的 1 按 AUTO 走
  Bool keep = r.csmode == 2 || r.csmode == 3;

  // swmod 的脉冲与寄存器的新值差一拍，先记脉冲、下一拍再取值
  rule mark;
    txPend <= r.txdata_data_wr;
  endrule

  rule push (txPend && txq.notFull);
    txq.enq(r.txdata_data);
    txIn <= txIn + 1;
  endrule

  rule csTrack;
    idWas   <= r.csid;
    modeWas <= r.csmode;
    defWas  <= r.csdef;
    Bool moved = r.csid != idWas || r.csmode != modeWas
              || ((r.csdef ^ defWas) & (1 << r.csid)) != 0;
    if (moved && csOn) csDrop[0] <= True;
  endrule

  // 该放的先放；有帧要发时，没按着就按下再数 cssck，按着或 OFF 就直接装帧
  rule idle (st == Idle);
    if (csOn && (csDrop[1] || !keep)) begin
      st   <= Tail;
      dly  <= r.delay0_sckcs;
      div  <= r.sckdiv;
      half <= False;
    end else if (txq.notEmpty) begin
      if (csOn || r.csmode == 3)
        st <= Load;
      else begin
        csOn  <= True;
        csAct <= r.csdef ^ (1 << r.csid);
        st    <= Lead;
        dly   <= r.delay0_cssck;
        div   <= r.sckdiv;
        half  <= False;
      end
    end
  endrule

  rule load (st == Load);
    shTx  <= txq.first;
    txq.deq;
    txOut <= txOut + 1;
    st    <= Xfer;
    half  <= False;
    bitn  <= 0;
    div   <= r.sckdiv;
  endrule

  Bool waiting = st == Lead || st == Tail || st == Rest || st == Gap;

  rule tick (waiting && dly != 0);
    if (div != 0)
      div <= div - 1;
    else begin
      div  <= r.sckdiv;
      half <= !half;
      if (half) dly <= dly - 1;
    end
  endrule

  rule waited (waiting && dly == 0);
    case (st)
      Lead: st <= Load;
      Tail: begin
        st   <= Rest;
        csOn <= False;
        csDrop[1] <= False;
        dly  <= r.delay1_intercs;
        div  <= r.sckdiv;
        half <= False;
      end
      default: st <= Idle;
    endcase
  endrule

  rule step (st == Xfer);
    if (div != 0)
      div <= div - 1;
    else begin
      div  <= r.sckdiv;
      half <= !half;
      // 每一位前半拍末采样、后半拍末移位；pha 只翻 SCK 在两个半拍里的电平（见 sck）。
      // 于是 pha = 0 采样落在前沿、pha = 1 落在后沿，而 pha = 1 的第一个前沿就是帧头，
      // 手册里 cssck 与 sckcs 各自那半个隐含周期由此而来。原来让 pha 去换采样与
      // 移位的先后：第一个前沿就把最高位移走，收尾那次采样也赶不上入队，收发都错一位
      // 单线只有 MISO 那一根，多线是整组；nl 与 8 的加减都在更宽的类型里做
      Bit#(8) got = multi ? zeroExtend(ioIn) : zeroExtend(ioIn[1]);
      Bit#(5) nxt = zeroExtend(bitn) + fromInteger(nl);
      if (!half)
        shRx <= lsb ? ((shRx >> nl) | (got << (8 - nl))) : ((shRx << nl) | got);
      else if (nxt >= zeroExtend(flen)) begin
        if (!sendOnly && rxq.notFull) begin rxq.enq(shRx); rxIn <= rxIn + 1; end
        st  <= keep ? Gap : Tail;
        dly <= keep ? r.delay1_interxfr : r.delay0_sckcs;
      end else begin
        shTx <= lsb ? (shTx >> nl) : (shTx << nl);
        bitn <= truncate(nxt);
      end
    end
  endrule

  // volatile 字段：没有存储，硬件每拍驱动
  rule status;
    r.txdata_full_in(txq.notFull ? 0 : 1);
    r.rxdata_data_in(rxq.first);
    r.rxdata_empty_in(rxq.notEmpty ? 0 : 1);
    // 19.15：发送队列**严格少于** txmark 才抬，接收队列**严格多于** rxmark 才抬。
    // 原来只看「非满」「非空」，两个门限寄存器压根不存在。
    r.ip_txwm_in(txLevel < zeroExtend(r.txmark) ? 1 : 0);
    r.ip_rxwm_in(rxLevel > zeroExtend(r.rxmark) ? 1 : 0);
  endrule

  // swacc：软件读过 rxdata 就弹一个
  rule pop (r.rxdata_data_rd && rxq.notEmpty);
    rxq.deq;
    rxOut <= rxOut + 1;
  endrule

  Bit#(1) phase = pack(half);

  interface regs = r.regs;
  interface SpiPins pins;
    method Bit#(1) sck = (st == Xfer) ? (r.sckmode_pol ^ r.sckmode_pha ^ phase)
                                      : r.sckmode_pol;
    // 表 73 与 19.7：非活动电平由 csdef 给；OFF 时硬件不碰，引脚停在 csdef 上，
    // 软件改 csdef 就能自己控片选
    method Bit#(csWidth) cs_n = (csOn && r.csmode != 3) ? csAct : r.csdef;
    // 高位先出取顶上 nl 位，低位先出取底下 nl 位；单线只驱 IO0
    method Bit#(TMax#(2, lines)) io_o =
        truncate((lsb ? shTx : (shTx >> (8 - nl))) & (multi ? ((1 << nl) - 1) : 1));
    // 共享一组 IO 时收方向要全放开，否则主机与从机对着驱动；单线只驱 MOSI
    method Bit#(TMax#(2, lines)) io_oe = multi ? (recvOnly ? 0 : '1) : 1;
    method Action io_i(Bit#(TMax#(2, lines)) v); ioIn._write(v); endmethod
  endinterface
  // 与喂给 ip 的是同一个表达式
  method Bool irq = ((r.ie_txwm == 1) && txLevel < zeroExtend(r.txmark))
                 || ((r.ie_rxwm == 1) && rxLevel > zeroExtend(r.rxmark));
endmodule

endpackage
