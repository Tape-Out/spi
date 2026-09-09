package Spi;

import FIFOF::*;
import ConfigReg::*;
import RegIf::*;
import SpiRegs::*;

// 本包不认识任何总线：对外只给中立的 RegIf，接哪种总线由 wrap 或装配决定。
typedef struct {
  Bool quad;
} SpiCfg;

// 四根数据线一直在接口上，单线模式只用到 io[0] 与 io[1]——真实的 QSPI 焊盘就是
// 这么接的（MOSI=IO0，MISO=IO1），单线时把 IO2/IO3 的方向关掉即可。
interface SpiPins#(numeric type csWidth);
  (* always_ready, result = "sck"    *) method Bit#(1) sck;
  (* always_ready, result = "cs_n"   *) method Bit#(csWidth) cs_n;
  (* always_ready, result = "io_o"   *) method Bit#(4) io_o;
  (* always_ready, result = "io_oe"  *) method Bit#(4) io_oe;
  (* always_ready, always_enabled, prefix = "" *)
  method Action io_i((* port = "io_i" *) Bit#(4) v);
endinterface

interface SpiIfc#(numeric type aw, numeric type dw,
                  numeric type fifoDepth, numeric type csWidth);
  interface RegIf#(aw, dw) regs;
  interface SpiPins#(csWidth) pins;
  (* always_ready *) method Bool irq;
endinterface

module mkSpi#(SpiCfg cfg)(SpiIfc#(aw, dw, fifoDepth, csWidth))
    provisos (Mul#(TDiv#(dw, 8), 8, dw), Add#(_a, 8, aw), Add#(_b, 12, dw),
              Add#(_c, 1, dw), Add#(_d, csWidth, dw), Add#(_e, 2, dw),
              Add#(_f, 4, dw), Add#(_g, 8, dw), Add#(_h, csWidth, 8), Add#(_i, 3, dw));

  SpiRegsIfc#(aw, dw, fifoDepth, csWidth) r <- mkSpiRegs(
      SpiRegsCfg { quad: cfg.quad });

  // 去守卫按端口给：软件那一端走 always_ready 的总线方法，硬件那一端走规则
  FIFOF#(Bit#(8)) txq <- mkGSizedFIFOF(True,  False, valueOf(fifoDepth));
  FIFOF#(Bit#(8)) rxq <- mkGSizedFIFOF(False, True,  valueOf(fifoDepth));

  Reg#(Bool)      busy  <- mkConfigReg(False);
  Reg#(Bit#(12))  div   <- mkReg(0);
  Reg#(Bool)      half  <- mkReg(False);   // 一位两个半周期
  Reg#(Bit#(4))   bitn  <- mkReg(0);
  Reg#(Bit#(8))   shTx  <- mkReg(0);
  Reg#(Bit#(8))   shRx  <- mkReg(0);
  Reg#(Bool)      txPend <- mkReg(False);
  Wire#(Bit#(4))  ioIn  <- mkBypassWire;
  // HOLD 模式下片选已经按下了没有。写 csmode 或 csid 都放开（19.8）。
  Reg#(Bool)      csHeld <- mkConfigReg(False);
  Reg#(Bool)      csWrPend <- mkReg(False);

  // 队列里有几条：FIFOF 不给计数，而入队与出队在不同规则里，共用一个计数器
  // 会抢同一个写口——各自一个自由计数器，相减即占用数。宽度够，回绕不影响差值。
  Reg#(Bit#(8)) txIn  <- mkReg(0);
  Reg#(Bit#(8)) txOut <- mkReg(0);
  Reg#(Bit#(8)) rxIn  <- mkReg(0);
  Reg#(Bit#(8)) rxOut <- mkReg(0);
  Bit#(8) txLevel = txIn - txOut;
  Bit#(8) rxLevel = rxIn - rxOut;

  // 帧方向。这个寄存器本来就被 quad 门控，所以关掉 quad 时它读回零、
  // 两个判据都不成立，行为跟以前一模一样。
  //   dir = 1：只发，线上回来的丢掉——共享线的双线/四线模式下，
  //            主机正在驱动，采到的是自己
  //   dir = 0：只收，共享线上放开驱动让从机送
  Bool sendOnly = cfg.quad && r.fmt_dir == 1;
  Bool recvOnly = cfg.quad && r.fmt_dir == 0;

  // 帧长只有开了 quad 才可配，否则恒 8 位
  Bit#(4) flen = cfg.quad ? (r.fmt_len == 0 ? 8 : r.fmt_len) : 8;

  // swmod 的脉冲与寄存器的新值差一拍，先记脉冲、下一拍再取值
  rule mark;
    txPend <= r.txdata_data_wr;
  endrule

  rule push (txPend && txq.notFull);
    txq.enq(r.txdata_data);
    txIn <= txIn + 1;
  endrule

  // 写脉冲与寄存器不能在同一条规则里读：前者逼着排在总线方法之后、后者逼着
  // 排在之前，bsc 判规则永不触发（G0021，装配一级才现形）。与上面 mark/push
  // 是同一条——先记脉冲，下一拍再动。
  rule csMark;
    csWrPend <= (r.csmode_wr || r.csid_wr);
  endrule

  // 写过 csmode 或 csid 就放开 HOLD 按住的片选；否则一开始传就按下
  rule csTrack;
    if (csWrPend) csHeld <= False;
    else if (busy && r.csmode == 2) csHeld <= True;
  endrule

  rule begin_frame (!busy && txq.notEmpty);
    shTx  <= txq.first;
    txq.deq;
    txOut <= txOut + 1;
    busy  <= True;
    half  <= False;
    bitn  <= 0;
    div   <= r.sckdiv;
  endrule

  rule step (busy);
    if (div != 0)
      div <= div - 1;
    else begin
      div  <= r.sckdiv;
      half <= !half;
      // pha=0 在前沿采样、后沿移位；pha=1 反过来
      Bool sampleNow = (r.sckmode_pha == 0) ? !half : half;
      if (sampleNow)
        shRx <= {shRx[6:0], ioIn[1]};
      else
        shTx <= {shTx[6:0], 1'b0};
      if (half) begin
        if (bitn + 1 == flen) begin
          busy <= False;
          // shRx 到这一拍已经攒够八位了：一个位周期是「先采样、后移位」，
          // 收尾这一拍走的是移位那一边，采样早在上半拍做完。
          // 这里再补一次采样就等于把整个字节循环左移一位——
          // 0xA5 收回来成 0x4B，而 0x00 与 0xFF 转不转都一样，所以只有
          // 非对称的字节看得出来。
          if (!sendOnly && rxq.notFull) begin rxq.enq(shRx); rxIn <= rxIn + 1; end
        end else
          bitn <= bitn + 1;
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

  interface regs = r.regs;
  interface SpiPins pins;
    // 空闲电平由 pol 定；csmode 为 2 时片选常抬，给软件手动控时序留口子
    method Bit#(1) sck = busy ? (half ? ~r.sckmode_pol : r.sckmode_pol)
                              : r.sckmode_pol;
    // 19.8 表 73：0=AUTO 一帧一起一落 · 2=HOLD 第一帧之后一直按住 ·
    // 3=OFF 硬件完全不管。原来写的是 `busy && csmode != 2`——
    // HOLD 从不拉低、OFF 反而跟着帧起落，两个模式的行为正好错位。
    method Bit#(csWidth) cs_n =
      (r.csmode == 3) ? '1
      : ((busy || (r.csmode == 2 && csHeld)) ? ~(1 << r.csid) : '1);
    method Bit#(4) io_o  = {3'b000, r.fmt_endian == 1 && cfg.quad
                                    ? shTx[0] : shTx[7]};
    // 四线模式下收方向要放开全部四根，否则主机与从机对着驱动
    method Bit#(4) io_oe = (cfg.quad && r.fmt_proto == 2)
                         ? (recvOnly ? 4'b0000 : 4'b1111) : 4'b0001;
    method Action io_i(Bit#(4) v); ioIn._write(v); endmethod
  endinterface
  // 与喂给 ip 的是同一个表达式
  method Bool irq = ((r.ie_txwm == 1) && txLevel < zeroExtend(r.txmark))
                 || ((r.ie_rxwm == 1) && rxLevel > zeroExtend(r.rxmark));
endmodule

endpackage
