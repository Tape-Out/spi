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
              Add#(_f, 4, dw), Add#(_g, 8, dw), Add#(_h, csWidth, 8));

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

  // 帧长只有开了 quad 才可配，否则恒 8 位
  Bit#(4) flen = cfg.quad ? (r.fmt_len == 0 ? 8 : r.fmt_len) : 8;

  // swmod 的脉冲与寄存器的新值差一拍，先记脉冲、下一拍再取值
  rule mark;
    txPend <= r.txdata_data_wr;
  endrule

  rule push (txPend && txq.notFull);
    txq.enq(r.txdata_data);
  endrule

  rule begin_frame (!busy && txq.notEmpty);
    shTx  <= txq.first;
    txq.deq;
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
          if (rxq.notFull) rxq.enq({shRx[6:0], ioIn[1]});
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
    r.ip_txwm_in(txq.notFull ? 1 : 0);
    r.ip_rxwm_in(rxq.notEmpty ? 1 : 0);
  endrule

  // swacc：软件读过 rxdata 就弹一个
  rule pop (r.rxdata_data_rd && rxq.notEmpty);
    rxq.deq;
  endrule

  interface regs = r.regs;
  interface SpiPins pins;
    // 空闲电平由 pol 定；csmode 为 2 时片选常抬，给软件手动控时序留口子
    method Bit#(1) sck = busy ? (half ? ~r.sckmode_pol : r.sckmode_pol)
                              : r.sckmode_pol;
    method Bit#(csWidth) cs_n = (busy && r.csmode != 2)
                              ? ~(1 << r.csid) : '1;
    method Bit#(4) io_o  = {3'b000, r.fmt_endian == 1 && cfg.quad
                                    ? shTx[0] : shTx[7]};
    method Bit#(4) io_oe = cfg.quad && r.fmt_proto == 2 ? 4'b1111 : 4'b0001;
    method Action io_i(Bit#(4) v); ioIn._write(v); endmethod
  endinterface
  method Bool irq = ((r.ie_txwm == 1) && txq.notFull)
                 || ((r.ie_rxwm == 1) && rxq.notEmpty);
endmodule

endpackage
