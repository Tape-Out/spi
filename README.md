# spi

SPI master and slave, with QSPI and execute-in-place as features.

![maturity](https://img.shields.io/badge/maturity-planned-lightgrey) ![license](https://img.shields.io/badge/license-MulanPSL--2.0-blue)

Part of the [Tape-Out](https://github.com/Tape-Out) IP library: Bluespec IP over the
bus-neutral contracts in [`hwcore`](https://github.com/Tape-Out/hwcore), assembled by
[`xirang`](https://github.com/Tape-Out/xirang). Maturity runs `planned` -> `simulated` ->
`fpga-proven` -> `asic-ready` -> `silicon-proven`.

## Status

Planned. What sits in this repository today is the retired picorv32-era Verilog, kept for
provenance. The Bluespec rewrite against the [`spec`](https://github.com/Tape-Out/spec)
contracts has not landed yet, and it will not reuse this source.

## Notes from the original

暂时实现一个只有一个主设备的SPI，日后扩展多主

## License

Mulan PSL v2.
