# spi

SPI master, one to eight data lines.

![maturity](https://img.shields.io/badge/maturity-simulated-yellow) ![license](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0%20OR%20MulanPSL--2.0-blue)

Part of the [Tape-Out](https://github.com/Tape-Out) IP library: Bluespec IP over the
bus-neutral contracts in [`hwcore`](https://github.com/Tape-Out/hwcore), assembled by
[`xirang`](https://github.com/Tape-Out/xirang). Maturity runs `planned` -> `simulated` ->
`fpga-proven` -> `asic-ready` -> `silicon-proven`.

## Status

Simulated. The register map follows chapter 19 of the SiFive FE310-G002 manual. `lines`
picks 1, 2, 4 or 8 data lines and the shifter moves that many bits a beat, so a frame
takes 8, 4, 2 or 1 beats; the behavioural testbench counts the edges on the pins.

Not implemented: per-frame protocol selection (`fmt.proto` is accepted and ignored,
since the line count is fixed at build time), the memory-mapped flash interface
(`fctrl`/`ffmt`), execute-in-place, and the slave side.

## License

任选其一：

- [MIT](LICENSE-MIT)
- [Apache 2.0](LICENSE-APACHE)
- [木兰宽松许可证 第2版](LICENSE-MULAN)

`SPDX-License-Identifier: MIT OR Apache-2.0 OR MulanPSL-2.0`

除非另行说明，你提交的贡献按上述三者同时授权，不附加其他条件。
