GW-Spark-10 educational GPGPU device
====================================

.. Copyright (c) 2024-2025

   SPDX-License-Identifier: GPL-2.0-or-later

The GW-Spark-10 is an educational PCIe 3D accelerator model. It provides a
control register area, VRAM, a synchronous DMA engine, and a small RISC-V
instruction interpreter for course kernels.

The device can be added to a PCIe machine with::

    -device gw-spark-10[,num_cus=N,warps_per_cu=N,warp_size=N,vram_size=SIZE]

The default properties are four compute units, four warps per compute unit, a
32-lane warp, and 64 MiB of VRAM. The compute-unit and warp properties are
reported through the capability register. They do not change execution
scheduling.

PCI configuration
-----------------

The PCI vendor and device IDs are ``1234:1337``. The class code identifies a
3D controller.

BAR0 contains a 1 MiB control register region. BAR2 contains VRAM and defaults
to 64 MiB. BAR4 contains a 64 KiB doorbell region; reads return zero and writes
have no effect.

The control registers expose device identity, status and error state, interrupt
status, DMA addresses and control, kernel and grid/block parameters, and the
current SIMT thread context. The register definitions are in
``include/hw/misc/gw-spark-10.h``.

Execution model
---------------

Writing the dispatch register executes the configured kernel synchronously.
Blocks, warps, and lanes run serially. A warp has 32 lanes. The interpreter
implements the RV32 instruction subset used by the course tests, including
basic RV32F operations and BF16, FP8 E4M3/E5M2, and FP4 E2M1 conversions.
Kernel code and data are addressed within VRAM using 32-bit core addresses.

The execution core does not consume kernel arguments or shared memory. The
barrier register records writes but does not synchronize lanes. The doorbell
region is inert. Interrupt status bits are latched in the control registers;
MSI-X and INTx notification delivery is not implemented.

The model is intended for functional exercises. Its compute-unit properties,
serial execution, and limited interpreter do not represent GPU throughput or
hardware scheduling behavior.

Source provenance
-----------------

The implementation is based on the QEMU Camp professional-stage experiment
from ``qemu-camp/qemu-camp-2026-exper-dweeqhd``, commit
``c817145e1106a05c5446fb406f30e1cb51e8f273``. The source files retain their
GPL-2.0-or-later notices.
