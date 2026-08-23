# QEMU Downstream

<p align="center">
  <strong>Community-maintained QEMU for RISC-V, ARM, and dynamic translation</strong><br>
  A practical workspace for upstream-oriented engineering, experiments, and ecosystem enablement.
</p>

<p align="center">
  <a href="https://www.qemu.org/">QEMU</a> ·
  <a href="https://www.qemu.org/documentation/">Documentation</a> ·
  <a href="https://lore.kernel.org/qemu-devel/">qemu-devel archive</a> ·
  <a href="https://gitlab.com/qemu-project/qemu">Upstream source</a>
</p>

> This repository is our community-maintained downstream tree of QEMU. Its primary focus is the QEMU RISC-V target and the wider RISC-V ecosystem, with related work in ARM support and TCG-based binary translation. It is a development tree for work that is being prepared for, discussed with, or carried toward upstream QEMU.

This tree complements upstream QEMU rather than replacing it. Downstream branches may contain work in progress, experimental implementations, RFC snapshots, and integration support that are not yet part of an upstream release.

## What we work on upstream

Our upstream contributions span the complete path from target architecture semantics to generated host code. The goal is to make QEMU more capable, measurable, and efficient while keeping the changes maintainable for the upstream community.

| Area | Focus |
| --- | --- |
| **QEMU RISC-V** | ISA and CPU support, machine and platform enablement, device models, architecture-level improvements, and correctness or performance testing. |
| **QEMU ARM** | Architecture support, system components, IOMMU/SMMU-related work, test coverage, and optimizations that improve emulation quality and performance. |
| **TCG and binary translation** | Improvements across instruction lowering, TCG IR construction and optimization, host code generation, fast paths, and target/backend coordination. |

### Architecture-aware optimization

Architecture work is not limited to adding an instruction or a device. We also look at how architectural behavior can be represented more directly and efficiently in QEMU: CPU models, system instructions, memory and exception behavior, platform devices, and the tests that protect them.

### TCG optimization

TCG is the bridge between guest instructions and executable host code. Our optimization work therefore covers both sides of that bridge: reducing unnecessary translation work, improving generated code, exposing useful target capabilities, and keeping the resulting behavior correct across different hosts and guests.

Where possible, upstream submissions include the implementation, tests, benchmarks, and design context needed for review.

## Core active contributors

The following contributors are among the core active members of our community in upstream QEMU. The links search the `qemu-devel` mailing-list archive by name, making it easy to follow patch series, reviews, and RFC discussions.

| Contributor | Representative areas | Upstream discussions |
| --- | --- | --- |
| **Chao Liu** | RISC-V CPU and platform support, architecture-level work, ARM/IOMMU, and TCG-related contributions | [Search qemu-devel](https://lore.kernel.org/qemu-devel/?q=Chao+Liu) |
| **Tao Tang** | RISC-V and TCG implementation, optimization, and review work | [Search qemu-devel](https://lore.kernel.org/qemu-devel/?q=Tao+Tang) |
| **Junze Cao** | RISC-V/TCG development and early-stage RFC discussions | [Search qemu-devel](https://lore.kernel.org/qemu-devel/?q=Junze+Cao) |

The focus labels above are intentionally broad. Contributions often cross the boundary between a target architecture and TCG, and many upstream changes are collaborative.

## What lives in downstream branches

Our downstream branches provide a focused place to develop and validate work before it is ready for upstream review. They may include:

| Category | What to expect |
| --- | --- |
| **Community developments** | New features, optimizations, prototypes, and other progress developed by our community. |
| **RFC discussions** | Early implementations, alternative designs, benchmarks, and patch snapshots used to collect feedback from upstream and ecosystem developers. |
| **Project enablement** | QEMU changes and integration support required by adjacent projects and workflows, including `cosim-gpu`. |

The downstream tree is deliberately fast-moving. If you need a stable, broadly supported QEMU release, use an official upstream release or a distribution package. If you want to evaluate a new idea, reproduce a benchmark, or help shape an upstream change, this is the right place to start.

## From experiment to upstream

```text
Idea → Downstream implementation → Tests and benchmarks → RFC on qemu-devel → Upstream patch series
```

Downstream work is most valuable when it leaves a clear trail: why the change is needed, how it affects translation or emulation, how it was measured, and what feedback is still open. The downstream branch is a staging ground; upstream QEMU remains the long-term home for generally useful changes.

## Build from source

This tree follows the standard QEMU build system. A minimal build looks like this:

```sh
mkdir build
cd build
../configure
make -j"$(nproc)"
```

For platform-specific dependencies and configuration options, see the [QEMU documentation](https://www.qemu.org/documentation/) and the upstream [build documentation](https://www.qemu.org/docs/master/devel/build-environment.html). The original upstream-oriented notes in this source tree are also available in [`README.rst`](README.rst).

## Contributing

Issues, design feedback, benchmarks, and implementation suggestions are welcome. For work intended to enter upstream QEMU:

1. Explain the problem and the architectural or translation-level impact.
2. Include focused tests and performance data where relevant.
3. Send the patch series or RFC to the [`qemu-devel` mailing list](https://lore.kernel.org/qemu-devel/) and follow the [QEMU contribution guidelines](https://www.qemu.org/docs/master/devel/submitting-a-patch.html).

For downstream-only experiments, keep the scope and status visible in the branch or commit message so that users can distinguish exploratory work from upstream-ready changes.

## License

QEMU is released under the GNU General Public License, version 2. See [`LICENSE`](LICENSE) and the headers of individual files for the applicable licensing terms.

<p align="center">
  <sub>Built in the open · measured in practice · upstream when ready</sub>
</p>
