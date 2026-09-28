# Vendored third-party plugin: Visual Studio Integration Tool

- Source: https://github.com/microsoft/vc-ue-extensions
- Commit: eaa20e73d1ba2f5ffa5c32cea063e3213c68ed98 (2026-09-21, "fix/ue58-compat")
- Plugin version: 2.8
- License: MIT (see LICENSE)
- Copied: VisualStudioTools.uplugin, Source/, Config/, LICENSE (repo tooling and docs omitted)

Editor-only. It lets Visual Studio 2026 show Blueprint references and Unreal reflection info
in C++ code, and discover and run Unreal automation tests from VS's Test Explorer.
It has no effect on the simulation and is never packaged into the game.

To update: clone the repo at a newer commit, replace these files, update this note, and rebuild.
