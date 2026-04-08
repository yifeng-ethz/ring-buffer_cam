# Agent Notes

- Never modify generated RTL under any `functional/` output directory in-place. If a change is needed, create a copy and ask for approval before wiring it in.
- Semantics-preserving edits for tool/simulator compatibility are OK; functional/behavior changes require explicit approval.
- Shared System Console toolkit sources live under `system_console_toolkit/`.
- The FE SciFi toolkit source of truth is `system_console_toolkit/fe_scifi/`.
- `online_dpv2` consumes the FE SciFi toolkit through the compatibility path `/home/yifeng/packages/online_dpv2/online/mu3e-ip-cores/system_console_toolkit`, which must resolve back to this repo.
- `/home/yifeng/packages/online_dpv2/online/mu3e-ip-cores` is only a compatibility symlink into the deprecated snapshot area. Do not add or update toolkit sources under its `system_console_toolkits/` tree.
