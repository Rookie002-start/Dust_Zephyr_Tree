# Submodule migration history

On 2026-10-26 the following Git submodules were converted to ordinary
directories in `Dust_Zephyr_Tree`:

| Directory | Imported commit | Original repository |
| --- | --- | --- |
| `framework/algorithm` | `a22b79e7b59f300cbf409a743c1862f94e080c4f` | `Dust_Zephyr_Architecture_Algorithm` |
| `framework/cmd` | `0c4f973c33a384472dcee9ffe50c8c07b38e6778` | `Dust_Zephyr_Architecture_Cmd` |
| `framework/drivers` | `0e5218cd2aae61eb51ce95757374de56d7b7f145` | `Dust_Zephyr_Architecture_Drivers` |
| `framework/init` | `b23b617694b51a31db9fa8847bcbfa9ed36ad00c` | `Dust_Zephyr_Architecture_Init` |
| `framework/modules` | `ff19d18e1a282fb764e9c1ed2b8edb30fd0fed57` | `Dust_Zephyr_Architecture_Modules` |
| `framework/topic` | `d959b398c1e14480cd04a977ba2c3aa6bd07c1af` | `Dust_Zephyr_Architecture_Topic` |
| `project` | `955ba4b56a29b2b6b8a31ff3e8ebae8e4c871618` | `Dust_Zephyr_Architecture_Project` |

The five commits created during the migration are stored as text patches in
this directory. Full Git bundles were created outside the repository at:

```text
~/zephyrproject/zephyr_user-submodule-backup-20261026/
```

The migrated directory contents were verified against each imported commit's
Git tree before accepting the conversion.
