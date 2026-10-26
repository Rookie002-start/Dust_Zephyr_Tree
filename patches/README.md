# Zephyr upstream patches

This repository keeps changes that are required in a community Zephyr checkout
without maintaining a separate Zephyr fork.

## Active patch

| Target | Upstream base | Patch |
| --- | --- | --- |
| Zephyr | `v4.4.1`, `1f6485eca25431b5ff27ce9a754218c9e559bbbb` | `zephyr/0001-zephyr-v4.4.1-local.diff` |

The patch contains the complete local delta for:

- STM32 PWM/SPI integration fixes.

- CMSIS compatibility headers and module configuration.

- Kconfig parsing compatibility.

- HPM PLIC handling and the `damiao_mc02` board integration.

Apply it to a clean Zephyr v4.4.1 checkout:

```bash
./patches/apply-zephyr-patches.sh \
  --zephyr ~/zephyrproject/zephyr
```

Validate without changing the checkout:

```bash
./patches/apply-zephyr-patches.sh \
  --zephyr ~/zephyrproject/zephyr \
  --check
```

The script is idempotent: it skips patches that are already applied and refuses
to modify the wrong upstream revision.
