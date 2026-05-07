# Building the SCAMP Python Module on Windows (Python 3.12+)

This guide replicates the build performed on this machine. It targets **Windows + 64-bit
Python 3.12 or newer** (where `distutils` was removed from the standard library) and
uses **MSVC** from a Visual Studio install.

The end result is a CPython extension module `scamp.<tag>.pyd` that wraps the prebuilt
`scamp5d_interface.lib` and lets a Python program talk to a SCAMP-5d board over USB or TCP.

---

## 1. Prerequisites

| Requirement | This machine | Notes |
|---|---|---|
| 64-bit Python 3.x | `D:\miniconda\python.exe` (3.13.12) | 32-bit also works, see *32-bit build* below. |
| `setuptools`     | 80.x | `python -m pip install setuptools` if missing. |
| MSVC C++ toolchain | VS Community 2026 at `C:\Program Files\Microsoft Visual Studio\18\Community` | VS 2019 / 2022 / 2026 all work. The "Desktop development with C++" workload installs everything needed. |
| Windows SDK | 10.0.26100 | Installed by the C++ workload. |
| Prebuilt `scamp5d_interface.lib` | [scamp5-sol/interface/x64/Release/scamp5d_interface.lib](../x64/Release/scamp5d_interface.lib) | Already in the repo — you do **not** need to rebuild it. |

Verify your VS install with:

```powershell
& "C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe" -latest -property displayName
```

If `vswhere` reports your VS edition, `setuptools` will auto-detect MSVC and you
can build from any shell. Otherwise, use the **x64 Native Tools Command Prompt for VS**
from the Start menu — it pre-loads `vcvarsall.bat`.

---

## 2. The `distutils` problem (Python 3.12+)

The original [setup.py](setup.py) starts with:

```python
from distutils.core import setup, Extension      # ← removed in Python 3.12
```

`distutils` was deprecated in Python 3.10 and **removed** in Python 3.12.
Switching to `setuptools` is a one-line change and is fully compatible with the
rest of the file:

```python
from setuptools import setup, Extension
```

That single edit is the only source change needed for modern Python. (This repo
has already been patched — see [setup.py:7](setup.py#L7).)

---

## 3. Build commands

From the [scamp5-sol/interface/scamp_python_module/](.) directory:

```powershell
# 1. Compile and link in-place (produces scamp.cp3XX-win_amd64.pyd next to setup.py)
D:\miniconda\python.exe setup.py build_ext --inplace

# 2. Install into your site-packages so `import scamp` works from anywhere
D:\miniconda\python.exe setup.py install
```

`build_ext --inplace` is the fastest dev loop — it drops the `.pyd` next to your
script and you can iterate without reinstalling. `install` is for sharing the
module across projects.

After a successful build you'll see:

```
build/temp.win-amd64-cpython-3XX/Release/scampmodule.obj
build/temp.win-amd64-cpython-3XX/Release/scampmodule_packet_switch.obj
scamp.cp3XX-win_amd64.pyd        ← the importable module
```

Sanity check:

```powershell
D:\miniconda\python.exe -c "import scamp; print(sorted(a for a in dir(scamp) if not a.startswith('_')))"
```

You should see `['VS_MSG_GUI_UPDATE', 'VS_MSG_HOST_DC', 'VS_MSG_HOST_ON', 'VS_MSG_HOST_VALUE', 'VS_MSG_PACKET', 'VS_MSG_USER_VALUE', 'close', 'get_packet', 'get_packet_raw', 'open_tcp', 'open_usb', 'routine', 'send_file', 'send_gui_value', 'send_image', 'send_message']`.

---

## 4. What `setup.py` is actually doing

```python
ext_module_1 = Extension(
    name = 'scamp',
    sources = ['scampmodule.cpp', 'scampmodule_packet_switch.cpp'],
    include_dirs = ['../scamp5d_interface'],   # SCAMP C++ headers
    library_dirs = ['../x64/Release/'],         # prebuilt .lib lives here
    libraries = ['scamp5d_interface',           # the static SCAMP lib
                 'kernel32','user32','advapi32'],# Win32 libs the .lib pulls in
    extra_compile_args = ['/O2'],
    extra_link_args = ['/LTCG:OFF'],            # disable LTCG to avoid
                                                #   mismatch with the .lib
    language='c++'
    )
```

- The Python wrapper at [scampmodule.cpp](scampmodule.cpp) exposes the C API
  via `PyMethodDef` ([line 214](scampmodule.cpp#L214)).
- [scampmodule_packet_switch.cpp](scampmodule_packet_switch.cpp) decodes
  on-wire SCAMP packets into Python dicts.
- The prebuilt static lib at [../x64/Release/scamp5d_interface.lib](../x64/Release/scamp5d_interface.lib)
  contains all the USB / TCP / packet-decoding code, plus already-resolved
  imports for `SETUPAPI`, `USERENV`, `PSAPI`, `IPHLPAPI`, `WS2_32`, etc.
- `/LTCG:OFF` is necessary because the prebuilt `.lib` was compiled without
  link-time code generation; turning LTCG on at the Python build would mismatch.

---

## 5. 32-bit build

Switch the lib path in [setup.py:15](setup.py#L15) from
`../x64/Release/` to `../Release/` and use a 32-bit Python interpreter.
Everything else is identical.

---

## 6. Module API (what got compiled in)

| Function | Signature | Purpose |
|---|---|---|
| `open_usb(serial)` | `(str) -> int` | Open USB direct connection. Returns 0 on success. Pass `'0'` to grab the first device. |
| `open_tcp(host, port)` | `(str, int) -> int` | Open TCP connection (e.g. to `scamp5d_proxy`). |
| `close()` | `() -> int` | Close the connection and free queued packets. |
| `routine()` | `() -> None` | Pump the SCAMP I/O state machine. Must be called regularly. |
| `get_packet()` | `() -> dict\|None` | Pop one decoded packet, or `None` if queue is empty. |
| `get_packet_raw()` | `() -> bytearray\|None` | Same but returns the raw on-wire bytes. |
| `send_gui_value(id, value)` | `(int, int) -> None` | Update GUI item `id` on the chip. **This is how you push parameters to firmware.** |
| `send_message(code, arg, data)` | `(int, int, int) -> None` | Low-level message; codes are `VS_MSG_*` constants. |
| `send_image(buf, w, h, n_bits)` | `(bytes, int, int, int) -> None` | Reply to a `REQUEST/IMAGE` packet. |
| `send_file(buf)` | `(bytes) -> None` | Reply to a `REQUEST/FILE` packet. |

Constants exposed: `VS_MSG_PACKET`, `VS_MSG_HOST_ON`, `VS_MSG_HOST_DC`,
`VS_MSG_GUI_UPDATE`, `VS_MSG_USER_VALUE`, `VS_MSG_HOST_VALUE`.

### Packet shape

`get_packet()` returns either `None` or a dict. For `type='data'` packets the
`datatype` field selects the rest of the schema (defined in
[scampmodule_packet_switch.cpp](scampmodule_packet_switch.cpp)):

| `datatype` | Extra keys |
|---|---|
| `TEXT` | `text` (str) |
| `SCAMP5_AOUT` | `width`, `height`, `buffer` (bytes, 8-bit greyscale) |
| `SCAMP5_DOUT` | `width`, `height`, `buffer` (bytes, 8-bit, values 0/255) |
| `INT8` / `INT16` / `INT32` / `FLOAT` | `n_rows`, `n_cols`, `data` (nested list) |
| `BBOX` / `EVENTS` | `n_rows`, `n_cols`, `data` (list of `(x,y)` tuples) |
| `REQUEST` | `filetype` (`IMAGE` or `FILE`), `filepath`, `n_bits` |

Common keys on every data packet: `loopcounter` (frame index), `channel` (which
output channel the data came from), `size` (bytes on the wire).

The image buffer needs to be flipped before display. The original [test.py](test.py#L34)
uses Pillow's transpose:

```python
img = Image.frombytes('L', (w, h), packet['buffer']) \
           .transpose(Image.Transpose.FLIP_LEFT_RIGHT) \
           .transpose(Image.Transpose.ROTATE_180)
```

(In Pillow ≥10, the legacy `Image.FLIP_LEFT_RIGHT` and `Image.ROTATE_180`
module-level constants were removed — use `Image.Transpose.*` instead.)

---

## 7. Troubleshooting

**`ModuleNotFoundError: No module named 'distutils'`**
You're on Python 3.12+. Apply the `setuptools` patch in §2.

**`error: Microsoft Visual C++ 14.0 or greater is required`**
MSVC is not on `PATH`. Either install the "Desktop development with C++" workload
in the Visual Studio Installer, or run from the *x64 Native Tools Command Prompt for VS*.

**`LINK : fatal error LNK1181: cannot open input file 'scamp5d_interface.lib'`**
The `library_dirs` path in `setup.py` is wrong for your build, or you're trying
a 32-bit build with the 64-bit `.lib`. Cross-check `library_dirs` against the
folder that actually contains the `.lib`.

**`LNK4006 __NULL_IMPORT_DESCRIPTOR already defined ... second definition ignored`**
Harmless. The prebuilt `.lib` aggregates several Windows DLL imports and the
linker dedupes the descriptor.

**`LNK4098: defaultlib 'LIBCMT' conflicts with use of other libs`**
Also harmless in this build. The prebuilt `.lib` was compiled `/MT` (static
CRT) but the Python extension uses `/MD` (dynamic CRT) by default; the linker
prefers `/MD` and the result still works because the static-lib path inside
`scamp5d_interface.lib` doesn't actually call any conflicting CRT symbols.
If it ever did cause a real failure, add `extra_link_args=['/NODEFAULTLIB:LIBCMT']`.

**`ImportError: DLL load failed while importing scamp`**
Almost always a Python-architecture mismatch — 32-bit Python trying to load
the 64-bit `.pyd`, or vice versa. Confirm with
`python -c "import struct; print(struct.calcsize('P')*8)"` (prints `64` for 64-bit).

**`AttributeError: module 'PIL.Image' has no attribute 'FLIP_LEFT_RIGHT'`**
Pillow ≥ 10 removed the legacy constants. Either pin `pip install "pillow<10"`
or update to `Image.Transpose.FLIP_LEFT_RIGHT` etc.

---

## 8. References

- [SCAMP-5d Python Support](https://scamp.gitlab.io/scamp5d_doc/_p_a_g_e__p_y_t_h_o_n__s_u_p_p_o_r_t.html)
- [SCAMP-5d I/O Guide](https://scamp.gitlab.io/scamp5d_doc/_p_a_g_e__g_u_i_d_e__i_o.html)
- Minimal headless example: [../scamp_python_minimal/](../scamp_python_minimal/)
