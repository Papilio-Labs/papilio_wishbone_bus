from pathlib import Path
import re

VCD_FILE = Path(__file__).with_name("tb_spi_wb_register.vcd")

HEADER_RE = re.compile(r"^\$var\s+\w+\s+(\d+)\s+([^\s]+)\s+([^\s]+)")
SCALAR_RE = re.compile(r"^([01xz])(.+)")
VECTOR_RE = re.compile(r"^b([01xz]+)\s+(.+)")


def parse_header(lines):
    id_to_name = {}
    for line in lines:
        line = line.strip()
        if line.startswith("$enddefinitions"):
            break
        m = HEADER_RE.match(line)
        if not m:
            continue
        _width, identifier, name = m.groups()
        # Keep the first occurrence to avoid inner-scope duplicates stomping top-level signals
        id_to_name.setdefault(identifier, name)
    return id_to_name


def stream(lines, id_to_name):
    t = 0
    for line in lines:
        line = line.strip()
        if not line:
            continue
        if line.startswith("#"):
            t = int(line[1:])
            continue
        m = VECTOR_RE.match(line)
        if m:
            bits, ident = m.groups()
            yield t, id_to_name.get(ident), bits
            continue
        m = SCALAR_RE.match(line)
        if m:
            val, ident = m.groups()
            yield t, id_to_name.get(ident), val


def summarize():
    if not VCD_FILE.exists():
        raise SystemExit(f"Missing VCD file: {VCD_FILE}")

    lines = VCD_FILE.read_text().splitlines()
    id_to_name = parse_header(lines)

    ack_pulses = 0
    writes = []
    reads = []
    cur = {}

    for t, name, val in stream(lines, id_to_name):
        if name is None:
            continue
        if name == "wb_we":
            cur["we"] = val
        if name == "wb_dat_o":
            cur["dat_o"] = val
        if name == "wb_dat_i":
            cur["dat_i"] = val
        if name == "wb_ack" and val == "1":
            ack_pulses += 1
            if cur.get("we") == "1":
                writes.append((t, cur.get("dat_o")))
            else:
                reads.append((t, cur.get("dat_i")))

    print("=== VCD Parse Summary ===")
    print(f"Ack pulses: {ack_pulses}")
    print(f"Writes observed: {len(writes)}")
    for t, d in writes[:5]:
        print(f"  t={t} dat_o={d}")
    print(f"Reads observed: {len(reads)}")
    for t, d in reads[:5]:
        print(f"  t={t} dat_i={d}")
    if ack_pulses == 0:
        print("WARNING: no ack pulses detected; check SPI stimulus")


if __name__ == "__main__":
    summarize()
