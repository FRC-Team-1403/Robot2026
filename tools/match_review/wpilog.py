"""Read primitive WPILOG records without changing the source log."""
import struct


def records(path):
    with open(path, 'rb') as f:
        if f.read(6) != b'WPILOG':
            raise ValueError('Not a WPILOG file')
        header = f.read(6)
        if len(header) != 6:
            raise ValueError('Truncated WPILOG header')
        version, extra = struct.unpack('<HI', header)
        if version != 0x100:
            raise ValueError(f'Unsupported WPILOG version {version:#x}')
        if len(f.read(extra)) != extra:
            raise ValueError('Truncated WPILOG header')
        entries = {}
        while True:
            b = f.read(1)
            if not b:
                return
            h = b[0]
            sizes = ((h & 3) + 1, ((h >> 2) & 3) + 1, ((h >> 4) & 7) + 1)
            raw = f.read(sum(sizes))
            if len(raw) != sum(sizes):
                raise ValueError('Truncated record header')
            a, b, c = sizes
            entry = int.from_bytes(raw[:a], 'little')
            size = int.from_bytes(raw[a:a+b], 'little')
            timestamp = int.from_bytes(raw[a+b:a+b+c], 'little')
            data = f.read(size)
            if len(data) != size:
                raise ValueError('Truncated record payload')
            if entry == 0:
                if data and data[0] == 0:
                    if len(data) < 5:
                        raise ValueError('Invalid start record')
                    number = int.from_bytes(data[1:5], 'little')
                    offset = 5
                    strings = []
                    for _ in range(3):
                        if offset + 4 > len(data):
                            raise ValueError('Invalid start record string')
                        length = int.from_bytes(data[offset:offset+4], 'little')
                        offset += 4
                        if offset + length > len(data):
                            raise ValueError('Invalid start record string')
                        strings.append(data[offset:offset+length].decode('utf-8', errors='replace'))
                        offset += length
                    entries[number] = strings[:2]
                elif data and data[0] == 1 and len(data) >= 5:
                    entries.pop(int.from_bytes(data[1:5], 'little'), None)
                continue
            if entry not in entries:
                continue
            name, kind = entries[entry]
            try:
                if kind == 'double' and len(data) == 8:
                    value = struct.unpack('<d', data)[0]
                elif kind == 'float' and len(data) == 4:
                    value = struct.unpack('<f', data)[0]
                elif kind == 'int64' and len(data) == 8:
                    value = struct.unpack('<q', data)[0]
                elif kind == 'boolean' and len(data) == 1:
                    value = bool(data[0])
                elif kind in ('string', 'json'):
                    value = data.decode('utf-8', errors='replace')
                else:
                    continue
            except (ValueError, struct.error):
                continue
            yield timestamp / 1_000_000, name, kind, value
