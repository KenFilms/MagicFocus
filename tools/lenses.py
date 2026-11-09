#!/usr/bin/env python3
"""Reads or replaces MagicFocus's lens names over USB, without taking out the SD card.

    python3 tools/lenses.py show           print each slot's name and number of calibration points
    python3 tools/lenses.py pull [file]    write the M5's lenses, names and points, into a list
    python3 tools/lenses.py push [file]    send a list to MagicFocus, which saves it on the SD card

The default list is lenses.txt in the repository's top folder, which is also built into the firmware
for SD cards without a list of their own. A list has a line per lens, "<slot>: <name>", with the
slots numbered 1 to 8 as on the M5's screen, each followed by up to six calibration points as lines
"<metres> m: <motor position>". A lens line that is just a name takes the first free slot, and lines
starting with # are ignored. On the SD card it is /tiltafocus/lenses.txt (see the README, "Calibrate
a lens").

A list with points replaces all the M5's lenses, so pull, edit and push it back; push refuses a list
that would delete points the M5 has unless given --force. A list without points only names lenses,
and every lens keeps its points.

MagicFocus must be running and connected by USB. Needs pyserial: pip install pyserial (the Python that
comes with PlatformIO already has it). Use --port when more than one board is connected.
"""
import argparse
import re
import sys
import time
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit('This needs pyserial: pip install pyserial')

SLOT_LINE = re.compile(r'^\[LENS\] slot (\d+), (\d+) points?:\s*(.*)$')
POINT_LINE = re.compile(r'^\s*([0-9]*\.?[0-9]+)\s*m\s*:\s*(\d+)\s*$')
LENS_LINE = re.compile(r'^\s*(\d+)\s*:\s*(.*?)\s*$')
# USB vendor ids: Espressif's own USB on the CoreS3, and the Core2's CP210x or CH9102 serial chips.
BOARD_VIDS = {0x303A, 0x10C4, 0x1A86}
NAME_MAX = 31
DEFAULT_LIST = Path(__file__).resolve().parent.parent / 'lenses.txt'


def find_port():
    """The one connected board's serial port."""
    ports = [port.device for port in list_ports.comports() if port.vid in BOARD_VIDS]
    if len(ports) == 1:
        return ports[0]
    if not ports:
        sys.exit('No MagicFocus board found on USB. Connect it, or name its port with --port.')
    sys.exit('More than one board found; pick one with --port: ' + ', '.join(ports))


def open_board(port):
    """Opens the port without the DTR and RTS changes that would restart the board."""
    board = serial.Serial()
    board.port = port
    board.baudrate = 115200
    board.timeout = 0.1
    board.dtr = False
    board.rts = False
    board.open()
    time.sleep(0.2)
    board.reset_input_buffer()
    return board


def read_lens_lines(board, done, seconds, prefix='[LENS]'):
    """Collects the board's lines that start with prefix until done(lines) or the time is up."""
    lines, buffer, end = [], b'', time.time() + seconds
    while time.time() < end:
        buffer += board.read(512)
        while b'\n' in buffer:
            raw, buffer = buffer.split(b'\n', 1)
            text = raw.decode('utf-8', errors='replace').rstrip()
            if text.startswith(prefix):
                lines.append(text)
                if done(lines):
                    return lines
    return lines


def read_slots(board):
    """Each slot as (number, saved points, name)."""
    board.write(b'N\n')
    lines = read_lens_lines(board, lambda lines: lines[-1] == '[LENS] end', 3)
    if not lines or lines[-1] != '[LENS] end':
        sys.exit("MagicFocus didn't list its lenses. Is it running, with firmware that has the N command?")
    return [(int(m[1]), int(m[2]), m[3]) for m in map(SLOT_LINE.match, lines) if m]


def read_list(board):
    """The M5's lens list, names and points, as it saves it on the SD card."""
    board.write(b'F\n')
    lines = read_lens_lines(board, lambda lines: lines[-1] == '[LIST] end', 3, '[LIST]')
    if not lines or lines[-1] != '[LIST] end':
        sys.exit("MagicFocus didn't send its lens list. Is it running, with firmware that has the F command?")
    return ''.join(line[len('[LIST] '):] + '\n' for line in lines[:-1])


def list_entries(lines):
    """Each lens in a list as [slot number or None, name, number of points]."""
    entries = []
    for line in lines:
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        if POINT_LINE.match(line):
            if entries:
                entries[-1][2] += 1
            continue
        lens = LENS_LINE.match(line)
        entries.append([int(lens[1]), lens[2], 0] if lens else [None, line.strip(), 0])
    return entries


def show(slots):
    for number, points, name in slots:
        calibration = f'  ({points} point{"" if points == 1 else "s"})' if points else ''
        print(f'{number}: {name or "-"}{calibration}')


def pull(board, path):
    text = read_list(board)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    show(read_slots(board))
    print(f'Saved to {path}')


def push(board, path, force):
    with open(path, encoding='utf-8-sig') as f:
        lines = [line.rstrip('\r\n') for line in f]
    entries = list_entries(lines)
    if any(points for _, _, points in entries) and not force:
        # A list with points replaces all the M5's lenses: make sure none loses its points by mistake.
        lost = [(number, name) for number, points, name in read_slots(board) if points and not any(
            listed_points and (listed == number or (listed is None and listed_name.lower() == name.lower()))
            for listed, listed_name, listed_points in entries)]
        if lost:
            names = ', '.join(f'{number} ({name or "no name"})' for number, name in lost)
            sys.exit(f'This list would delete the calibration points of slot {names}. Run pull first to get '
                     'them into the list, or push with --force to delete them.')
    for count, line in enumerate(lines, 1):
        name = line.split(':', 1)[1].strip() if re.match(r'^\s*\d+\s*:', line) else line.strip()
        if not line.lstrip().startswith('#') and len(name) > NAME_MAX:
            print(f'Line {count}: names have at most {NAME_MAX} characters, so "{name}" will be cut short')
        if not line.isascii():
            print(f'Line {count}: the M5 shows only plain ASCII characters')
    if any(line.strip() == '.' for line in lines):
        sys.exit('A line holding only "." would end the list early; remove it.')
    board.write(b'L\n')
    for line in lines:
        board.write(line.encode('utf-8')[:63] + b'\n')
        time.sleep(0.01)
    board.write(b'.\n')
    answer = read_lens_lines(board, lambda lines: lines[-1].startswith('[LENS] list'), 5)
    result = [line for line in answer if line.startswith('[LENS] list')]
    if not result:
        sys.exit("MagicFocus didn't answer. Is it running, with firmware that has the L command?")
    if result[-1] != '[LENS] list saved':
        sys.exit(result[-1].removeprefix('[LENS] '))
    show(read_slots(board))
    print('Saved on the SD card as /tiltafocus/lenses.txt')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('command', choices=['show', 'pull', 'push'])
    parser.add_argument('file', nargs='?', default=DEFAULT_LIST,
                        help=f'list to write (pull) or send (push); by default {DEFAULT_LIST}')
    parser.add_argument('--port', help='serial port of the board, found by itself when only one is connected')
    parser.add_argument('--force', action='store_true', help='push even if lenses on the M5 would lose their points')
    args = parser.parse_args()
    board = open_board(args.port or find_port())
    try:
        if args.command == 'show':
            show(read_slots(board))
        elif args.command == 'pull':
            pull(board, args.file)
        else:
            push(board, args.file, args.force)
    finally:
        board.close()


if __name__ == '__main__':
    main()
