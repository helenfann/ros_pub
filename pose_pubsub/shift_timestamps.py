# Usage: python shift_timestamps.py input.csv output.csv
# Adds 8.43 seconds to each timestamp (sec/nsec) in the CSV.

import csv
import sys

OFFSET_SEC = 50
OFFSET_NSEC = 430_000_000  # 0.43 s

def normalize_sec_nsec(sec: int, nsec: int):
    while nsec >= 1_000_000_000:
        nsec -= 1_000_000_000
        sec += 1
    while nsec < 0:
        nsec += 1_000_000_000
        sec -= 1
    return sec, nsec

def read_rows(path):
    with open(path, newline='') as f:
        reader = csv.reader(f)
        return list(reader)

def write_rows(path, rows):
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(rows)

def main(input_csv, output_csv):
    rows = read_rows(input_csv)
    if not rows:
        print("Error: input file is empty.", file=sys.stderr)
        sys.exit(1)

    header = rows[0]
    data = rows[1:]

    adjusted = [header]
    for row in data:
        if len(row) < 10:
            print(f"Error: row has too few columns: {row}", file=sys.stderr)
            sys.exit(1)
        try:
            sec = int(row[1])
            nsec = int(row[2])
        except (ValueError, IndexError):
            print(f"Error: could not parse sec/nsec for row: {row}", file=sys.stderr)
            sys.exit(1)

        sec += OFFSET_SEC
        nsec += OFFSET_NSEC
        sec, nsec = normalize_sec_nsec(sec, nsec)

        new_row = row[:]
        new_row[1] = str(sec)
        new_row[2] = str(nsec)
        adjusted.append(new_row)

    write_rows(output_csv, adjusted)
    print(f"Wrote adjusted CSV to {output_csv}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python add_offset.py input.csv output.csv")
        sys.exit(1)
    main(sys.argv[1], sys.argv[2])

    