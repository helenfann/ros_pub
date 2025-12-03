# Usage: python adjust_timestamps.py first.csv second.csv output.csv

import csv
import sys

def read_rows(path):
    with open(path, newline='') as f:
        reader = csv.reader(f)
        rows = list(reader)
    return rows

def write_rows(path, rows):
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(rows)

def parse_header_and_data(rows):
    # Expect header with columns: "# counter, sec, nsec, x, y, z, qx, qy, qz, qw"
    header = rows[0]
    data = rows[1:]
    return header, data

def normalize_sec_nsec(sec, nsec):
    # Ensure nsec in [0, 1_000_000_000)
    while nsec >= 1_000_000_000:
        nsec -= 1_000_000_000
        sec += 1
    while nsec < 0:
        nsec += 1_000_000_000
        sec -= 1
    return sec, nsec

def main(first_csv, second_csv, output_csv):
    rows1 = read_rows(first_csv)
    rows2 = read_rows(second_csv)
    if not rows1 or not rows2:
        print("Error: One of the input files is empty.", file=sys.stderr)
        sys.exit(1)

    header1, data1 = parse_header_and_data(rows1)
    header2, data2 = parse_header_and_data(rows2)

    if not data1 or not data2:
        print("Error: One of the input files has no data rows.", file=sys.stderr)
        sys.exit(1)

    # Get timestamps from first rows
    try:
        sec1 = int(data1[0][1])
        nsec1 = int(data1[0][2])
        sec2 = int(data2[0][1])
        nsec2 = int(data2[0][2])
    except (ValueError, IndexError):
        print("Error: Could not parse sec/nsec in first data rows.", file=sys.stderr)
        sys.exit(1)

    # Compute delta = (first_csv first timestamp) - (second_csv first timestamp)
    delta_sec = sec1 - sec2
    delta_nsec = nsec1 - nsec2
    if delta_nsec < 0:
        delta_nsec += 1_000_000_000
        delta_sec -= 1

    adjusted_rows = []
    adjusted_rows.append(header2)  # keep header from second CSV

    for row in data2:
        new_row = row[:]
        try:
            sec = int(row[1])
            nsec = int(row[2])
        except (ValueError, IndexError):
            print("Error: Invalid row encountered; cannot parse sec/nsec.", file=sys.stderr)
            sys.exit(1)

        sec += delta_sec
        nsec += delta_nsec
        sec, nsec = normalize_sec_nsec(sec, nsec)

        new_row[1] = str(sec)
        new_row[2] = str(nsec)
        adjusted_rows.append(new_row)

    write_rows(output_csv, adjusted_rows)
    print(f"Adjusted timestamps written to {output_csv}")

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python adjust_timestamps.py first.csv second.csv output.csv")
        sys.exit(1)
    main(sys.argv[1], sys.argv[2], sys.argv[3])