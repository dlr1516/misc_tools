import csv
from pathlib import Path

import re


results_filename = "../build/results_icp_vfc_arspw_arsgraph_20260213_1139_08.csv"

rows = []
path = Path(results_filename)
if not path.exists():
    raise FileNotFoundError(f"{path} does not exist")

with path.open(newline="", encoding="utf-8") as fh:
    reader = csv.DictReader(fh)
    # print("type(reader.fieldnames)")
    # print(type(reader.fieldnames))
    reader_fieldnames_stripped = []
    for field in reader.fieldnames:
        field = re.sub("\\s+", "", field)  # remove \n, \t from header
        field = field.replace(" ", "") # remove spaces from header
        # print(f"Field: '{field}'")
        reader_fieldnames_stripped.append(field)
    for row in reader:
        row_stripped = {}
        # print("type(row)")
        # print(type(row))
        for key in row:
            value = row[key]
            value = re.sub("\\s+", "", value)  # remove \n, \t from header
            value = value.replace(" ", "") # remove spaces from header
            key_stripped = re.sub("\\s+", "", key)  # remove \n, \t from header
            key_stripped = key_stripped.replace(" ", "") # remove spaces from header
            row_stripped[key_stripped] = value
        rows.append(row_stripped)
        print(f"row_stripped: {row_stripped}")

print(f"Loaded {len(rows)} rows, fields: {reader_fieldnames_stripped}")
