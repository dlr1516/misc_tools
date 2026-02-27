import csv
from pathlib import Path
import os

import re

def comparator(elem):
    return elem[0]['id']

#results_filename = "../build/results_icp_vfc_arspw_arsgraph_20260213_1139_08.csv"
dir = "../build/results_batch/"

results = []

for file in os.listdir(dir):
    rows = []
    results_filename = dir + file
    if not os.path.isfile(results_filename):
        continue
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
                if not isinstance(value, str):
                    continue
                value = re.sub("\\s+", "", value)  # remove \n, \t from header
                value = value.replace(" ", "") # remove spaces from header
                key_stripped = re.sub("\\s+", "", key)  # remove \n, \t from header
                key_stripped = key_stripped.replace(" ", "") # remove spaces from header
                row_stripped[key_stripped] = float(value)
            rows.append(row_stripped)
            print(f"row_stripped: {row_stripped}")

    print(f"Loaded {len(rows)} rows, fields: {reader_fieldnames_stripped}")
    results.append(rows)

print(f"Loaded results for {len(results)} graphs")

results.sort(key=comparator)

count = 0
print("Scans with simmetry issue: ")
for rows in results:
    err = abs(rows[-1]['gt'] - rows[-1]['ars_graph'])
    if(err > 20):
        print(f"{rows[0]['id']}: err in last {err}")
        count += 1
print(f"total count: {count}")

avg_errors = []
last_errors = []

for rows in results:
    avg_err = {}
    last_err = {}
    avg_err['id'] = rows[0]['id']
    last_err['id'] = rows[0]['id']
    size = len(rows)
    for row in rows:
        for key, val in row.items():
            if(key == 'gt' or key == 'id' or key == 'odom'):
                continue
            if not key in avg_err.keys():
                avg_err[key] = 0.0
            err = abs(val - row['gt'])
            avg_err[key] = avg_err[key] + err
        for key in avg_err.keys():
            if(key != 'id'):
                avg_err[key] = avg_err[key]/size
    for key, val in rows[-1].items():
        if(key == 'gt' or key == 'id' or key == 'odom'):
            continue
        last_err[key] = abs(val - rows[-1]['gt'])

    avg_errors.append(avg_err)
    last_errors.append(last_err)

with open("errors.txt", "w") as f:
    for i in range(len(avg_errors)):
        avg_err = avg_errors[i]
        last_err = last_errors[i]
        id = avg_err['id']
        f.write(f"{int(id)}: \taverage:\t")
        for key, val in avg_err.items():
            if(key == 'gt' or key == 'id'):
                continue
            f.write(f"{key}: {val:.4f}\t")
        f.write("\n\tlast node:\t")
        for key, val in last_err.items():
            if(key == 'gt' or key == 'id'):
                continue
            f.write(f"{key}: {val:.4f}\t")
        f.write("\n\n")        

