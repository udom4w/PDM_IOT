import json, hashlib

SRC = r'C:\Users\HP\AppData\Local\Temp\claude\flows_patch_source.json'
OUT = r'C:\Users\HP\Downloads\PDM_IOT\docs\engineering\evidence\line_audit_20260915\staging\staged_flows_ISOLATED_COPY.json'
HEALTH_JS = r'C:\Users\HP\Downloads\PDM_IOT\docs\engineering\evidence\line_audit_20260915\staging\health_logic_patched.js'
LINE_JS = r'C:\Users\HP\Downloads\PDM_IOT\docs\engineering\evidence\line_audit_20260915\staging\line_message_builder_patched.js'

with open(SRC, encoding='utf-8') as f:
    data = json.load(f)

with open(SRC, encoding='utf-8') as f:
    data_original = json.load(f)  # separate, untouched copy for comparison

with open(HEALTH_JS, encoding='utf-8') as f:
    health_new = f.read().rstrip('\n')

with open(LINE_JS, encoding='utf-8') as f:
    line_new = f.read().rstrip('\n')

byid = {n['id']: n for n in data}
old_health = byid['adf3dc5f003a516c']['func']
old_line = byid['e982d76b3ebe0b06']['func']

byid['adf3dc5f003a516c']['func'] = health_new
byid['e982d76b3ebe0b06']['func'] = line_new

with open(OUT, 'w', encoding='utf-8') as f:
    json.dump(data, f, indent=4, ensure_ascii=False)
    f.write('\n')

# Verify node count and only these two changed, against the untouched original
data2 = json.load(open(OUT, encoding='utf-8'))
byid_orig = {n['id']: n for n in data_original}
assert len(data2) == len(data_original), "node count changed!"
byid2 = {n['id']: n for n in data2}
changed = []
for nid in byid_orig:
    if json.dumps(byid_orig[nid], sort_keys=True, ensure_ascii=False) != json.dumps(byid2[nid], sort_keys=True, ensure_ascii=False):
        changed.append(nid)
print("node count:", len(data_original), "->", len(data2))
print("changed node ids:", changed)
assert changed == ['adf3dc5f003a516c', 'e982d76b3ebe0b06'] or set(changed) == {'adf3dc5f003a516c', 'e982d76b3ebe0b06'}, "unexpected extra changes!"

with open(OUT, 'rb') as f:
    print("staged file SHA256:", hashlib.sha256(f.read()).hexdigest())
