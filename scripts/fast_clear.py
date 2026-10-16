import urllib.request
import json
from concurrent.futures import ThreadPoolExecutor

raw = json.loads(urllib.request.urlopen('http://182.254.135.92:8080/api/v1/fall/events?device_id=MM001&only_unhandled=true&limit=500').read().decode('utf-8'))
events = raw.get('data', [])
print(f"Fetched {len(events)} events to handle.")

def handle_one(e):
    eid = e['id']
    try:
        req = urllib.request.Request(f'http://182.254.135.92:8080/api/v1/fall/events/{eid}/handle', data=b'', method='POST')
        urllib.request.urlopen(req, timeout=3)
        return True
    except Exception:
        return False

with ThreadPoolExecutor(max_workers=20) as executor:
    results = list(executor.map(handle_one, events))

print(f"Handled {sum(results)} / {len(events)} events.")

# 再检查一次
raw_after = json.loads(urllib.request.urlopen('http://182.254.135.92:8080/api/v1/fall/events?device_id=MM001&only_unhandled=true&limit=500').read().decode('utf-8'))
print("Remaining unhandled:", len(raw_after.get('data', [])))

# 检查 latest
latest = json.loads(urllib.request.urlopen('http://182.254.135.92:8080/api/data/latest').read().decode('utf-8'))
print("Latest API fall status:", latest.get('data', {}).get('fall'))
